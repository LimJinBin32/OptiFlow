import sys
sys.path.append("/home/jinbin/.local/lib/python3.11/site-packages")  # Add your package path

import time
import threading
from gpiozero import DistanceSensor
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2
import os
from datetime import datetime
from ultralytics import YOLO
import smbus2
from grove.display.jhd1802 import JHD1802
import mysql.connector
import requests
import json
import datetime as DT

# Pi setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pins for ultrasonic sensors
TRIGGER_PIN1, ECHO_PIN1 = 18, 24  # Entry Sensor for Row 1
TRIGGER_PIN2, ECHO_PIN2 = 23, 16  # Exit Sensor for Row 1
TRIGGER_PIN3, ECHO_PIN3 = 20, 21  # Entry Sensor for Row 2
TRIGGER_PIN4, ECHO_PIN4 = 26, 14  # Exit Sensor for Row 2

# Pins for servo motors
SERVO_PIN1 = 12
SERVO_PIN2 = 13
SERVO_PIN3 = 19

# Initialize the Grove LCD at I2C address 0x3E
lcd = JHD1802(0x3E)

# Initialize distance sensors
sensor1 = DistanceSensor(echo=ECHO_PIN1, trigger=TRIGGER_PIN1)
sensor2 = DistanceSensor(echo=ECHO_PIN2, trigger=TRIGGER_PIN2)
sensor3 = DistanceSensor(echo=ECHO_PIN3, trigger=TRIGGER_PIN3)
sensor4 = DistanceSensor(echo=ECHO_PIN4, trigger=TRIGGER_PIN4)

# Setup servos
GPIO.setup(SERVO_PIN1, GPIO.OUT)
GPIO.setup(SERVO_PIN2, GPIO.OUT)
GPIO.setup(SERVO_PIN3, GPIO.OUT)

servo1 = GPIO.PWM(SERVO_PIN1, 50)
servo2 = GPIO.PWM(SERVO_PIN2, 50)
servo3 = GPIO.PWM(SERVO_PIN3, 50)

servo1.start(2.5)
servo2.start(2.5)
servo3.start(2.5)

# Directory to save snapshots
snapshot_dir = "snapshots"
os.makedirs(snapshot_dir, exist_ok=True)

# Initialize PiCamera2
camera = Picamera2()
camera_config = camera.create_preview_configuration(main={"size": (640, 480)})
camera.configure(camera_config)
camera.start()
print("[YOLO] Camera started for object detection.")

# Initialize YOLO model (detect people only)
model = YOLO("yolov8n.pt")

# Global variables
row1 = 0
row2 = 0
yolo_person_count = 0
Gate1 = None
Gate2 = None

max_people_per_row = 3  # for testing
unlock_threshold = 5    # for testing

lock = threading.Lock()  # Thread safety
stop_event = threading.Event()

# We'll track the current camera servo angle
camera_servo_angle = 90  # start centered

# Laptop Flask server URL
SERVER_URL = "http://172.27.154.31:5000/receive_data"


def send_data(row1_count, row2_count, yolo_count, Gate1, Gate2):
    """Send data to laptop Flask server."""
    timestamp = DT.datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Format timestamp

    data = {
        "row1_count": row1_count,
        "row2_count": row2_count,
        "yolo_count": yolo_count,
        "Gate1": Gate1,
        "Gate2": Gate2,
        "timestamp": timestamp
    }

    try:
        response = requests.post(SERVER_URL, json=data)
        if response.status_code == 200:
            print("? Data sent successfully!")
        else:
            print(f"?? Error: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"? Failed to send data: {e}")

        
#------------------------Code-----------------------
def capture_snapshot():
    """ Capture an image and return the file path """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    image_path = os.path.join(snapshot_dir, f"snapshot_{timestamp}.jpg")
    camera.capture_file(image_path)
    print(f"[??] Captured snapshot: {image_path}")
    return image_path


def analyze_snapshot(image_path):
    """ Run YOLO on the snapshot and save processed image """
    frame = cv2.imread(image_path)
    results = model(frame)

    person_count = 0
    for r in results:
        for box in r.boxes:
            person_count += 1
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = round(box.conf[0].item(), 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Person {conf}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

    processed_path = image_path.replace(".jpg", "_processed.jpg")
    cv2.imwrite(processed_path, frame)
    print(f"[? YOLO] Processed snapshot saved: {processed_path} (Detected: {person_count})")
    return person_count


def set_servo_angle(servo, pin, angle):
    """Rotate servo to given angle (0-180) at 50Hz, then stop."""
    duty = angle / 18 + 2
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)  # let servo finish
    servo.ChangeDutyCycle(0)


# ------------------ Row 1 Handler ------------------ #
def row1_handler():
    global row1
    sensor1_prev = 0
    sensor2_prev = 0

    while not stop_event.is_set():
        sensor1_near = sensor1.distance < 0.2
        sensor2_near = sensor2.distance < 0.2

        sensor1_state = 1 if sensor1_near else 0
        sensor2_state = 1 if sensor2_near else 0

        with lock:
            # If sensor1 transitions 1->0 => row1++
            if sensor1_prev == 1 and sensor1_state == 0 and row1 < 999:
                row1 += 1
                print(f"âœ… [ENTRY] Row 1 => {row1}")

            # If sensor2 transitions 1->0 => row1--
            if sensor2_prev == 1 and sensor2_state == 0 and row1 > 0:
                row1 -= 1
                print(f"ðŸšª [EXIT] Row 1 => {row1}")

        sensor1_prev = sensor1_state
        sensor2_prev = sensor2_state
        time.sleep(0.1)

    print("[THREAD STOP] Row 1 handler has stopped.")


# ------------------ Row 2 Handler ------------------ #
def row2_handler():
    global row2
    sensor3_prev = 0
    sensor4_prev = 0

    while not stop_event.is_set():
        sensor3_near = sensor3.distance < 0.2
        sensor4_near = sensor4.distance < 0.2

        sensor3_state = 1 if sensor3_near else 0
        sensor4_state = 1 if sensor4_near else 0

        with lock:
            if sensor3_prev == 1 and sensor3_state == 0 and row2 < 999:
                row2 += 1
                print(f"âœ… [ENTRY] Row 2 => {row2}")

            if sensor4_prev == 1 and sensor4_state == 0 and row2 > 0:
                row2 -= 1
                print(f"ðŸšª [EXIT] Row 2 => {row2}")

        sensor3_prev = sensor3_state
        sensor4_prev = sensor4_state
        time.sleep(0.1)

    print("[THREAD STOP] Row 2 handler has stopped.")


# ------------------ Servo Controller ------------------ #

def servo_controller():
    global Gate1, Gate2
    while not stop_event.is_set():
        with lock:
            total_people = row1 + row2
            print(f"[DEBUG] row1={row1}, row2={row2}, total={total_people}")
            
            # If Row1 is full, lock Row1 & open Row2
            if row1 >= max_people_per_row and row2 < max_people_per_row:  #90 IS OPEN, 0 IS CLOSE FOR SERVO 1
                set_servo_angle(servo1, SERVO_PIN1, 0)
                set_servo_angle(servo2, SERVO_PIN2, 90)
                Gate1 = 'Close'
                Gate2 = 'Open'

            # If Row2 is full, lock Row2 & open Row1 # 90 IS OPEN, 180 US CLOSE FOR SERVO 2
            elif row2 >= max_people_per_row and row1 < max_people_per_row:
                set_servo_angle(servo1, SERVO_PIN1, 90)
                set_servo_angle(servo2, SERVO_PIN2, 180)
                Gate1 = 'Open'
                Gate2 = 'Close'

            # If both full, lock both
            elif row1 >= max_people_per_row and row2 >= max_people_per_row:
                set_servo_angle(servo1, SERVO_PIN1, 0)
                set_servo_angle(servo2, SERVO_PIN2, 180)
                Gate1 = 'Close'
                Gate2 = 'Close'

            else:
                # Neither full => unlock both
                set_servo_angle(servo1, SERVO_PIN1, 90)
                set_servo_angle(servo2, SERVO_PIN2, 90)
                Gate1 = 'Open'
                Gate2 = 'Open'

        time.sleep(0.5)

    print("[THREAD STOP] Servo controller has stopped.")


# ------------------ Camera Servo Controller ------------------ #
def camera_servo_controller():
    global camera_servo_angle
    set_servo_angle(servo3, SERVO_PIN3, 90)
    camera_servo_angle = 90

    while not stop_event.is_set():
        with lock:
            r1_full = (row1 >= max_people_per_row)
            r2_full = (row2 >= max_people_per_row)

            if r1_full and not r2_full:
                set_servo_angle(servo3, SERVO_PIN3, 120)
                camera_servo_angle = 120

            elif r2_full and not r1_full:
                set_servo_angle(servo3, SERVO_PIN3, 60)
                camera_servo_angle = 60

            elif r1_full and r2_full:
                # both full -> maybe center or some specific angle
                set_servo_angle(servo3, SERVO_PIN3, 120)
                camera_servo_angle = 120
                time.sleep(3)
                set_servo_angle(servo3, SERVO_PIN3, 60)
                camera_servo_angle = 60

            else:
                # neither full => center
                set_servo_angle(servo3, SERVO_PIN3, 90)
                camera_servo_angle = 90

        time.sleep(1)

    print("[THREAD STOP] camera_servo_controller has stopped.")


# ------------------ LCD Updater ------------------ #
def lcd_updater():
    while not stop_event.is_set():
        with lock:
            r1 = row1
            r2 = row2

        lcd.clear()
        time.sleep(0.05)
        lcd.setCursor(0, 0)
        lcd.write(f"Row1: {r1}")
        lcd.setCursor(1, 0)
        lcd.write(f"Row2: {r2}")
        time.sleep(0.5)

    print("[THREAD STOP] LCD updater has stopped.")


# ------------------ Add a scrolling helper, if you want to show override messages ------------------ #
def scroll_long_text(message, row=0):
    """
    Scrolls 'message' across the specified LCD row (0 or 1) once.
    If message fits in 16 chars, it just displays for 2s.
    """
    lcd.clear()
    time.sleep(0.1)
    lcd_width = 16

    if len(message) <= lcd_width:
        lcd.setCursor(row, 0)
        lcd.write(message)
        time.sleep(2)
        return

    for i in range(len(message) - lcd_width + 1):
        lcd.setCursor(row, 0)
        lcd.write(message[i : i + lcd_width])
        time.sleep(0.3)


# ------------------ Camera Handler (Override Logic) ------------------ #
def camera_handler():
    global yolo_person_count, camera_servo_angle, row1, row2
    print("[?? YOLO] Snapshot-based object detection started.")
    last_angle = None

    try:
        while not stop_event.is_set():
            with lock:
                angle_now = camera_servo_angle

            # If the servo angle just changed
            if angle_now != last_angle:
                # We ONLY snapshot if it's now 60 or 120
                if angle_now in [60, 120]:
                    image_path = capture_snapshot()
                    detected_count = analyze_snapshot(image_path)

                    # Override row1 or row2 if different
                    with lock:
                        if angle_now == 120:
                            # We assume 120 => Row1
                            if detected_count != row1:
                                old_val = row1
                                row1 = detected_count
                                yolo_person_count = detected_count
                                print(f"[YOLO] Overriding Row1 from {old_val} to {detected_count}")
                                scroll_long_text(f"YOLO Overrided R1: {old_val} to {detected_count} ")

                        elif angle_now == 60:
                            # We assume 60 => Row2
                            if detected_count != row2:
                                old_val = row2
                                row2 = detected_count
                                yolo_person_count = detected_count
                                print(f"[YOLO] Overriding Row2 from {old_val} to {detected_count}")
                                scroll_long_text(f"YOLO Overrided R2: {old_val} to {detected_count} ")

                # Update last_angle
                last_angle = angle_now

            # If angle hasn't changed, do nothing
            time.sleep(1)

    finally:
        camera.stop()
        print("[?? YOLO] Camera and detection stopped.")

# ------------------ Data Logging ------------------ #

def log_thread_function():
    """ Continuously sends data to laptop Flask server every 5 seconds. """
    while True:
        row1_count = row1  # Replace with actual value
        row2_count = row2  # Replace with actual value
        yolo_count = yolo_person_count  # Replace with actual value

        send_data(row1_count, row2_count, yolo_count, Gate1, Gate2)  # Send to laptop
        time.sleep(5)  # Send every 5 seconds


# ------------------ Start Threads ------------------ #
t1 = threading.Thread(target=row1_handler, daemon=True)
t2 = threading.Thread(target=row2_handler, daemon=True)
t3 = threading.Thread(target=servo_controller, daemon=True)
t4 = threading.Thread(target=lcd_updater, daemon=True)
t5 = threading.Thread(target=camera_handler, daemon=True)
t6 = threading.Thread(target=camera_servo_controller, daemon=True)
t7 = threading.Thread(target=log_thread_function, daemon=True)  # NEW LOGGING THREAD


t1.start()
t2.start()
t3.start()
t4.start()
t5.start()
t6.start()
t7.start()

print("ðŸš€ System Started")

# ------------------ Main Loop ------------------ #
try:
    # Set initial servo angles
    set_servo_angle(servo1, SERVO_PIN1, 90)
    set_servo_angle(servo2, SERVO_PIN2, 90)
    set_servo_angle(servo3, SERVO_PIN3, 90)

    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n[INFO] Caught KeyboardInterrupt â€” stopping threads.")
    stop_event.set()
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
    t6.join()
    t7.join()

    # Cleanup and shutdown
    servo1.stop()
    servo2.stop()
    servo3.stop()
    GPIO.cleanup()

    print("ðŸ›‘ System Stopped")