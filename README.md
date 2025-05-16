# OptiFlow
An intelligent queue management system powered by Raspberry Pi that uses sensors, computer vision, and automation to manage two queues in real-time.

---

## 📌 Description

This project combines hardware and software components to monitor and control two entry lines. It uses:
- Ultrasonic sensors to track queue entries and exits  
- A PiCamera and YOLOv8 for real-time person detection  
- Servo motors to control access gates  
- An LCD to display live queue counts  
- Flask integration to send live data to a remote server

The system supports snapshot capture, override logic via YOLO, and multi-threaded queue management for robust performance.

---

## ⚙️ Technologies & Components

- Raspberry Pi  
- Python  
- YOLOv8 (Ultralytics)  
- PiCamera2  
- gpiozero & RPi.GPIO  
- Grove LCD (JHD1802)  
- MySQL (for optional logging)  
- Flask (server-side logging)  
- OpenCV  

---

## 🚀 Features

- Real-time person counting with YOLOv8  
- Dual-queue monitoring using ultrasonic sensors  
- Servo-controlled gate management  
- LCD display for live queue updates  
- Periodic data logging to a remote server  
- Snapshot capture with object detection overlay  

---

## 📂 File Overview

- `smart_queue_control_system.py`: Main system script with all core functionality  
- `/snapshots`: Folder for storing captured and processed images - this folder is automcatically created when the script runs

---

## 📝 Notes

- Ensure all packages are installed and accessible on the Pi  
- Modify `SERVER_URL` to point to your Flask endpoint  
- Adjust pin assignments if your hardware differs  

---

# 📦 Required Packages

Install these packages on your Raspberry Pi environment:

```bash
pip install ultralytics
pip install opencv-python
pip install gpiozero
pip install RPi.GPIO
pip install smbus2
pip install mysql-connector-python
pip install requests
```
You may also need to manually install Grove display libraries and PiCamera2 support if not preloaded.

---

## 👤 Author

**Lim Jin Bin**  
Y2 AI & Data Engineering — Nanyang Polytechnic  
Module: Internet Of Things Application

