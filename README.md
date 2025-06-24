# ROS 2 Humble Setup for CubePilot and RealSense

## 📦 Prerequisites

Ensure you have ROS 2 Humble installed on your Raspberry Pi (or Ubuntu system).  
Refer to the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

---

## 🧠 Install RTAB-Map

RTAB-Map is a real-time appearance-based mapping system.

```bash
sudo apt update
sudo apt install ros-humble-rtabmap
```

## 🛰️ Connect CubePilot to Raspberry Pi via Serial
Identify the serial device
```bash
ls /dev/tty*
sudo usermod -a -G dialout $USER
newgrp dialout

