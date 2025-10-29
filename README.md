# xrce_ros2_control

**xrce_ros2_control** provides different controllers and useful tools to interface ROS 2 with **MicroXRCEAgent** and **PX4-Autopilot**.  
It enables remote control of UAVs and other robots using XRCE-DDS, while integrating standard ROS 2 control abstractions.

---

## Features

- Multiple ROS 2 controllers for PX4 UAVs  
- Tools for MicroXRCEAgent communication  
- Supports multi-UAV setups with topic remapping and namespaces  
- Includes launch files, test scripts, and example configurations  

---

## Prerequisites

- ROS 2 Humble / Rolling / Foxy (depending on your environment)  
- PX4-Autopilot (with ROS 2 bridge support)  
- MicroXRCEAgent (for XRCE-DDS communication)  
- Python 3.10+  

---

## Installation

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/viswans2132/xrce_ros2_control.git
