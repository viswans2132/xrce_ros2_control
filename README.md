# xrce_ros2_control

xrce_ros2_control integrates ROS 2 control abstractions with MicroXRCEAgent and PX4 autopilot to enable remote control of UAVs (and other robots) over XRCE-DDS. The package provides controllers, utilities, launch files, and examples to run the controller with an external positioning setup using ROS 2 and XRCE-DDS.

- Project: https://github.com/viswans2132/xrce_ros2_control

Table of contents
- About
- Features
- Supported platforms
- Prerequisites
- Installation
- Build & source
- Quick start (single UAV)
- Multi-UAV / namespaces & remapping
- Configuration (controllers & topics)
- Running MicroXRCEAgent
- Tests & examples
- Troubleshooting
- Contributing
- License & contact

About
-----
This repository provides ROS 2 controllers and helper tools for relaying control commands and state between ROS 2 and embedded devices running Micro XRCE-DDS clients (for example PX4 on microcontrollers). It exposes familiar ROS 2 control interfaces while using MicroXRCEAgent to communicate over XRCE-DDS.

Features
--------
- ROS 2 controllers tailored for PX4 and embedded XRCE clients
- Tools and launch files to run MicroXRCEAgent and bridge to PX4
- Support for multi-UAV setups with namespaces and topic remapping
- Example configurations, test scripts and launch files
- Designed to work with ROS 2 (Foxy / Humble / Rolling), PX4 and MicroXRCEAgent

Supported platforms
-------------------
- ROS 2: Foxy, Humble, Rolling (confirm compatibility for your target ROS 2 distro)
- PX4-Autopilot (with ROS 2 bridge or when used with MicroXRCE clients)
- MicroXRCEAgent (XRCE-DDS agent for microcontrollers)
- Development tested on Ubuntu LTS (see specific distro docs for exact versions)

Prerequisites
-------------
- ROS 2 installed (Foxy / Humble / Rolling) and sourced
- PX4 Autopilot and its ROS 2 bridge (if using PX4-SITL or onboard PX4)
- MicroXRCEAgent (https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- Python 3.8+ (3.10+ recommended for scripts)
- colcon (for building ROS 2 workspaces)
- Typical PX4 build dependencies if you run PX4 SITL

Installation
------------
1. Create and enter your ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone this repository:
   ```bash
   git clone https://github.com/viswans2132/xrce_ros2_control.git
   ```

3. Install OS / ROS dependencies (example using rosdep):
   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

Build & source
---------------
- Always source your ROS 2 distribution and the workspace after building:
  ```bash
  source /opt/ros/<distro>/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```
- Replace `<distro>` with `foxy`, `humble`, or your ROS 2 version.

Quick start (single UAV)
------------------------
1. Start MicroXRCEAgent on the machine that will bridge ROS 2 to XRCE clients. Agent can run in UDP or TCP mode. Example for Simulation (UDP server on port 8888):
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
   See "Running MicroXRCEAgent" below for more detail.

2. Launch PX4 (SITL) or power your vehicle running Micro XRCE-DDS client and point it to the MicroXRCEAgent.

3. Launch the ROS 2 nodes and controllers provided in this repository:
   - Use the provided launch files (examples in `launch/`) to spawn controllers and any helper nodes. Example:
     ```bash
     ros2 launch xrce_ros2_control px4_control.launch.py
     ```
   - Adjust the launch file name and parameters according to the controller and robot.

Configuration (controllers & topics)
------------------------------------
- Controller configurations, topic names, and parameter files are typically YAML files in `config/` (or similar).
- Controller types supported (examples):
  - Position controller
  - Velocity controller
  - Attitude / rate controllers
  - PX4-specific command interfaces

Inspect the repository's `config/` and `launch/` directories to find and adapt the presets for your platform and use-case.

Running MicroXRCEAgent
----------------------
- MicroXRCEAgent is the bridge between ROS 2 and XRCE-DDS clients. Install it from the eProsima Micro XRCE-DDS repository and run it before the client connects.
- Common usage for simulation:
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```
  - With PixHawk Hardware:
    ```bash
    sudo MicroXRCEAgent serial --dev /dev/ttyACM0 -b 115200
    ```

Tests & examples
----------------
- Look in `test/`, `launch/` and `examples/` (if present) for automated test scripts and example launch files.
- Use ROS 2 bag and rqt tools to inspect topics during integration testing.
- If you have a PX4 SITL setup, integrate this repo's launch files with the PX4 SITL startup sequence.

Troubleshooting
---------------
- If the MicroXRCEAgent does not show connections:
  - Check firewall rules / ports.
  - Ensure the XRCE-DDS client is configured to connect to the agent’s host and port.
- If ROS 2 topics are missing:
  - Ensure workspace is sourced and nodes are running (use `ros2 node list`, `ros2 topic list`).
- Controller failing to start:
  - Check parameter YAML syntax and controller manager logs.

Contributing
------------
Contributions, issues and pull requests are welcome.
- Open an issue describing the bug or feature request.
- Provide a minimal reproduction or steps to test.
- Follow common ROS 2 package guidelines and include tests where possible.

Suggested workflow:
1. Fork the repo
2. Create a feature branch
3. Run and test locally
4. Open a pull request with a clear description and testing instructions

License & contact
-----------------
- License: Please check the repository's LICENSE file for the license used.
- Contact / author: See the repository owner (https://github.com/viswans2132) for more details.

Acknowledgements
----------------
- Based on Micro XRCE-DDS by eProsima
- PX4 Autopilot integration examples and community

Notes
-----
- This README is a general, user-friendly guide. Check the repository for more detailed launch files and configuration examples tailored to your ROS 2 distribution, PX4 version and MicroXRCEAgent setup.
