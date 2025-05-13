# AIR5051-Team3-FinalProject
AIR 5051-Team3-FinalProject
# Guard Robot

**Autonomous Security Patrol Robotics System**

---

## Overview

Guard Robot is an advanced intelligent security robot designed for autonomous patrolling and rapid intruder response. Leveraging state-of-the-art computer vision (YOLO), robust SLAM-based navigation, and a dexterous robotic manipulator, the system delivers comprehensive real-time surveillance, threat mitigation, and obstacle management. The project demonstrates a holistic integration of perception, motion planning, and human-robot interaction in dynamic indoor environments.

---

## Features

- **Autonomous Patrolling**
  - Utilizes SLAM (Gmapping) for real-time map construction and AMCL for precise localization.
  - Executes looped patrols along pre-defined routes with dynamic path planning and robust obstacle avoidance via `move_base`.

- **Intruder Detection & Response**
  - Deploys a YOLOv5n model for high-accuracy, real-time human detection.
  - Upon detecting an intruder, the robot tracks the target, activates an audible buzzer alarm, and engages the robotic arm to physically block unauthorized access.

- **Obstacle Removal**
  - Employs a depth camera to acquire raw 3D point clouds and computes Axis-Aligned Bounding Boxes (AABB) for obstacle representation.
  - Plans collision-free arm trajectories to safely push movable obstacles aside, ensuring uninterrupted patrol.

- **Interactive GUI**
  - Real-time video feed, mode toggling (Auto / Manual / Guard), and comprehensive robot status monitoring.
  - Manual controls for robot mobility and manipulator actions, fostering seamless human-robot collaboration.

---

## System Architecture

- **Navigation**: SLAM-based mapping and localization, MoveBase for path planning and obstacle avoidance.
- **Perception**: Real-time video analytics via YOLOv5n for intruder detection.
- **Manipulation**: Safe and efficient arm motion planning using MoveIt.
- **User Interface**: Python-based GUI for real-time interaction and supervision.

---

## Workflow

1. The robot autonomously patrols the designated route using SLAM and localizes itself with AMCL.
2. Live video is continuously analyzed for human presence; intruders are detected via YOLOv5n.
3. On intruder detection, the system issues an alert, tracks the target, and engages the manipulator for deterrence.
4. Any detected obstacles are processed using point cloud data; the manipulator removes them from the path.
5. Users can monitor and control the system via an intuitive GUI, with real-time status feedback.

---

## Installation

> **Platform**: Ubuntu 20.04 + ROS Noetic recommended.

1. **Clone the Repository**
    ```bash
    git clone https://github.com/yourusername/guard-robot.git
    cd guard-robot
    ```

2. **Install Dependencies**
    - ROS packages
    - Python 3.8, PyTorch, OpenCV, MoveIt, PCL, and other required libraries
    - YOLOv5 environment setup

3. **Build the Workspace**
    ```bash
    catkin_make
    source devel/setup.bash
    ```

4. **Launch the System**
    ```bash
    roslaunch guard_robot bringup.launch
    ```

5. **Start the GUI**
    ```bash
    python scripts/guard_robot_gui.py
    ```

---

## Future Work

- Resolve Qualcomm board compatibility for enhanced real-time performance and accelerated vision processing.
- Further optimize manipulator motion planning and multi-sensor data fusion.
- Expand towards multi-robot coordination and collaborative patrolling in large-scale environments.

---

## Contributors

- **Shujian Yu** (224040221)
- **Kaiyu Hu** (224040210)
- **Yongkang Qin** (224040247)

---

## License

This project is licensed under the MIT License.

---

**We welcome pull requests, suggestions, and collaborations!**
