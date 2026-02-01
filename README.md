# Argos — ROS2 Quadruped Locomotion Project



**Argos** is a ROS2-based software framework for controlling a quadruped robot — currently focused on basic locomotion behaviors such as walking forward, stopping, and default pose control.

The project demonstrates:
- ROS2 node design in C++ and Python
- Keyboard teleoperation (WASD + Space for commands)
- Gait interpolation logic
- Stand / default posture control
- Modular architecture for motion commands

This repository is developed as an open-source robotics project for both personal learning and community collaboration.

---

## Features

### Motion Control
- Interpolated gait controller for walking
- Smooth transitions between pose keyframes
- Support for:
  - `walk`
  - `stop`
  - `jump` 

### ⌨️ Teleoperation
- Keyboard input node:
  - `W` – walk forward
  - `S` – backward
  - `X` – Stop
  - `Space` – jump 

The keyboard driver publishes ROS2 messages that control the locomotion system.

---

---

## 🧠 Design Overview

### Modular ROS2 Architecture

**Teleop Node**
- Reads keyboard inputs
- Publishes to a custom `/motion_cmd` topic as ROS2 String messages

**Interpolated Publisher**
- Subscribes to `/motion_cmd`
- Tracks current motion state (`forward`, `idle`)
- Smoothly interpolates between key leg positions (gait phases)

**Default Pose Controller**
- Subscribes to `/motion_cmd`
- When receiving `"stop"`, sends default joint positions to `/joint_position_controller/commands`

This design enforces separation of input, gait logic, and pose output, which is essential for production-grade robotics.

---

# Roadmap
- Add IMU and LiDAR sensors for SLAM
- Improve gait


