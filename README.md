# 🚗 Autonomous Vehicle Navigation System

### 📚 Table of Contents
- [Project Overview](#project-overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Technologies Used](#technologies-used)
- [Installation](#installation)

## Project Overview
This project is focused on developing an **Autonomous Vehicle Navigation System** that enables vehicles to navigate safely and efficiently from a source to a destination. The system addresses key challenges in autonomous navigation, including path following, path planning, obstacle detection, and avoidance.

The project leverages a combination of advanced simulation, robotics, and programming technologies to achieve robust and reliable autonomous navigation in complex environments.

## System Architecture
The architecture of the Autonomous Vehicle Navigation System is **layered** and modular, with each layer focusing on specific tasks to achieve autonomous navigation.

![System Architecture](Artifacts/Diagrams/Final%20Architection.png)

## Proposed Solution
The data flow through the system is illustrated below, showcasing interactions among the perception, planning, control, and simulation interface modules.

![Data Flow Diagram](Artifacts/Diagrams/Proposed%20Solution.png)

## Key Features
- **🛤️ Path Planning**: Generates an optimal path from the source to the destination.
- **➡️ Path Following**: Ensures the vehicle follows the planned path accurately.
- **🔍 Obstacle Detection**: Uses sensors to detect obstacles in the environment.
- **🚧 Obstacle Avoidance**: Implements algorithms to navigate around obstacles.
- **🖥️ Simulation**: Utilizes the CARLA simulator for realistic testing environments.
- **🔴 Detecting Traffic Lights**: Utilizes a monocular camera to identify traffic light colors (red, yellow, green) and their states.
- **📸 Image Processing**: Applies computer vision techniques using OpenCV for effective recognition.
- **🚦 Decision Making**: Integrates with the planning module to make decisions based on the detected traffic light states, ensuring compliance with traffic rules.


## Technologies Used
- **🚙 CARLA Simulator**: For realistic vehicle simulation.
- **🤖 ROS Noetic**: For robotic system development.
- **🔌 CARLA-ROS Bridge**: For seamless integration between CARLA and ROS.
- **🐍 Python (rospy)**: For ROS-based programming.
- **🖼️ OpenCV**: For computer vision tasks.


## Installation

### ⚙️ Prerequisites
- **ROS Noetic** installed on Ubuntu 20.04.
- **CARLA Simulator** installed.
- **Python 3.8** or higher.
