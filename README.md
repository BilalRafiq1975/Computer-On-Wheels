# Autonomous Vehicle Navigation System

### Table of Contents
- [Project Overview](#project-overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Technologies Used](#technologies-used)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Testing](#testing)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Project Overview
This project is focused on developing an **Autonomous Vehicle Navigation System** that enables vehicles to navigate safely and efficiently from a source to a destination. The system addresses key challenges in autonomous navigation, including path following, path planning, obstacle detection, and avoidance.

The project leverages a combination of advanced simulation, robotics, and programming technologies to achieve robust and reliable autonomous navigation in complex environments.

## Key Features
- **Path Planning**: Generates an optimal path from the source to the destination.
- **Path Following**: Ensures the vehicle follows the planned path accurately.
- **Obstacle Detection**: Uses sensors to detect obstacles in the environment.
- **Obstacle Avoidance**: Implements algorithms to navigate around obstacles.
- **Simulation**: Utilizes the CARLA simulator for realistic testing environments.

## System Architecture

The system is divided into several modules:

- **Perception Module**: Processes sensor data for obstacle detection.
- **Planning Module**: Handles path planning and decision-making.
- **Control Module**: Manages vehicle control to follow the planned path.
- **Simulation Interface**: Integrates with CARLA via the CARLA-ROS bridge.

## Technologies Used
- **CARLA Simulator**: For realistic vehicle simulation.
- **ROS Noetic**: For robotic system development.
- **CARLA-ROS Bridge**: For seamless integration between CARLA and ROS.
- **Python (rospy)**: For ROS-based programming.
- **OpenCV**: For computer vision tasks.

## Installation

### Prerequisites
- **ROS Noetic** installed on Ubuntu 20.04.
- **CARLA Simulator** installed.
- **Python 3.8** or higher.
