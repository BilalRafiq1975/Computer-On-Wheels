#!/bin/bash

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Source CARLA setup
export PYTHONPATH=$PYTHONPATH:/home/maanz-ai/Downloads/CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
export CARLA_ROOT=/home/maanz-ai/Downloads/CARLA_0.9.13
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg

# Source CARLA ROS bridge setup
source /home/maanz-ai/carla-ros-bridge/catkin_ws/devel/setup.bash

# Set ROS workspace
export ROS_WORKSPACE=/home/maanz-ai/carla-ros-bridge/catkin_ws
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WORKSPACE/src
source $ROS_WORKSPACE/devel/setup.bash

# Set CARLA PythonAPI path
export CARLA_PYTHONAPI_PATH="/home/maanz-ai/Downloads/CARLA_0.9.13/PythonAPI"
export PYTHONPATH="$PYTHONPATH:$CARLA_PYTHONAPI_PATH"
export PYTHONPATH="$PYTHONPATH:/home/maanz-ai/Downloads/CARLA_0.9.13/PythonAPI/carla/agents"
export PYTHONPATH="$PYTHONPATH:/home/maanz-ai/Downloads/CARLA_0.9.13/PythonAPI/carla"

# Run CARLA
cd /home/maanz-ai/Downloads/CARLA_0.9.13/
./CarlaUE4.sh &

# Wait for CARLA to launch
sleep 10

# Source ROS bridge setup
source /home/maanz-ai/carla-ros-bridge/catkin_ws/devel/setup.bash

# Run ROS bridge
roslaunch carla_ros_bridge carla_ros_bridge.launch town:=Town10UHD_opt passive:=True

