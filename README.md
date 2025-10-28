# Image-Based Visual Servoing - Franka Panda

IBVS demo where a camera mounted on the Panda robot's end-effector tracks and follows a red ball.

## Setup

### Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-kdl-parser \
    libopencv-dev \
    libvisp-dev \
    liborocos-kdl-dev
```

### Build Workspace

```bash
# Create workspace
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src

# Copy your packages here:
# - ibvs_controller
# - visual_servoing
# - my_simple_controllers
# - panda_ros2_gazebo

# Build
cd ~/panda_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Source workspace
cd ~/panda_ws
source install/setup.bash

# Launch everything
ros2 launch panda_ros2_gazebo gazebo.launch.py
```

This starts Gazebo, spawns the robot and ball, and launches all control nodes. The robot will automatically start tracking the red ball.

