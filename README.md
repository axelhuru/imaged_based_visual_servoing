# Image-Based Visual Servoing - Franka Panda

IBVS demo where a camera mounted on the Panda robot's end-effector tracks and follows a red ball.


Building VIsP from source is required to build project, this sequence worked for us:
## Setup
```bash
sudo apt install -y ros-humble-vision-msgs ros-humble-image-transport \
ros-humble-cv-bridge ros-humble-rclcpp ros-humble-geometry-msgs


mkdir -p ~/visp_ws/src
cd ~/visp_ws/src
git clone https://github.com/lagadic/visp.git
cd visp
git checkout 3.6.1  


cd ~/visp_ws
mkdir build
cd build
cmake ../src/visp \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=ON \
    -DUSE_FFMPEG=ON \
    -DUSE_OPENGL=ON \
    -DUSE_QT=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local


make -j$(nproc)
sudo make install


cd ~/visp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install


```


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

