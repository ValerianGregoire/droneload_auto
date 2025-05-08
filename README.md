# Drone Automation for the Safran Droneload Challenge

This repository contains the source code, configuration files, and documentation for our drone automation system developed for the **Safran Droneload Challenge**. The system is based on **ROS 2 Humble Hawksbill** and integrates with the **PX4 Autopilot** to enable autonomous mission execution.

## Project Overview

- **Platform**: PX4-compatible drone with a Pixhawk 6C and a Raspberry Pi
- **Middleware**: ROS 2 Humble Hawksbill
- **OS**: Ubuntu 22.04 Jammy Jellyfish
- **Main Features**:
  - Autonomous mission planning and execution
  - Real-time telemetry monitoring
  - Payload delivery and pickup logic
  - Simulation support with Gazebo and PX4 SITL
  - Multi-node ROS 2 architecture for modularity

---

## Automatic ROS 2 Humble Setup (Ubuntu 22.04)

This repository must be placed in `~/ros2_ws/src/`, but also contains the code to install ROS 2 if not done already. The repository needs to be moved to the correct path once ROS 2 is installed to run.

### Shell script execution

Run this command in a terminal within the repository folder to install ROS 2 automatically. Note that the automatic installation will create a copy of the current directory in `~/ros2_ws/src/`.

```shell
chmod +x install_ros2.sh
./install_ros2.sh
```

---

## Manual ROS 2 Humble Setup (Ubuntu 22.04)

Run the commands below in a terminal to set up your development environment on a clean Ubuntu 22.04 installation without using the shell script. 

### 1. Setup Locale

```shell
bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Add the ROS 2 apt repository

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install development tools and ROS tools

```shell
sudo apt update && sudo apt install -y \
  build-essential \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```

### 4. Get ROS 2 code

```shell
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
```

### 5. Install dependencies using rosdep

```shell
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

### 6. Build the code in the workspace

```shell
cd ~/ros2_humble/
colcon build --symlink-install
```

### 7. Source ROS 2 in all terminals
```shell
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 8. Create a workspace
```shell
mkdir -p ~/ros2_ws/src
```

### 9. Add the packages to the workspace 
Clone the package in the workspace (again) and clone the interfaces package to use topics and services.

```shell
cd ~/ros2_ws/src
git clone https://github.com/ValerianGregoire/droneload_auto.git
git clone https://github.com/ValerianGregoire/droneload_interfaces.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
```

### 10. Build the workspace
```shell
cd ~/ros2_ws
colcon build
```

### 11. Source the workspace in all terminals
```shell
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Manual PX4-ROS2 Bridge Setup

Run the commands below in a terminal to set up the PX4-ROS2 bridge without using the shell script. 

### 12. Download Micro XRCE-DDS Agent
```shell
sudo snap install micro-xrce-dds-agent --edge
```

### 13. Add the agent package and its dependencies to the workspace
```shell
cd ~/ros2_ws/src
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ~/ros2_ws/
colcon build
```