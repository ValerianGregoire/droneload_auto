# Setup locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
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

# Get ROS 2 code
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install dependencies using rosdep
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers rosidl_defaults"

# Build the code in the workspace
cd ~/ros2_humble/
colcon build --symlink-install

# Source ROS 2 in all terminals
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create a workspace
mkdir -p ~/ros2_ws/src

# Add the packages to the workspace 
cd ~/ros2_ws/src
git clone https://github.com/ValerianGregoire/droneload_auto.git
git clone https://github.com/ValerianGregoire/droneload_interfaces.git
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace in all terminals
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc

# Download Micro XRCE-DDS Agent
sudo snap install micro-xrce-dds-agent --edge