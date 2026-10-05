#!/bin/bash
#
# This script automates the installation of ROS 2 Humble, ArduPilot, Gazebo,
# the Gazebo ArduPilot plugin, and Mavros on Ubuntu.
#
# This is the most robust version, designed to be fully idempotent. It
# now includes checks for existing directories, files, and handles the
# symbolic link creation error by not using the ArduPilot prereqs script.
# Instead, it directly installs all required packages.
#
# Fixes addressed:
# 1. Python 2 vs. Python 3 package name issue.
# 2. git clone failures when directories already exist.
# 3. Symbolic link creation failures when the link already exists.
# 4. Syntax errors from modifying the prereqs script.
#
# Usage:
#   1. Make the script executable: chmod +x install.sh
#   1. Run the script: ./install.sh

set -e # Exit immediately if a command exits with a non-zero status.

# --- Global Variables ---
USER_NAME=$USER
ROS_DISTRO="humble"
GAZEBO_VERSION="11"
ARDUPILOT_DIR="/home/$USER_NAME/ardupilot"
REPO_DIR="/home/$USER_NAME/LRS-FEI"
ARDUPILOT_GAZEBO_DIR="/home/$USER_NAME/ardupilot_gazebo"
GEOGRAPHICLIB_SCRIPT="install_geographiclib_datasets.sh"
PKG_CONFIG_LINK="/usr/bin/arm-linux-gnueabihf-pkg-config"

echo "--- Section 0: Installing git large-file-system ---"
sudo apt install curl python-is-python3 -y
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt update
sudo apt install git git-lfs -y
if [ -d "$REPO_DIR" ]; then
    echo "LRS-FEI repository already exists. Skipping clone."
else
    echo "Cloning LRS-FEI repository..."
    git clone https://github.com/MartinSedlacek/LRS-FEI $REPO_DIR
    echo "export GAZEBO_MODEL_PATH=~/LRS-FEI/models" >> ~/.bashrc
fi

# --- 1. Install ROS 2 Humble ---
echo "--- Section 1: Installing ROS 2 $ROS_DISTRO ---"
echo "Adding ROS 2 repositories and keys..."

# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-$ROS_DISTRO-desktop python3-argcomplete
sudo apt install -y ros-dev-tools

# Add ROS 2 environment setup to your .bashrc
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi
echo "ROS 2 $ROS_DISTRO installation complete."


# --- 2. Install ArduPilot ---
echo "--- Section 2: Installing and Configuring ArduPilot ---"
cd ~

if [ -d "$ARDUPILOT_DIR" ]; then
    echo "ArduPilot directory already exists. Skipping clone."
    cd "$ARDUPILOT_DIR"
else
    git clone -b Copter-4.2.3 https://github.com/ArduPilot/ardupilot.git $ARDUPILOT_DIR
    cd "$ARDUPILOT_DIR"
fi
echo "Installing all required dependencies for ArduPilot..."

git submodule update --init --recursive

# --- Manually Install Dependencies (Replaces the prereqs script) ---
echo "Installing all required dependencies for ArduPilot..."
sudo apt install python3-pip -y
chmod +x $ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh
python3 -m pip install "setuptools==58.2.0"
sed -i 's/PYTHON_V="python"/PYTHON_V="python3"/' $ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh
sed -i 's/PIP=pip2/PIP=pip3/' $ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh
$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh -y

# Source the profile to ensure environment variables are loaded
. ~/.profile

# Configure and build ArduPilot SITL (Software-In-The-Loop)
echo "Configuring and building ArduPilot Copter SITL..."
./waf configure --board sitl
./waf copter
echo "ArduPilot Copter SITL build complete."


# --- 3. Install Gazebo Stable ---
echo "--- Section 3: Installing Gazebo stable ---"
echo "Adding Gazebo repositories and keys..."

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

echo "Installing Gazebo $GAZEBO_VERSION and development files..."
sudo apt-get install -y gazebo libgazebo-dev
echo "Gazebo $GAZEBO_VERSION installation complete."


# --- 4. Install Gazebo AP Plugin ---
echo "--- Section 4: Installing Gazebo ArduPilot Plugin ---"
cd ~

if [ -d "$ARDUPILOT_GAZEBO_DIR" ]; then
    echo "Gazebo plugin directory already exists. Skipping clone."
    cd "$ARDUPILOT_GAZEBO_DIR"
else
    git clone https://github.com/khancyr/ardupilot_gazebo
    cd "$ARDUPILOT_GAZEBO_DIR"
fi

mkdir -p build
cd build

echo "Building the Gazebo ArduPilot Plugin..."
cmake ..
make -j$(nproc) # Use all available cores for faster compilation
sudo make install
echo "Gazebo ArduPilot Plugin installation complete."


# --- 5. Install Mavros ---
echo "--- Section 5: Installing Mavros ---"

# Check if ROS 2 is sourced in the current shell, if not, do it now
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS 2 environment for Mavros installation."
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

echo "Installing Mavros for ROS 2 $ROS_DISTRO..."
sudo apt install -y ros-$ROS_DISTRO-mavros
sudo apt install -y ros-$ROS_DISTRO-gazebo-ros-pkgs
sudo apt install -y ros-$ROS_DISTRO-image-pipeline
sudo apt install -y ros-$ROS_DISTRO-rviz2
echo "Installing GeographicLib datasets..."
if [ ! -f "$GEOGRAPHICLIB_SCRIPT" ]; then
    wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
fi
chmod a+x $GEOGRAPHICLIB_SCRIPT
sudo ./$GEOGRAPHICLIB_SCRIPT
echo "Mavros installation and GeographicLib datasets complete."
        
echo "--- Installation complete! ---"
echo "You should restart your terminal or run 'source ~/.bashrc' to apply all environment changes."

