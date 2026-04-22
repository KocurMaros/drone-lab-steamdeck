FROM ubuntu:22.04

# Define environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# --- Stage 1: Base dependencies & Setup ---
RUN apt-get update && apt-get install -y \
    sudo curl wget git lsb-release gnupg locales tzdata python-is-python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash \
    && apt-get update && apt-get install -y git-lfs \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create user 'deck' with sudo privileges
RUN useradd -m -s /bin/bash deck && \
    echo "deck ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to 'deck' user
USER deck
WORKDIR /home/deck
ENV USER=deck


# --- Stage 2: Install ROS 2 Humble ---
RUN sudo apt-get update && sudo apt-get install -y software-properties-common \
    && sudo add-apt-repository universe \
    && sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install -y ros-humble-desktop python3-argcomplete ros-dev-tools \
    && sudo rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/humble/setup.bash" >> /home/deck/.bashrc


# --- Stage 3: Install Gazebo 11 ---
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && sudo apt-get update \
    && sudo apt-get install -y gazebo libgazebo-dev cmake \
    && sudo rm -rf /var/lib/apt/lists/*


# --- Stage 4: ArduPilot ---
RUN git clone -b Copter-4.2.3 https://github.com/ArduPilot/ardupilot.git /home/deck/ardupilot
WORKDIR /home/deck/ardupilot
RUN git submodule update --init --recursive

# Run Ardupilot's prerequisite installer
RUN python3 -m pip install "setuptools==58.2.0" \
    && sed -i 's/PYTHON_V="python"/PYTHON_V="python3"/' Tools/environment_install/install-prereqs-ubuntu.sh \
    && sed -i 's/PIP=pip2/PIP=pip3/' Tools/environment_install/install-prereqs-ubuntu.sh \
    && chmod +x Tools/environment_install/install-prereqs-ubuntu.sh \
    && USER=deck ./Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build SITL Copter
RUN ./waf configure --board sitl \
    && ./waf copter


# --- Stage 5: Ardupilot Gazebo Plugin ---
WORKDIR /home/deck
RUN git clone https://github.com/khancyr/ardupilot_gazebo /home/deck/ardupilot_gazebo
WORKDIR /home/deck/ardupilot_gazebo
RUN mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && sudo make install


# --- Stage 6: Install Mavros & Remaining Utilities ---
WORKDIR /home/deck
# We need to run this shell with standard bash so 'source' works
SHELL ["/bin/bash", "-c"]
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-mavros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-image-pipeline \
    ros-humble-rviz2 \
    && wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod a+x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh \
    && sudo rm -rf /var/lib/apt/lists/*

# Final environment variables for LRS-FEI repo which user mounts natively
ENV GAZEBO_MODEL_PATH=/home/deck/LRS-FEI/models
RUN echo "export GAZEBO_MODEL_PATH=/home/deck/LRS-FEI/models" >> /home/deck/.bashrc

CMD ["/bin/bash"]
