FROM ubuntu:22.04

# Prevent interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install prerequisites for the install script
RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    wget \
    git \
    lsb-release \
    gnupg \
    locales \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# Set up a non-root user 'deck' to match Steam Deck environment
RUN useradd -m -s /bin/bash deck && \
    echo "deck ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER deck
WORKDIR /home/deck

# Copy the install script
COPY --chown=deck:deck install.sh /home/deck/install.sh

# Make the script executable and run it
RUN chmod +x /home/deck/install.sh && \
    /home/deck/install.sh

# Sourcing .bashrc automatically for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /home/deck/.bashrc

# Set the default command to bash
CMD ["/bin/bash"]
