# Use ROS2 Jazzy as base image
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS2 packages for your robot project
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-urdf \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-rqt-robot-steering \
    ros-jazzy-rviz-imu-plugin \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user for safety (use next available UID)
RUN useradd -m -s /bin/bash rosuser && \
    usermod -aG sudo rosuser

# Set up workspace directory with proper ownership
WORKDIR /home/rosuser/ros2_ws
RUN chown -R rosuser:rosuser /home/rosuser

# Switch to non-root user
USER rosuser

# Copy source code and configuration files
COPY --chown=rosuser:rosuser . /home/rosuser/ros2_ws/

# Build the workspace during image build
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/rosuser/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys 'ros-jazzy-gz-ros2-control-demos' || true && \
    colcon build --symlink-install"

# Create a script to run the workspace
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
source /home/rosuser/ros2_ws/install/setup.bash\n\
exec "$@"' > /home/rosuser/run.sh && \
    chmod +x /home/rosuser/run.sh && \
    chown rosuser:rosuser /home/rosuser/run.sh

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/rosuser/.bashrc

# Expose ROS2 ports
EXPOSE 11311 11312

# Override the base image's entrypoint
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/jazzy/setup.bash && bash"]

