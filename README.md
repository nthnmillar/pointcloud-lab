# Pointcloud-lab

A simulated mecanum robot with 3D LiDAR for testing, recording processing lidar point cloud data.

## What This Solves

3D LiDAR attached to a previous robot sim I built appeared to smear in Rviz regardless of IMU and odometry sensor fusion which I attempted with an EKF filter. So I rebuilt another yahboom mecanum robot which included nav2 slam data into its EKF filter.

## The Problem

To minimize lidar scan smearing in Rviz, I attempted some EKF calibration with cursor to minimize smearing, tweaking noise covariance and how much odom vs IMU effects the robot's yaw rotation. Then, switching between using odom and map as the world reference, and then back to using odom again but using nav2's amcl_pose instead to get localisation.

However, the lidar data visualised in Rviz from rotations were consistently out of sync with the SLAM map when the yahboom robot rotated, whereas the lidar data snapped back on the map when translating in position. I planned on attempting to make a filter that only recorded accurate lidar data during robot translation, and disregarded lidar data from rotation.

## The Solution

I noticed that the default teleop controls were smearing the lidar data, whereas another script controlling the robot with acceleration and deceleration did not smear lidar point clouds. So I later found that I resolved the rotation lidar smearing issue by building slam friendly controls. This is where instead of controlling a robot with sudden and erratic movements such as with the default ros2 teleop keyboard controls. These controls have acceleration and decelerate for all movement, which allow for SLAM friendly trajectories and predictable motion for consistent lidar position tracking.

## Quick Start

### Pull Image
```bash
docker pull nthnmillar/pointcloud-lab:latest
```

### Run Container
```bash
xhost +local:docker

docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e QT_X11_NO_MITSHM=1 \
  nthnmillar/pointcloud-lab:latest
```

### Install & Run

```bash
colcon build
source install/setup.bash
ros2 launch yahboom_rosmaster_bringup yahboom_slam.launch.py gz_headless:=true
```

### Robot Controls


Open additional terminal
if using docker:
```bash
docker exec -it <container_name_or_id> /bin/bash
```

```bash
ros2 run yahboom_rosmaster_system_tests smooth_keyboard_controller


## Docker Hub

https://hub.docker.com/repository/docker/nthnmillar/pointcloud-lab/
