# ROS2 Jazzy Docker Setup

Simple Docker container for ROS2 Jazzy development.

## Quick Start

### 1. Build the Container
```bash
docker build -t lidar-project .
```

### 2. Run the Container
```bash
docker run -it --rm \
  -v $(pwd)/src:/ros2_ws/src \
  --network host \
  lidar-project
```

## What This Gives You

- ✅ **ROS2 Jazzy** - Latest ROS2 version
- ✅ **Gazebo & RViz** - 3D simulation and visualization
- ✅ **All Dependencies** - Robot control, navigation, localization
- ✅ **Portable** - Works on any Linux with Docker

## Inside the Container

```bash
# Check ROS2 is working
ros2 --help

# List your packages
ros2 pkg list

# Build your workspace
colcon build --symlink-install

# Run your robot
ros2 launch robot_bringup robot_gazebo.launch.py
```

## GUI Support (Gazebo, RViz)

For GUI applications to work properly:

```bash
# Allow X11 connections (run on host before starting container)
xhost +local:docker

# Then run your container normally
docker run -it --rm \
  -v $(pwd)/src:/ros2_ws/src \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  lidar-project
```

## Development Workflow

1. **Edit code** in `./src/` on your host machine
2. **Run container** - your code is automatically mounted
3. **Build inside container** - `colcon build --symlink-install`
4. **Test your robot** - `ros2 launch robot_bringup robot_gazebo.launch.py`

## Why This Approach

- **Simple** - Just build and run
- **Fast** - No rebuilding for code changes
- **Portable** - Works anywhere with Docker
- **Clean** - No complex setup or configuration files

That's it! Simple, clean, and focused on what you actually need.

