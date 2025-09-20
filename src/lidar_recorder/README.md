# LiDAR Recorder

Records LiDAR point clouds to PCD, PLY, LAS, and LAZ formats with optional filtering.

## Setup

First, start the robot simulation as described in the [main README](../README.md):

```bash
ros2 launch yahboom_rosmaster_bringup yahboom_slam.launch.py gz_headless:=true
```

Then open another terminal and run:

```bash
source install/setup.bash
```

## Commands

Default (LAZ format):
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py
```

PCD format:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py pcd:=true
```

PLY format:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py ply:=true
```

LAS format:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py las:=true
```

Multiple formats:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py pcd:=true ply:=true las:=true laz:=true
```

Raw data only:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py raw:=true filtered:=false
```

Filtered data only:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py raw:=false filtered:=true
```

Custom filter level:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py filter_lvl:=0.05
```

Custom topic:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py topic_name:=/velodyne_points
```

Custom output directory:
```bash
ros2 launch lidar_recorder lidar_recorder.launch.py output_dir:=/path/to/output
```

Stop recording:
```bash
ros2 param set /lidar_recorder stop_recording true
```
