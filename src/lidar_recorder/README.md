# LiDAR Recorder Package

A comprehensive ROS2 package for LiDAR point cloud recording, filtering, and conversion. This package provides multiple tools for flexible workflows:

1. **LiDAR Recorder** - Records LiDAR data directly to PCD, PLY, and LAZ formats
2. **Bag Recorder** - Records raw LiDAR data to ROS bag files
3. **Bag Filter** - Loads bag files, applies PCL filtering, and saves as PCD
4. **PCD to LAZ Converter** - Converts PCD files to compressed LAZ format

Each tool includes visualization capabilities to monitor the process and verify results.

## Features

- **Direct recording** - Record LiDAR data directly to PCD, PLY, and LAZ formats
- **Flexible workflow** - Record first, then filter and convert
- **Real-time recording** to ROS bag files with auto-stop
- **Advanced filtering** with voxel grid, spatial filtering, and outlier removal
- **Visualization** at each step using PCL visualizer and RViz
- **Multiple output formats** - ROS bag, PCD, PLY, and LAZ
- **Configurable parameters** for different use cases
- **Metadata logging** for all operations

## Dependencies

- ROS2 (Humble or later)
- PCL (Point Cloud Library)
- pcl_ros and pcl_conversions
- rosbag2_cpp
- laszip (for LAZ compression)

## Installation

1. Clone this package to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

2. Install dependencies:
```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions libpcl-dev ros-humble-rosbag2-cpp liblaszip-dev
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select lidar_recorder
source install/setup.bash
```

## Workflow

### Option 1: Direct Recording (Recommended)

Record LiDAR data directly to multiple formats:

```bash
# Basic recording (PCD and PLY formats)
ros2 launch lidar_recorder lidar_recorder.launch.py

# Enable LAZ format (requires liblaszip)
ros2 launch lidar_recorder lidar_recorder.launch.py \
  laz:=true \
  output_dir:=~/direct_recording

# Custom recording with filtering
ros2 launch lidar_recorder lidar_recorder.launch.py \
  topic_name:=/lidar/points \
  output_dir:=~/my_recording \
  pcd:=true \
  ply:=true \
  laz:=true \
  filtered:=true \
  filter_lvl:=0.1
```

**Features:**
- Records directly to PCD, PLY, and LAZ formats
- Real-time point cloud combination
- Optional voxel grid filtering
- Auto-saves when stopped (Ctrl+C)
- Metadata logging

### Option 2: Bag-based Workflow

### Step 1: Record LiDAR Data to Bag

Record raw LiDAR data from your robot:

```bash
# Basic recording (5 minutes, auto-stop)
ros2 launch lidar_recorder bag_recorder.launch.py

# Custom recording
ros2 launch lidar_recorder bag_recorder.launch.py \
  output_bag:=yahboom_lidar_data \
  topic_name:=/lidar/points \
  max_duration:=600.0
```

**Features:**
- Records to ROS bag format
- Auto-stop after specified duration
- Real-time status updates
- RViz visualization available

### Step 2: Filter Bag Data

Load the recorded bag file and apply filtering:

```bash
# Basic filtering
ros2 launch lidar_recorder bag_filter.launch.py \
  input_bag:=/path/to/yahboom_lidar_data \
  output_dir:=~/filtered_data

# Advanced filtering with custom parameters
ros2 launch lidar_recorder bag_filter.launch.py \
  input_bag:=/path/to/yahboom_lidar_data \
  output_dir:=~/filtered_data \
  voxel_size:=0.15 \
  filter_x_min:=-30.0 \
  filter_x_max:=30.0 \
  filter_y_min:=-30.0 \
  filter_y_max:=30.0 \
  filter_z_min:=-5.0 \
  filter_z_max:=5.0
```

**Features:**
- Loads ROS bag files
- Combines all point clouds
- Applies spatial filtering (X, Y, Z bounds)
- Voxel grid downsampling
- Statistical outlier removal
- PCL visualizer shows before/after comparison
- Saves intermediate results at each step

**Output files:**
- `original_combined.pcd` - Raw combined data
- `spatial_filtered.pcd` - After spatial filtering
- `voxel_filtered.pcd` - After voxel filtering
- `final_filtered.pcd` - Final result
- `filtering_metadata.txt` - Detailed statistics

### Step 3: Convert to LAZ

Convert the filtered PCD to compressed LAZ format:

```bash
# Convert final filtered result
ros2 launch lidar_recorder pcd_to_laz.launch.py \
  input_pcd:=~/filtered_data/final_filtered.pcd \
  output_laz:=~/final_result.laz

# Custom compression settings
ros2 launch lidar_recorder pcd_to_laz.launch.py \
  input_pcd:=~/filtered_data/final_filtered.pcd \
  compression_level:=5 \
  point_format:=0
```

**Features:**
- Converts PCD to LAZ format
- Configurable compression levels (0-9)
- Multiple point formats (XYZ, XYZI, XYZRGB)
- PCL visualizer shows the point cloud
- Metadata logging

## Visualization

Each tool provides visualization capabilities:

### Bag Recorder
- **RViz**: View real-time point clouds during recording
- **Status topics**: Monitor recording progress

### Bag Filter
- **PCL Visualizer**: Side-by-side comparison of original vs filtered
- **Color coding**: Red (original), Green (filtered)
- **Interactive**: Rotate, zoom, and explore point clouds

### PCD to LAZ Converter
- **PCL Visualizer**: View the point cloud being converted
- **Quality check**: Verify data before compression

## Complete Example Workflow

```bash
# 1. Start your robot simulation
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py

# 2. Record LiDAR data (in another terminal)
ros2 launch lidar_recorder bag_recorder.launch.py \
  output_bag:=~/yahboom_recording \
  max_duration:=300.0

# 3. Filter the recorded data (after recording completes)
ros2 launch lidar_recorder bag_filter.launch.py \
  input_bag:=~/yahboom_recording \
  output_dir:=~/yahboom_filtered \
  voxel_size:=0.1 \
  filter_x_min:=-20.0 \
  filter_x_max:=20.0 \
  filter_y_min:=-20.0 \
  filter_y_max:=20.0

# 4. Convert to LAZ (after filtering completes)
ros2 launch lidar_recorder pcd_to_laz.launch.py \
  input_pcd:=~/yahboom_filtered/final_filtered.pcd \
  output_laz:=~/yahboom_final.laz
```

## Parameters

### LiDAR Recorder (Direct Recording)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_dir` | `recorded_lidar_data` | Output directory for recorded data |
| `topic_name` | `/lidar/points` | LiDAR topic to subscribe to |
| `raw` | `true` | Save raw (unfiltered) data |
| `filtered` | `true` | Apply filtering and save filtered data |
| `filter_lvl` | `0.1` | Voxel grid filter level (0.0 = no filtering) |
| `save_individual_files` | `false` | Save individual PCD files for each message |
| `pcd` | `true` | Save data in PCD format |
| `ply` | `true` | Save data in PLY format |
| `laz` | `false` | Save data in LAZ format (requires liblaszip) |
| `stop_recording` | `false` | Set to true to stop recording and save data |

### Bag Recorder
| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_bag` | `lidar_recording` | Output bag file name |
| `topic_name` | `/lidar/points` | LiDAR topic to subscribe to |
| `enable_visualization` | `true` | Enable RViz visualization |
| `max_duration` | `300.0` | Maximum recording duration (seconds) |

### Bag Filter
| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_bag` | (required) | Input bag file path |
| `output_dir` | `filtered_lidar_data` | Output directory |
| `voxel_size` | `0.1` | Voxel grid size (0.0 = no filtering) |
| `remove_outliers` | `true` | Enable outlier removal |
| `filter_x_min/max` | `-50.0/50.0` | X-axis spatial bounds |
| `filter_y_min/max` | `-50.0/50.0` | Y-axis spatial bounds |
| `filter_z_min/max` | `-10.0/10.0` | Z-axis spatial bounds |

### PCD to LAZ Converter
| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_pcd` | (required) | Input PCD file path |
| `output_laz` | (auto) | Output LAZ file path |
| `compression_level` | `3` | LAZ compression (0-9) |
| `point_format` | `0` | Point format (0=XYZ, 1=XYZI, 2=XYZRGB) |

## Troubleshooting

### Common Issues

1. **Build errors**: Ensure all dependencies are installed
2. **Bag file not found**: Check file path and permissions
3. **Visualization not working**: Ensure X11 forwarding is enabled
4. **Memory issues**: Reduce voxel size for large point clouds

### Debug Mode

Enable verbose logging for any tool:
```bash
ros2 launch lidar_recorder <tool>.launch.py --ros-args --log-level debug
```

## Why This Workflow?

**Advantages of the three-step approach:**
- **Flexibility**: Experiment with different filtering parameters without re-recording
- **Debugging**: Visualize data at each step to understand the process
- **Reprocessing**: Apply multiple filtering strategies to the same dataset
- **Storage efficiency**: Raw data in bag format, filtered results in PCD, final output in LAZ
- **Quality control**: Verify results before compression

## Contributing

Feel free to submit issues and enhancement requests!

## License

MIT License - see LICENSE file for details.
