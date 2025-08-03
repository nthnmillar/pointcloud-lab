#!/bin/bash
echo "SCRIPT STARTING"
# Single script to launch the Yahboom ROSMASTERX3 with Gazebo, Nav2 and ROS 2 Controllers

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"

}

# Function to check if simulation is ready
check_simulation_ready() {
    local max_attempts=60  # 60 attempts * 2 seconds = 2 minutes max wait
    local attempt=0
    
    echo "Waiting for simulation to be ready..."
    
    while [ $attempt -lt $max_attempts ]; do
        # Check if Gazebo is running and responding
        if gz service -s /world/test_world/stats --reqtype gz.msgs.WorldStatistics --reptype gz.msgs.WorldStatistics --timeout 1000 --req "" >/dev/null 2>&1; then
            echo "✓ Gazebo is responding"
            # Check if robot_state_publisher is running
            if ros2 node list | grep -q "robot_state_publisher"; then
                echo "✓ robot_state_publisher is running"
                # Check if controller_server is running (this is the correct nav2 controller node)
                if ros2 node list | grep -q "controller_server"; then
                    echo "✓ controller_server is running"
                    # Check if SLAM is ready (if SLAM is enabled)
                    if [ "$1" = "slam" ]; then
                        if ros2 node list | grep -q "slam_toolbox"; then
                            echo "✓ slam_toolbox is running"
                            # Wait for SLAM map to appear in RViz
                            echo "Waiting for SLAM map to appear in RViz..."
                            if ros2 topic list | grep -q "/map" && ros2 topic echo /map --once --timeout 5 >/dev/null 2>&1; then
                                echo "✓ SLAM map is available"
                                echo "Simulation and SLAM are ready!"
                                return 0
                            else
                                echo "Waiting for SLAM map to be published..."
                            fi
                        else
                            echo "Waiting for slam_toolbox..."
                        fi
                    else
                        echo "Simulation is ready!"
                        return 0
                    fi
                else
                    echo "Waiting for controller_server..."
                fi
            else
                echo "Waiting for robot_state_publisher..."
            fi
        else
            echo "Waiting for Gazebo to respond..."
        fi
        
        echo "Waiting for simulation to initialize... (attempt $((attempt + 1))/$max_attempts)"
        sleep 2
        attempt=$((attempt + 1))
    done
    
    echo "Warning: Simulation may not be fully ready, but proceeding anyway..."
    return 1
}

# Function to adjust camera position
adjust_camera_position() {
    echo "Attempting to adjust camera position..."
    if gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}" >/dev/null 2>&1; then
        echo "✓ Camera position adjusted successfully"
    else
        echo "⚠ Camera adjustment failed (this is normal if running headless or if GUI is not available)"
    fi
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

# Check if SLAM argument is provided
if [ "$1" = "slam" ]; then
    SLAM_ARG="slam:=True"
else
    SLAM_ARG="slam:=False"
fi

# For cafe.world -> z:=0.20
# For house.world -> z:=0.05
# To change Gazebo camera pose: gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

echo "Launching Gazebo simulation with Nav2..."
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
    headless:=False \
    load_controllers:=true \
    world_file:=test_world.sdf \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.05 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 \
    "$SLAM_ARG" &

# Wait for simulation to be ready
check_simulation_ready "$1"

echo "Adjusting camera position..."
adjust_camera_position

# Keep the script running until Ctrl+C
wait
