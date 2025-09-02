#ifndef LIDAR_RECORDER_HPP
#define LIDAR_RECORDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <vector>
#include <string>

class LidarRecorder : public rclcpp::Node
{
public:
    LidarRecorder();

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void status_timer_callback();
    void save_recorded_data();

    std::string output_dir_;
    std::string topic_name_;
    double voxel_size_;
    bool remove_outliers_;
    double outlier_std_dev_;
    int outlier_min_neighbors_;
    
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_clouds_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LIDAR_RECORDER_HPP

