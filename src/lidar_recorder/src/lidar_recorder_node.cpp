#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>

#ifdef HAVE_LASZIP
#include <laszip/laszip.h>
#endif

class LidarRecorder : public rclcpp::Node
{
public:
    LidarRecorder() : Node("lidar_recorder")
    {
        // Parameters
        this->declare_parameter("output_dir", "recorded_lidar_data");
        this->declare_parameter("topic_name", "/lidar/points");
        this->declare_parameter("voxel_size", 0.1);
        this->declare_parameter("remove_outliers", true);
        this->declare_parameter("outlier_std_dev", 2.0);
        this->declare_parameter("outlier_min_neighbors", 10);
        this->declare_parameter("save_individual_files", false);
        this->declare_parameter("output_format", "ply");
        
        output_dir_ = this->get_parameter("output_dir").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        remove_outliers_ = this->get_parameter("remove_outliers").as_bool();
        outlier_std_dev_ = this->get_parameter("outlier_std_dev").as_double();
        outlier_min_neighbors_ = this->get_parameter("outlier_min_neighbors").as_int();
        save_individual_files_ = this->get_parameter("save_individual_files").as_bool();
        output_format_ = this->get_parameter("output_format").as_string();
        
        // Create output directory
        std::filesystem::create_directories(output_dir_);
        
        // Subscribe to LiDAR topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_, 10, std::bind(&LidarRecorder::lidar_callback, this, std::placeholders::_1));
        
        // Status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/lidar_recorder/status", 10);
        
        // Timer for status updates
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&LidarRecorder::status_timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Recorder started. Recording from topic: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output format: %s", output_format_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save individual files: %s", save_individual_files_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to stop recording and save data");
    }
    
    ~LidarRecorder()
    {
        if (!point_clouds_.empty()) {
            save_recorded_data();
        }
    }
    
private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Convert ROS message to PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *pcl_cloud);
            
            if (!pcl_cloud->empty()) {
                // Add timestamp for ordering
                auto timestamp = std::chrono::high_resolution_clock::now();
                point_clouds_.push_back({timestamp, pcl_cloud});
                
                if (point_clouds_.size() % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Recorded %zu point clouds", point_clouds_.size());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }
    
    void status_timer_callback()
    {
        std_msgs::msg::String status_msg;
        status_msg.data = "Recording: " + std::to_string(point_clouds_.size()) + " point clouds captured";
        status_pub_->publish(status_msg);
    }
    
    void save_to_ply(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        try {
            // Open file for writing
            std::ofstream ply_file(filename);
            if (!ply_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open PLY file: %s", filename.c_str());
                return;
            }
            
            // Write PLY header
            ply_file << "ply\n";
            ply_file << "format ascii 1.0\n";
            ply_file << "element vertex " << cloud->size() << "\n";
            ply_file << "property float x\n";
            ply_file << "property float y\n";
            ply_file << "property float z\n";
            ply_file << "end_header\n";
            
            // Write point data
            for (const auto& point : cloud->points) {
                ply_file << point.x << " " << point.y << " " << point.z << "\n";
            }
            
            ply_file.close();
            RCLCPP_INFO(this->get_logger(), "Saved PLY file: %s", filename.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving PLY file: %s", e.what());
        }
    }
    
    void save_recorded_data()
    {
        if (point_clouds_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No point clouds recorded");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Saving %zu point clouds...", point_clouds_.size());
        
        // Create timestamped directory
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::string timestamp = std::to_string(time_t);
        std::string save_dir = output_dir_ + "/recording_" + timestamp;
        std::filesystem::create_directories(save_dir);
        
        // Save individual PCD files only if requested
        if (save_individual_files_) {
            for (size_t i = 0; i < point_clouds_.size(); ++i) {
                std::string filename = save_dir + "/pointcloud_" + std::to_string(i) + ".pcd";
                pcl::io::savePCDFile(filename, *point_clouds_[i].second);
            }
            RCLCPP_INFO(this->get_logger(), "Individual PCD files: %zu files", point_clouds_.size());
        }
        
        // Save combined and filtered point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& [timestamp, cloud] : point_clouds_) {
            *combined_cloud += *cloud;
        }
        
        // Apply voxel grid filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(combined_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);
            RCLCPP_INFO(this->get_logger(), "Voxel filtering: %zu -> %zu points", 
                        combined_cloud->size(), filtered_cloud->size());
        } else {
            *filtered_cloud = *combined_cloud;
        }
        
        // Remove outliers if requested
        if (remove_outliers_ && !filtered_cloud->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
            outlier_filter.setInputCloud(filtered_cloud);
            outlier_filter.setMeanK(outlier_min_neighbors_);
            outlier_filter.setStddevMulThresh(outlier_std_dev_);
            outlier_filter.filter(*outlier_filtered_cloud);
            RCLCPP_INFO(this->get_logger(), "Outlier removal: %zu -> %zu points", 
                        filtered_cloud->size(), outlier_filtered_cloud->size());
            *filtered_cloud = *outlier_filtered_cloud;
        }
        
        // Save filtered point cloud in specified format
        std::string filtered_filename;
        if (output_format_ == "ply") {
            filtered_filename = save_dir + "/filtered_pointcloud.ply";
            save_to_ply(filtered_filename, filtered_cloud);
        } else {
            filtered_filename = save_dir + "/filtered_pointcloud.pcd";
            pcl::io::savePCDFile(filtered_filename, *filtered_cloud);
        }
        
        // Save metadata
        std::string metadata_file = save_dir + "/metadata.txt";
        std::ofstream metadata(metadata_file);
        if (metadata.is_open()) {
            metadata << "Recording session: " << timestamp << "\n";
            metadata << "Topic: " << topic_name_ << "\n";
            metadata << "Total point clouds: " << point_clouds_.size() << "\n";
            metadata << "Combined point cloud size: " << combined_cloud->size() << " points\n";
            metadata << "Filtered point cloud size: " << filtered_cloud->size() << " points\n";
            metadata << "Voxel size: " << voxel_size_ << "\n";
            metadata << "Outlier removal: " << (remove_outliers_ ? "enabled" : "disabled") << "\n";
            metadata.close();
        }
        
        RCLCPP_INFO(this->get_logger(), "Data saved to: %s", save_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "Filtered point cloud: %s", filtered_filename.c_str());
        RCLCPP_INFO(this->get_logger(), "Individual PCD files: %zu files", point_clouds_.size());
    }
    
    std::string output_dir_;
    std::string topic_name_;
    double voxel_size_;
    bool remove_outliers_;
    double outlier_std_dev_;
    int outlier_min_neighbors_;
    bool save_individual_files_;
    std::string output_format_;
    
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_clouds_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LidarRecorder>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

