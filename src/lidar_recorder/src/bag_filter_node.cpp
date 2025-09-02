#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>

class BagFilter : public rclcpp::Node
{
public:
    BagFilter() : Node("bag_filter")
    {
        // Parameters
        this->declare_parameter("input_bag", "");
        this->declare_parameter("output_dir", "filtered_lidar_data");
        this->declare_parameter("voxel_size", 0.1);
        this->declare_parameter("remove_outliers", true);
        this->declare_parameter("outlier_std_dev", 2.0);
        this->declare_parameter("outlier_min_neighbors", 10);
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("filter_x_min", -50.0);
        this->declare_parameter("filter_x_max", 50.0);
        this->declare_parameter("filter_y_min", -50.0);
        this->declare_parameter("filter_y_max", 50.0);
        this->declare_parameter("filter_z_min", -10.0);
        this->declare_parameter("filter_z_max", 10.0);
        
        input_bag_ = this->get_parameter("input_bag").as_string();
        output_dir_ = this->get_parameter("output_dir").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        remove_outliers_ = this->get_parameter("remove_outliers").as_bool();
        outlier_std_dev_ = this->get_parameter("outlier_std_dev").as_double();
        outlier_min_neighbors_ = this->get_parameter("outlier_min_neighbors").as_int();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        
        // Spatial filter parameters
        filter_x_min_ = this->get_parameter("filter_x_min").as_double();
        filter_x_max_ = this->get_parameter("filter_x_max").as_double();
        filter_y_min_ = this->get_parameter("filter_y_min").as_double();
        filter_y_max_ = this->get_parameter("filter_y_max").as_double();
        filter_z_min_ = this->get_parameter("filter_z_min").as_double();
        filter_z_max_ = this->get_parameter("filter_z_max").as_double();
        
        if (input_bag_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input bag file not specified!");
            rclcpp::shutdown();
            return;
        }
        
        // Create output directory
        std::filesystem::create_directories(output_dir_);
        
        // Status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/bag_filter/status", 10);
        
        RCLCPP_INFO(this->get_logger(), "Bag Filter started. Processing: %s", input_bag_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Voxel size: %f", voxel_size_);
        RCLCPP_INFO(this->get_logger(), "Outlier removal: %s", remove_outliers_ ? "enabled" : "disabled");
        
        // Start processing in a separate thread to allow visualization
        if (enable_visualization_) {
            RCLCPP_INFO(this->get_logger(), "Visualization enabled - PCL visualizer will open");
            processing_thread_ = std::thread(&BagFilter::process_bag, this);
        } else {
            process_bag();
        }
    }
    
    ~BagFilter()
    {
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }
    
private:
    void process_bag()
    {
        try {
            // Open bag file
            rosbag2_cpp::readers::SequentialReader reader;
            
            // Set up storage options
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = input_bag_;
            storage_options.storage_id = "sqlite3";
            
            // Set up converter options
            rosbag2_cpp::ConverterOptions converter_options;
            converter_options.input_serialization_format = "cdr";
            converter_options.output_serialization_format = "cdr";
            
            reader.open(storage_options, converter_options);
            
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds;
            size_t message_count = 0;
            
            RCLCPP_INFO(this->get_logger(), "Reading bag file...");
            
            // Read all messages
            while (reader.has_next()) {
                auto bag_message = reader.read_next();
                
                if (bag_message->topic_name == "/lidar/points") {
                    // Deserialize message
                    rclcpp::SerializedMessage serialized_msg;
                    serialized_msg.get_rcl_serialized_message() = *bag_message->serialized_data;
                    
                    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = 
                        std::make_shared<sensor_msgs::msg::PointCloud2>();
                    
                    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                    serialization.deserialize_message(&serialized_msg, cloud_msg.get());
                    
                    // Convert to PCL
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
                    
                    if (!pcl_cloud->empty()) {
                        point_clouds.push_back(pcl_cloud);
                        message_count++;
                        
                        if (message_count % 10 == 0) {
                            RCLCPP_INFO(this->get_logger(), "Read %zu messages", message_count);
                        }
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Finished reading bag. Processing %zu point clouds...", point_clouds.size());
            
            // Process and filter point clouds
            process_point_clouds(point_clouds);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing bag: %s", e.what());
        }
    }
    
    void process_point_clouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& point_clouds)
    {
        if (point_clouds.empty()) {
            RCLCPP_WARN(this->get_logger(), "No point clouds to process");
            return;
        }
        
        // Combine all point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& cloud : point_clouds) {
            *combined_cloud += *cloud;
        }
        
        RCLCPP_INFO(this->get_logger(), "Combined cloud: %zu points", combined_cloud->size());
        
        // Apply spatial filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr spatial_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_through;
        
        // Filter X
        pass_through.setInputCloud(combined_cloud);
        pass_through.setFilterFieldName("x");
        pass_through.setFilterLimits(filter_x_min_, filter_x_max_);
        pass_through.filter(*spatial_filtered_cloud);
        
        // Filter Y
        pass_through.setInputCloud(spatial_filtered_cloud);
        pass_through.setFilterFieldName("y");
        pass_through.setFilterLimits(filter_y_min_, filter_y_max_);
        pass_through.filter(*spatial_filtered_cloud);
        
        // Filter Z
        pass_through.setInputCloud(spatial_filtered_cloud);
        pass_through.setFilterFieldName("z");
        pass_through.setFilterLimits(filter_z_min_, filter_z_max_);
        pass_through.filter(*spatial_filtered_cloud);
        
        RCLCPP_INFO(this->get_logger(), "After spatial filtering: %zu points", spatial_filtered_cloud->size());
        
        // Apply voxel grid filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(spatial_filtered_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*voxel_filtered_cloud);
            RCLCPP_INFO(this->get_logger(), "After voxel filtering: %zu points", voxel_filtered_cloud->size());
        } else {
            *voxel_filtered_cloud = *spatial_filtered_cloud;
        }
        
        // Remove outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (remove_outliers_ && !voxel_filtered_cloud->empty()) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
            outlier_filter.setInputCloud(voxel_filtered_cloud);
            outlier_filter.setMeanK(outlier_min_neighbors_);
            outlier_filter.setStddevMulThresh(outlier_std_dev_);
            outlier_filter.filter(*final_cloud);
            RCLCPP_INFO(this->get_logger(), "After outlier removal: %zu points", final_cloud->size());
        } else {
            *final_cloud = *voxel_filtered_cloud;
        }
        
        // Save results
        save_results(combined_cloud, spatial_filtered_cloud, voxel_filtered_cloud, final_cloud);
        
        // Visualization
        if (enable_visualization_) {
            visualize_results(combined_cloud, final_cloud);
        }
    }
    
    void save_results(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& spatial_filtered,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_filtered,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& final)
    {
        // Save original
        std::string original_file = output_dir_ + "/original_combined.pcd";
        pcl::io::savePCDFile(original_file, *original);
        
        // Save spatial filtered
        std::string spatial_file = output_dir_ + "/spatial_filtered.pcd";
        pcl::io::savePCDFile(spatial_file, *spatial_filtered);
        
        // Save voxel filtered
        std::string voxel_file = output_dir_ + "/voxel_filtered.pcd";
        pcl::io::savePCDFile(voxel_file, *voxel_filtered);
        
        // Save final result
        std::string final_file = output_dir_ + "/final_filtered.pcd";
        pcl::io::savePCDFile(final_file, *final);
        
        // Save metadata
        std::string metadata_file = output_dir_ + "/filtering_metadata.txt";
        std::ofstream metadata(metadata_file);
        if (metadata.is_open()) {
            metadata << "Bag Filter Results\n";
            metadata << "==================\n";
            metadata << "Input bag: " << input_bag_ << "\n";
            metadata << "Original points: " << original->size() << "\n";
            metadata << "After spatial filtering: " << spatial_filtered->size() << "\n";
            metadata << "After voxel filtering: " << voxel_filtered->size() << "\n";
            metadata << "Final result: " << final->size() << "\n";
            metadata << "Voxel size: " << voxel_size_ << "\n";
            metadata << "Outlier removal: " << (remove_outliers_ ? "enabled" : "disabled") << "\n";
            metadata << "Spatial bounds: X[" << filter_x_min_ << "," << filter_x_max_ << "] ";
            metadata << "Y[" << filter_y_min_ << "," << filter_y_max_ << "] ";
            metadata << "Z[" << filter_z_min_ << "," << filter_z_max_ << "]\n";
            metadata.close();
        }
        
        RCLCPP_INFO(this->get_logger(), "Results saved to: %s", output_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Files: original_combined.pcd, spatial_filtered.pcd, voxel_filtered.pcd, final_filtered.pcd");
    }
    
    void visualize_results(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered)
    {
        // Create visualizer
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Bag Filter Results"));
        viewer->setBackgroundColor(0, 0, 0);
        
        // Add original cloud (red)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(original, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(original, red, "original");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original");
        
        // Add filtered cloud (green)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(filtered, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ>(filtered, green, "filtered");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered");
        
        // Add coordinate axes
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        
        RCLCPP_INFO(this->get_logger(), "Visualization opened. Press 'q' to close.");
        
        // Keep visualizer open
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::string input_bag_;
    std::string output_dir_;
    double voxel_size_;
    bool remove_outliers_;
    double outlier_std_dev_;
    int outlier_min_neighbors_;
    bool enable_visualization_;
    
    // Spatial filter parameters
    double filter_x_min_, filter_x_max_;
    double filter_y_min_, filter_y_max_;
    double filter_z_min_, filter_z_max_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std::thread processing_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BagFilter>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
