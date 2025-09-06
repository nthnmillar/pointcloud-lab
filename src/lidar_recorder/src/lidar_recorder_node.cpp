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
#include <laszip/laszip_api.h>
#endif

class LidarRecorder : public rclcpp::Node
{
public:
    LidarRecorder() : Node("lidar_recorder")
    {
        // Parameters
        this->declare_parameter("output_dir", "recorded_lidar_data");
        this->declare_parameter("topic_name", "/lidar/points");
        this->declare_parameter("raw", true);
        this->declare_parameter("filtered", true);
        this->declare_parameter("filter_lvl", 0.1);
        this->declare_parameter("save_individual_files", false);
        this->declare_parameter("pcd", true);
        this->declare_parameter("ply", true);
        this->declare_parameter("laz", false);
        this->declare_parameter("las", false);
        this->declare_parameter("stop_recording", false);
        
        output_dir_ = this->get_parameter("output_dir").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        raw_ = this->get_parameter("raw").as_bool();
        filtered_ = this->get_parameter("filtered").as_bool();
        filter_lvl_ = this->get_parameter("filter_lvl").as_double();
        save_individual_files_ = this->get_parameter("save_individual_files").as_bool();
        pcd_ = this->get_parameter("pcd").as_bool();
        ply_ = this->get_parameter("ply").as_bool();
        laz_ = this->get_parameter("laz").as_bool();
        las_ = this->get_parameter("las").as_bool();
        stop_recording_ = this->get_parameter("stop_recording").as_bool();
        
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
        RCLCPP_INFO(this->get_logger(), "PCD format: %s", pcd_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "PLY format: %s", ply_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "LAZ format: %s", laz_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "LAS format: %s", las_ ? "enabled" : "disabled");
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
        // Check if stop recording was requested (check parameter dynamically)
        if (this->get_parameter("stop_recording").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Stop recording requested. Saving data and shutting down...");
            save_recorded_data();
            rclcpp::shutdown();
            return;
        }
        
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
    
    #ifdef HAVE_LASZIP
    void save_to_laz(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty point cloud for LAZ saving!");
            return;
        }
        
        try {
            // Initialize LASzip
            laszip_POINTER laszip_writer;
            if (laszip_create(&laszip_writer) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create LASzip writer!");
                return;
            }
            
            // Get header and point pointers
            laszip_header_struct* header;
            laszip_point_struct* point;
            if (laszip_get_header_pointer(laszip_writer, &header) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get LASzip header pointer!");
                laszip_destroy(laszip_writer);
                return;
            }
            
            if (laszip_get_point_pointer(laszip_writer, &point) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get LASzip point pointer!");
                laszip_destroy(laszip_writer);
                return;
            }
            
            // Set header information
            header->version_major = 1;
            header->version_minor = 2;
            header->point_data_format = 0; // XYZ format
            header->point_data_record_length = 20;
            
            // Calculate bounds
            double min_x = std::numeric_limits<double>::max();
            double min_y = std::numeric_limits<double>::max();
            double min_z = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double max_y = std::numeric_limits<double>::lowest();
            double max_z = std::numeric_limits<double>::lowest();
            
            for (const auto& pcl_point : cloud->points) {
                min_x = std::min(min_x, static_cast<double>(pcl_point.x));
                min_y = std::min(min_y, static_cast<double>(pcl_point.y));
                min_z = std::min(min_z, static_cast<double>(pcl_point.z));
                max_x = std::max(max_x, static_cast<double>(pcl_point.x));
                max_y = std::max(max_y, static_cast<double>(pcl_point.y));
                max_z = std::max(max_z, static_cast<double>(pcl_point.z));
            }
            
            header->min_x = min_x;
            header->min_y = min_y;
            header->min_z = min_z;
            header->max_x = max_x;
            header->max_y = max_y;
            header->max_z = max_z;
            
            header->x_offset = (min_x + max_x) / 2.0;
            header->y_offset = (min_y + max_y) / 2.0;
            header->z_offset = (min_z + max_z) / 2.0;
            
            // Calculate appropriate scale factors to avoid overflow
            double max_range = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
            double scale_factor = std::max(0.01, max_range / 1000000.0); // Ensure reasonable precision
            
            header->x_scale_factor = scale_factor;
            header->y_scale_factor = scale_factor;
            header->z_scale_factor = scale_factor;
            
            header->number_of_point_records = cloud->size();
            header->number_of_points_by_return[0] = cloud->size();
            
            // Set the header before opening
            if (laszip_set_header(laszip_writer, header) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set LAZ header!");
                laszip_destroy(laszip_writer);
                return;
            }
            
            // Open for writing with compression
            if (laszip_open_writer(laszip_writer, filename.c_str(), 1) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open LAZ file for writing: %s", filename.c_str());
                laszip_destroy(laszip_writer);
                return;
            }
            
            // Write points
            size_t points_written = 0;
            bool write_success = true;
            RCLCPP_INFO(this->get_logger(), "Starting to write %zu points to LAZ file...", cloud->size());
            
            for (const auto& pcl_point : cloud->points) {
                point->X = static_cast<laszip_I32>((pcl_point.x - header->x_offset) / header->x_scale_factor);
                point->Y = static_cast<laszip_I32>((pcl_point.y - header->y_offset) / header->y_scale_factor);
                point->Z = static_cast<laszip_I32>((pcl_point.z - header->z_offset) / header->z_scale_factor);
                
                int result = laszip_write_point(laszip_writer);
                if (result != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to write point %zu! LASzip error: %d", points_written, result);
                    write_success = false;
                    break;
                }
                
                points_written++;
                
                // Progress update for large datasets
                if (points_written % 100000 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Written %zu/%zu points to LAZ...", points_written, cloud->size());
                }
            }
            
            // Close and cleanup
            if (laszip_close_writer(laszip_writer) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to close LAZ writer!");
                write_success = false;
            }
            laszip_destroy(laszip_writer);
            
            if (write_success && points_written == cloud->size()) {
                RCLCPP_INFO(this->get_logger(), "Saved LAZ file: %s (%zu points)", filename.c_str(), points_written);
            } else {
                RCLCPP_ERROR(this->get_logger(), "LAZ file save incomplete: %s (%zu/%zu points written)", 
                            filename.c_str(), points_written, cloud->size());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving LAZ file: %s", e.what());
        }
    }
    #endif
    
    #ifdef HAVE_LASZIP
    void save_to_las(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty point cloud for LAS saving!");
            return;
        }
        
        try {
            // Initialize LASzip
            laszip_POINTER laszip_writer;
            if (laszip_create(&laszip_writer) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create LASzip writer!");
                return;
            }
            
            // Get header and point pointers
            laszip_header_struct* header;
            laszip_point_struct* point;
            if (laszip_get_header_pointer(laszip_writer, &header) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get LASzip header pointer!");
                laszip_destroy(laszip_writer);
                return;
            }
            
            if (laszip_get_point_pointer(laszip_writer, &point) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get LASzip point pointer!");
                laszip_destroy(laszip_writer);
                return;
            }
            
            // Set header information
            header->version_major = 1;
            header->version_minor = 2;
            header->point_data_format = 0; // XYZ format
            header->point_data_record_length = 20;
            
            // Calculate bounds
            double min_x = std::numeric_limits<double>::max();
            double min_y = std::numeric_limits<double>::max();
            double min_z = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double max_y = std::numeric_limits<double>::lowest();
            double max_z = std::numeric_limits<double>::lowest();
            
            for (const auto& pcl_point : cloud->points) {
                min_x = std::min(min_x, static_cast<double>(pcl_point.x));
                min_y = std::min(min_y, static_cast<double>(pcl_point.y));
                min_z = std::min(min_z, static_cast<double>(pcl_point.z));
                max_x = std::max(max_x, static_cast<double>(pcl_point.x));
                max_y = std::max(max_y, static_cast<double>(pcl_point.y));
                max_z = std::max(max_z, static_cast<double>(pcl_point.z));
            }
            
            header->min_x = min_x;
            header->min_y = min_y;
            header->min_z = min_z;
            header->max_x = max_x;
            header->max_y = max_y;
            header->max_z = max_z;
            
            header->x_offset = (min_x + max_x) / 2.0;
            header->y_offset = (min_y + max_y) / 2.0;
            header->z_offset = (min_z + max_z) / 2.0;
            
            // Calculate appropriate scale factors to avoid overflow
            double max_range = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
            double scale_factor = std::max(0.01, max_range / 1000000.0); // Ensure reasonable precision
            
            header->x_scale_factor = scale_factor;
            header->y_scale_factor = scale_factor;
            header->z_scale_factor = scale_factor;
            
            header->number_of_point_records = cloud->size();
            header->number_of_points_by_return[0] = cloud->size();
            
            // Set the header before opening
            if (laszip_set_header(laszip_writer, header) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set LAS header!");
                laszip_destroy(laszip_writer);
                return;
            }
            
            // Open for writing WITHOUT compression (LAS format)
            if (laszip_open_writer(laszip_writer, filename.c_str(), 0) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open LAS file for writing: %s", filename.c_str());
                laszip_destroy(laszip_writer);
                return;
            }
            
            // Write points
            size_t points_written = 0;
            bool write_success = true;
            RCLCPP_INFO(this->get_logger(), "Starting to write %zu points to LAS file...", cloud->size());
            
            for (const auto& pcl_point : cloud->points) {
                point->X = static_cast<laszip_I32>((pcl_point.x - header->x_offset) / header->x_scale_factor);
                point->Y = static_cast<laszip_I32>((pcl_point.y - header->y_offset) / header->y_scale_factor);
                point->Z = static_cast<laszip_I32>((pcl_point.z - header->z_offset) / header->z_scale_factor);
                
                int result = laszip_write_point(laszip_writer);
                if (result != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to write point %zu! LASzip error: %d", points_written, result);
                    write_success = false;
                    break;
                }
                
                points_written++;
                
                // Progress update for large datasets
                if (points_written % 100000 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Written %zu/%zu points to LAS...", points_written, cloud->size());
                }
            }
            
            // Close and cleanup
            if (laszip_close_writer(laszip_writer) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to close LAS writer!");
                write_success = false;
            }
            laszip_destroy(laszip_writer);
            
            if (write_success && points_written == cloud->size()) {
                RCLCPP_INFO(this->get_logger(), "Saved LAS file: %s (%zu points)", filename.c_str(), points_written);
            } else {
                RCLCPP_ERROR(this->get_logger(), "LAS file save incomplete: %s (%zu/%zu points written)", 
                            filename.c_str(), points_written, cloud->size());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving LAS file: %s", e.what());
        }
    }
    #endif
    
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
        
        // Save raw point cloud if requested
        std::string raw_filename;
        if (raw_) {
            // Clean the raw data by removing invalid points
            pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& point : combined_cloud->points) {
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    cleaned_cloud->points.push_back(point);
                }
            }
            cleaned_cloud->width = cleaned_cloud->points.size();
            cleaned_cloud->height = 1;
            cleaned_cloud->is_dense = true;
            
            RCLCPP_INFO(this->get_logger(), "Cleaned raw data: %zu -> %zu points", 
                       combined_cloud->size(), cleaned_cloud->size());
            
            if (pcd_) {
                raw_filename = save_dir + "/pointcloud_raw.pcd";
                pcl::io::savePCDFile(raw_filename, *cleaned_cloud);
            }
            if (ply_) {
                raw_filename = save_dir + "/pointcloud_raw.ply";
                save_to_ply(raw_filename, cleaned_cloud);
            }
            #ifdef HAVE_LASZIP
            if (laz_) {
                raw_filename = save_dir + "/pointcloud_raw.laz";
                save_to_laz(raw_filename, cleaned_cloud);
            }
            if (las_) {
                raw_filename = save_dir + "/pointcloud_raw.las";
                save_to_las(raw_filename, cleaned_cloud);
            }
            #endif
        }
        
        // Apply filtering if requested
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string filtered_filename;
        if (filtered_ && filter_lvl_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(combined_cloud);
            voxel_filter.setLeafSize(filter_lvl_, filter_lvl_, filter_lvl_);
            voxel_filter.filter(*filtered_cloud);
            RCLCPP_INFO(this->get_logger(), "Filtering applied: %zu -> %zu points", 
                        combined_cloud->size(), filtered_cloud->size());
            
            // Save filtered point cloud
            if (pcd_) {
                filtered_filename = save_dir + "/pointcloud_filtered.pcd";
                pcl::io::savePCDFile(filtered_filename, *filtered_cloud);
            }
            if (ply_) {
                filtered_filename = save_dir + "/pointcloud_filtered.ply";
                save_to_ply(filtered_filename, filtered_cloud);
            }
            #ifdef HAVE_LASZIP
            if (laz_) {
                filtered_filename = save_dir + "/pointcloud_filtered.laz";
                save_to_laz(filtered_filename, filtered_cloud);
            }
            if (las_) {
                filtered_filename = save_dir + "/pointcloud_filtered.las";
                save_to_las(filtered_filename, filtered_cloud);
            }
            #endif
        } else {
            RCLCPP_INFO(this->get_logger(), "No filtering applied, raw data only");
        }
        
        // Save metadata
        std::string metadata_file = save_dir + "/metadata.txt";
        std::ofstream metadata(metadata_file);
        if (metadata.is_open()) {
            metadata << "Recording session: " << timestamp << "\n";
            metadata << "Topic: " << topic_name_ << "\n";
            metadata << "Total point clouds: " << point_clouds_.size() << "\n";
            metadata << "Combined point cloud size: " << combined_cloud->size() << " points\n";
            if (raw_) {
                metadata << "Raw data: enabled\n";
                if (pcd_) metadata << "Raw PCD: enabled\n";
                if (ply_) metadata << "Raw PLY: enabled\n";
                #ifdef HAVE_LASZIP
                if (laz_) metadata << "Raw LAZ: enabled\n";
                if (las_) metadata << "Raw LAS: enabled\n";
                #endif
            } else {
                metadata << "Raw data: disabled\n";
            }
            if (filtered_ && filter_lvl_ > 0.0) {
                metadata << "Filtered point cloud size: " << filtered_cloud->size() << " points\n";
                metadata << "Filter level: " << filter_lvl_ << "\n";
                if (pcd_) metadata << "Filtered PCD: enabled\n";
                if (ply_) metadata << "Filtered PLY: enabled\n";
                #ifdef HAVE_LASZIP
                if (laz_) metadata << "Filtered LAZ: enabled\n";
                if (las_) metadata << "Filtered LAS: enabled\n";
                #endif
            } else {
                metadata << "Filtering: disabled\n";
            }
            metadata.close();
        }
        
        RCLCPP_INFO(this->get_logger(), "Data saved to: %s", save_dir.c_str());
        if (raw_) {
            RCLCPP_INFO(this->get_logger(), "Raw point cloud: %s", raw_filename.c_str());
        }
        if (filtered_ && filter_lvl_ > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Filtered point cloud: %s", filtered_filename.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Individual files: %zu files", point_clouds_.size());
    }
    
    std::string output_dir_;
    std::string topic_name_;
    bool raw_;
    bool filtered_;
    double filter_lvl_;
    bool save_individual_files_;
    bool pcd_;
    bool ply_;
    bool laz_;
    bool las_;
    bool stop_recording_;
    
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

