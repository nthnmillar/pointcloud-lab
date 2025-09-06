#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifdef HAVE_LASZIP
#include <laszip/laszip_api.h>
#endif

#include <chrono>
#include <string>
#include <filesystem>
#include <thread>
#include <vector>

class PcdToLazConverter : public rclcpp::Node
{
public:
    PcdToLazConverter() : Node("pcd_to_laz_converter")
    {
        #ifndef HAVE_LASZIP
        RCLCPP_ERROR(this->get_logger(), "PCD to LAZ conversion requires liblaszip. Please install liblaszip-dev.");
        rclcpp::shutdown();
        return;
        #endif
        
        // Parameters
        this->declare_parameter("input_pcd", "");
        this->declare_parameter("output_laz", "");
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("compression_level", 3);
        this->declare_parameter("point_format", 0); // 0=XYZ, 1=XYZI, 2=XYZRGB
        
        input_pcd_ = this->get_parameter("input_pcd").as_string();
        output_laz_ = this->get_parameter("output_laz").as_string();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        compression_level_ = this->get_parameter("compression_level").as_int();
        point_format_ = this->get_parameter("point_format").as_int();
        
        if (input_pcd_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input PCD file not specified!");
            rclcpp::shutdown();
            return;
        }
        
        if (output_laz_.empty()) {
            // Generate output filename
            std::filesystem::path input_path(input_pcd_);
            output_laz_ = input_path.parent_path().string() + "/" + 
                         input_path.stem().string() + ".laz";
        }
        
        // Status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/pcd_to_laz/status", 10);
        
        RCLCPP_INFO(this->get_logger(), "PCD to LAZ Converter started.");
        RCLCPP_INFO(this->get_logger(), "Input PCD: %s", input_pcd_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output LAZ: %s", output_laz_.c_str());
        RCLCPP_INFO(this->get_logger(), "Compression level: %d", compression_level_);
        RCLCPP_INFO(this->get_logger(), "Point format: %d", point_format_);
        
        // Start processing in a separate thread to allow visualization
        if (enable_visualization_) {
            RCLCPP_INFO(this->get_logger(), "Visualization enabled - PCL visualizer will open");
            processing_thread_ = std::thread(&PcdToLazConverter::process_conversion, this);
        } else {
            process_conversion();
        }
    }
    
    ~PcdToLazConverter()
    {
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }
    
private:
    void process_conversion()
    {
        try {
            // Load PCD file
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            RCLCPP_INFO(this->get_logger(), "Loading PCD file: %s", input_pcd_.c_str());
            
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_, *pcl_cloud) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file!");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loaded %zu points from PCD", pcl_cloud->size());
            
            #ifdef HAVE_LASZIP
            // Convert to LAZ
            bool success = convert_to_laz(pcl_cloud);
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Successfully converted to LAZ: %s", output_laz_.c_str());
                
                // Visualization
                if (enable_visualization_) {
                    visualize_point_cloud(pcl_cloud);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert to LAZ format!");
            }
            #else
            RCLCPP_ERROR(this->get_logger(), "LAZ conversion not available - liblaszip not found");
            #endif
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during conversion: %s", e.what());
        }
    }
    
    #ifdef HAVE_LASZIP
    bool convert_to_laz(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
    {
        if (pcl_cloud->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty point cloud!");
            return false;
        }
        
        // Initialize LASzip
        laszip_POINTER laszip_writer;
        if (laszip_create(&laszip_writer) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create LASzip writer!");
            return false;
        }
        
        // Get header and point pointers
        laszip_header_struct* header;
        laszip_point_struct* point;
        if (laszip_get_header_pointer(laszip_writer, &header) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get LASzip header pointer!");
            laszip_destroy(laszip_writer);
            return false;
        }
        
        if (laszip_get_point_pointer(laszip_writer, &point) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get LASzip point pointer!");
            laszip_destroy(laszip_writer);
            return false;
        }
        
        header->version_major = 1;
        header->version_minor = 2;
        header->point_data_format = point_format_;
        header->point_data_record_length = 20; // XYZ format
        
        // Calculate bounds
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double min_z = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        double max_z = std::numeric_limits<double>::lowest();
        
        for (const auto& point : pcl_cloud->points) {
            min_x = std::min(min_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            min_z = std::min(min_z, static_cast<double>(point.z));
            max_x = std::max(max_x, static_cast<double>(point.x));
            max_y = std::max(max_y, static_cast<double>(point.y));
            max_z = std::max(max_z, static_cast<double>(point.z));
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
        header->x_scale_factor = 0.001; // 1mm precision
        header->y_scale_factor = 0.001;
        header->z_scale_factor = 0.001;
        
        header->number_of_point_records = pcl_cloud->size();
        header->number_of_points_by_return[0] = pcl_cloud->size();
        
        // Open for writing with compression
        if (laszip_open_writer(laszip_writer, output_laz_.c_str(), 1) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open LAZ file for writing!");
            laszip_destroy(laszip_writer);
            return false;
        }
        
        // Write points
        size_t points_written = 0;
        for (const auto& pcl_point : pcl_cloud->points) {
            point->X = static_cast<laszip_I32>((pcl_point.x - header->x_offset) / header->x_scale_factor);
            point->Y = static_cast<laszip_I32>((pcl_point.y - header->y_offset) / header->y_scale_factor);
            point->Z = static_cast<laszip_I32>((pcl_point.z - header->z_offset) / header->z_scale_factor);
            
            if (laszip_write_point(laszip_writer) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write point %zu!", points_written);
                break;
            }
            
            points_written++;
            
            if (points_written % 10000 == 0) {
                RCLCPP_INFO(this->get_logger(), "Written %zu points...", points_written);
            }
        }
        
        // Close and cleanup
        laszip_close_writer(laszip_writer);
        laszip_destroy(laszip_writer);
        
        RCLCPP_INFO(this->get_logger(), "Successfully wrote %zu points to LAZ file", points_written);
        
        // Save metadata
        std::string metadata_file = std::filesystem::path(output_laz_).parent_path().string() + "/laz_metadata.txt";
        std::ofstream metadata(metadata_file);
        if (metadata.is_open()) {
            metadata << "LAZ Conversion Metadata\n";
            metadata << "=======================\n";
            metadata << "Input PCD: " << input_pcd_ << "\n";
            metadata << "Output LAZ: " << output_laz_ << "\n";
            metadata << "Points converted: " << points_written << "\n";
            metadata << "Compression level: " << compression_level_ << "\n";
            metadata << "Point format: " << point_format_ << "\n";
            metadata << "Bounds: X[" << min_x << "," << max_x << "] ";
            metadata << "Y[" << min_y << "," << max_y << "] ";
            metadata << "Z[" << min_z << "," << max_z << "]\n";
            metadata << "Scale factors: " << header->x_scale_factor << "\n";
            metadata.close();
        }
        
        return points_written == pcl_cloud->size();
    }
    #endif
    
    void visualize_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
    {
        // Create visualizer
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD to LAZ Conversion"));
        viewer->setBackgroundColor(0, 0, 0);
        
        // Add point cloud
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(pcl_cloud, 255, 255, 255);
        viewer->addPointCloud<pcl::PointXYZ>(pcl_cloud, white, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        
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
    
    std::string input_pcd_;
    std::string output_laz_;
    bool enable_visualization_;
    int compression_level_;
    int point_format_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std::thread processing_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PcdToLazConverter>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
