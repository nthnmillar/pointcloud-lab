#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <chrono>
#include <string>
#include <memory>

class BagRecorder : public rclcpp::Node
{
public:
    BagRecorder() : Node("bag_recorder")
    {
        // Parameters
        this->declare_parameter("output_bag", "lidar_recording");
        this->declare_parameter("topic_name", "/lidar/points");
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("max_duration", 300.0); // 5 minutes default
        
        output_bag_ = this->get_parameter("output_bag").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        max_duration_ = this->get_parameter("max_duration").as_double();
        
        // Initialize bag writer
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        
        // Set up storage options
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = output_bag_;
        storage_options.storage_id = "sqlite3";
        
        // Set up converter options
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        // Open the bag
        writer_->open(storage_options, converter_options);
        
        // Subscribe to LiDAR topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_, 10, std::bind(&BagRecorder::lidar_callback, this, std::placeholders::_1));
        
        // Status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/bag_recorder/status", 10);
        
        // Timer for status updates and auto-stop
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&BagRecorder::status_timer_callback, this));
        
        // Start time for duration tracking
        start_time_ = std::chrono::high_resolution_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Recorder started. Recording from topic: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output bag: %s", output_bag_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max duration: %.1f seconds", max_duration_);
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to stop recording or wait for auto-stop");
        
        if (enable_visualization_) {
            RCLCPP_INFO(this->get_logger(), "Visualization enabled - use RViz to view point clouds");
        }
    }
    
    ~BagRecorder()
    {
        if (writer_) {
            writer_->close();
            RCLCPP_INFO(this->get_logger(), "Bag file saved: %s", output_bag_.c_str());
        }
    }
    
private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Check if we've exceeded max duration
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
            
            if (elapsed >= max_duration_) {
                RCLCPP_INFO(this->get_logger(), "Max duration reached. Stopping recording...");
                rclcpp::shutdown();
                return;
            }
            
            // Write message to bag
            rclcpp::SerializedMessage serialized_msg;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            serialization.serialize_message(msg.get(), &serialized_msg);
            
            rosbag2_storage::SerializedBagMessage bag_msg;
            bag_msg.serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t(serialized_msg.get_rcl_serialized_message()));
            bag_msg.recv_timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
            bag_msg.send_timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
            bag_msg.topic_name = topic_name_;
            
            writer_->write(std::make_shared<rosbag2_storage::SerializedBagMessage>(bag_msg));
            
            // Update statistics
            message_count_++;
            total_points_ += msg->width * msg->height;
            
            if (message_count_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "Recorded %zu messages, %zu total points", 
                            message_count_, total_points_);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error recording message: %s", e.what());
        }
    }
    
    void status_timer_callback()
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
        auto remaining = max_duration_ - elapsed;
        
        std_msgs::msg::String status_msg;
        status_msg.data = "Recording: " + std::to_string(message_count_) + 
                         " messages, " + std::to_string(total_points_) + 
                         " points, " + std::to_string(remaining) + "s remaining";
        status_pub_->publish(status_msg);
        
        // Check if we should auto-stop
        if (elapsed >= max_duration_) {
            RCLCPP_INFO(this->get_logger(), "Max duration reached. Stopping recording...");
            rclcpp::shutdown();
        }
    }
    
    std::string output_bag_;
    std::string topic_name_;
    bool enable_visualization_;
    double max_duration_;
    
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    std::chrono::high_resolution_clock::time_point start_time_;
    size_t message_count_ = 0;
    size_t total_points_ = 0;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BagRecorder>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
