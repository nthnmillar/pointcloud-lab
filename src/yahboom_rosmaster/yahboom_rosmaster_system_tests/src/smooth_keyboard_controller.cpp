#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;

class SmoothKeyboardController : public rclcpp::Node
{
public:
    SmoothKeyboardController() : Node("smooth_keyboard_controller")
    {
        // Create publisher for movement commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create publisher for status messages
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_status", 10);
        
        // Movement parameters for smooth control
        max_linear_velocity_ = 0.5;    // m/s
        max_angular_velocity_ = 1.0;   // rad/s
        acceleration_rate_ = 0.1;      // m/s² per update
        deceleration_rate_ = 0.15;     // m/s² per update
        
        // Current velocity state
        current_linear_x_ = 0.0;
        current_linear_y_ = 0.0;
        current_angular_z_ = 0.0;
        
        // Target velocity state
        target_linear_x_ = 0.0;
        target_linear_y_ = 0.0;
        target_angular_z_ = 0.0;
        
        // Control loop parameters
        control_rate_ = 20.0;  // Hz
        control_timer_ = this->create_wall_timer(
            1000ms / static_cast<int>(control_rate_), 
            std::bind(&SmoothKeyboardController::control_loop, this)
        );
        
        // Start keyboard input thread
        keyboard_thread_ = std::thread(&SmoothKeyboardController::keyboard_input_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "Smooth Keyboard Controller Started!");
        RCLCPP_INFO(this->get_logger(), "Controls: WASD (movement), QE (rotation), X (stop), SPACE (emergency)");
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to exit");
    }
    
    ~SmoothKeyboardController()
    {
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        emergency_stop();
        RCLCPP_INFO(this->get_logger(), "Smooth Keyboard Controller Shutdown Complete");
    }
    
    void emergency_stop()
    {
        // Emergency stop - immediately set all velocities to zero
        target_linear_x_ = 0.0;
        target_linear_y_ = 0.0;
        target_angular_z_ = 0.0;
        current_linear_x_ = 0.0;
        current_linear_y_ = 0.0;
        current_angular_z_ = 0.0;
        
        // Publish immediate stop
        publish_velocity(0.0, 0.0, 0.0);
        
        // Publish status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "EMERGENCY STOP!";
        status_pub_->publish(status_msg);
        
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
    }
    
private:
    void keyboard_input_loop()
    {
        // Set terminal to raw mode for immediate key reading
        struct termios old_settings, new_settings;
        tcgetattr(STDIN_FILENO, &old_settings);
        new_settings = old_settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        
        // Set stdin to non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        
        while (rclcpp::ok()) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) > 0) {
                process_key(ch);
            }
            std::this_thread::sleep_for(10ms);
        }
        
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    }
    
    void process_key(char key)
    {
        switch (std::tolower(key)) {
            case 'w':  // Forward
                execute_movement("FORWARD");
                break;
            case 's':  // Backward
                execute_movement("BACKWARD");
                break;
            case 'a':  // Left strafe (mecanum)
                execute_movement("LEFT");
                break;
            case 'd':  // Right strafe (mecanum)
                execute_movement("RIGHT");
                break;
            case 'q':  // Rotate left
                execute_movement("ROTATE_LEFT");
                break;
            case 'e':  // Rotate right
                execute_movement("ROTATE_RIGHT");
                break;
            case 'x':  // Stop
                execute_movement("STOP");
                break;
            case ' ':  // Spacebar - emergency stop
                emergency_stop();
                break;
            case 3:    // Ctrl+C
                rclcpp::shutdown();
                break;
        }
    }
    
    void execute_movement(const std::string& action)
    {
        if (action == "FORWARD") {
            target_linear_x_ = max_linear_velocity_;
            target_linear_y_ = 0.0;
            target_angular_z_ = 0.0;
        }
        else if (action == "BACKWARD") {
            target_linear_x_ = -max_linear_velocity_;
            target_linear_y_ = 0.0;
            target_angular_z_ = 0.0;
        }
        else if (action == "LEFT") {
            // Mecanum strafe left - Y velocity positive
            target_linear_x_ = 0.0;
            target_linear_y_ = max_linear_velocity_;
            target_angular_z_ = 0.0;
        }
        else if (action == "RIGHT") {
            // Mecanum strafe right - Y velocity negative
            target_linear_x_ = 0.0;
            target_linear_y_ = -max_linear_velocity_;
            target_angular_z_ = 0.0;
        }
        else if (action == "ROTATE_LEFT") {
            target_linear_x_ = 0.0;
            target_linear_y_ = 0.0;
            target_angular_z_ = max_angular_velocity_;
        }
        else if (action == "ROTATE_RIGHT") {
            target_linear_x_ = 0.0;
            target_linear_y_ = 0.0;
            target_angular_z_ = -max_angular_velocity_;
        }
        else if (action == "STOP") {
            target_linear_x_ = 0.0;
            target_linear_y_ = 0.0;
            target_angular_z_ = 0.0;
        }
        
        // Publish status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Action: " + action;
        status_pub_->publish(status_msg);
    }
    
    void control_loop()
    {
        // Smoothly adjust current velocities toward target velocities
        smooth_velocity_transition();
        
        // Publish current velocities
        publish_velocity(current_linear_x_, current_linear_y_, current_angular_z_);
    }
    
    void smooth_velocity_transition()
    {
        // Linear X velocity
        if (std::abs(target_linear_x_ - current_linear_x_) > 0.01) {
            if (target_linear_x_ > current_linear_x_) {
                current_linear_x_ = std::min(
                    current_linear_x_ + acceleration_rate_ / control_rate_, 
                    target_linear_x_
                );
            } else {
                current_linear_x_ = std::max(
                    current_linear_x_ - deceleration_rate_ / control_rate_, 
                    target_linear_x_
                );
            }
        } else {
            current_linear_x_ = target_linear_x_;
        }
        
        // Linear Y velocity (mecanum strafing)
        if (std::abs(target_linear_y_ - current_linear_y_) > 0.01) {
            if (target_linear_y_ > current_linear_y_) {
                current_linear_y_ = std::min(
                    current_linear_y_ + acceleration_rate_ / control_rate_, 
                    target_linear_y_
                );
            } else {
                current_linear_y_ = std::max(
                    current_linear_y_ - deceleration_rate_ / control_rate_, 
                    target_linear_y_
                );
            }
        } else {
            current_linear_y_ = target_linear_y_;
        }
        
        // Angular Z velocity
        if (std::abs(target_angular_z_ - current_angular_z_) > 0.01) {
            if (target_angular_z_ > current_angular_z_) {
                current_angular_z_ = std::min(
                    current_angular_z_ + acceleration_rate_ / control_rate_, 
                    target_angular_z_
                );
            } else {
                current_angular_z_ = std::max(
                    current_angular_z_ - deceleration_rate_ / control_rate_, 
                    target_angular_z_
                );
            }
        } else {
            current_angular_z_ = target_angular_z_;
        }
    }
    
    void publish_velocity(double linear_x, double linear_y, double angular_z)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.linear.y = linear_y;  // Mecanum strafing
        twist_msg.angular.z = angular_z;
        
        cmd_vel_pub_->publish(twist_msg);
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Threads
    std::thread keyboard_thread_;
    
    // Movement parameters
    double max_linear_velocity_;
    double max_angular_velocity_;
    double acceleration_rate_;
    double deceleration_rate_;
    double control_rate_;
    
    // Velocity state
    double current_linear_x_;
    double current_linear_y_;
    double current_angular_z_;
    double target_linear_x_;
    double target_linear_y_;
    double target_angular_z_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<SmoothKeyboardController>();
    
    try {
        rclcpp::spin(controller);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(controller->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
} 