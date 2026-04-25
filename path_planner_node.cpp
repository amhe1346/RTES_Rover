/**
 * @file path_planner_node.cpp
 * @brief DEVELOPMENT NODE: Service 2 - Path Planning (isolated testing)
 * 
 * Standalone node for testing path planning logic independently.
 * Generates navigation commands based on sensor data.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

extern "C" {
    #include "path_planner.h"
    #include "motor_control.h"
}

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner") {
        
        // Initialize C library
        path_planner_init();
        
        // Subscriptions
        ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensor/ultrasonic", 10,
            std::bind(&PathPlannerNode::ultrasonic_callback, this, std::placeholders::_1));
        
        emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            std::bind(&PathPlannerNode::emergency_callback, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_planned", 10);
        
        // Timer: 10Hz (100ms periodic updates)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathPlannerNode::plan_path, this));
        
        RCLCPP_INFO(this->get_logger(), "Path Planner Node (DEV) initialized");
    }

private:
    void ultrasonic_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        distance_m_ = msg->range;
    }
    
    void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        emergency_active_ = msg->data;
    }
    
    void plan_path() {
        // Don't plan if emergency stop active
        if (emergency_active_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Path planning suspended - emergency stop active");
            return;
        }
        
        // Call C library function
        PathCommand path_cmd = compute_path_command(distance_m_, 0.0f);
        
        // Convert to ROS2 Twist message
        auto twist = geometry_msgs::msg::Twist();
        
        switch (path_cmd.command) {
            case MOTOR_FORWARD:
                twist.linear.x = 0.5;  // Forward velocity
                twist.angular.z = 0.0;
                break;
            case MOTOR_BACKWARD:
                twist.linear.x = -0.3;  // Backward velocity
                twist.angular.z = 0.0;
                break;
            case MOTOR_LEFT:
                twist.linear.x = 0.2;
                twist.angular.z = 0.5;  // Turn left
                break;
            case MOTOR_RIGHT:
                twist.linear.x = 0.2;
                twist.angular.z = -0.5;  // Turn right
                break;
            case MOTOR_STOP:
            default:
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                break;
        }
        
        cmd_vel_pub_->publish(twist);
        
        // Logging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Path: Dist=%.2fm, Cmd=%c, Confidence=%.2f",
            distance_m_, motor_command_to_char(path_cmd.command), path_cmd.confidence);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    float distance_m_ = 10.0f;
    bool emergency_active_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
