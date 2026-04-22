/**
 * @file emergency_stop_node.cpp
 * @brief DEVELOPMENT NODE: Service 1 - Emergency Stop (isolated testing)
 * 
 * Standalone node for testing emergency stop logic independently.
 * Monitors sensors and publishes emergency stop signals.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

extern "C" {
    #include "emergency_stop.h"
}

class EmergencyStopNode : public rclcpp::Node {
public:
    EmergencyStopNode() : Node("emergency_stop") {
        
        // Initialize C library
        emergency_stop_init();
        
        // Subscriptions
        ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensor/ultrasonic", 10,
            std::bind(&EmergencyStopNode::ultrasonic_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&EmergencyStopNode::odom_callback, this, std::placeholders::_1));
        
        // Publisher
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        
        // Timer: 100Hz for development/testing (slower than production)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EmergencyStopNode::check_safety, this));
        
        RCLCPP_INFO(this->get_logger(), "Emergency Stop Node (DEV) initialized");
    }

private:
    void ultrasonic_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        distance_m_ = msg->range;
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        velocity_ms_ = msg->twist.twist.linear.x;
    }
    
    void check_safety() {
        EmergencyStopState state;
        
        // Call C library function
        bool emergency = check_emergency_stop(distance_m_, velocity_ms_, &state);
        
        // Publish result
        auto msg = std_msgs::msg::Bool();
        msg.data = emergency;
        emergency_pub_->publish(msg);
        
        // Detailed logging for development
        if (emergency) {
            RCLCPP_WARN(this->get_logger(),
                "⚠ EMERGENCY! Dist: %.2fm, Vel: %.2fm/s, TTC: %.3fs, StopDist: %.2fm",
                state.distance_m, state.velocity_ms, state.ttc, state.safe_stopping_distance);
        }
        
        // Periodic status
        if (check_count_++ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Distance: %.2fm, Velocity: %.2fm/s, Emergency: %s",
                distance_m_, velocity_ms_, emergency ? "YES" : "no");
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    float distance_m_ = 10.0f;
    float velocity_ms_ = 0.0f;
    uint32_t check_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyStopNode>());
    rclcpp::shutdown();
    return 0;
}
