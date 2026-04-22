/**
 * @file motor_control_node.cpp
 * @brief DEVELOPMENT NODE: Service 3 - Motor Control & Logging (isolated testing)
 * 
 * Standalone node for testing motor control with priority arbitration.
 * Receives commands from emergency stop and path planning services.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

extern "C" {
    #include "motor_control.h"
}

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control") {
        
        // Declare parameters
        this->declare_parameter("serial_port_front", "/dev/ttyACM1");
        this->declare_parameter("serial_port_rear", "/dev/ttyACM0");
        
        serial_port_front_ = this->get_parameter("serial_port_front").as_string();
        serial_port_rear_ = this->get_parameter("serial_port_rear").as_string();
        
        // Initialize C library
        motor_control_init();
        
        // Initialize serial
        init_serial_ports();
        
        // Subscriptions
        emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            std::bind(&MotorControlNode::emergency_callback, this, std::placeholders::_1));
        
        cmd_vel_planned_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_planned", 10,
            std::bind(&MotorControlNode::cmd_vel_planned_callback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // Timer: 20Hz (50ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MotorControlNode::execute_commands, this));
        
        RCLCPP_INFO(this->get_logger(), "Motor Control Node (DEV) initialized");
        RCLCPP_INFO(this->get_logger(), "Front: %s, Rear: %s", 
                    serial_port_front_.c_str(), serial_port_rear_.c_str());
    }
    
    ~MotorControlNode() {
        send_motor_command(MOTOR_STOP);
        if (serial_fd_front_ >= 0) close(serial_fd_front_);
        if (serial_fd_rear_ >= 0) close(serial_fd_rear_);
    }

private:
    void init_serial_ports() {
        serial_fd_front_ = open(serial_port_front_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_front_ >= 0) {
            configure_serial_port(serial_fd_front_);
            RCLCPP_INFO(this->get_logger(), "✓ Front Arduino connected");
        }
        
        serial_fd_rear_ = open(serial_port_rear_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_rear_ >= 0) {
            configure_serial_port(serial_fd_rear_);
            RCLCPP_INFO(this->get_logger(), "✓ Rear Arduino connected");
        }
    }
    
    void configure_serial_port(int fd) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd, &tty);
        
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        tty.c_cflag |= (CLOCAL | CREAD);
        
        tcsetattr(fd, TCSANOW, &tty);
    }
    
    void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        emergency_active_ = msg->data;
    }
    
    void cmd_vel_planned_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        planned_cmd_ = twist_to_motor_command(*msg);
        planned_cmd_time_ = this->now();
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        teleop_cmd_ = twist_to_motor_command(*msg);
        teleop_cmd_time_ = this->now();
    }
    
    MotorCommand twist_to_motor_command(const geometry_msgs::msg::Twist& twist) {
        if (std::abs(twist.angular.z) > 0.1) {
            return (twist.angular.z > 0) ? MOTOR_LEFT : MOTOR_RIGHT;
        } else if (std::abs(twist.linear.x) > 0.1) {
            return (twist.linear.x > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        }
        return MOTOR_STOP;
    }
    
    void execute_commands() {
        MotorCommand cmd_to_execute = MOTOR_STOP;
        uint8_t priority = 255;
        std::string source = "none";
        
        // Priority 1: Emergency stop
        if (emergency_active_) {
            cmd_to_execute = MOTOR_STOP;
            priority = 1;
            source = "EMERGENCY";
        }
        // Priority 2: Path planning
        else if ((this->now() - planned_cmd_time_).seconds() < 0.5) {
            cmd_to_execute = planned_cmd_;
            priority = 2;
            source = "PATH_PLAN";
        }
        // Priority 3: Teleop
        else if ((this->now() - teleop_cmd_time_).seconds() < 0.5) {
            cmd_to_execute = teleop_cmd_;
            priority = 3;
            source = "TELEOP";
        }
        
        // Execute via C library
        MotorControlState state;
        bool executed = execute_motor_command(cmd_to_execute, priority, &state);
        
        if (executed) {
            send_motor_command(cmd_to_execute);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Executing: %c (priority: %u, source: %s, count: %u)",
                motor_command_to_char(cmd_to_execute), priority, 
                source.c_str(), state.command_count);
        }
    }
    
    void send_motor_command(MotorCommand command) {
        char cmd_char = motor_command_to_char(command);
        
        if (serial_fd_front_ >= 0) {
            write(serial_fd_front_, &cmd_char, 1);
        }
        if (serial_fd_rear_ >= 0) {
            write(serial_fd_rear_, &cmd_char, 1);
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_planned_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int serial_fd_front_ = -1;
    int serial_fd_rear_ = -1;
    std::string serial_port_front_;
    std::string serial_port_rear_;
    
    bool emergency_active_ = false;
    MotorCommand planned_cmd_ = MOTOR_STOP;
    MotorCommand teleop_cmd_ = MOTOR_STOP;
    rclcpp::Time planned_cmd_time_;
    rclcpp::Time teleop_cmd_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
