/**
 * @file realtime_system_node.cpp
 * @brief PRODUCTION NODE: Integrated real-time control system with RT hardening
 * 
 * Single monolithic node containing all 3 services for <10ms response:
 * - Service 1: Emergency Stop (highest priority)
 * - Service 2: Path Planning (medium priority)
 * - Service 3: Motor Control & Logging (lowest priority)
 * 
 * Real-Time Features:
 * - POSIX SCHED_FIFO scheduling with configurable priority
 * - Memory locking (mlockall) to prevent page faults
 * - CPU affinity pinning for dedicated core execution
 * - Performance tracking and deadline monitoring
 * - CSV logging for offline analysis
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <fstream>
#include <cmath>
#include <limits>

// POSIX Real-Time headers
#include <sys/mman.h>    // mlockall
#include <sched.h>       // sched_setscheduler
#include <pthread.h>     // pthread_setaffinity_np
#include <errno.h>

extern "C" {
    #include "emergency_stop.h"
    #include "path_planner.h"
    #include "motor_control.h"
}

// C++ Ultrasonic sensor drivers
#include "ultrasonic_hcsr04.h"     // sysfs version (fallback)
#include "ultrasonic_hcsr04_rt.h"  // Memory-mapped RT version (preferred)

// Performance tracking structure (similar to fifo_example)
struct PerformanceStats {
    int64_t start_us;
    int64_t end_us;
    int64_t exec_us;
    int64_t delta_us;
    int deadline_miss;
    MotorCommand command;
    float distance;
    float velocity;
    bool emergency;
};

class RealtimeSystemNode : public rclcpp::Node {
public:
    RealtimeSystemNode() : Node("realtime_system") {
        
        // Declare RT parameters
        this->declare_parameter("serial_port_front", "/dev/ttyACM0");
        this->declare_parameter("emergency_distance_threshold", 0.25);
        this->declare_parameter("control_frequency_hz", 1000.0);
        // the control loop is a 1 ms period and 10 ms deadline 
        this->declare_parameter("enable_rt", true);
        this->declare_parameter("rt_priority", 80);
        this->declare_parameter("cpu_affinity", -1);
        this->declare_parameter("enable_mlockall", false);
        this->declare_parameter("deadline_us", 10000);  // 10ms default
            // this deadline was made for performanmce tracking 
            //its  10 ms reaction time 

        this->declare_parameter("enable_csv_logging", true);
        this->declare_parameter("csv_output_path", "/tmp/rtes_performance.csv");
        
        // Get parameters
        serial_port_front_ = this->get_parameter("serial_port_front").as_string();
        double control_freq = this->get_parameter("control_frequency_hz").as_double();
        enable_rt_ = this->get_parameter("enable_rt").as_bool();
        rt_priority_ = this->get_parameter("rt_priority").as_int();
        cpu_affinity_ = this->get_parameter("cpu_affinity").as_int();
        enable_mlockall_ = this->get_parameter("enable_mlockall").as_bool();
        deadline_us_ = this->get_parameter("deadline_us").as_int();
        enable_csv_logging_ = this->get_parameter("enable_csv_logging").as_bool();
        csv_output_path_ = this->get_parameter("csv_output_path").as_string();
        
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "Realtime System Node INITIALIZING");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        
        // Apply RT features if enabled
        if (enable_rt_) {
            setup_realtime_features();
        } else {
            RCLCPP_INFO(this->get_logger(), "Real-time features DISABLED (use enable_rt:=true)");
        }
        
        // Initialize performance logging
        if (enable_csv_logging_) {
            init_csv_logging();
        }
        
        // Initialize C libraries
        emergency_stop_init();
        path_planner_init();
        motor_control_init();
        
        // Initialize serial ports to Arduinos
        init_serial_ports();
        
        // Initialize direct GPIO ultrasonic sensor (optional - can use subscription instead)
        this->declare_parameter("enable_ultrasonic_gpio", false);
        this->declare_parameter("ultrasonic_trigger_pin", 24);
        this->declare_parameter("ultrasonic_echo_pin", 18);
        this->declare_parameter("use_rt_gpio", true);  // Try memory-mapped RT version first
        enable_ultrasonic_gpio_ = this->get_parameter("enable_ultrasonic_gpio").as_bool();
        use_rt_gpio_ = this->get_parameter("use_rt_gpio").as_bool();
        
        if (enable_ultrasonic_gpio_) {
            int trig_pin = this->get_parameter("ultrasonic_trigger_pin").as_int();
            int echo_pin = this->get_parameter("ultrasonic_echo_pin").as_int();
            
            bool initialized = false;
            
            // Try RT version first (requires gpio group membership)
            if (use_rt_gpio_) {
                ultrasonic_sensor_rt_ = UltrasonicHCSR04RT(trig_pin, echo_pin);
                if (ultrasonic_sensor_rt_.initialize()) {
                    RCLCPP_INFO(this->get_logger(), " Ultrasonic RT GPIO initialized (memory-mapped, GPIO%d/GPIO%d)", 
                               trig_pin, echo_pin);
                    RCLCPP_INFO(this->get_logger(), "  • <100μs GPIO latency, 5ms timeout, CLOCK_MONOTONIC");
                    initialized = true;
                } else {
                    RCLCPP_WARN(this->get_logger(), " Failed to initialize RT GPIO, trying sysfs fallback...");
                    use_rt_gpio_ = false;
                }
            }
            
            // Fallback to sysfs version if RT version failed or not requested
            if (!initialized) {
                ultrasonic_sensor_ = UltrasonicHCSR04(trig_pin, echo_pin);
                if (ultrasonic_sensor_.initialize()) {
                    RCLCPP_INFO(this->get_logger(), " Ultrasonic GPIO sensor initialized (sysfs, GPIO%d/GPIO%d)", 
                               trig_pin, echo_pin);
                    RCLCPP_WARN(this->get_logger(), "   Using sysfs (1-5ms latency) - not hard RT compliant");
                    initialized = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), " Failed to initialize ultrasonic GPIO sensor");
                    enable_ultrasonic_gpio_ = false;
                }
            }
            
            if (initialized) {
                // Create publisher for direct sensor readings
                ultrasonic_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic", 10);
            }
        }
        
        // ROS2 Subscriptions (only subscribe if NOT using direct GPIO)
        if (!enable_ultrasonic_gpio_) {
            ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
                "/sensor/ultrasonic", 10,
                std::bind(&RealtimeSystemNode::ultrasonic_callback, this, std::placeholders::_1));
        }
        
        
        // ROS2 Publishers (for monitoring/debugging)
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_internal", 10);
        
        // High-speed control loop timer
        // this is only used for logging 

        auto period_ms = std::chrono::microseconds(static_cast<int>(1000000.0 / control_freq));
        timer_ = this->create_wall_timer(
            period_ms,
            std::bind(&RealtimeSystemNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Control frequency: %.0f Hz", control_freq);
        RCLCPP_INFO(this->get_logger(), "Deadline: %ld μs (%.2f ms)", deadline_us_, deadline_us_ / 1000.0);
        RCLCPP_INFO(this->get_logger(), "Serial Port: %s", serial_port_front_.c_str());
        if (enable_csv_logging_) {
            RCLCPP_INFO(this->get_logger(), "CSV Logging: %s", csv_output_path_.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
    }
    
    ~RealtimeSystemNode() {
        // Send stop command before shutdown
        send_motor_command(MOTOR_STOP);
        
        // Cleanup ultrasonic sensor
        if (enable_ultrasonic_gpio_) {
            if (use_rt_gpio_) {
                ultrasonic_sensor_rt_.cleanup();
            } else {
                ultrasonic_sensor_.cleanup();
            }
        }
        
        // Close serial port
        if (serial_fd_front_ >= 0) {
            close(serial_fd_front_);
        }
        
        // Write final statistics and close CSV
        if (csv_file_.is_open()) {
            write_performance_summary();
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), " Performance data saved to %s", csv_output_path_.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Realtime System Node SHUTDOWN");
    }

private:
    // ═════════════════════════════════════════════════════════════════════
    // REAL-TIME SETUP FUNCTIONS
    // ═════════════════════════════════════════════════════════════════════
    
    void setup_realtime_features() {
        RCLCPP_INFO(this->get_logger(), "Applying real-time features...");
        
        // Phase 1: Memory Locking
        if (enable_mlockall_) {
            if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
                RCLCPP_INFO(this->get_logger(), " Memory locked (mlockall)");
            } else {
                RCLCPP_WARN(this->get_logger(), " mlockall failed: %s", strerror(errno));
                RCLCPP_WARN(this->get_logger(), "  Try: sudo setcap cap_ipc_lock=eip realtime_system_node");
            }
        }
        
        // Phase 2: POSIX Real-Time Scheduling
        struct sched_param param;
        param.sched_priority = rt_priority_;
        
        if (sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
            RCLCPP_INFO(this->get_logger(), " SCHED_FIFO enabled (priority %d)", rt_priority_);
        } else {
            RCLCPP_WARN(this->get_logger(), " sched_setscheduler failed: %s", strerror(errno));
            RCLCPP_WARN(this->get_logger(), "  Try: sudo setcap cap_sys_nice=eip realtime_system_node");
        }
        
        // Phase 3: CPU Affinity
        if (cpu_affinity_ >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(cpu_affinity_, &cpuset);
            
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) == 0) {
                RCLCPP_INFO(this->get_logger(), " CPU affinity set to core %d", cpu_affinity_);
            } else {
                RCLCPP_WARN(this->get_logger(), "ERROR: pthread_setaffinity_np failed: %s", strerror(errno));
            }
        }
    }
    
    void init_csv_logging() {
        csv_file_.open(csv_output_path_);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_output_path_.c_str());
            enable_csv_logging_ = false;
            return;
        }
        
        // Write CSV header
        csv_file_ << "frame,start_us,end_us,exec_us,delta_us,deadline_us,miss,"
                  << "distance_m,velocity_ms,command,emergency\n";
        csv_file_.flush();
        
        RCLCPP_INFO(this->get_logger(), "Working: CSV logging initialized");
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // SERIAL PORT SETUP
    // ═════════════════════════════════════════════════════════════════════
    void init_serial_ports() {
        // Open front Arduino
        serial_fd_front_ = open(serial_port_front_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_front_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open front serial port: %s", 
                         serial_port_front_.c_str());
        } else {
            configure_serial_port(serial_fd_front_);
            RCLCPP_INFO(this->get_logger(), "Working: Front Arduino connected");
        }
        
       
    }
    
    void configure_serial_port(int fd) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            return;
        }
        
        // 115200 baud for minimal latency (~0.087ms per byte vs 1.04ms at 9600)
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        tcsetattr(fd, TCSANOW, &tty);
    }
    
    void ultrasonic_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        ultrasonic_distance_ = msg->range;
    }
    
    // Read ultrasonic sensor directly from GPIO (C++ deterministic driver)
    void read_ultrasonic_sensor() {
        float distance_m = 0.0f;
        bool success = false;
        
        // Use appropriate driver version
        if (use_rt_gpio_) {
            success = ultrasonic_sensor_rt_.read_distance(distance_m);
        } else {
            success = ultrasonic_sensor_.read_distance(distance_m);
        }
        
        if (success) {
            // Valid reading - update distance
            ultrasonic_distance_ = distance_m;
            
            // Publish Range message for ROS2 ecosystem
            auto msg = sensor_msgs::msg::Range();
            msg.header.stamp = this->now();
            msg.header.frame_id = "ultrasonic_link";
            msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            msg.field_of_view = 0.26f;  // ~15 degrees in radians
            msg.min_range = 0.02f;      // 2cm
            msg.max_range = 4.0f;       // 4m
            msg.range = distance_m;
            ultrasonic_pub_->publish(msg);
        } else {
            // Read failed - use last valid distance
            // (Don't update ultrasonic_distance_, keep last good value)
            if (loop_count_ % 1000 == 0) {  // Log errors every 1 second at 1kHz
                RCLCPP_WARN(this->get_logger(), "Ultrasonic read failed, using last value: %.2fm", 
                           ultrasonic_distance_);
            }
        }
    }
    
    void control_loop() {
        // Start timing (high-resolution monotonic clock)
        struct timespec start_ts, end_ts;
        clock_gettime(CLOCK_MONOTONIC, &start_ts);
        
        // Read ultrasonic sensor at ~14 Hz (every 70ms) instead of 1kHz
        // HC-SR04 requires minimum 60ms measurement cycle
        if (enable_ultrasonic_gpio_) {
            if (ultrasonic_decimation_counter_++ >= 60) {  // 60 frames * 1.18ms = 70ms (> 60ms min)
                ultrasonic_decimation_counter_ = 0;
                read_ultrasonic_sensor();
            }
        }
        
        // ═══════════════════════════════════════════════════════════════════
        // SERVICE 1: EMERGENCY STOP (HIGHEST PRIORITY)
        // ═══════════════════════════════════════════════════════════════════
        EmergencyStopState emergency_state;
        bool emergency = check_emergency_stop(
            ultrasonic_distance_, 
            current_velocity_,
            &emergency_state);
        
        MotorCommand current_command;
        
        if (emergency) {
            // IMMEDIATE STOP - Preempt all lower services
            current_command = MOTOR_STOP;
            send_motor_command(MOTOR_STOP);
            
            // Publish emergency status for monitoring
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            emergency_pub_->publish(msg);
            
            // Log critical event
            if (emergency_count_++ % 100 == 0) {  // Log every 100ms at 1kHz
                RCLCPP_ERROR(this->get_logger(), 
                    "EMERGENCY STOP! Distance: %.2fm, Velocity: %.2fm/s, TTC: %.3fs",
                    ultrasonic_distance_, current_velocity_, emergency_state.ttc);
            }

        } else {
            // Reset emergency counter
            emergency_count_ = 0;
            
            // ═══════════════════════════════════════════════════════════════════
            // SERVICE 2: PATH PLANNING (MEDIUM PRIORITY)
            // ═══════════════════════════════════════════════════════════════════
            PathCommand path_cmd = compute_path_command(
                ultrasonic_distance_,
                current_velocity_);
            
            // ═══════════════════════════════════════════════════════════════════
            // SERVICE 3: MOTOR CONTROL & LOGGING (LOWEST PRIORITY)
            // ═══════════════════════════════════════════════════════════════════
            MotorControlState motor_state;
            bool executed = execute_motor_command(
                path_cmd.command, 
                2,  // Priority 2 (Service 2)
                &motor_state);
            
            if (executed) {
                send_motor_command(path_cmd.command);
            }
            
            current_command = path_cmd.command;
        }
        
        // End timing (before any periodic logging to exclude that overhead)
        clock_gettime(CLOCK_MONOTONIC, &end_ts);
        
        // Calculate timing metrics
        int64_t start_us = timespec_to_us(&start_ts);
        int64_t end_us = timespec_to_us(&end_ts);
        int64_t exec_us = end_us - start_us;
        int64_t delta_us = (loop_count_ > 0) ? (end_us - prev_end_us_) : 0;
        int deadline_miss = (delta_us > deadline_us_) ? 1 : 0;
        
        // Update statistics (exclude first frame)
        if (loop_count_ > 0) {
            exec_sum_us_ += exec_us;
            delta_sum_us_ += delta_us;
            miss_count_ += deadline_miss;
            
            if (exec_us > exec_max_us_) exec_max_us_ = exec_us;
            if (exec_us < exec_min_us_) exec_min_us_ = exec_us;
            if (delta_us > delta_max_us_) delta_max_us_ = delta_us;
            if (delta_us < delta_min_us_) delta_min_us_ = delta_us;
            
            exec_sum_sq_ += (double)exec_us * (double)exec_us;
            delta_sum_sq_ += (double)delta_us * (double)delta_us;
        }
        
        // Log to CSV
        if (enable_csv_logging_ && csv_file_.is_open()) {
            csv_file_ << loop_count_ << ","
                      << start_us << ","
                      << end_us << ","
                      << exec_us << ","
                      << delta_us << ","
                      << deadline_us_ << ","
                      << deadline_miss << ","
                      << ultrasonic_distance_ << ","
                      << current_velocity_ << ","
                      << motor_command_to_char(current_command) << ","
                      << (emergency ? 1 : 0) << "\n";
            
            // Flush periodically
            if (loop_count_ % 100 == 0) {
                csv_file_.flush();
            }
        }
        
        // Periodic status reporting (every 1 second at 1kHz)
        if (loop_count_ % 1000 == 0 && loop_count_ > 0) {
            double avg_exec_us = (double)exec_sum_us_ / (double)loop_count_;
            double avg_delta_us = (double)delta_sum_us_ / (double)loop_count_;
            double fps = 1000000.0 / avg_delta_us;
            double miss_percent = (100.0 * (double)miss_count_) / (double)loop_count_;
            
            RCLCPP_INFO(this->get_logger(), 
                "RT Stats: Exec=%.2fμs, FPS=%.1f, Misses=%d (%.2f%%), Dist=%.2fm, Vel=%.2fm/s",
                avg_exec_us, fps, miss_count_, miss_percent,
                ultrasonic_distance_, current_velocity_);
        }
        
        prev_end_us_ = end_us;
        loop_count_++;
    }
    
    void send_motor_command(MotorCommand command) {
        char cmd_char = motor_command_to_char(command);
        
        // Send to Arduino
        if (serial_fd_front_ >= 0) {
            write(serial_fd_front_, &cmd_char, 1);
        }
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // PERFORMANCE TRACKING UTILITIES
    // ═════════════════════════════════════════════════════════════════════
    
    static int64_t timespec_to_us(const struct timespec *ts) {
        return ((int64_t)ts->tv_sec * 1000000LL) + (ts->tv_nsec / 1000LL);
    }
    
    void write_performance_summary() {
        if (loop_count_ <= 1) {
            return;  // Not enough data
        }
        
        int stats_count = loop_count_ - 1;  // Exclude first frame
        double avg_exec_us = (double)exec_sum_us_ / (double)stats_count;
        double avg_delta_us = (double)delta_sum_us_ / (double)stats_count;
        double fps = 1000000.0 / avg_delta_us;
        double miss_percent = (100.0 * (double)miss_count_) / (double)stats_count;
        
        // Calculate standard deviations
        double exec_variance = (exec_sum_sq_ / (double)stats_count) - (avg_exec_us * avg_exec_us);
        double delta_variance = (delta_sum_sq_ / (double)stats_count) - (avg_delta_us * avg_delta_us);
        double exec_std = (exec_variance > 0) ? sqrt(exec_variance) : 0.0;
        double delta_std = (delta_variance > 0) ? sqrt(delta_variance) : 0.0;
        
        // Calculate jitter
        int64_t exec_jitter = exec_max_us_ - exec_min_us_;
        int64_t delta_jitter = delta_max_us_ - delta_min_us_;
        
        // Calculate coefficient of variation
        double exec_cv = (avg_exec_us > 0) ? (exec_std / avg_exec_us) : 0.0;
        double delta_cv = (avg_delta_us > 0) ? (delta_std / avg_delta_us) : 0.0;
        
        // Write summary to CSV
        csv_file_ << "\n# Performance Summary (excluding frame 0)\n";
        csv_file_ << "# Frames analyzed," << stats_count << "\n";
        csv_file_ << "# Average exec time us," << avg_exec_us << "\n";
        csv_file_ << "# Exec std dev us," << exec_std << "\n";
        csv_file_ << "# Min exec time us," << exec_min_us_ << "\n";
        csv_file_ << "# Max exec time us," << exec_max_us_ << "\n";
        csv_file_ << "# Exec jitter us," << exec_jitter << "\n";
        csv_file_ << "# Exec CV," << exec_cv << "\n";
        csv_file_ << "# Average frame delta us," << avg_delta_us << "\n";
        csv_file_ << "# Delta std dev us," << delta_std << "\n";
        csv_file_ << "# Min frame delta us," << delta_min_us_ << "\n";
        csv_file_ << "# Max frame delta us," << delta_max_us_ << "\n";
        csv_file_ << "# Delta jitter us," << delta_jitter << "\n";
        csv_file_ << "# Delta CV," << delta_cv << "\n";
        csv_file_ << "# Estimated FPS," << fps << "\n";
        csv_file_ << "# Deadline us," << deadline_us_ << "\n";
        csv_file_ << "# Deadline misses," << miss_count_ << "\n";
        csv_file_ << "# Miss percentage," << miss_percent << "\n";
        
        // Log summary to console
        RCLCPP_INFO(this->get_logger(), "\n═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "PERFORMANCE SUMMARY");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "Frames: %d", stats_count);
        RCLCPP_INFO(this->get_logger(), "Exec Time: %.2f±%.2f μs (%.2f-%.2f μs)",
                    avg_exec_us, exec_std, (double)exec_min_us_, (double)exec_max_us_);
        RCLCPP_INFO(this->get_logger(), "Exec Jitter: %ld μs (%.2f ms)", exec_jitter, exec_jitter / 1000.0);
        RCLCPP_INFO(this->get_logger(), "Frame Rate: %.2f FPS (%.2f ms period)", fps, avg_delta_us / 1000.0);
        RCLCPP_INFO(this->get_logger(), "Deadline: %ld μs (%.2f ms)", deadline_us_, deadline_us_ / 1000.0);
        RCLCPP_INFO(this->get_logger(), "Misses: %d / %d (%.2f%%)", miss_count_, stats_count, miss_percent);
        
        if (miss_percent < 1.0) {
            RCLCPP_INFO(this->get_logger(), " Excellent real-time performance");
        } else if (miss_percent < 5.0) {
            RCLCPP_INFO(this->get_logger(), " Good real-time performance");
        } else if (miss_percent < 10.0) {
            RCLCPP_WARN(this->get_logger(), " Marginal real-time performance");
        } else {
            RCLCPP_ERROR(this->get_logger(), " Poor real-time performance");
        }
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // MEMBER VARIABLES
    // ═════════════════════════════════════════════════════════════════════
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_pub_;  // Direct sensor publishing
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Direct GPIO ultrasonic sensor (C++)
    UltrasonicHCSR04RT ultrasonic_sensor_rt_;  // Memory-mapped RT version
    UltrasonicHCSR04 ultrasonic_sensor_;       // sysfs fallback version
    bool enable_ultrasonic_gpio_ = false;
    bool use_rt_gpio_ = true;  // Prefer RT version
    
    // Serial port
    int serial_fd_front_ = -1;
    std::string serial_port_front_;
    
    // Real-time configuration
    bool enable_rt_;
    int rt_priority_;
    int cpu_affinity_;
    bool enable_mlockall_;
    int64_t deadline_us_;
    
    // Performance logging
    bool enable_csv_logging_;
    std::string csv_output_path_;
    std::ofstream csv_file_;
    
    // Sensor data
    float ultrasonic_distance_ = 10.0f;
    float current_velocity_ = 0.0f;
    
    // Performance tracking
    uint64_t loop_count_ = 0;
    uint32_t ultrasonic_decimation_counter_ = 0;
    uint32_t emergency_count_ = 0;
    int64_t prev_end_us_ = 0;
    int miss_count_ = 0;
    
    // Statistics (excluding first frame)
    int64_t exec_sum_us_ = 0;
    int64_t delta_sum_us_ = 0;
    int64_t exec_max_us_ = 0;
    int64_t delta_max_us_ = 0;
    int64_t exec_min_us_ = std::numeric_limits<int64_t>::max();
    int64_t delta_min_us_ = std::numeric_limits<int64_t>::max();
    double exec_sum_sq_ = 0.0;
    double delta_sum_sq_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RealtimeSystemNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
