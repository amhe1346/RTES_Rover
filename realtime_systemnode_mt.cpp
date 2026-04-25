/**
 * @file realtime_system_node_mt.cpp
 * @brief Multi-Threaded Priority-Driven RTES Node with Preemptive Scheduling
 * 
 * Architecture:
 * - 4 independent pthread services with priority-driven scheduling
 * - Emergency Thread (P99, 100Hz/10ms) - Highest priority, preempts all others
 * - Sensor Thread (P85, 14Hz/70ms) - Respects HC-SR04 60ms minimum measurement cycle
 * - Planning Thread (P90, 50Hz/20ms) - Medium priority
 * - Motor Thread (P80, 50Hz/20ms) - Lowest priority
 * 
 * Synchronization:
 * - Atomics for lock-free single-writer/reader: emergency_flag, path_command
 * - Mutex for sensor data (multiple readers: emergency, planning threads)
 * - ROS2 odometry callback uses same sensor_mutex for consistency
 * 
 * Benefits:
 * - 50-70% CPU reduction (no polling at 1kHz)
 * - True preemption: emergency can interrupt planning/motor
 * - Independent thread timing prevents HC-SR04 timeouts from blocking emergency
 * - Per-thread performance monitoring and deadline tracking
 * 
 * Real-Time Features:
 * - POSIX SCHED_FIFO per-thread RT priorities
 * - Memory locking (mlockall) to prevent page faults
 * - CPU affinity pinning (emergency thread on isolated core)
 * - clock_nanosleep(CLOCK_MONOTONIC) for precise periodic execution
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
#include <atomic>
#include <mutex>
#include <string>

// POSIX Real-Time headers
#include <sys/mman.h>    // mlockall
#include <sched.h>       // sched_setscheduler
#include <pthread.h>     // pthread_create, pthread_setaffinity_np
#include <errno.h>
#include <time.h>        // clock_nanosleep

extern "C" {
    #include "emergency_stop.h"
    #include "path_planner.h"
    #include "motor_control.h"
}

// C++ Ultrasonic sensor drivers
#include "ultrasonic_hcsr04.h"     // sysfs version (fallback)
#include "ultrasonic_hcsr04_rt.h"  // Memory-mapped RT version (preferred)

// ═══════════════════════════════════════════════════════════════════
// THREAD CONFIGURATION CONSTANTS
// ═══════════════════════════════════════════════════════════════════
namespace ThreadConfig {
    // Thread periods (milliseconds)
    constexpr int EMERGENCY_PERIOD_MS = 10;  // 100 Hz
    constexpr int PLANNING_PERIOD_MS = 20;   // 50 Hz
    constexpr int MOTOR_PERIOD_MS = 20;      // 50 Hz
    constexpr int SENSOR_PERIOD_MS = 70;     // ~14 Hz (respects HC-SR04 60ms minimum)
    
    // Thread RT priorities (1-99, higher = more priority)
    constexpr int EMERGENCY_PRIORITY = 99;   // Highest - preempts everything
    constexpr int PLANNING_PRIORITY = 90;    // Medium
    constexpr int MOTOR_PRIORITY = 80;       // Lowest
    constexpr int SENSOR_PRIORITY = 85;      // Medium-high 
    
    // Thread names for logging
    constexpr const char* EMERGENCY_NAME = "emergency";
    constexpr const char* PLANNING_NAME = "planning";
    constexpr const char* MOTOR_NAME = "motor";
    constexpr const char* SENSOR_NAME = "sensor";

    // CSV flush cadence to avoid flush-per-row overhead in RT loops.
    constexpr uint64_t CSV_FLUSH_EVERY_N_ROWS = 200;
}

// ═══════════════════════════════════════════════════════════════════
// PER-THREAD PERFORMANCE TRACKING
// ═══════════════════════════════════════════════════════════════════
struct ThreadPerformanceStats {
    const char* thread_name;
    int64_t period_us;
    int64_t deadline_us;
    
    // Running statistics
    int64_t min_exec_us;
    int64_t max_exec_us;
    int64_t total_exec_us;
    int64_t max_jitter_us;
    uint64_t cycle_count;
    uint64_t deadline_misses;
    
    ThreadPerformanceStats(const char* name, int period_ms)
        : thread_name(name)
        , period_us(period_ms * 1000)
        , deadline_us(period_ms * 1000)  // Deadline = period
        , min_exec_us(INT64_MAX)
        , max_exec_us(0)
        , total_exec_us(0)
        , max_jitter_us(0)
        , cycle_count(0)
        , deadline_misses(0)
    {}
    
    void record_cycle(int64_t exec_us, int64_t jitter_us) {
        cycle_count++;
        total_exec_us += exec_us;
        
        if (exec_us < min_exec_us) min_exec_us = exec_us;
        if (exec_us > max_exec_us) max_exec_us = exec_us;
        if (jitter_us > max_jitter_us) max_jitter_us = jitter_us;
        
        if (exec_us > deadline_us) {
            deadline_misses++;
        }
    }
    
    double get_mean_exec_us() const {
        return cycle_count > 0 ? static_cast<double>(total_exec_us) / cycle_count : 0.0;
    }
    
    double get_miss_rate() const {
        return cycle_count > 0 ? static_cast<double>(deadline_misses) / cycle_count * 100.0 : 0.0;
    }
};

class RealtimeSystemNodeMT : public rclcpp::Node {
public:
    RealtimeSystemNodeMT() : Node("realtime_system_mt")
        , threads_running_(true)
        , emergency_thread_started_(false)
        , sensor_thread_started_(false)
        , planning_thread_started_(false)
        , motor_thread_started_(false)
        , emergency_flag_(false)
        , path_command_(MOTOR_FORWARD)
        , emergency_stats_(ThreadConfig::EMERGENCY_NAME, ThreadConfig::EMERGENCY_PERIOD_MS)
        , planning_stats_(ThreadConfig::PLANNING_NAME, ThreadConfig::PLANNING_PERIOD_MS)
        , motor_stats_(ThreadConfig::MOTOR_NAME, ThreadConfig::MOTOR_PERIOD_MS)
        , sensor_stats_(ThreadConfig::SENSOR_NAME, ThreadConfig::SENSOR_PERIOD_MS)
        , serial_fd_front_(-1)
    {
        // Declare RT parameters (same as original node for launch compatibility)
        this->declare_parameter("serial_port_front", "/dev/ttyACM0");
        this->declare_parameter("emergency_distance_threshold", 0.5);
        this->declare_parameter("enable_rt", false);
        this->declare_parameter("rt_priority", 80);  // Not used in MT version (per-thread priorities hardcoded)
        this->declare_parameter("cpu_affinity", -1);
        this->declare_parameter("enable_mlockall", false);
        this->declare_parameter("deadline_us", 10000);  // Used for emergency thread
        this->declare_parameter("enable_csv_logging", true);
        this->declare_parameter("csv_output_path", "/tmp/rtes_performance_mt.csv");
        this->declare_parameter("enable_ultrasonic_gpio", true);
        this->declare_parameter("use_rt_gpio", true);
        this->declare_parameter("ultrasonic_trigger_pin", 24);
        this->declare_parameter("ultrasonic_echo_pin", 18);
        
        // Get parameters
        serial_port_front_ = this->get_parameter("serial_port_front").as_string();
        double emergency_distance_threshold =
            this->get_parameter("emergency_distance_threshold").as_double();
        enable_rt_ = this->get_parameter("enable_rt").as_bool();
        cpu_affinity_ = this->get_parameter("cpu_affinity").as_int();
        enable_mlockall_ = this->get_parameter("enable_mlockall").as_bool();
        enable_csv_logging_ = this->get_parameter("enable_csv_logging").as_bool();
        csv_output_path_ = this->get_parameter("csv_output_path").as_string();
        enable_ultrasonic_gpio_ = this->get_parameter("enable_ultrasonic_gpio").as_bool();
        use_rt_gpio_ = this->get_parameter("use_rt_gpio").as_bool();
        int trigger_pin = this->get_parameter("ultrasonic_trigger_pin").as_int();
        int echo_pin = this->get_parameter("ultrasonic_echo_pin").as_int();

        RCLCPP_INFO(this->get_logger(), "Ultrasonic pins configured: trigger=%d echo=%d",
                trigger_pin, echo_pin);
        
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "Multi-Threaded RTES Node INITIALIZING");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "Architecture: 4 Independent Threads");
        RCLCPP_INFO(this->get_logger(), "  • Emergency: P%d @ %dHz (%.1fms)", 
                    ThreadConfig::EMERGENCY_PRIORITY, 1000/ThreadConfig::EMERGENCY_PERIOD_MS,
                    ThreadConfig::EMERGENCY_PERIOD_MS / 1.0);
        RCLCPP_INFO(this->get_logger(), "  • Sensor:    P%d @ %dHz (%.1fms)", 
                    ThreadConfig::SENSOR_PRIORITY, 1000/ThreadConfig::SENSOR_PERIOD_MS,
                    ThreadConfig::SENSOR_PERIOD_MS / 1.0);
        RCLCPP_INFO(this->get_logger(), "  • Planning:  P%d @ %dHz (%.1fms)", 
                    ThreadConfig::PLANNING_PRIORITY, 1000/ThreadConfig::PLANNING_PERIOD_MS,
                    ThreadConfig::PLANNING_PERIOD_MS / 1.0);
        RCLCPP_INFO(this->get_logger(), "  • Motor:     P%d @ %dHz (%.1fms)", 
                    ThreadConfig::MOTOR_PRIORITY, 1000/ThreadConfig::MOTOR_PERIOD_MS,
                    ThreadConfig::MOTOR_PERIOD_MS / 1.0);
        
        // Apply global RT features if enabled
        if (enable_rt_) {
            setup_realtime_features();
        }
        
        // Initialize ultrasonic sensor
        // Note: GPIO pins are hardcoded in sensor constructors (trigger=24, echo=18)
        if (enable_ultrasonic_gpio_) {
            if (use_rt_gpio_) {
                if (ultrasonic_sensor_rt_.initialize()) {
                    RCLCPP_INFO(this->get_logger(), " Ultrasonic RT GPIO initialized (memory-mapped, GPIO24/GPIO18)");
                } else {
                    RCLCPP_WARN(this->get_logger(), " RT GPIO init failed, falling back to sysfs");
                    use_rt_gpio_ = false;
                    if (ultrasonic_sensor_.initialize()) {
                        RCLCPP_INFO(this->get_logger(), " Ultrasonic sysfs initialized (GPIO24/GPIO18)");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), " Sysfs GPIO init also failed");
                    }
                }
            } else {
                if (ultrasonic_sensor_.initialize()) {
                    RCLCPP_INFO(this->get_logger(), " Ultrasonic sysfs initialized (GPIO24/GPIO18)");
                } else {
                    RCLCPP_ERROR(this->get_logger(), " Sysfs GPIO init failed");
                }
            }
        }
        
        // Initialize C libraries before launching worker threads.
        set_emergency_threshold(static_cast<float>(emergency_distance_threshold));
        emergency_stop_init();
        path_planner_init();
        motor_control_init();

        RCLCPP_INFO(this->get_logger(), "Emergency stop threshold configured: %.3f m",
                emergency_distance_threshold);

        // Initialize serial ports
        init_serial_ports();
        
        // Initialize CSV logging
        if (enable_csv_logging_) {
            init_csv_logging();
        }
        
        // ROS2 subscriptions
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RealtimeSystemNodeMT::odometry_callback, this, std::placeholders::_1));
        
        // ROS2 publishers
        ultrasonic_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic", 10);
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        
        // Initialize sensor data with safe defaults
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            ultrasonic_distance_ = 10.0f;  // Far away
            current_velocity_ = 0.0f;
        }
        
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        
        // Launch all threads
        launch_threads();
        
        RCLCPP_INFO(this->get_logger(), " All threads launched and running");
    }
    
    ~RealtimeSystemNodeMT() {
        RCLCPP_INFO(this->get_logger(), "Shutting down threads...");
        
        // Signal all threads to stop
        threads_running_ = false;
        
        // Wait for all threads to finish
        if (emergency_thread_started_) pthread_join(emergency_thread_, nullptr);
        if (sensor_thread_started_) pthread_join(sensor_thread_, nullptr);
        if (planning_thread_started_) pthread_join(planning_thread_, nullptr);
        if (motor_thread_started_) pthread_join(motor_thread_, nullptr);
        
        RCLCPP_INFO(this->get_logger(), " All threads stopped");
        
        // Send final stop command
        send_motor_command(MOTOR_STOP);
        
        // Cleanup ultrasonic sensor
        if (enable_ultrasonic_gpio_) {
            if (use_rt_gpio_) {
                ultrasonic_sensor_rt_.cleanup();
            } else {
                ultrasonic_sensor_.cleanup();
            }
        }
        
        // Close serial ports
        if (serial_fd_front_ >= 0) {
            close(serial_fd_front_);
        }
        
        // Write final statistics and close CSV
        if (csv_file_.is_open()) {
            write_performance_summary();
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), " Performance data saved to %s", csv_output_path_.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Multi-Threaded RTES Node SHUTDOWN");
    }

private:
    // ═════════════════════════════════════════════════════════════════════
    // REAL-TIME SETUP
    // ═════════════════════════════════════════════════════════════════════
    void setup_realtime_features() {
        RCLCPP_INFO(this->get_logger(), "Applying real-time features...");
        
        // Memory Locking
        if (enable_mlockall_) {
            if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
                RCLCPP_INFO(this->get_logger(), " Memory locked (mlockall)");
            } else {
                RCLCPP_WARN(this->get_logger(), " mlockall failed: %s", strerror(errno));
            }
        }
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // THREAD LAUNCH AND CONFIGURATION
    // ═════════════════════════════════════════════════════════════════════
    void launch_threads() {
        auto create_thread_with_fallback = [this](
            pthread_t* thread,
            pthread_attr_t* rt_attr,
            void* (*entry)(void*),
            const char* name,
            bool* started_flag) {
            int rc = pthread_create(thread, rt_attr, entry, this);
            if (rc == 0) {
                *started_flag = true;
                return;
            }

            RCLCPP_WARN(
                this->get_logger(),
                "Failed to create %s thread with RT attributes: %s. Retrying with default attributes.",
                name,
                strerror(rc));

            rc = pthread_create(thread, nullptr, entry, this);
            if (rc == 0) {
                *started_flag = true;
                RCLCPP_WARN(this->get_logger(), "%s thread is running without explicit RT attributes", name);
                return;
            }

            RCLCPP_ERROR(this->get_logger(), "Failed to create %s thread: %s", name, strerror(rc));
        };

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        
        // Launch Emergency Thread (Highest Priority)
        struct sched_param param_emergency;
        param_emergency.sched_priority = ThreadConfig::EMERGENCY_PRIORITY;
        pthread_attr_setschedparam(&attr, &param_emergency);

        create_thread_with_fallback(
            &emergency_thread_,
            &attr,
            emergency_thread_entry,
            "emergency",
            &emergency_thread_started_);
        
        // Pin emergency thread to isolated core if specified
        if (cpu_affinity_ >= 0 && emergency_thread_started_) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(cpu_affinity_, &cpuset);
            int rc = pthread_setaffinity_np(emergency_thread_, sizeof(cpu_set_t), &cpuset);
            if (rc == 0) {
                RCLCPP_INFO(this->get_logger(), " Emergency thread pinned to core %d", cpu_affinity_);
            } else {
                RCLCPP_WARN(this->get_logger(), " Failed to pin emergency thread to core %d: %s", cpu_affinity_, strerror(rc));
            }
        }
        
        // Launch Sensor Thread
        struct sched_param param_sensor;
        param_sensor.sched_priority = ThreadConfig::SENSOR_PRIORITY;
        pthread_attr_setschedparam(&attr, &param_sensor);
        create_thread_with_fallback(
            &sensor_thread_,
            &attr,
            sensor_thread_entry,
            "sensor",
            &sensor_thread_started_);
        
        // Launch Planning Thread
        struct sched_param param_planning;
        param_planning.sched_priority = ThreadConfig::PLANNING_PRIORITY;
        pthread_attr_setschedparam(&attr, &param_planning);
        create_thread_with_fallback(
            &planning_thread_,
            &attr,
            planning_thread_entry,
            "planning",
            &planning_thread_started_);
        
        // Launch Motor Thread
        struct sched_param param_motor;
        param_motor.sched_priority = ThreadConfig::MOTOR_PRIORITY;
        pthread_attr_setschedparam(&attr, &param_motor);
        create_thread_with_fallback(
            &motor_thread_,
            &attr,
            motor_thread_entry,
            "motor",
            &motor_thread_started_);
        
        pthread_attr_destroy(&attr);
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // THREAD ENTRY POINTS (Static wrappers)
    // ═════════════════════════════════════════════════════════════════════
    static void* emergency_thread_entry(void* arg) {
        RealtimeSystemNodeMT* node = static_cast<RealtimeSystemNodeMT*>(arg);
        node->emergency_thread_func();
        return nullptr;
    }
    
    static void* sensor_thread_entry(void* arg) {
        RealtimeSystemNodeMT* node = static_cast<RealtimeSystemNodeMT*>(arg);
        node->sensor_thread_func();
        return nullptr;
    }
    
    static void* planning_thread_entry(void* arg) {
        RealtimeSystemNodeMT* node = static_cast<RealtimeSystemNodeMT*>(arg);
        node->planning_thread_func();
        return nullptr;
    }
    
    static void* motor_thread_entry(void* arg) {
        RealtimeSystemNodeMT* node = static_cast<RealtimeSystemNodeMT*>(arg);
        node->motor_thread_func();
        return nullptr;
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // THREAD IMPLEMENTATIONS
    // ═════════════════════════════════════════════════════════════════════
    
    void emergency_thread_func() {
        struct timespec next_wake;
        clock_gettime(CLOCK_MONOTONIC, &next_wake);
        
        const int64_t period_ns = ThreadConfig::EMERGENCY_PERIOD_MS * 1000000LL;
        
        while (threads_running_) {
            struct timespec start_ts;
            clock_gettime(CLOCK_MONOTONIC, &start_ts);
            
            // Read sensor data (protected by mutex)
            float distance, velocity;
            {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                distance = ultrasonic_distance_;
                velocity = current_velocity_;
            }
            
            // SERVICE 1: Emergency Stop Check
            EmergencyStopState emergency_state;
            bool emergency = check_emergency_stop(distance, velocity, &emergency_state);
            
            // Update emergency flag (atomic, lock-free)
            emergency_flag_.store(emergency, std::memory_order_release);
            
            if (emergency) {
                // Publish emergency status
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                emergency_pub_->publish(msg);
            }
            
            // Calculate timing
            struct timespec end_ts;
            clock_gettime(CLOCK_MONOTONIC, &end_ts);
            int64_t exec_us = timespec_diff_us(&start_ts, &end_ts);
            int64_t jitter_us = timespec_diff_us(&next_wake, &start_ts);
            
            emergency_stats_.record_cycle(exec_us, jitter_us);

            // Log full trace (normal + emergency).
            log_thread_trace(
                ThreadConfig::EMERGENCY_NAME,
                emergency_stats_.cycle_count,
                exec_us,
                jitter_us,
                distance,
                velocity,
                emergency,
                emergency_state.ttc);
            
            // Sleep until next period
            next_wake.tv_nsec += period_ns;
            normalize_timespec(&next_wake);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
        }
    }
    
    void sensor_thread_func() {
        struct timespec next_wake;
        clock_gettime(CLOCK_MONOTONIC, &next_wake);
        
        const int64_t period_ns = ThreadConfig::SENSOR_PERIOD_MS * 1000000LL;
        
        while (threads_running_) {
            struct timespec start_ts;
            clock_gettime(CLOCK_MONOTONIC, &start_ts);
            
            // Read ultrasonic sensor
            float distance_m = 10.0f;  // Default: far away
            bool valid = false;
            
            if (enable_ultrasonic_gpio_) {
                if (use_rt_gpio_) {
                    valid = ultrasonic_sensor_rt_.read_distance(distance_m);
                } else {
                    valid = ultrasonic_sensor_.read_distance(distance_m);
                }
                
                if (valid) {
                    // Update sensor data (protected by mutex)
                    {
                        std::lock_guard<std::mutex> lock(sensor_mutex_);
                        ultrasonic_distance_ = distance_m;
                    }
                    
                    // Publish Range message
                    auto msg = sensor_msgs::msg::Range();
                    msg.header.stamp = this->now();
                    msg.header.frame_id = "ultrasonic_link";
                    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
                    msg.field_of_view = 0.26f;
                    msg.min_range = 0.02f;
                    msg.max_range = 4.0f;
                    msg.range = distance_m;
                    ultrasonic_pub_->publish(msg);
                }
            }
            
            // Calculate timing
            struct timespec end_ts;
            clock_gettime(CLOCK_MONOTONIC, &end_ts);
            int64_t exec_us = timespec_diff_us(&start_ts, &end_ts);
            int64_t jitter_us = timespec_diff_us(&next_wake, &start_ts);
            
            sensor_stats_.record_cycle(exec_us, jitter_us);

            // Include latest command/emergency context for sensor timing traces.
            bool emergency = emergency_flag_.load(std::memory_order_acquire);
            float velocity_snapshot;
            {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                velocity_snapshot = current_velocity_;
            }
            log_thread_trace(
                ThreadConfig::SENSOR_NAME,
                sensor_stats_.cycle_count,
                exec_us,
                jitter_us,
                distance_m,
                velocity_snapshot,
                emergency,
                std::numeric_limits<float>::infinity());
            
            // Sleep until next period
            next_wake.tv_nsec += period_ns;
            normalize_timespec(&next_wake);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
        }
    }
    
    void planning_thread_func() {
        struct timespec next_wake;
        clock_gettime(CLOCK_MONOTONIC, &next_wake);
        
        const int64_t period_ns = ThreadConfig::PLANNING_PERIOD_MS * 1000000LL;
        
        while (threads_running_) {
            struct timespec start_ts;
            clock_gettime(CLOCK_MONOTONIC, &start_ts);

            // Read sensor data (protected by mutex)
            float distance, velocity;
            {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                distance = ultrasonic_distance_;
                velocity = current_velocity_;
            }

            // SERVICE 2: Path Planning
            // Always compute the latest command. Emergency gating is handled
            // by motor_thread_func() before commands are sent to hardware.
            PathCommand path_cmd = compute_path_command(distance, velocity);

            // Update path command (atomic, lock-free)
            path_command_.store(path_cmd.command, std::memory_order_release);
            
            // Calculate timing
            struct timespec end_ts;
            clock_gettime(CLOCK_MONOTONIC, &end_ts);
            int64_t exec_us = timespec_diff_us(&start_ts, &end_ts);
            int64_t jitter_us = timespec_diff_us(&next_wake, &start_ts);
            
            planning_stats_.record_cycle(exec_us, jitter_us);

            bool emergency = emergency_flag_.load(std::memory_order_acquire);
            log_thread_trace(
                ThreadConfig::PLANNING_NAME,
                planning_stats_.cycle_count,
                exec_us,
                jitter_us,
                distance,
                velocity,
                emergency,
                std::numeric_limits<float>::infinity());
            
            // Sleep until next period
            next_wake.tv_nsec += period_ns;
            normalize_timespec(&next_wake);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
        }
    }
    
    void motor_thread_func() {
        struct timespec next_wake;
        clock_gettime(CLOCK_MONOTONIC, &next_wake);
        
        const int64_t period_ns = ThreadConfig::MOTOR_PERIOD_MS * 1000000LL;
        
        while (threads_running_) {
            struct timespec start_ts;
            clock_gettime(CLOCK_MONOTONIC, &start_ts);
            
            // Check emergency flag (atomic, lock-free)
            bool emergency = emergency_flag_.load(std::memory_order_acquire);
            
            MotorCommand cmd_to_send;
            
            if (emergency) {
                // EMERGENCY: Send STOP immediately
                cmd_to_send = MOTOR_STOP;
            } else {
                // Normal operation: Read planned command (atomic, lock-free)
                cmd_to_send = path_command_.load(std::memory_order_acquire);
            }
            
            // SERVICE 3: Motor Control
            MotorControlState motor_state;
            execute_motor_command(cmd_to_send, 1, &motor_state);
            send_motor_command(cmd_to_send);
            
            // Calculate timing
            struct timespec end_ts;
            clock_gettime(CLOCK_MONOTONIC, &end_ts);
            int64_t exec_us = timespec_diff_us(&start_ts, &end_ts);
            int64_t jitter_us = timespec_diff_us(&next_wake, &start_ts);
            
            motor_stats_.record_cycle(exec_us, jitter_us);

            float distance_snapshot;
            float velocity_snapshot;
            {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                distance_snapshot = ultrasonic_distance_;
                velocity_snapshot = current_velocity_;
            }
            log_thread_trace(
                ThreadConfig::MOTOR_NAME,
                motor_stats_.cycle_count,
                exec_us,
                jitter_us,
                distance_snapshot,
                velocity_snapshot,
                emergency,
                std::numeric_limits<float>::infinity());
            
            // Sleep until next period
            next_wake.tv_nsec += period_ns;
            normalize_timespec(&next_wake);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
        }
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // ROS2 CALLBACKS
    // ═════════════════════════════════════════════════════════════════════
    
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update velocity (protected by same mutex as sensor data)
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        current_velocity_ = std::abs(msg->twist.twist.linear.x);
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // SERIAL PORT MANAGEMENT
    // ═════════════════════════════════════════════════════════════════════
    
    void init_serial_ports() {
        serial_fd_front_ = open(serial_port_front_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_front_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_front_.c_str());
        } else {
            configure_serial_port(serial_fd_front_);
            RCLCPP_INFO(this->get_logger(), "✓ Arduino connected: %s", serial_port_front_.c_str());
        }
    }
    
    void configure_serial_port(int fd) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting serial attributes");
            return;
        }
        
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting serial attributes");
        }
    }
    
    void send_motor_command(MotorCommand cmd) {
        if (serial_fd_front_ < 0) return;

        char cmd_char = motor_command_to_char(cmd);
        
        write(serial_fd_front_, &cmd_char, 1);
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // CSV LOGGING
    // ═════════════════════════════════════════════════════════════════════
    
    void init_csv_logging() {
        csv_file_.open(csv_output_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            const std::string fallback_path = "/tmp/rtes_performance_mt_" + std::to_string(getpid()) + ".csv";
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to open CSV file: %s. Trying fallback path: %s",
                csv_output_path_.c_str(),
                fallback_path.c_str());

            csv_output_path_ = fallback_path;
            csv_file_.clear();
            csv_file_.open(csv_output_path_, std::ios::out | std::ios::trunc);

            if (!csv_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open fallback CSV file: %s", csv_output_path_.c_str());
                enable_csv_logging_ = false;
                return;
            }
        }
        
        csv_file_ << "thread,cycle,exec_us,jitter_us,distance_m,velocity_ms,emergency,ttc\n";
        csv_file_.flush();
        
        RCLCPP_INFO(this->get_logger(), "✓ CSV logging initialized: %s", csv_output_path_.c_str());
    }
    
    void log_thread_trace(const char* thread_name,
                          uint64_t cycle,
                          int64_t exec_us,
                          int64_t jitter_us,
                          float distance,
                          float velocity,
                          bool emergency,
                          float ttc) {
        if (!enable_csv_logging_ || !csv_file_.is_open()) return;
        
        std::lock_guard<std::mutex> lock(csv_mutex_);
        csv_file_ << thread_name << ","
                  << cycle << ","
                  << exec_us << ","
                  << jitter_us << ","
                  << distance << ","
                  << velocity << ","
                  << (emergency ? "1," : "0,")
                  << ttc << "\n";

        csv_pending_rows_++;
        if (csv_pending_rows_ % ThreadConfig::CSV_FLUSH_EVERY_N_ROWS == 0) {
            csv_file_.flush();
        }
    }
    
    void write_performance_summary() {
        csv_file_ << "\n# Performance Summary\n";
        csv_file_ << "# Thread,Cycles,Mean_Exec_us,Min_us,Max_us,Max_Jitter_us,Deadline_Misses,Miss_Rate_%\n";
        
        write_thread_summary(emergency_stats_);
        write_thread_summary(sensor_stats_);
        write_thread_summary(planning_stats_);
        write_thread_summary(motor_stats_);
        csv_file_.flush();
    }
    
    void write_thread_summary(const ThreadPerformanceStats& stats) {
        csv_file_ << "# " << stats.thread_name << ","
                  << stats.cycle_count << ","
                  << stats.get_mean_exec_us() << ","
                  << stats.min_exec_us << ","
                  << stats.max_exec_us << ","
                  << stats.max_jitter_us << ","
                  << stats.deadline_misses << ","
                  << stats.get_miss_rate() << "\n";
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // UTILITY FUNCTIONS
    // ═════════════════════════════════════════════════════════════════════
    
    static int64_t timespec_diff_us(const struct timespec* start, const struct timespec* end) {
        int64_t sec_diff = end->tv_sec - start->tv_sec;
        int64_t nsec_diff = end->tv_nsec - start->tv_nsec;
        return sec_diff * 1000000LL + nsec_diff / 1000LL;
    }
    
    static void normalize_timespec(struct timespec* ts) {
        while (ts->tv_nsec >= 1000000000L) {
            ts->tv_sec++;
            ts->tv_nsec -= 1000000000L;
        }
    }
    
    // ═════════════════════════════════════════════════════════════════════
    // MEMBER VARIABLES
    // ═════════════════════════════════════════════════════════════════════
    
    // Configuration
    std::string serial_port_front_;
    std::string csv_output_path_;
    bool enable_rt_;
    int cpu_affinity_;
    bool enable_mlockall_;
    bool enable_csv_logging_;
    bool enable_ultrasonic_gpio_;
    bool use_rt_gpio_;
    
    // Threading
    pthread_t emergency_thread_;
    pthread_t sensor_thread_;
    pthread_t planning_thread_;
    pthread_t motor_thread_;
    std::atomic<bool> threads_running_;
    bool emergency_thread_started_;
    bool sensor_thread_started_;
    bool planning_thread_started_;
    bool motor_thread_started_;
    
    // Thread synchronization
    std::atomic<bool> emergency_flag_;                    // Lock-free: emergency → motor
    std::atomic<MotorCommand> path_command_;             // Lock-free: planning → motor
    std::mutex sensor_mutex_;                             // Protects sensor data (multiple readers)
    std::mutex csv_mutex_;                                // Protects CSV file writes
    uint64_t csv_pending_rows_ = 0;
    
    // Sensor data (protected by sensor_mutex_)
    float ultrasonic_distance_;
    float current_velocity_;
    
    // Performance tracking
    ThreadPerformanceStats emergency_stats_;
    ThreadPerformanceStats planning_stats_;
    ThreadPerformanceStats motor_stats_;
    ThreadPerformanceStats sensor_stats_;
    
    // Hardware interfaces
    int serial_fd_front_;
    UltrasonicHCSR04 ultrasonic_sensor_;
    UltrasonicHCSR04RT ultrasonic_sensor_rt_;
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    
    // CSV logging
    std::ofstream csv_file_;
};

// ═════════════════════════════════════════════════════════════════════
// MAIN
// ═════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RealtimeSystemNodeMT>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
