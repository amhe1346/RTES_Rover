/**
 * @file ultrasonic_hcsr04.cpp
 * @brief Real-time HC-SR04 ultrasonic sensor driver using direct GPIO access
 * 
 * Uses Linux sysfs GPIO interface for direct hardware control
 * No Python, no libraries - just direct file I/O for minimal latency
 */

#include "ultrasonic_hcsr04.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <sys/time.h>

UltrasonicHCSR04::UltrasonicHCSR04(int trigger_pin, int echo_pin)
    : trigger_pin_(trigger_pin)
    , echo_pin_(echo_pin)
    , trigger_fd_(-1)
    , echo_fd_(-1)
    , initialized_(false)
{
}

UltrasonicHCSR04::~UltrasonicHCSR04() {
    cleanup();
}

bool UltrasonicHCSR04::initialize() {
    // Export GPIO pins if not already exported
    if (!export_gpio(trigger_pin_) || !export_gpio(echo_pin_)) {
        return false;
    }
    
    // Set trigger as output
    if (!set_gpio_direction(trigger_pin_, "out")) {
        return false;
    }
    
    // Set echo as input
    if (!set_gpio_direction(echo_pin_, "in")) {
        return false;
    }
    
    // Open value files for fast access
    char path[64];
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", trigger_pin_);
    trigger_fd_ = open(path, O_WRONLY);
    if (trigger_fd_ < 0) {
        fprintf(stderr, "Failed to open trigger value file\n");
        return false;
    }
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", echo_pin_);
    echo_fd_ = open(path, O_RDONLY);
    if (echo_fd_ < 0) {
        fprintf(stderr, "Failed to open echo value file\n");
        close(trigger_fd_);
        trigger_fd_ = -1;
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool UltrasonicHCSR04::read_distance(float &distance_m) {
    if (!initialized_) {
        return false;
    }
    
    // Send 10μs pulse on trigger
    write(trigger_fd_, "0", 1);
    usleep(2);  // 2μs LOW
    
    write(trigger_fd_, "1", 1);
    usleep(10); // 10μs HIGH
    
    write(trigger_fd_, "0", 1);
    
    // Measure echo pulse width
    struct timeval start, end;
    struct timeval timeout_start;
    gettimeofday(&timeout_start, NULL);
    
    char value[3];
    long timeout_us = 100000;  // 100ms timeout
    
    // Wait for echo to go HIGH
    while (true) {
        lseek(echo_fd_, 0, SEEK_SET);
        if (read(echo_fd_, value, 2) < 1) {
            return false;
        }
        value[2] = '\0';
        
        if (value[0] == '1') {
            gettimeofday(&start, NULL);
            break;
        }
        
        // Check timeout
        struct timeval now;
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - timeout_start.tv_sec) * 1000000 +
                      (now.tv_usec - timeout_start.tv_usec);
        if (elapsed > timeout_us) {
            return false;  // Timeout
        }
    }
    
    // Wait for echo to go LOW
    gettimeofday(&timeout_start, NULL);
    while (true) {
        lseek(echo_fd_, 0, SEEK_SET);
        if (read(echo_fd_, value, 2) < 1) {
            return false;
        }
        value[2] = '\0';
        
        if (value[0] == '0') {
            gettimeofday(&end, NULL);
            break;
        }
        
        // Check timeout
        struct timeval now;
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - timeout_start.tv_sec) * 1000000 +
                      (now.tv_usec - timeout_start.tv_usec);
        if (elapsed > timeout_us) {
            return false;  // Timeout
        }
    }
    
    // Calculate pulse duration in microseconds
    long pulse_us = (end.tv_sec - start.tv_sec) * 1000000 +
                    (end.tv_usec - start.tv_usec);
    
    // Speed of sound: 343 m/s = 0.0343 cm/μs
    // Distance = (pulse_duration * 0.0343) / 2
    // Simplified: distance_cm = pulse_us / 58.0
    float distance_cm = pulse_us / 58.0f;
    distance_m = distance_cm / 100.0f;
    
    // Validate range (HC-SR04: 2cm to 400cm)
    if (distance_m < 0.02f || distance_m > 4.0f) {
        return false;
    }
    
    return true;
}

void UltrasonicHCSR04::cleanup() {
    if (trigger_fd_ >= 0) {
        close(trigger_fd_);
        trigger_fd_ = -1;
    }
    
    if (echo_fd_ >= 0) {
        close(echo_fd_);
        echo_fd_ = -1;
    }
    
    // Note: We don't unexport GPIOs to allow reuse
    initialized_ = false;
}

bool UltrasonicHCSR04::export_gpio(int pin) {
    // Check if already exported
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin);
    if (access(path, F_OK) == 0) {
        return true;  // Already exported
    }
    
    // Export the GPIO
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open GPIO export file\n");
        return false;
    }
    
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", pin);
    ssize_t bytes_written = write(fd, buf, strlen(buf));
    close(fd);
    
    if (bytes_written <= 0) {
        fprintf(stderr, "Failed to export GPIO %d\n", pin);
        return false;
    }
    
    // Wait for GPIO to be ready
    usleep(100000);  // 100ms
    
    return true;
}

bool UltrasonicHCSR04::set_gpio_direction(int pin, const char* direction) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open direction file for GPIO %d\n", pin);
        return false;
    }
    
    ssize_t bytes_written = write(fd, direction, strlen(direction));
    close(fd);
    
    if (bytes_written <= 0) {
        fprintf(stderr, "Failed to set direction for GPIO %d\n", pin);
        return false;
    }
    
    return true;
}
