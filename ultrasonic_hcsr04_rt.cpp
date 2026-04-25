/**
 * @file ultrasonic_hcsr04_rt.cpp
 * @brief TRUE HARD REAL-TIME HC-SR04 ultrasonic sensor driver implementation
 * 
 * Memory-mapped GPIO with CLOCK_MONOTONIC for deterministic <100μs latency
 */

#include "ultrasonic_hcsr04_rt.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cstring>
#include <cstdio>
#include <time.h>
#include <errno.h>

UltrasonicHCSR04RT::UltrasonicHCSR04RT(int trigger_pin, int echo_pin)
    : trigger_pin_(trigger_pin)
    , echo_pin_(echo_pin)
    , gpio_map_(nullptr)
    , mem_fd_(-1)
    , trigger_mask_(0)
    , echo_mask_(0)
    , initialized_(false)
{
}

UltrasonicHCSR04RT::~UltrasonicHCSR04RT() {
    cleanup();
}

bool UltrasonicHCSR04RT::setup_gpio_memory() {
    // Open /dev/gpiomem (doesn't require root, just gpio group membership)
    mem_fd_ = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (mem_fd_ < 0) {
        fprintf(stderr, "Failed to open /dev/gpiomem: %s\n", strerror(errno));
        fprintf(stderr, "Make sure user is in 'gpio' group: sudo usermod -a -G gpio $USER\n");
        return false;
    }
    
    // Memory map the GPIO registers
    void* map = mmap(
        nullptr,                    // Let kernel choose address
        GPIO_MAP_SIZE,              // 4KB page
        PROT_READ | PROT_WRITE,     // Read/write access
        MAP_SHARED,                 // Shared with other processes
        mem_fd_,                    // File descriptor
        0                           // Offset (gpiomem already at GPIO base)
    );
    
    if (map == MAP_FAILED) {
        fprintf(stderr, "Failed to mmap GPIO: %s\n", strerror(errno));
        close(mem_fd_);
        mem_fd_ = -1;
        return false;
    }
    
    gpio_map_ = static_cast<volatile uint32_t*>(map);
    return true;
}

void UltrasonicHCSR04RT::set_gpio_output(int pin) {
    // Each GPFSEL register controls 10 pins (3 bits per pin)
    int reg_index = pin / 10;
    int bit_offset = (pin % 10) * 3;
    
    // Read-modify-write: clear 3 bits then set to 001 (output)
    uint32_t reg = gpio_map_[GPFSEL_OFFSET/4 + reg_index];
    reg &= ~(0x7 << bit_offset);  // Clear 3 bits
    reg |= (0x1 << bit_offset);   // Set to output (001)
    gpio_map_[GPFSEL_OFFSET/4 + reg_index] = reg;
}

void UltrasonicHCSR04RT::set_gpio_input(int pin) {
    // Each GPFSEL register controls 10 pins (3 bits per pin)
    int reg_index = pin / 10;
    int bit_offset = (pin % 10) * 3;
    
    // Read-modify-write: clear 3 bits (000 = input)
    uint32_t reg = gpio_map_[GPFSEL_OFFSET/4 + reg_index];
    reg &= ~(0x7 << bit_offset);  // Clear 3 bits = input
    gpio_map_[GPFSEL_OFFSET/4 + reg_index] = reg;
}

inline uint64_t UltrasonicHCSR04RT::get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

inline void UltrasonicHCSR04RT::delay_ns(uint32_t ns) {
    struct timespec req;
    req.tv_sec = 0;
    req.tv_nsec = ns;
    nanosleep(&req, nullptr);
}

bool UltrasonicHCSR04RT::initialize() {
    if (initialized_) {
        return true;
    }
    
    // Setup memory-mapped GPIO
    if (!setup_gpio_memory()) {
        return false;
    }
    
    // Calculate pin masks for fast bitwise operations
    trigger_mask_ = 1 << trigger_pin_;
    echo_mask_ = 1 << echo_pin_;
    
    // Configure GPIO pins
    set_gpio_output(trigger_pin_);
    set_gpio_input(echo_pin_);
    
    // Ensure trigger starts LOW
    gpio_clear(trigger_mask_);
    delay_ns(100000);  // 100μs settle time
    
    initialized_ = true;
    fprintf(stderr, "✓ Memory-mapped GPIO initialized (trigger: GPIO%d, echo: GPIO%d)\n", 
            trigger_pin_, echo_pin_);
    return true;
}

bool UltrasonicHCSR04RT::read_distance(float &distance_m) {
    if (!initialized_) {
        return false;
    }
    
    // Send 10μs pulse on trigger
    gpio_clear(trigger_mask_);
    delay_ns(2000);  // 2μs LOW
    
    gpio_set(trigger_mask_);
    delay_ns(10000); // 10μs HIGH
    
    gpio_clear(trigger_mask_);
    
    // Measure echo pulse width with monotonic clock
    uint64_t timeout_start = get_time_ns();
    uint64_t start_time, end_time;
    
    // Wait for echo to go HIGH
    while (!(gpio_read() & echo_mask_)) {
        uint64_t now = get_time_ns();
        if ((now - timeout_start) > TIMEOUT_NS) {
            return false;  // Timeout
        }
    }
    start_time = get_time_ns();
    
    // Wait for echo to go LOW
    timeout_start = start_time;
    while (gpio_read() & echo_mask_) {
        uint64_t now = get_time_ns();
        if ((now - timeout_start) > TIMEOUT_NS) {
            return false;  // Timeout
        }
    }
    end_time = get_time_ns();
    
    // Calculate pulse duration in microseconds
    uint64_t pulse_ns = end_time - start_time;
    float pulse_us = pulse_ns / 1000.0f;
    
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

void UltrasonicHCSR04RT::cleanup() {
    if (gpio_map_ != nullptr) {
        // Set trigger LOW before cleanup
        if (initialized_) {
            gpio_clear(trigger_mask_);
        }
        
        munmap((void*)gpio_map_, GPIO_MAP_SIZE);
        gpio_map_ = nullptr;
    }
    
    if (mem_fd_ >= 0) {
        close(mem_fd_);
        mem_fd_ = -1;
    }
    
    initialized_ = false;
}
