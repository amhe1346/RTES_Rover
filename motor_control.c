/**
 * @file motor_control.c
 * @brief Service 3: Motor Control & Logging Implementation
 * 
 * Priority-based command execution and system telemetry.
 */

#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// Module state
static MotorControlState g_motor_state;
static bool g_initialized = false;

// Logging buffer (simple circular buffer)
#define LOG_BUFFER_SIZE 1000
static struct {
    float distance;
    float velocity;
    MotorCommand command;
    uint64_t timestamp_us;
} g_log_buffer[LOG_BUFFER_SIZE];
static size_t g_log_index = 0;

void motor_control_init(void) {
    memset(&g_motor_state, 0, sizeof(MotorControlState));
    g_motor_state.current_command = MOTOR_STOP;
    g_motor_state.priority = 255;  // Lowest priority initially
    g_initialized = true;
}

char motor_command_to_char(MotorCommand command) {
    switch (command) {
        case MOTOR_FORWARD:  return 'F';
        case MOTOR_BACKWARD: return 'B';
        case MOTOR_LEFT:     return 'L';
        case MOTOR_RIGHT:    return 'R';
        case MOTOR_STOP:     return 'S';
        default:             return 'S';
    }
}

bool execute_motor_command(MotorCommand command, uint8_t priority,
                           MotorControlState *state) {
    
    if (!g_initialized) {
        motor_control_init();
    }
    
    // Priority check: only execute if equal or higher priority
    // (lower number = higher priority)
    if (priority > g_motor_state.priority && 
        g_motor_state.current_command != MOTOR_STOP) {
        // Command preempted by higher priority
        if (state != NULL) {
            memcpy(state, &g_motor_state, sizeof(MotorControlState));
        }
        return false;
    }
    
    // Execute command
    g_motor_state.current_command = command;
    g_motor_state.priority = priority;
    g_motor_state.command_count++;
    
    // In real implementation, this would write to serial port
    // For now, just log the command
    // char cmd_char = motor_command_to_char(command);
    // fprintf(stderr, "[MOTOR] Command: %c (priority: %d)\n", cmd_char, priority);
    
    if (state != NULL) {
        memcpy(state, &g_motor_state, sizeof(MotorControlState));
    }
    
    return true;
}

void get_motor_state(MotorControlState *state) {
    if (!g_initialized) {
        motor_control_init();
    }
    
    if (state != NULL) {
        memcpy(state, &g_motor_state, sizeof(MotorControlState));
    }
}

void log_telemetry(float distance_m, float velocity_ms, MotorCommand command) {
    if (!g_initialized) {
        motor_control_init();
    }
    
    // Get timestamp (microseconds)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    
    // Store in circular buffer
    g_log_buffer[g_log_index].distance = distance_m;
    g_log_buffer[g_log_index].velocity = velocity_ms;
    g_log_buffer[g_log_index].command = command;
    g_log_buffer[g_log_index].timestamp_us = timestamp_us;
    
    g_log_index = (g_log_index + 1) % LOG_BUFFER_SIZE;
    
    // Optional: Print to console for debugging
    // fprintf(stderr, "[LOG] dist: %.2fm, vel: %.2fm/s, cmd: %c\n", 
    //         distance_m, velocity_ms, motor_command_to_char(command));
}
