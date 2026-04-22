/**
 * @file safety_types.h
 * @brief Common type definitions for TrashBot RTES safety system
 */

#ifndef SAFETY_TYPES_H
#define SAFETY_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Motor command enumeration
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT
} MotorCommand;

// Emergency stop state structure
typedef struct {
    float distance_m;              // Current distance to obstacle
    float velocity_ms;             // Current velocity (m/s)
    float ttc;                     // Time-to-collision (seconds)
    float safe_stopping_distance;  // Required stopping distance
    bool emergency_triggered;      // Emergency stop active
    uint64_t timestamp_us;         // Timestamp in microseconds
} EmergencyStopState;

// Path planning decision structure
typedef struct {
    MotorCommand command;          // Recommended motor command
    float confidence;              // Confidence score (0.0-1.0)
    float clearance_left;          // Left clearance distance
    float clearance_right;         // Right clearance distance
    float clearance_front;         // Front clearance distance
} PathCommand;

// Motor control state structure
typedef struct {
    MotorCommand current_command;  // Currently executing command
    uint8_t priority;              // Command priority (1=highest)
    float execution_time_ms;       // Time spent executing (ms)
    uint32_t command_count;        // Total commands executed
} MotorControlState;

// System constants
#define DECELERATION_MS2 1.0f      // Assumed deceleration (m/s²)
#define SAFETY_MARGIN_M 0.5f       // Safety margin buffer (meters)
#define MIN_OBSTACLE_DISTANCE_M 0.3f  // Minimum safe distance
#define MAX_VELOCITY_MS 1.0f       // Maximum velocity

#endif // SAFETY_TYPES_H
