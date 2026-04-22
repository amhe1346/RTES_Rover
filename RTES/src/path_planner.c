/**
 * @file path_planner.c
 * @brief Service 2: Path Planning Implementation
 * 
 * Obstacle avoidance logic and navigation decision making.
 */

#include "path_planner.h"
#include <string.h>

// Planning parameters
#define OBSTACLE_THRESHOLD_M 0.8f   // Distance to start avoiding (meters)
#define CLEAR_DISTANCE_M 1.5f       // Distance considered clear
#define TURN_DURATION_MS 800        // Typical turn duration

static PathCommand g_last_command;
static bool g_initialized = false;

void path_planner_init(void) {
    memset(&g_last_command, 0, sizeof(PathCommand));
    g_last_command.command = MOTOR_FORWARD;
    g_last_command.confidence = 1.0f;
    g_initialized = true;
}

MotorCommand determine_avoidance_direction(float front_dist, 
                                            float left_dist, 
                                            float right_dist) {
    // If front is clear, go forward
    if (front_dist > CLEAR_DISTANCE_M) {
        return MOTOR_FORWARD;
    }
    
    // If front is blocked, determine turn direction
    if (front_dist < OBSTACLE_THRESHOLD_M) {
        // Compare left and right clearances
        if (left_dist > right_dist) {
            return MOTOR_LEFT;
        } else if (right_dist > left_dist) {
            return MOTOR_RIGHT;
        } else {
            // Equal clearance or no side sensors, default to right
            return MOTOR_RIGHT;
        }
    }
    
    // Intermediate distance - slow approach
    return MOTOR_FORWARD;
}

PathCommand compute_path_command(float front_distance_m, 
                                 float current_velocity_ms __attribute__((unused))) {
    
    if (!g_initialized) {
        path_planner_init();
    }
    
    PathCommand cmd;
    memset(&cmd, 0, sizeof(PathCommand));
    
    // Store clearances
    cmd.clearance_front = front_distance_m;
    cmd.clearance_left = 0.0f;   // TODO: Add side sensors if available
    cmd.clearance_right = 0.0f;  // TODO: Add side sensors if available
    
    // Decision logic based on front distance
    if (front_distance_m < MIN_OBSTACLE_DISTANCE_M) {
        // Very close obstacle - STOP
        cmd.command = MOTOR_STOP;
        cmd.confidence = 1.0f;
    } 
    else if (front_distance_m < OBSTACLE_THRESHOLD_M) {
        // Obstacle detected - turn to avoid
        cmd.command = determine_avoidance_direction(
            front_distance_m, 
            cmd.clearance_left, 
            cmd.clearance_right);
        cmd.confidence = 0.8f;
    } 
    else if (front_distance_m < CLEAR_DISTANCE_M) {
        // Caution zone - continue forward slowly
        cmd.command = MOTOR_FORWARD;
        cmd.confidence = 0.6f;
    } 
    else {
        // Clear path - full speed forward
        cmd.command = MOTOR_FORWARD;
        cmd.confidence = 1.0f;
    }
    
    // If currently moving backwards and obstacle cleared, switch to forward
    if (g_last_command.command == MOTOR_BACKWARD && 
        front_distance_m > CLEAR_DISTANCE_M) {
        cmd.command = MOTOR_FORWARD;
    }
    
    // Store for next iteration
    memcpy(&g_last_command, &cmd, sizeof(PathCommand));
    
    return cmd;
}
