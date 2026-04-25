/**
 * @file path_planner.c
 * @brief Service 2: Path Planning Implementation
 * 
 * Obstacle avoidance logic and navigation decision making.
 */

#include "path_planner.h"
#include <string.h>

// Planning parameters
#define OBSTACLE_THRESHOLD_M 0.68f   // Distance to start avoiding (meters)
#define CLEAR_DISTANCE_M 1.5f       // Distance considered clear
#define TURN_DURATION_MS 800        // Typical turn duration
#define PATH_PLANNER_STOP_DISTANCE_M 0.5f // planner-specific hard stop distance

// Robot dimensions and opening detection
#define ROBOT_WIDTH_M 0.23f         // Robot width: 9 inches
#define MIN_OPENING_WIDTH_M 0.35f   // Minimum safe opening (with margin)
#define TURN_RATE_DEG_S 60.0f       // Estimated turn rate (degrees/second)
#define CONTROL_PERIOD_S 0.01f      // 100Hz control loop = 0.01s period

static PathCommand g_last_command;
static bool g_initialized = false;

// Opening width tracking
static uint32_t clear_space_cycles = 0;  // Consecutive cycles with clear space while turning
static float clear_space_total_dist = 0.0f;  // Sum of distances during clear cycles

void path_planner_init(void) {
    memset(&g_last_command, 0, sizeof(PathCommand));
    g_last_command.command = MOTOR_FORWARD;
    g_last_command.confidence = 1.0f;
    clear_space_cycles = 0;
    clear_space_total_dist = 0.0f;
    g_initialized = true;
}

/**
 * Estimate opening width based on turn duration and distance
 * 
 * Formula: width ≈ (turn_angle × π/180) × distance
 * where turn_angle = turn_rate × time
 */
static float estimate_opening_width(void) {
    if (clear_space_cycles == 0) {
        return 0.0f;
    }
    
    // Calculate average distance during clear space
    float avg_distance = clear_space_total_dist / (float)clear_space_cycles;
    
    // Calculate total turn time
    float turn_time_s = (float)clear_space_cycles * CONTROL_PERIOD_S;
    
    // Calculate turn angle in radians
    float turn_angle_deg = TURN_RATE_DEG_S * turn_time_s;
    float turn_angle_rad = turn_angle_deg * 3.14159f / 180.0f;
    
    // Arc length approximates opening width
    float estimated_width = turn_angle_rad * avg_distance;
    
    return estimated_width;
}

MotorCommand determine_avoidance_direction(float front_dist) {
    // If front is clear, go forward
    if (front_dist > CLEAR_DISTANCE_M) {
        return MOTOR_FORWARD;
    }
    
    // If front is blocked, default to right turn (no side sensors)
    if (front_dist < OBSTACLE_THRESHOLD_M) {
        return MOTOR_RIGHT;
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
   
    
    // Decision logic based on front distance
    if (front_distance_m < PATH_PLANNER_STOP_DISTANCE_M) {
        // Very close obstacle - STOP
        cmd.command = MOTOR_STOP;
        cmd.confidence = 1.0f;
        
        // Reset clear space tracking
        clear_space_cycles = 0;
        clear_space_total_dist = 0.0f;
    } 
    else if (front_distance_m < OBSTACLE_THRESHOLD_M) {
        // Obstacle detected - turn to avoid
        cmd.command = MOTOR_RIGHT;
        cmd.confidence = 0.8f;
        
        // Reset clear space tracking (obstacle blocking)
        clear_space_cycles = 0;
        clear_space_total_dist = 0.0f;
    } 
    else if (front_distance_m < CLEAR_DISTANCE_M) {
        // Space is clearing - check if we're currently turning
        bool currently_turning = (g_last_command.command == MOTOR_RIGHT || 
                                  g_last_command.command == MOTOR_LEFT);
        
        if (currently_turning) {
            // Track continuous clear space while turning
            clear_space_cycles++;
            clear_space_total_dist += front_distance_m;
            
            // Estimate opening width
            float opening_width = estimate_opening_width();
            
            // Only switch to forward if opening is wide enough
            if (opening_width >= MIN_OPENING_WIDTH_M) {
                cmd.command = MOTOR_FORWARD;
                cmd.confidence = 0.7f;
                
                // Reset tracking for next obstacle
                clear_space_cycles = 0;
                clear_space_total_dist = 0.0f;
            } else {
                // Keep turning - opening not wide enough yet
                cmd.command = MOTOR_RIGHT;
                cmd.confidence = 0.6f;
            }
        } else {
            // Not turning, just proceed forward in caution zone
            cmd.command = MOTOR_FORWARD;
            cmd.confidence = 0.6f;
        }
    } 
    else {
        // Clear path - full speed forward
        cmd.command = MOTOR_FORWARD;
        cmd.confidence = 1.0f;
        
        // Reset clear space tracking
        clear_space_cycles = 0;
        clear_space_total_dist = 0.0f;
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
