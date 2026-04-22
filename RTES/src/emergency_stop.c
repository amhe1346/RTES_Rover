/**
 * @file emergency_stop.c
 * @brief Service 1: Emergency Stop Implementation
 * 
 * Hard real-time collision detection and emergency stop logic.
 * All functions designed for <1ms execution time.
 */

#include "emergency_stop.h"
#include <math.h>
#include <string.h>

// Module state
static EmergencyStopState g_state;
static bool g_initialized = false;

void emergency_stop_init(void) {
    memset(&g_state, 0, sizeof(EmergencyStopState));
    g_state.distance_m = INFINITY;
    g_state.velocity_ms = 0.0f;
    g_state.ttc = INFINITY;
    g_initialized = true;
}

float calculate_ttc(float distance_m, float velocity_ms) {
    // Handle edge cases
    if (distance_m <= 0.0f) {
        return 0.0f;  // Already at collision
    }
    
    if (velocity_ms <= 0.01f) {
        return INFINITY;  // Not moving towards obstacle
    }
    
    // TTC = distance / velocity
    return distance_m / velocity_ms;
}

float calculate_stopping_distance(float velocity_ms, float deceleration_ms2) {
    // Handle edge cases
    if (velocity_ms <= 0.0f) {
        return 0.0f;
    }
    
    if (deceleration_ms2 <= 0.0f) {
        return INFINITY;  // Cannot stop
    }
    
    // Stopping distance: d = v² / (2 * a)
    return (velocity_ms * velocity_ms) / (2.0f * deceleration_ms2);
}

bool check_emergency_stop(float distance_m, float velocity_ms, 
                          EmergencyStopState *state) {
    
    if (!g_initialized) {
        emergency_stop_init();
    }
    
    // Update internal state
    g_state.distance_m = distance_m;
    g_state.velocity_ms = velocity_ms;
    
    // Calculate TTC
    g_state.ttc = calculate_ttc(distance_m, velocity_ms);
    
    // Calculate required stopping distance
    g_state.safe_stopping_distance = calculate_stopping_distance(
        velocity_ms, DECELERATION_MS2);
    
    // Add safety margin
    float threshold_distance = g_state.safe_stopping_distance + SAFETY_MARGIN_M;
    
    // Emergency conditions:
    // 1. Distance less than stopping distance + margin
    // 2. Moving forward (velocity > 0)
    // 3. Distance less than minimum safe distance OR TTC < critical threshold
    
    bool distance_critical = (distance_m < threshold_distance);
    bool moving_forward = (velocity_ms > 0.01f);
    bool too_close = (distance_m < MIN_OBSTACLE_DISTANCE_M);
    bool ttc_critical = (g_state.ttc < 1.0f);  // Less than 1 second to collision
    
    g_state.emergency_triggered = moving_forward && (distance_critical || too_close || ttc_critical);
    
    // Copy state to output if provided
    if (state != NULL) {
        memcpy(state, &g_state, sizeof(EmergencyStopState));
    }
    
    return g_state.emergency_triggered;
}
