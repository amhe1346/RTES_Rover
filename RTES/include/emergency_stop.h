/**
 * @file emergency_stop.h
 * @brief Service 1: Emergency Stop - High-priority collision avoidance
 * 
 * Monitors ultrasonic sensor data and computes Time-to-Collision (TTC).
 * Generates emergency stop commands when collision risk detected.
 * Target: <1ms execution time for hard real-time response.
 */

#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include "safety_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize emergency stop system
 */
void emergency_stop_init(void);

/**
 * Check if emergency stop should be triggered
 * 
 * @param distance_m Current distance to obstacle (meters)
 * @param velocity_ms Current velocity (m/s)
 * @param state Output state structure (can be NULL)
 * @return true if emergency stop required, false otherwise
 * 
 * Execution time: <1ms (deterministic)
 */
bool check_emergency_stop(float distance_m, float velocity_ms, 
                          EmergencyStopState *state);

/**
 * Calculate Time-to-Collision (TTC)
 * 
 * @param distance_m Distance to obstacle (meters)
 * @param velocity_ms Current velocity (m/s)
 * @return TTC in seconds, or INFINITY if no collision
 */
float calculate_ttc(float distance_m, float velocity_ms);

/**
 * Calculate safe stopping distance
 * 
 * @param velocity_ms Current velocity (m/s)
 * @param deceleration_ms2 Deceleration rate (m/s²)
 * @return Required stopping distance (meters)
 */
float calculate_stopping_distance(float velocity_ms, float deceleration_ms2);

#ifdef __cplusplus
}
#endif

#endif // EMERGENCY_STOP_H
