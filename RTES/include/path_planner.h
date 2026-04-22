/**
 * @file path_planner.h
 * @brief Service 2: Path Planning - Obstacle avoidance navigation
 * 
 * Analyzes sensor data to determine navigation decisions (left, right, straight).
 * Generates directional motor commands for obstacle avoidance.
 * Runs at medium priority after emergency stop checks.
 */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "safety_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize path planner
 */
void path_planner_init(void);

/**
 * Compute path command based on sensor data
 * 
 * @param front_distance_m Front ultrasonic distance (meters)
 * @param current_velocity_ms Current velocity (m/s)
 * @return PathCommand structure with navigation decision
 * 
 * Execution time: <5ms
 */
PathCommand compute_path_command(float front_distance_m, 
                                 float current_velocity_ms);

/**
 * Determine best avoidance direction
 * 
 * @param front_dist Front clearance (meters)
 * @param left_dist Left clearance (meters, 0 if no sensor)
 * @param right_dist Right clearance (meters, 0 if no sensor)
 * @return Recommended motor command (LEFT, RIGHT, or STOP)
 */
MotorCommand determine_avoidance_direction(float front_dist, 
                                            float left_dist, 
                                            float right_dist);

#ifdef __cplusplus
}
#endif

#endif // PATH_PLANNER_H
