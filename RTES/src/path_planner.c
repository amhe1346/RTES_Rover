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

