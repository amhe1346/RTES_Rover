/**
 * @file motor_control.h
 * @brief Service 3: Motor Control & Logging - Command execution and telemetry
 * 
 * Executes motor commands with priority-based arbitration.
 * Maintains system state logging and ensures reliable actuation.
 * Lowest priority service, preempted by higher services.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "safety_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize motor control system
 */
void motor_control_init(void);

/**
 * Execute motor command with priority handling
 * 
 * @param command Motor command to execute
 * @param priority Command priority (1=highest, 3=lowest)
 * @param state Output motor control state (can be NULL)
 * @return true if command executed, false if preempted
 */
bool execute_motor_command(MotorCommand command, uint8_t priority,
                           MotorControlState *state);

/**
 * Get current motor control state
 * 
 * @param state Output state structure
 */
void get_motor_state(MotorControlState *state);

/**
 * Convert motor command enum to character representation
 * 
 * @param command Motor command to convert
 * @return Character representation (F/B/L/R/S)
 */
char motor_command_to_char(MotorCommand command);

/**
 * Log system telemetry data
 * 
 * @param distance_m Current distance reading
 * @param velocity_ms Current velocity
 * @param command Current motor command
 */
void log_telemetry(float distance_m, float velocity_ms, MotorCommand command);

/**
 * Convert MotorCommand enum to serial command character
 * 
 * @param command Motor command enum
 * @return Serial command character ('F', 'B', 'L', 'R', 'S')
 */
char motor_command_to_char(MotorCommand command);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
