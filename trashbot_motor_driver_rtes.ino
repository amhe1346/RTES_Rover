/**
 * TrashBot Motor Driver - RTES Optimized
 * 
 * Real-time optimized Arduino Uno motor driver for TrashBot RTES system.
 * 
 * Features:
 * - 115200 baud for minimal latency (~0.087ms per byte vs 1.04ms at 9600)
 * - Non-blocking serial processing
 * - Direct GPIO control (~10-50μs execution time)
 * - No delays or blocking operations
 * - Simple single-character command protocol
 * 
 * Command Protocol:
 * - 'F' = Forward
 * - 'B' = Backward
 * - 'L' = Left (differential)
 * - 'R' = Right (differential)
 * - 'S' = Stop
 * 
 * Hardware Setup:
 * - Connect L298N or similar H-bridge motor driver
 * - Adjust pin assignments below for your configuration
 * 
 * Performance:
 * - Serial transmission: ~0.087ms (115200 baud)
 * - Command execution: <50μs
 * - Total Pi->Arduino->Motors: <200μs
 * 
 * Author: TrashBot RTES Team
 * Target: <10ms emergency stop latency
 */

// ═══════════════════════════════════════════════════════════════════
// PIN CONFIGURATION - L298N Motor Driver (Working Configuration)
// ═══════════════════════════════════════════════════════════════════

// Left Motor (Motor A)
#define PWMA  2    // Left motor PWM (speed control)
#define AIN1  4    // Left motor direction pin 1
#define AIN2  3    // Left motor direction pin 2

// Right Motor (Motor B)
#define PWMB  8    // Right motor PWM (speed control)
#define BIN1  6    // Right motor direction pin 1
#define BIN2  7    // Right motor direction pin 2

// Standby pin
#define STBY  5    // Standby - must be HIGH to enable motors

// Status LED
#define LED_STATUS   13   // Built-in LED for status indication

// Default motor speed
int motorSpeed = 200;  // 0-255, adjustable per command

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
    // Initialize serial at 115200 baud (12x faster than 9600)
    Serial.begin(115200);
    
    // Configure all pins as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(LED_STATUS, OUTPUT);
    
    // Enable motor driver (STBY must be HIGH)
    digitalWrite(STBY, HIGH);
    
    // Initialize in STOP state
    stop_motors();
    
    // Flash LED to indicate ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
        digitalWrite(LED_STATUS, LOW);
        delay(100);
    }
    
    // Optional: Send ready signal
    Serial.println("READY");
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP - Non-blocking, minimal latency
// ═══════════════════════════════════════════════════════════════════

void loop() {
    // Check for incoming serial commands (non-blocking)
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        // Execute command immediately
        execute_command(cmd);
        
        // Blink LED to indicate command received
        digitalWrite(LED_STATUS, HIGH);
        // Note: No delay here! LED will turn off on next command or timeout
    } else {
        // Turn off LED when no commands (visual feedback)
        digitalWrite(LED_STATUS, LOW);
    }
    
    // NO DELAYS! This loop executes as fast as possible
    // Typical loop time: <10μs when no serial data
}

// ═══════════════════════════════════════════════════════════════════
// COMMAND EXECUTION - Direct GPIO control, <50μs execution time
// ═══════════════════════════════════════════════════════════════════

void execute_command(char cmd) {
    switch(cmd) {
        case 'F':  // Forward
        case 'f':
            move_forward();
            break;
            
        case 'B':  // Backward
        case 'b':
            move_backward();
            break;
            
        case 'L':  // Left (differential turn)
        case 'l':
            turn_left();
            break;
            
        case 'R':  // Right (differential turn)
        case 'r':
            turn_right();
            break;
            
        case 'S':  // Stop
        case 's':
        case ' ':  // Space also stops
            stop_motors();
            break;
            
        default:
            // Unknown command - stop for safety
            stop_motors();
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════
// MOTOR CONTROL FUNCTIONS - L298N with PWM control
// ═══════════════════════════════════════════════════════════════════

// Low-level motor control
void motorA(int speed, bool dir) {
    digitalWrite(AIN1, dir ? HIGH : LOW);
    digitalWrite(AIN2, dir ? LOW : HIGH);
    analogWrite(PWMA, speed);
}

void motorB(int speed, bool dir) {
    digitalWrite(BIN1, dir ? HIGH : LOW);
    digitalWrite(BIN2, dir ? LOW : HIGH);
    analogWrite(PWMB, speed);
}

// High-level movement commands
void move_forward() {
    motorA(motorSpeed, false);  // Left motor forward
    motorB(motorSpeed, false);  // Right motor forward
}

void move_backward() {
    motorA(motorSpeed, true);   // Left motor backward
    motorB(motorSpeed, true);   // Right motor backward
}

void turn_left() {
    // TRUE differential turn - left backward, right forward
    motorA(motorSpeed, true);   // Left motor backward
    motorB(motorSpeed, false);  // Right motor forward
}

void turn_right() {
    // TRUE differential turn - left forward, right backward
    motorA(motorSpeed, false);  // Left motor forward
    motorB(motorSpeed, true);   // Right motor backward
}

void stop_motors() {
    motorA(0, true);
    motorB(0, true);
}

// ═══════════════════════════════════════════════════════════════════
// PERFORMANCE NOTES
// ═══════════════════════════════════════════════════════════════════
// 
// Latency Breakdown (from Pi command to motor activation):
// 1. Pi serial write: ~0.087ms (1 byte @ 115200 baud)
// 2. Arduino serial receive: ~0ms (interrupt-driven)
// 3. Command execution: ~0.01-0.05ms (digitalWrite calls)
// Total: ~0.1-0.2ms (well under 1ms target)
// 
// Comparison to 9600 baud:
// - 9600 baud:   ~1.04ms per byte transmission
// - 115200 baud: ~0.087ms per byte transmission
// - Speedup: 12x faster
// 
// This leaves most of the 10ms emergency stop budget for:
// - Sensor reading on Pi
// - Emergency stop logic
// - Path planning
// - ROS2 overhead
// - Physical motor response time
// 
// ═══════════════════════════════════════════════════════════════════
