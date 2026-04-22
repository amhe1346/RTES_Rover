/**
 * @file test_emergency.c
 * @brief Unit tests for emergency stop logic
 * 
 * Simple test cases to validate emergency stop calculations.
 * Run with: ./test_emergency
 */

#include "emergency_stop.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

#define TEST_PASS printf("✓ PASS: %s\n", __func__)
#define TEST_FAIL printf("✗ FAIL: %s\n", __func__)

void test_ttc_calculation() {
    // Test normal case
    float ttc = calculate_ttc(10.0f, 2.0f);
    assert(fabs(ttc - 5.0f) < 0.01f);  // 10m / 2m/s = 5s
    
    // Test stationary (should return infinity)
    ttc = calculate_ttc(10.0f, 0.0f);
    assert(isinf(ttc));
    
    // Test already at collision
    ttc = calculate_ttc(0.0f, 2.0f);
    assert(ttc == 0.0f);
    
    TEST_PASS;
}

void test_stopping_distance() {
    // Test normal case: v=10m/s, a=5m/s²
    float stop_dist = calculate_stopping_distance(10.0f, 5.0f);
    assert(fabs(stop_dist - 10.0f) < 0.01f);  // v²/2a = 100/10 = 10m
    
    // Test stationary
    stop_dist = calculate_stopping_distance(0.0f, 1.0f);
    assert(stop_dist == 0.0f);
    
    // Test v=2m/s, a=1m/s²
    stop_dist = calculate_stopping_distance(2.0f, 1.0f);
    assert(fabs(stop_dist - 2.0f) < 0.01f);  // 4/2 = 2m
    
    TEST_PASS;
}

void test_emergency_stop_trigger() {
    emergency_stop_init();
    EmergencyStopState state;
    
    // Test 1: Safe distance, no emergency
    bool emergency = check_emergency_stop(10.0f, 0.5f, &state);
    assert(emergency == false);
    printf("  Test 1: Safe distance (10m, 0.5m/s) - No emergency\n");
    
    // Test 2: Close obstacle, should trigger
    emergency = check_emergency_stop(0.2f, 0.5f, &state);
    assert(emergency == true);
    printf("  Test 2: Close obstacle (0.2m, 0.5m/s) - EMERGENCY!\n");
    
    // Test 3: Moderate distance but high speed
    emergency = check_emergency_stop(1.0f, 2.0f, &state);
    assert(emergency == true);
    printf("  Test 3: High speed approach (1.0m, 2.0m/s) - EMERGENCY!\n");
    
    // Test 4: Stationary, no emergency even if close
    emergency = check_emergency_stop(0.5f, 0.0f, &state);
    assert(emergency == false);
    printf("  Test 4: Stationary (0.5m, 0m/s) - No emergency\n");
    
    TEST_PASS;
}

void test_state_output() {
    emergency_stop_init();
    EmergencyStopState state;
    
    check_emergency_stop(2.0f, 1.0f, &state);
    
    assert(state.distance_m == 2.0f);
    assert(state.velocity_ms == 1.0f);
    assert(state.ttc == 2.0f);  // 2m / 1m/s = 2s
    
    printf("  State: dist=%.2fm, vel=%.2fm/s, ttc=%.2fs, stop_dist=%.2fm\n",
           state.distance_m, state.velocity_ms, state.ttc, state.safe_stopping_distance);
    
    TEST_PASS;
}

int main() {
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Emergency Stop Unit Tests\n");
    printf("═══════════════════════════════════════════════════════════\n\n");
    
    test_ttc_calculation();
    test_stopping_distance();
    test_emergency_stop_trigger();
    test_state_output();
    
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("All tests passed! ✓\n");
    printf("═══════════════════════════════════════════════════════════\n");
    
    return 0;
}
