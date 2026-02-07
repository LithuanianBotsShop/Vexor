/**
 * Example 04: Motor Control without Start Module
 * Demonstrates manual motor control for testing
 * 
 * This example shows how to manually enable/disable
 * motors when not using the physical start button.
 */

#include <Vexor.h>

Vexor robot;  // Create robot object

void setup() {
    Serial.begin(9600);
    robot.begin();
    
    // DISABLE start module for manual control
    robot.useStartModule(false);
    
    // MANUALLY enable motors (required when start module is disabled)
    robot.motorEnable();

    //robot.reverseRightMotor(true);  // Inverts  right motor direction when is true (useful for wiring differences).
    //robot.reverseLeftMotor(true);   // Inverts  left motor direction when is true (useful for wiring differences).
    
    Serial.println("=== Vexor Manual Motor Control ===");
    Serial.println("Start module DISABLED");
    Serial.println("Motors MANUALLY enabled");
    Serial.println();
    delay(4000);
    
    robot.setColor(COLOR_BLUE);  // Blue = manual mode
}

void loop() {
    Serial.println("\n=== Motor Test Sequence ===");
    
    // Test 1: Forward
    Serial.println("1. Forward (70% speed)");
    robot.motor(70, 70);
    delay(2000);
    
    // Test 2: Backward
    Serial.println("2. Backward (50% speed)");
    robot.motor(-50, -50);
    delay(2000);
    
    // Test 3: Turn left
    Serial.println("3. Turn left");
    robot.motor(-40, 40);
    delay(1500);
    
    // Test 4: Turn right
    Serial.println("4. Turn right");
    robot.motor(40, -40);
    delay(1500);
    
    // Test 5: Spin clockwise
    Serial.println("5. Spin clockwise");
    robot.motor(100, -100);
    delay(1000);
    
    // Test 6: Stop
    Serial.println("6. Stop");
    robot.stop();
    delay(2000);
    
    // Optional: Manually disable motors
    // robot.motorDisable();
    // delay(5000);
    // robot.motorEnable();
}