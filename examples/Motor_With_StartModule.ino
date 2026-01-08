/**
 * Example 03: Motor Control with Start Module
 * Demonstrates motor control using competition start module
 * 
 * This example shows how to use the physical start button
 * to enable/disable motors during competitions.
 */

#include <Vexor.h>

Vexor robot;  // Create robot object

void setup() {
    Serial.begin(9600);
    robot.begin();
    
    // IMPORTANT: Start module is enabled by default
    Serial.println("=== Vexor Motor Control with Start Module ===");
    Serial.println("Waiting for start signal...");
    Serial.println("(Press start button to begin)");
    Serial.println();
    
    robot.setColor(COLOR_RED);  // Red = waiting for start
}

void loop() {
    // Update required components
    robot.ledUpdate();
    
    // Check if start button is pressed
    bool startSignal = robot.startSignal();
    
    if (!startSignal) {
        // Waiting for start - motors disabled
        robot.setColor(COLOR_RED);
        robot.ledBlink(500, COLOR_RED);  // Blink red while waiting
        
        Serial.println("⏳ Waiting for start signal...");
    } else {
        // Start button pressed - motors enabled
        robot.ledBlinkStop();
        robot.setColor(COLOR_GREEN);  // Green = running
        
        // Example motor movements
        Serial.println("Start signal received! Motors enabled.");
        
        // Forward for 2 seconds
        Serial.println("Moving FORWARD (50%)");
        robot.motor(50, 50);
        delay(2000);
        
        // Turn right for 1 second
        Serial.println("Turning RIGHT");
        robot.motor(30, -30);
        delay(1000);
        
        // Backward for 1.5 seconds
        Serial.println("Moving BACKWARD (30%)");
        robot.motor(-30, -30);
        delay(1500);
        
        // Stop
        Serial.println("STOPPING");
        robot.stop();
        delay(2000);
    }
    
    delay(100);  // Small delay to prevent Serial flooding
}