/**
 * Example 01: Sensor Basics
 * Demonstrates how to read each sensor individually
 * 
 * This example shows how to read every sensor on the
 * Vexor robot and display their status in Serial Monitor.
 */

#include <Vexor.h>

Vexor robot;  // Create robot object

void setup() {
    Serial.begin(9600);      // Start serial communication
    robot.begin();           // Initialize robot
	
	// robot.sensorPullUp(true)   //Recommended when using inverted sensor logic.
	// robot.reverseSensorSignal(true) //Useful for sensors with active-low output.
	
    Serial.println("=== Vexor Sensor Basics Example ===");
    Serial.println("Reading individual sensors...");
    Serial.println();
}

void loop() {
    // Read front sensors individually
    bool fl90 = robot.readSensor(FL90);
    bool fl45 = robot.readSensor(FL45);
    bool cc   = robot.readSensor(CC);
    bool fd45 = robot.readSensor(FD45);
    bool fd90 = robot.readSensor(FD90);
    
    // Read line sensors individually
    bool lineLeft  = robot.readSensor(LINE_LEFT);
    bool lineRight = robot.readSensor(LINE_RIGHT);
    
    // Display sensor status
    Serial.println("=== Front Sensors ===");
    Serial.print("FL90: "); Serial.println(fl90 ? "OBSTACLE" : "CLEAR");
    Serial.print("FL45: "); Serial.println(fl45 ? "OBSTACLE" : "CLEAR");
    Serial.print("CC:   "); Serial.println(cc   ? "OBSTACLE" : "CLEAR");
    Serial.print("FD45: "); Serial.println(fd45 ? "OBSTACLE" : "CLEAR");
    Serial.print("FD90: "); Serial.println(fd90 ? "OBSTACLE" : "CLEAR");
    
    Serial.println("\n=== Line Sensors ===");
    Serial.print("LEFT:  "); Serial.println(lineLeft  ? "ON LINE" : "NO LINE");
    Serial.print("RIGHT: "); Serial.println(lineRight ? "ON LINE" : "NO LINE");
    
    Serial.println("\n-----------------------------------\n");
    
    delay(500);  // Wait 0.5 seconds between readings
}