/**
   Example 08: Current Measurement
   Demonstrates motor current monitoring

   This example shows how to read motor current
   and implement overload protection.
*/

#include <Vexor.h>

Vexor robot;  // Create robot object

void setup() {
  Serial.begin(9600);
  robot.begin();

  // Disable start module for manual testing
  robot.useStartModule(false);
  robot.motorEnable();

  Serial.println("=== Vexor Current Measurement ===");
  Serial.println("Monitoring motor current draw");
  Serial.println();
}

void loop() {

  // Read current from both motors
  float rightCurrent = robot.rightMotorCurrent();
  float leftCurrent = robot.leftMotorCurrent();

  // Display current values
  Serial.print("Right Motor: ");
  Serial.print(rightCurrent, 3);  // 3 decimal places
  Serial.print(" A\t");

  Serial.print("Left Motor: ");
  Serial.print(leftCurrent, 3);
  Serial.print(" A");

  // Simple current level indicators
  Serial.print("\t[");
  for (int i = 0; i < (int)(rightCurrent * 10); i++) Serial.print("#");
  for (int i = (int)(rightCurrent * 10); i < 20; i++) Serial.print(" ");
  Serial.print("]");

  Serial.println();

  // Overload protection example
  float overloadThreshold = 1.5;  // 1.5 Amps

  if (rightCurrent > overloadThreshold || leftCurrent > overloadThreshold) {
    Serial.println("OVERLOAD DETECTED! Stopping motors.");


    // Stop motors
    robot.stop();

    // Visual warning
    robot.ledBlink(100, 10, COLOR_RED);


    // Wait for current to drop
    delay(3000);
    robot.setColor(COLOR_GREEN);

  } else if (rightCurrent > 1.0 || leftCurrent > 1.0) {
    // Warning level (yellow)
    robot.setColor(COLOR_YELLOW);
  } else {
    // Normal operation (green)
    robot.setColor(COLOR_GREEN);
  }

  // Test different motor loads
  static int testPhase = 0;
  static unsigned long lastChange = 0;

  if (millis() - lastChange > 3000) {
    lastChange = millis();
    testPhase = (testPhase + 1) % 4;

    switch (testPhase) {
      case 0:
        Serial.println("\n>>> Test: Motors OFF");
        robot.stop();
        break;
      case 1:
        Serial.println("\n>>> Test: 25% speed");
        robot.motor(25, 25);
        break;
      case 2:
        Serial.println("\n>>> Test: 50% speed");
        robot.motor(50, 50);
        break;
      case 3:
        Serial.println("\n>>> Test: 100% )");
        robot.motor(100, 100);
        break;
    }
  }

  delay(100);  // 10Hz sampling rate
}