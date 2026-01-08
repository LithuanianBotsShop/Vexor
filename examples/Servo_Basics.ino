/**
 * Example 08: External Servo Control
 * Demonstrates how to use an external Servo library
 * together with the Vexor controller
 *
 * Vexor only provides the servo pin number.
 * Servo timing and control is handled by the Servo library.
 */

#include <Vexor.h>
#include <Servo.h>

Vexor robot;
Servo flagServo;

void setup() {
    Serial.begin(9600);
    robot.begin();

    uint8_t servoPin = robot.getServoPin();

    Serial.println("=== External Servo Control Example ===");
    Serial.print("Using servo pin: ");
    Serial.println(servoPin);

    flagServo.attach(servoPin);
}

void loop() {

    flagServo.write(0);     // Left
    delay(1500);

    flagServo.write(90);    // Center
    delay(1500);

    flagServo.write(180);   // Right
    delay(3000);
}
