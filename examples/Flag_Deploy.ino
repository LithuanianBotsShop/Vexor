/**
 * Example 07: Flag Deployment
 * Demonstrates the built-in flag deployment mechanism
 *
 * This example uses the Vexor internal protection logic.
 * The flag can be deployed ONLY ONCE per power cycle.
 */

#include <Vexor.h>

Vexor robot;

void setup() {
    Serial.begin(9600);
    robot.begin();

    Serial.println("=== Vexor Flag Deployment Example ===");
    Serial.println("Flag will deploy once after 3 seconds...");
}

void loop() {

    delay(3000);

    Serial.println("Deploying flag!");
    robot.deployFlag(150);  // duration in milliseconds

    Serial.println("Flag deployed.");
    Serial.println("Further deploy attempts will be ignored.");

    while (1); // Stop execution
}