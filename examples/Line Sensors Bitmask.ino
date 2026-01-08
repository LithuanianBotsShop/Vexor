/**
 * Example 02B: Line Sensors (Switch-Case)
 * 
 * Demonstrates line sensor bitmask reading
 * and decision making using switch-case.
 */

#include <Vexor.h>

Vexor robot;

void setup() {
    Serial.begin(9600);
    robot.begin();

    Serial.println("=== Vexor Line Sensors (Switch-Case) ===");
}

void loop() {
    uint8_t line = robot.readLineSensors(); // 2-bit mask

    Serial.print("Line sensors: 0b");
    Serial.print((line >> 1) & 1);
    Serial.print((line >> 0) & 1);
    Serial.println();

    switch (line) {

        case 0b00:
            Serial.println("Safe: no line detected");
            break;

        case 0b01:
            Serial.println("LEFT line detected");
            break;

        case 0b10:
            Serial.println("RIGHT  line detected");          
            break;

        case 0b11:
            Serial.println("BOTH lines detected! DANGER!");
            break;
    }

    Serial.println("----------------------------");
    delay(300);
}
