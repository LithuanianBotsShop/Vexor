/**
 * Example 05: LED Color Control
 * Demonstrates all LED color functions
 * 
 * This example shows different ways to control
 * the RGB LED on the Vexor robot.
 */

#include <Vexor.h>

Vexor robot;  // Create robot object

void setup() {
    Serial.begin(9600);
    robot.begin();
    
    Serial.println("=== Vexor LED Color Control ===");
    Serial.println("Demonstrating all color modes");
    Serial.println();
}

void loop() {
    // Update required components
    
    Serial.println("\n=== BASIC COLORS ===");
    
    // 1. Basic color functions
    Serial.println("Red");
    robot.setRed(255);
    delay(1000);
    
    Serial.println("Green");
    robot.setGreen(255);
    delay(1000);
    
    Serial.println("Blue");
    robot.setBlue(255);
    delay(1000);
    
    Serial.println("White");
    robot.setWhite(200);
    delay(1000);
    
    Serial.println("Off");
    robot.ledOff();
    delay(500);
    
    // 2. Custom RGB colors
    Serial.println("\n=== CUSTOM RGB COLORS ===");
    
    Serial.println("Orange (255, 128, 0)");
    robot.setRGB(255, 128, 0);
    delay(1000);
    
    Serial.println("Purple (128, 0, 255)");
    robot.setRGB(128, 0, 255);
    delay(1000);
    
    Serial.println("Cyan (0, 255, 255)");
    robot.setRGB(0, 255, 255);
    delay(1000);
    
    Serial.println("Pink (255, 100, 180)");
    robot.setRGB(255, 100, 180);
    delay(1000);
    
    robot.ledOff();
    delay(500);
    
    // 3. HEX color codes
    Serial.println("\n=== HEX COLOR CODES ===");
    
    Serial.println("HEX: 0xFF0000 (Red)");
    robot.setHexColor(0xFF0000);
    delay(1000);
    
    Serial.println("HEX: 0x00FF00 (Green)");
    robot.setHexColor(0x00FF00);
    delay(1000);
    
    Serial.println("HEX: 0x0000FF (Blue)");
    robot.setHexColor(0x0000FF);
    delay(1000);
    
    Serial.println("HEX: 0xFFA500 (Orange)");
    robot.setHexColor(0xFFA500);
    delay(1000);
    
    Serial.println("HEX: 0x800080 (Purple) with 50% brightness");
    robot.setHexColor(0x800080, 128);
    delay(1000);
    
    robot.ledOff();
    delay(500);
    
    // 4. Predefined colors from enum
    Serial.println("\n=== PREDEFINED COLORS ===");
    
    Serial.println("COLOR_YELLOW");
    robot.setColor(COLOR_YELLOW);
    delay(1000);
    
    Serial.println("COLOR_CYAN");
    robot.setColor(COLOR_CYAN);
    delay(1000);
    
    Serial.println("COLOR_MAGENTA");
    robot.setColor(COLOR_MAGENTA);
    delay(1000);
    
    Serial.println("COLOR_ORANGE");
    robot.setColor(COLOR_ORANGE);
    delay(1000);
    
    Serial.println("COLOR_PURPLE with 75% brightness");
    robot.setColor(COLOR_PURPLE, 192);
    delay(1000);
    
    Serial.println("COLOR_OFF");
    robot.setColor(COLOR_OFF);
    delay(2000);
}