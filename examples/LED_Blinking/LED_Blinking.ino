/**
 * Example 06: LED Blinking Patterns
 * Demonstrates LED blinking functions
 * 
 * This example shows different blinking patterns
 * and how to control blinking behavior.
 */

#include "Vexor.h"

Vexor robot;  // Create robot object

void setup() {
    Serial.begin(9600);
    robot.begin();
    
    Serial.println("=== Vexor LED Blinking Patterns ===");
    Serial.println("Demonstrating blinking modes");
    Serial.println();
}

void loop() {
    
    Serial.println("\n=== SIMPLE BLINKING ===");
    
    // 1. Simple blink with predefined color
    Serial.println("Blinking RED (500ms interval)");
    robot.ledBlink(500,5,COLOR_RED);
    
    Serial.println("Blinking GREEN (250ms interval)");
    robot.ledBlink(250,5,COLOR_GREEN);
    
    Serial.println("Blinking BLUE (100ms interval) - fast blink");
    robot.ledBlink(100,15,COLOR_BLUE);

    
    // 2. Blink with HEX colors
    Serial.println("\n=== HEX COLOR BLINKING ===");
    
    Serial.println("Blinking ORANGE (0xFFA500)");
    robot.ledBlinkHex(400,5, 0xFFA500);

    
    Serial.println("Blinking PURPLE (0x800080) with 50% brightness");
    robot.ledBlinkHex(300,5, 0x800080, 128);
    
    Serial.println("Stopping blink");
    robot.ledOff(); 
    delay(1000);
    
}