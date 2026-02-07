/**
 * Simple Vexor sensor bitmask example
 * 
 * readFrontSensors() returns a 5-bit bitmask with states of all 5 front sensors
 * Function is very fast (0.4 µs)
 * 
 * Bit values:
 * Bit 0: FR90 (right 90°)
 * Bit 1: FR45 (right 45°)
 * Bit 2: CC   (center)
 * Bit 3: FL45 (left 45°)
 * Bit 4: FL90 (left 90°)
 * 
 * Bit = 1: sensor detects obstacle
 * Bit = 0: no obstacle detected
 */

#include <Vexor.h>

Vexor robot;

void setup() {
    Serial.begin(9600);
    robot.begin();
	
	// robot.sensorPullUp(true);   //Recommended when using inverted sensor logic.
	// robot.reverseSensorSignal(true); //Useful for sensors with active-low output.
    
    Serial.println("VEXOR SENSOR BITMASK EXAMPLE");
    Serial.println("============================");
    Serial.println();
    Serial.println("Bit mapping (LSB = bit 0):");
    Serial.println("Bit 0: FR90 (right 90°)");
    Serial.println("Bit 1: FR45 (right 45°)");
    Serial.println("Bit 2: CC   (center)");
    Serial.println("Bit 3: FL45 (left 45°)");
    Serial.println("Bit 4: FL90 (left 90°)");
    Serial.println();
    Serial.println("0 = no obstacle");
    Serial.println("1 = obstacle detected");
    Serial.println("============================");
    Serial.println();
	
	
}

void loop() {
    // Quickly read all 5 sensors at once
    uint8_t sensorMask = robot.readFrontSensors();
    
    // Show bitmask in binary format
    Serial.print("Bitmask: 0b");
    
    // Print 5 sensor bits (bits 4..0)
    for(int i = 4; i >= 0; i--) {
        Serial.print((sensorMask >> i) & 1);
    }
    Serial.println();
    
    // Use switch-case for analysis
    switch(sensorMask) {
        case 0b00000:
            Serial.println("All clear - no obstacles");
            break;
            
        case 0b00100:
            Serial.println("Center sensor detects obstacle");
            break;
            
        case 0b01010:
            Serial.println("Both 45° sensors detect obstacles");
            break;
            
        case 0b10001:
            Serial.println("Both 90° sensors detect obstacles");
            break;
            
        case 0b01110:
            Serial.println("Center and both 45° sensors detect obstacles");
            break;
            
        case 0b11111:
            Serial.println("ALL sensors detect obstacles - completely blocked!");
            break;
            
        case 0b11000:
            Serial.println("Right side sensors (FR45 & FR90) detect obstacles");
            break;
            
        case 0b00011:
            Serial.println("Left side sensors (FL45 & FL90) detect obstacles");
            break;
            
        case 0b01000:
            Serial.println("Only FR45 (right 45°) detects obstacle");
            break;
            
        case 0b00010:
            Serial.println("Only FL45 (left 45°) detects obstacle");
            break;
            
        case 0b10000:
            Serial.println("Only FR90 (right 90°) detects obstacle");
            break;
            
        case 0b00001:
            Serial.println("Only FL90 (left 90°) detects obstacle");
            break;
       
        default: 
            Serial.println("Unknown combo");
            break;
    }
    
    Serial.println("---");
    delay(1000);
}