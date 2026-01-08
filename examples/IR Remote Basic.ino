/**
 * Example 09: IR Remote Basic
 * Demonstrates how to use an IR remote receiver
 * together with the Vexor controller.
 *
 * Vexor provides the IR receiver pin via getIRPin().
 * IR decoding is handled by the IRremote library.
 */

#include <Vexor.h>
#include <IRremote.h>

Vexor robot;

void setup() {
    Serial.begin(9600);
    robot.begin();

    // Get IR receiver pin from Vexor
    uint8_t irPin = robot.getIRPin();

    Serial.println("=== Vexor IR Remote Example ===");
    Serial.print("IR receiver pin: ");
    Serial.println(irPin);

    // Initialize IR receiver
    IrReceiver.begin(irPin);

    Serial.println("Waiting for IR signals...");
    robot.setBlue(255);
}

void loop() {

    if (IrReceiver.decode()) {

        // Read received IR code
        uint32_t irCode = IrReceiver.decodedIRData.decodedRawData;

        Serial.print("IR code received: 0x");
        Serial.println(irCode, HEX);
        // Resume receiver for next signal
        IrReceiver.resume();
    }
}