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

        auto &data = IrReceiver.decodedIRData;

        // noise filtration
        if (data.protocol != UNKNOWN && data.command != 0) {

            Serial.print("Protocol: ");
            Serial.print(getProtocolString(data.protocol));

            Serial.print(" | Address: 0x");
            Serial.print(data.address, HEX);

            Serial.print(" | Command: 0x");
            Serial.println(data.command, HEX);
            
              // Command comparison can be done here
        }

        IrReceiver.resume();
    }
}