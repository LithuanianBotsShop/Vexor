/**
 * @file Vexor.cpp
 * @brief Implementation of Vexor Mini Sumo Robot Controller
 * @version 1.1.0
 * @date 2026-01-07
 * @author LithuanianBots
 * @copyright MIT License
 */

#include "Vexor.h"

// ==================== CONSTRUCTOR & INITIALIZATION ====================

Vexor::Vexor() {
    // Constructor - nothing to initialize here
}

/**
 * @brief Initialize all robot components
 * 
 * This method must be called in setup() before using any other functions.
 * It configures all pins, sensors, motors, and LED.
 */
void Vexor::begin() {
    _initPins();
}

/**
 * @brief Initialize all I/O pins
 * @private
 */
void Vexor::_initPins() {
    // Configure sensor pins
    sensorPullUp(_pullupState);
    
    // Line sensors and start pin
    pinMode(_LINE_LEFT, INPUT);
    pinMode(_LINE_RIGHT, INPUT);
    pinMode(_START_PIN, INPUT);
    
    // Servo and flag mechanism
    pinMode(_SERVO_PIN, OUTPUT);
    pinMode(_N20_PIN, OUTPUT);
    digitalWrite(_N20_PIN, LOW);
    
    // Motor control pins
    pinMode(_MOTOR_L_PWM, OUTPUT);
    pinMode(_MOTOR_R_PWM, OUTPUT);
    pinMode(_MOTOR_L_DIR, OUTPUT);
    pinMode(_MOTOR_R_DIR, OUTPUT);
    
    // Stop motors initially
    stop();
    
    // Configure RGB LED
    DDRB |= _BV(_RGB_LED_BIT);
    PORTB &= ~_BV(_RGB_LED_BIT);
    ledOff();
}

// ==================== SENSOR METHODS ====================

/**
 * @brief Read all front sensors as a 5-bit mask
 * @return Bitmask where bit 0=FL90, bit 1=FL45, bit 2=CC, bit 3=FR45, bit 4=FR90
 */
uint8_t Vexor::readFrontSensors() {
   // Read pins D3-D7 → bits 0–4
    uint8_t raw;

    if (!_reverseSensor) {
        raw = (PIND >> 3) & 0b11111;
    } else {
        raw = ~(PIND >> 3) & 0b11111;
    }

    /*
     * BIT REMAP FIX
     * Physical wiring error:
     * bit 3 <-> bit 4 must be swapped
     *
     * raw bits:
     * bit 0 = FL90
     * bit 1 = FL45
     * bit 2 = CC
     * bit 3 = RIGHT        (WRONG)
     * bit 4 = FRONT-RIGHT  (WRONG)
     */

    uint8_t fixed = raw;

    // Clear bits 3 and 4
    fixed &= ~(0b11000);

    // Move old bit3 -> bit4
    fixed |= (raw & 0b01000) << 1;

    // Move old bit4 -> bit3
    fixed |= (raw & 0b10000) >> 1;

    return fixed;
}

/**
 * @brief Read line sensors as a 2-bit mask
 * @return Bitmask where bit 0=LEFT_LINE, bit 1=RIGHT_LINE
 */
uint8_t Vexor::readLineSensors() {
    // Read pins A0-A1 (PINC register bits 0-1)
    return PINC & 0b11;
}

/**
 * @brief Read specific sensor
 * @param s Sensor to read
 * @return Sensor state (true = detected, false = not detected)
 */
bool Vexor::readSensor(Sensor s) {
    // Line sensors - no logic reversal
    if (s == LINE_LEFT || s == LINE_RIGHT) {
        return digitalRead((s == LINE_LEFT) ? _LINE_LEFT : _LINE_RIGHT);
    }
    
    // Front sensors
    bool sensorValue;
    switch (s) {
        case FL90: sensorValue = digitalRead(_FL90); break;
        case FL45: sensorValue = digitalRead(_FL45); break;
        case CC:   sensorValue = digitalRead(_CC);   break;
        case FR45: sensorValue = digitalRead(_FR45); break;
        case FR90: sensorValue = digitalRead(_FR90); break;
        default:   return false;
    }
    
    // Apply reversal only to front sensors
    return _reverseSensor ? !sensorValue : sensorValue;
}

/**
 * @brief Enable or disable internal pull-up resistors for front sensors
 * @param state true = enable pull-ups, false = disable
 */
void Vexor::sensorPullUp(bool state) {
    _pullupState = state;
    uint8_t mode = state ? INPUT_PULLUP : INPUT;
    
    pinMode(_FL90, mode);
    pinMode(_FL45, mode);
    pinMode(_CC, mode);
    pinMode(_FR45, mode);
    pinMode(_FR90, mode);
}

/**
 * @brief Reverse sensor logic (invert readings)
 * @param state true = reverse logic, false = normal logic
 */
void Vexor::reverseSensorSignal(bool state) {
    _reverseSensor = state;
}

// ==================== MOTOR CONTROL ====================

/**
 * @brief Control both motors with percentage values
 * @param leftPercent Left motor speed (-100 to 100)
 * @param rightPercent Right motor speed (-100 to 100)
 */
void Vexor::motor(int8_t leftPercent, int8_t rightPercent) {
    // Apply motor direction reversal
    if (_reverseRightMotor) rightPercent *= -1;
    if (_reverseLeftMotor)  leftPercent *= -1;
    
    // Constrain values
    leftPercent  = constrain(leftPercent, -100, 100);
    rightPercent = constrain(rightPercent, -100, 100);
    
    // Convert to PWM values
    uint8_t leftPWM  = _percentToPWM(leftPercent);
    uint8_t rightPWM = _percentToPWM(rightPercent);
    
    // Left motor control
    digitalWrite(_MOTOR_L_DIR, leftPercent >= 0 ? HIGH : LOW);
    analogWrite(_MOTOR_L_PWM, leftPWM);
    
    // Right motor control (direction logic may be inverted depending on wiring)
    digitalWrite(_MOTOR_R_DIR, rightPercent >= 0 ? LOW : HIGH);
    analogWrite(_MOTOR_R_PWM, rightPWM);
}

/**
 * @brief Stop both motors
 */
void Vexor::stop() {
    analogWrite(_MOTOR_L_PWM, 0);
    analogWrite(_MOTOR_R_PWM, 0);
}

/**
 * @brief Convert percentage to PWM value
 * @param percent Motor speed percentage (-100 to 100)
 * @return PWM value (0-255)
 * @private
 */
uint8_t Vexor::_percentToPWM(int8_t percent) {
    return map(abs(percent), 0, 100, 0, 255);
}

/**
 * @brief Reverse right motor direction
 * @param state true = reversed, false = normal
 */
void Vexor::reverseRightMotor(bool state) {
    _reverseRightMotor = state;
}

/**
 * @brief Reverse left motor direction
 * @param state true = reversed, false = normal
 */
void Vexor::reverseLeftMotor(bool state) {
    _reverseLeftMotor = state;
}

/**
 * @brief Get right motor current consumption
 * @return Current in Amperes
 */
float Vexor::rightMotorCurrent() {
    int adc = analogRead(_R_CURRENT_PIN);
    float voltage = (adc * _VREF) / _ADC_MAX;
    float current = voltage / (_RIPROPI * _AIPROPI);
    return current;
}

/**
 * @brief Get left motor current consumption
 * @return Current in Amperes
 */
float Vexor::leftMotorCurrent() {
    int adc = analogRead(_L_CURRENT_PIN);
    float voltage = (adc * _VREF) / _ADC_MAX;
    float current = voltage / (_RIPROPI * _AIPROPI);
    return current;
}

/**
 * @brief Manually enable motors (when start module is disabled)
 */
void Vexor::motorEnable() {
    if (!_useStartModule) {
        digitalWrite(_START_PIN, HIGH);
    }
}

/**
 * @brief Manually disable motors (when start module is disabled)
 */
void Vexor::motorDisable() {
    if (!_useStartModule) {
        digitalWrite(_START_PIN, LOW);
    }
}

// ==================== LED CONTROL ====================

/**
 * @brief Send data to WS2812B LED (optimized assembly)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @private
 */
void Vexor::_sendWS2812(uint8_t r, uint8_t g, uint8_t b) {
    // Optimized assembly for WS2812B timing
    asm volatile(
        // Save registers
        "push r16          \n\t"
        "push r17          \n\t"
        "push r18          \n\t"
        "push r19          \n\t"
        "push r20          \n\t"
        "push r21          \n\t"
        
        // Load color values: R16=G, R17=R, R18=B (GRB order)
        "mov r16, %[green] \n\t"
        "mov r17, %[red]   \n\t"
        "mov r18, %[blue]  \n\t"
        
        // Disable interrupts for precise timing
        "cli               \n\t"
        
        // 3 bytes = 24 bits
        "ldi r19, 3        \n\t"  // Byte counter
        "mov r20, r16      \n\t"  // First byte = Green
        
        "byte_loop:        \n\t"
        "ldi r21, 8        \n\t"  // Bit counter
        
        "bit_loop:         \n\t"
        // Send HIGH signal
        "sbi %[port], %[bit] \n\t"
        
        // Check bit and set timing
        "lsl r20           \n\t"  // Shift bit into carry
        "brcs one_bit      \n\t"  // Jump if carry=1
        
        // ZERO bit: short HIGH (300-400ns)
        "nop               \n\t"
        "nop               \n\t"
        "cbi %[port], %[bit] \n\t"  // Set LOW
        // Long LOW (~850ns)
        "nop               \n\t" "nop               \n\t"
        "nop               \n\t" "nop               \n\t"
        "nop               \n\t" "nop               \n\t"
        "nop               \n\t" "nop               \n\t"
        "rjmp bit_done     \n\t"
        
        "one_bit:          \n\t"
        // ONE bit: long HIGH (~800ns)
        "nop               \n\t" "nop               \n\t"
        "nop               \n\t" "nop               \n\t"
        "nop               \n\t" "nop               \n\t"
        "nop               \n\t"
        "cbi %[port], %[bit] \n\t"  // Set LOW
        // Short LOW (~450ns)
        "nop               \n\t" "nop               \n\t"
        
        "bit_done:         \n\t"
        "dec r21           \n\t"  // Decrement bit counter
        "brne bit_loop     \n\t"  // Repeat for 8 bits
        
        // Load next byte
        "dec r19           \n\t"
        "breq send_done    \n\t"  // All bytes sent
        
        "cpi r19, 2        \n\t"
        "brne not_first    \n\t"
        "mov r20, r17      \n\t"  // Second byte = Red
        "rjmp byte_loop    \n\t"
        
        "not_first:        \n\t"
        "mov r20, r18      \n\t"  // Third byte = Blue
        "rjmp byte_loop    \n\t"
        
        "send_done:        \n\t"
        // Re-enable interrupts
        "sei               \n\t"
        
        // Restore registers
        "pop r21           \n\t"
        "pop r20           \n\t"
        "pop r19           \n\t"
        "pop r18           \n\t"
        "pop r17           \n\t"
        "pop r16           \n\t"
        
        // RESET signal (minimum 50µs)
        "ldi r24, 160      \n\t"  // 160 cycles ≈ 10µs @16MHz
        "reset_wait:       \n\t"
        "nop               \n\t"
        "dec r24           \n\t"
        "brne reset_wait   \n\t"
        
        : 
        : [port] "I" (_SFR_IO_ADDR(PORTB)),  // PORTB address
          [bit] "I" (_RGB_LED_BIT),          // PB5 bit
          [red] "r" (r),
          [green] "r" (g),
          [blue] "r" (b)
        : "r16", "r17", "r18", "r19", "r20", "r21", "r24"
    );
}

/**
 * @brief Set RGB color directly
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void Vexor::setRGB(uint8_t r, uint8_t g, uint8_t b) {
    _sendWS2812(r, g, b);
}

/**
 * @brief Set red color
 * @param brightness Brightness (0-255)
 */
void Vexor::setRed(uint8_t brightness) {
    _sendWS2812(brightness, 0, 0);
}

/**
 * @brief Set green color
 * @param brightness Brightness (0-255)
 */
void Vexor::setGreen(uint8_t brightness) {
    _sendWS2812(0, brightness, 0);
}

/**
 * @brief Set blue color
 * @param brightness Brightness (0-255)
 */
void Vexor::setBlue(uint8_t brightness) {
    _sendWS2812(0, 0, brightness);
}

/**
 * @brief Set white color
 * @param brightness Brightness (0-255)
 */
void Vexor::setWhite(uint8_t brightness) {
    _sendWS2812(brightness, brightness, brightness);
}

/**
 * @brief Turn LED off
 */
void Vexor::ledOff() {
    _sendWS2812(0, 0, 0);
}

/**
 * @brief Convert HEX color to RGB components
 * @param hexColor HEX color value (0xRRGGBB or 0xRGB)
 * @param r Reference to red component
 * @param g Reference to green component
 * @param b Reference to blue component
 * @private
 */
void Vexor::_hexToRGB(uint32_t hexColor, uint8_t &r, uint8_t &g, uint8_t &b) {
    if (hexColor <= 0xFFF) {
        // 0xRGB format (short)
        r = ((hexColor >> 8) & 0xF) * 17;  // Multiply by 17 (0xF→0xFF)
        g = ((hexColor >> 4) & 0xF) * 17;
        b = (hexColor & 0xF) * 17;
    } else {
        // 0xRRGGBB format (full)
        r = (hexColor >> 16) & 0xFF;
        g = (hexColor >> 8) & 0xFF;
        b = hexColor & 0xFF;
    }
}

/**
 * @brief Apply brightness to RGB components
 * @param r Red component (modified in-place)
 * @param g Green component (modified in-place)
 * @param b Blue component (modified in-place)
 * @param brightness Brightness (0-255)
 * @private
 */
void Vexor::_applyBrightness(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t brightness) {
    if (brightness < 255) {
        r = (r * brightness) / 255;
        g = (g * brightness) / 255;
        b = (b * brightness) / 255;
    }
}

/**
 * @brief Set color from HEX value
 * @param hexColor HEX color (0xRRGGBB or 0xRGB)
 * @param brightness Brightness (0-255)
 */
void Vexor::setHexColor(uint32_t hexColor, uint8_t brightness) {
    uint8_t r, g, b;
    _hexToRGB(hexColor, r, g, b);
    _applyBrightness(r, g, b, brightness);
    setRGB(r, g, b);
}

/**
 * @brief Set predefined color
 * @param color Predefined color from LedColor enum
 * @param brightness Brightness (0-255)
 */
void Vexor::setColor(LedColor color, uint8_t brightness) {
    switch(color) {
        case COLOR_RED:     setRGB(brightness, 0, 0); break;
        case COLOR_GREEN:   setRGB(0, brightness, 0); break;
        case COLOR_BLUE:    setRGB(0, 0, brightness); break;
        case COLOR_YELLOW:  setRGB(brightness, brightness, 0); break;
        case COLOR_CYAN:    setRGB(0, brightness, brightness); break;
        case COLOR_MAGENTA: setRGB(brightness, 0, brightness); break;
        case COLOR_WHITE:   setRGB(brightness, brightness, brightness); break;
        case COLOR_ORANGE:  setRGB(brightness, brightness/2, 0); break;
        case COLOR_PURPLE:  setRGB(brightness/2, 0, brightness); break;
        case COLOR_PINK:    setRGB(brightness, brightness/4, brightness/2); break;
        case COLOR_OFF:     ledOff(); break;
    }
}

/**
 * @brief Blink LED with predefined color
 * @param intervalMS Blink interval in milliseconds
 * @param count Number of blink cycles
 * @param color LED color
 * @param brightness Brightness (0-255)
 */
void Vexor::ledBlink(uint16_t intervalMS, int count, LedColor color, uint8_t brightness) {
    _ledBlinkActive = true;
    _ledBlinkInterval = intervalMS;
    _ledBlinkBrightness = brightness;
    
    // Set color based on enum
    switch(color) {
        case COLOR_RED:     _ledBlinkR = brightness; _ledBlinkG = 0; _ledBlinkB = 0; break;
        case COLOR_GREEN:   _ledBlinkR = 0; _ledBlinkG = brightness; _ledBlinkB = 0; break;
        case COLOR_BLUE:    _ledBlinkR = 0; _ledBlinkG = 0; _ledBlinkB = brightness; break;
        case COLOR_YELLOW:  _ledBlinkR = brightness; _ledBlinkG = brightness; _ledBlinkB = 0; break;
        case COLOR_CYAN:    _ledBlinkR = 0; _ledBlinkG = brightness; _ledBlinkB = brightness; break;
        case COLOR_MAGENTA: _ledBlinkR = brightness; _ledBlinkG = 0; _ledBlinkB = brightness; break;
        case COLOR_WHITE:   _ledBlinkR = brightness; _ledBlinkG = brightness; _ledBlinkB = brightness; break;
        case COLOR_ORANGE:  _ledBlinkR = brightness; _ledBlinkG = brightness/2; _ledBlinkB = 0; break;
        case COLOR_PURPLE:  _ledBlinkR = brightness/2; _ledBlinkG = 0; _ledBlinkB = brightness; break;
        case COLOR_PINK:    _ledBlinkR = brightness; _ledBlinkG = brightness/4; _ledBlinkB = brightness/2; break;
        case COLOR_OFF:     _ledBlinkR = 0; _ledBlinkG = 0; _ledBlinkB = 0; break;
    }
    
    // Perform blinking
    for(int i = 0; i < count; i++) {
        setRGB(_ledBlinkR, _ledBlinkG, _ledBlinkB);
        delay(_ledBlinkInterval);
        ledOff();
        delay(_ledBlinkInterval);
    }
    
    _ledBlinkActive = false;
}

/**
 * @brief Blink LED with HEX color
 * @param intervalMS Blink interval in milliseconds
 * @param count Number of blink cycles
 * @param hexColor HEX color value
 * @param brightness Brightness (0-255)
 */
void Vexor::ledBlinkHex(uint16_t intervalMS, int count, uint32_t hexColor, uint8_t brightness) {
    _ledBlinkActive = true;
    _ledBlinkInterval = intervalMS;
    _ledBlinkBrightness = brightness;
    
    // Convert HEX to RGB
    _hexToRGB(hexColor, _ledBlinkR, _ledBlinkG, _ledBlinkB);
    _applyBrightness(_ledBlinkR, _ledBlinkG, _ledBlinkB, brightness);
    
    // Perform blinking
    for(int i = 0; i < count; i++) {
        setRGB(_ledBlinkR, _ledBlinkG, _ledBlinkB);
        delay(_ledBlinkInterval);
        ledOff();
        delay(_ledBlinkInterval);
    }
    
    _ledBlinkActive = false;
}

// ==================== START MODULE ====================

/**
 * @brief Check start button state
 * @return true if start button is pressed, false otherwise
 */
bool Vexor::startSignal() {
    return _useStartModule ? digitalRead(_START_PIN) : false;
}

/**
 * @brief Get IR sensor pin number
 * @return Pin number (12) for IR sensor pin 
 */
uint8_t Vexor:: getIRPin(){
   return _IR_PIN;
}
/**
 * @brief Enable or disable start module functionality
 * @param state true = use start module, false = manual control
 */
void Vexor::useStartModule(bool state) {
    _useStartModule = state;
    if (state) {
        pinMode(_START_PIN, INPUT);     // Use as input for start signal
    } else {
        pinMode(_START_PIN, OUTPUT);    // Use as output for manual enable
    }
}

// ==================== SERVO & FLAG ====================

/**
 * @brief Get servo pin number
 * @return Pin number (A3) for servo control
 */
uint8_t Vexor::getServoPin() {
    return _SERVO_PIN;
}

/**
 * @brief Deploy flag for specified duration
 * @param durationMS Flag deployment duration in milliseconds
 */
void Vexor::deployFlag(uint16_t durationMS) {
    static bool alreadyDeployed = false;
    static uint32_t startTime = 0;
    
    // Prevent multiple deployments
    if (alreadyDeployed) return;
    
    // Start flag deployment
    if (startTime == 0) {
        digitalWrite(_N20_PIN, HIGH);
        startTime = millis();
    }
    
    // Check if duration has elapsed
    if (millis() - startTime >= durationMS) {
        digitalWrite(_N20_PIN, LOW);
        alreadyDeployed = true;
    }
}