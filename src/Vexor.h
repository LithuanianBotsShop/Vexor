/**
 * @file Vexor.h
 * @brief Main library for Vexor Mini Sumo Robot Controller
 * @version 1.1.0
 * @date 2026-01-07
 * @author LithuanianBots
 * @copyright MIT License
 * @see https://github.com/LithuanianBots
 */

#ifndef VEXOR_H
#define VEXOR_H

#include <Arduino.h>

/**
 * @enum Sensor
 * @brief Robot front and line sensor enumeration
 */
enum Sensor {
    FL90,      ///< Front left 90° sensor
    FL45,      ///< Front left 45° sensor
    CC,        ///< Center sensor
    FR45,      ///< Front right 45° sensor
    FR90,      ///< Front right 90° sensor
    LINE_LEFT, ///< Left line sensor
    LINE_RIGHT ///< Right line sensor
};

/**
 * @enum LedColor
 * @brief Predefined RGB LED colors
 */
enum LedColor {
    COLOR_RED,     ///< Red color
    COLOR_GREEN,   ///< Green color
    COLOR_BLUE,    ///< Blue color
    COLOR_YELLOW,  ///< Yellow color
    COLOR_CYAN,    ///< Cyan color
    COLOR_MAGENTA, ///< Magenta color
    COLOR_WHITE,   ///< White color
    COLOR_ORANGE,  ///< Orange color
    COLOR_PURPLE,  ///< Purple color
    COLOR_PINK,    ///< Pink color
    COLOR_OFF      ///< LED off
};

/**
 * @class Vexor
 * @brief Main class for controlling Vexor Mini Sumo Robot
 * 
 * This library provides complete control over all Vexor robot components:
 * sensors, motors, RGB LED, servo, current monitoring and flag mechanism.
 * Designed for Arduino Nano with optimized performance for Mini Sumo competitions.
 */
class Vexor {
public:
    Vexor();
    void begin();
    
    // ==================== SENSOR METHODS ====================
    uint8_t readFrontSensors();   ///< Read all front sensors as bitmask
    uint8_t readLineSensors();    ///< Read line sensors as bitmask
    bool readSensor(Sensor s);    ///< Read specific sensor
    void sensorPullUp(bool state);///< Enable/disable sensor pull-up resistors
    void reverseSensorSignal(bool state); ///< Reverse sensor logic
    
    // ==================== MOTOR CONTROL ====================
    void motor(int8_t leftPercent, int8_t rightPercent); ///< Control both motors
    void stop();                                          ///< Stop motors
    void reverseRightMotor(bool state);                   ///< Reverse right motor direction
    void reverseLeftMotor(bool state);                    ///< Reverse left motor direction
    float rightMotorCurrent();                            ///< Get right motor current (A)
    float leftMotorCurrent();                             ///< Get left motor current (A)
    void  motorEnable();                                   ///< Manually enable motors
    void  motorDisable();                                  ///< Manually disable motors
    
    // ==================== LED CONTROL ====================
    void setRGB(uint8_t r, uint8_t g, uint8_t b);         ///< Set RGB color directly
    void setRed(uint8_t brightness = 255);                ///< Set red color
    void setGreen(uint8_t brightness = 255);              ///< Set green color
    void setBlue(uint8_t brightness = 255);               ///< Set blue color
    void setWhite(uint8_t brightness = 255);              ///< Set white color
    void ledOff();                                        ///< Turn LED off
    void setHexColor(uint32_t hexColor, uint8_t brightness = 255); ///< Set HEX color
    void setColor(LedColor color, uint8_t brightness = 255); ///< Set predefined color
    void ledBlink(uint16_t intervalMS, int count, LedColor color, uint8_t brightness = 255);
    void ledBlinkHex(uint16_t intervalMS, int count, uint32_t hexColor, uint8_t brightness = 255);
    
    // ==================== START MODULE ====================
    bool startSignal();                                   ///< Check start button state
    void useStartModule(bool state);                      ///< Enable/disable start module
    uint8_t getIRPin();                                   ///< Get IR pin number (12)
    
    // ==================== SERVO & FLAG ====================
    uint8_t getServoPin();                                ///< Get servo pin number (A3)
    void deployFlag(uint16_t durationMS = 100);           ///< Deploy flag for specified duration
    
private:
    // ==================== PIN DEFINITIONS ====================
    // Sensor pins
    static constexpr uint8_t _FL90 = 3;
    static constexpr uint8_t _FL45 = 4;
    static constexpr uint8_t _CC = 5;
    static constexpr uint8_t _FR45 = 7;
    static constexpr uint8_t _FR90 = 6;
    static constexpr uint8_t _LINE_LEFT = A0;
    static constexpr uint8_t _LINE_RIGHT = A1;
    
    // Motor pins
    static constexpr uint8_t _MOTOR_L_PWM = 9;
    static constexpr uint8_t _MOTOR_R_PWM = 10;
    static constexpr uint8_t _MOTOR_L_DIR = 8;
    static constexpr uint8_t _MOTOR_R_DIR = 11;
     static constexpr uint8_t _IR_PIN = 12;
    
    // Other pins
    static constexpr uint8_t _START_PIN = A2;
    static constexpr uint8_t _R_CURRENT_PIN = A6;
    static constexpr uint8_t _L_CURRENT_PIN = A7;
    static constexpr uint8_t _SERVO_PIN = A3;
    static constexpr uint8_t _N20_PIN = 2;
    
    // RGB LED
    static constexpr uint8_t _RGB_LED_PIN = 13;
    static constexpr uint8_t _RGB_LED_PORT = 0x05;  ///< PORTB address
    static constexpr uint8_t _RGB_LED_BIT = 5;      ///< PB5 (D13) bit
    
    // ==================== STATE VARIABLES ====================
    bool _pullupState = false;
    bool _useStartModule = true;
    bool _reverseRightMotor = false;
    bool _reverseLeftMotor = false;
    bool _reverseSensor = false;
    
    // ==================== CONSTANTS ====================
    static constexpr double _AIPROPI = 0.000455;   ///< Current sensor sensitivity (A/A)
    static constexpr double _RIPROPI = 2550.0;     ///< Current sensor resistor (ohm)
    static constexpr float _ADC_MAX = 1023.0;      ///< ADC maximum value
    static constexpr float _VREF = 5.0;            ///< ADC reference voltage (V)
    
    // ==================== LED BLINK VARIABLES ====================
    bool _ledBlinkActive = false;
    uint8_t _ledBlinkR = 0;
    uint8_t _ledBlinkG = 0;
    uint8_t _ledBlinkB = 0;
    uint8_t _ledBlinkBrightness = 255;
    uint16_t _ledBlinkInterval = 500;
    
    // ==================== PRIVATE METHODS ====================
    void _initPins();
    uint8_t _percentToPWM(int8_t percent);
    void _sendWS2812(uint8_t r, uint8_t g, uint8_t b);
    void _hexToRGB(uint32_t hexColor, uint8_t &r, uint8_t &g, uint8_t &b);
    void _applyBrightness(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t brightness);
};

#endif // VEXOR_H