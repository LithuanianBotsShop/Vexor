# Vexor Arduino Library

Professional Arduino library for the **Vexor Mini Sumo Robot Controller**, designed and maintained by **LithuanianBots**.

Vexor provides a **clean, competition-grade API** for controlling motors, sensors, RGB LED (WS2812B), start module, motor current monitoring, servo output, IR pin access and flag deployment mechanism.

The library is optimized for **Arduino Nano (ATmega328P, 16 MHz)** and designed around **real Mini Sumo competition requirements**: fast reaction time, deterministic behavior and clear, safe abstractions.

---

## 🚀 Features

* ✅ Percentage-based motor control (`-100 … +100`)
* ✅ Ultra-fast **front sensor bitmask reading** (direct port access)
* ✅ Line sensor reading (bitmask + single sensor access)
* ✅ Sensor logic inversion and pull-up control
* ✅ Motor current monitoring (left & right)
* ✅ WS2812B RGB LED control (no external libraries)
* ✅ LED blinking utilities (debug / status)
* ✅ Competition start module support
* ✅ IR sensor pin access for IRremote
* ✅ Servo output and automatic flag deployment

---

## 📦 Installation

### Arduino Library Manager (recommended)

1. Open **Arduino IDE**
2. Go to **Sketch → Include Library → Manage Libraries…**
3. Search for **Vexor**
4. Click **Install**

---

### Manual Installation

1. Download or clone this repository
2. Copy it to:

```
Documents/Arduino/libraries/Vexor
```

3. Restart Arduino IDE
4. Include the library:

```cpp
#include <Vexor.h>
```

---

## ⚙️ Initialization

### `Vexor()`

Creates a Vexor controller instance.

---

### `void begin()`

Initializes **all hardware resources**:

* sensor pins
* motors
* RGB LED
* start module
* servo & flag output

📌 **Must be called in `setup()` before any other method.**

---

## 🧲 Sensor API

### `enum Sensor`

Defines all available sensors.

```
FL90, FL45, CC, FR45, FR90, LINE_LEFT, LINE_RIGHT
```

---

### `uint8_t readFrontSensors()`

Reads **all front opponent sensors at once** using direct port access.

* Returns a **5-bit bitmask**
* Extremely fast (~0.4 µs)
* Ideal for competition logic

| Bit | Sensor |
| --: | ------ |
|   0 | FL90   |
|   1 | FL45   |
|   2 | CC     |
|   3 | FR45   |
|   4 | FR90   |

#### Example return values

* `0b00100` → opponent detected by **center sensor only**
* `0b11111` → opponent detected by **all sensors**

#### Example: switch-based logic

```cpp
switch (robot.readFrontSensors()) {
  case 0b00100: robot.motor(100, 100); break;   // straight
  case 0b01100: robot.motor(60, 100);  break;   // slight left
  case 0b00110: robot.motor(100, 60);  break;   // slight right
  default:      robot.motor(80, 80);   break;
}
```

---

### `uint8_t readLineSensors()`

Reads both line sensors as a **2-bit bitmask**.

| Bit | Sensor     |
| --: | ---------- |
|   0 | LINE_LEFT  |
|   1 | LINE_RIGHT |

#### Example

* `0b01` → left line detected
* `0b10` → right line detected

```cpp
if (robot.readLineSensors() == 0b01) {
  robot.motor(-80, 80);
}
```

---

### `bool readSensor(Sensor s)`

Reads a **single sensor state**.

* Uses `digitalRead()`
* Slower (~4–5 µs)
* Automatically applies sensor logic reversal for **front sensors only**

#### When to use

* Simple conditions
* Debugging
* Educational code

#### Example

```cpp
if (robot.readSensor(CC)) {
  robot.motor(100, 100);
}
```

---

### `void sensorPullUp(bool state)`

Enables or disables **internal pull-up resistors** for **front sensors**.

* `true` → INPUT_PULLUP
* `false` → INPUT

✔ Recommended when using inverted sensor logic.

---

### `void reverseSensorSignal(bool state)`

Inverts **front sensor logic only**.

* `true` → logical inversion
* `false` → normal logic

Useful for sensors with active-low output.

---

## 🏎️ Motor Control

### `void motor(int8_t leftPercent, int8_t rightPercent)`

Controls both motors using **percentage values**.

* Range: `-100 … +100`
* Positive → forward
* Negative → backward

---

### `void stop()`

Immediately stops both motors.

---

### `void reverseLeftMotor(bool state)`

### `void reverseRightMotor(bool state)`

Reverses motor direction in software.

Useful when motor wiring differs between robots.

---

### `void motorEnable()`

### `void motorDisable()`

Manually enables or disables motors **only when start module is disabled**.

---

## 🔋 Motor Current Monitoring

### `float leftMotorCurrent()`

### `float rightMotorCurrent()`

Returns motor current in **Amperes (A)**.

Used for stall detection, diagnostics and research.

---

## 🌈 RGB LED Control

### `void setRGB(uint8_t r, uint8_t g, uint8_t b)`

Sets LED color using raw RGB values.

---

### `void setRed()` / `setGreen()` / `setBlue()` / `setWhite()`

Sets predefined colors with optional brightness.

---

### `void setColor(LedColor color, uint8_t brightness = 255)`

Sets LED using predefined color enum.

---

### `void setHexColor(uint32_t hexColor, uint8_t brightness = 255)`

Sets LED color using HEX format (`0xRRGGBB` or `0xRGB`).

---

### `void ledOff()`

Turns the RGB LED off.

---

### `void ledBlink(uint16_t intervalMS, int count, LedColor color, uint8_t brightness)`

### `void ledBlinkHex(uint16_t intervalMS, int count, uint32_t hexColor, uint8_t brightness)`

Blocks execution and blinks LED for debugging or status indication.

---

## 🕹️ Start Module & IR

### `bool startSignal()`

Reads competition start signal.

Returns `true` when start is active.

---

### `void useStartModule(bool state)`

Enables or disables start module logic.

* `true` → competition mode
* `false` → manual motor enable

---

### `uint8_t getIRPin()`

Returns the Arduino pin number used for the IR sensor.

Designed for use with external libraries such as **IRremote**.

---

## 🚩 Servo & Flag

### `uint8_t getServoPin()`

Returns the Arduino pin used for servo control.

---

### `void deployFlag(uint16_t durationMS = 100)`

Deploys the flag using an N20 motor.

* One-time activation
* Protected against re-triggering

---

## 📂 Examples

The `examples/` folder contains ready-to-use sketches demonstrating all core features of the Vexor library.

### 🔍 Sensors
- **Sensor_Basics.ino**  
  Basic individual sensor reading using `readSensor()`.

- **Front_Sensor_Bitmask.ino**  
  Fast front sensor reading using a 5-bit bitmask with binary logic.  
  Demonstrates how to use binary expressions (e.g. `0b11010`) together with `switch-case`
  for efficient opponent position detection.

- **Line_Sensors_Bitmask.ino**  
  Line sensor reading using a 2-bit bitmask and `switch-case` logic for quick edge detection.

### ⚡ Motors
- **Motor_With_StartModule.ino**  
  Motor control with competition start module enabled.  
  Motors are activated only after a valid start signal is received.

- **Motor_Without_StartModule.ino**  
  Manual motor control without start module logic.  
  Useful for testing, tuning, and development outside competitions.

### 🎮 IR & Start Module
- **IR_Remote_Basic.ino**  
  Simple IR remote example using the **IRremote** library.  
  Demonstrates how to access the IR receiver pin via `getIRPin()`.

### 💡 RGB LED
- **LED_Colors.ino**  
  Predefined RGB colors and brightness control.

- **LED_Blinking.ino**  
  LED blinking effects with adjustable color and timing.

### 🚩 Mechanisms
- **Flag_Deploy.ino**  
  Automatic flag deployment using the onboard N20 motor.

### 🔋 Monitoring
- **Current_Measurement.ino**  
  Real-time motor current measurement and monitoring.

---

## 🏆 Designed For

* Mini Sumo robots
* Robotics competitions
* Educational robotics
* Deterministic embedded systems

---

## 👤 Author

**LithuanianBots**
🌐 [https://lithuanianbots.com](https://lithuanianbots.com)

---

## 📜 License

MIT License — free to use, modify and distribute.

---

> Built for competition. Designed for clarity. Optimized for speed.
