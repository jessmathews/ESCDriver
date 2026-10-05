# ESCDriver

[![Platform: RP2350](https://img.shields.io/badge/Platform-RP2350%20%2F%20Pico%202-blue.svg)](https://www.raspberrypi.com/products/raspberry-pi-pico-2/)
[![Framework: Arduino-Pico](https://img.shields.io/badge/Framework-arduino--pico-green.svg)](https://github.com/earlephilhower/arduino-pico)
[![Standard: C++17](https://img.shields.io/badge/Standard-C%2B%2B17-orange.svg)]()
[![Hardware: Zero CPU](https://img.shields.io/badge/CPU%20Load-0%25%20(Pure%20Hardware)-brightgreen.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A high-performance, glitch-free hardware PWM Electronic Speed Controller (ESC) driver for the **Raspberry Pi RP2350** (Raspberry Pi Pico 2 and RP2350 derivatives) using the [`arduino-pico`](https://github.com/earlephilhower/arduino-pico) core.

Unlike software PWM or timer-interrupt-driven libraries that suffer from CPU overhead, jitter, and interrupt latency, **ESCDriver** drives ESC outputs directly via the RP2350's dedicated hardware PWM slices. Waveforms are generated purely in silicon with sub-nanosecond precision, zero CPU load, and guaranteed phase alignment.

---

## Table of Contents

- [Features](#features)
- [Supported Protocols](#supported-protocols)
- [Hardware Architecture & Pin Rules](#hardware-architecture--pin-rules)
- [Recommended Pin Mappings](#recommended-pin-mappings)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Bench Test Example](#bench-test-example)
- [Repository Layout](#repository-layout)
- [API Reference](#api-reference)
  - [Initialization & Lifecycle](#initialization--lifecycle)
  - [Throttle & Output Control](#throttle--output-control)
  - [Timing & Refresh Rate](#timing--refresh-rate)
  - [Safety & Failsafe](#safety--failsafe)
- [Safety Features](#safety-features)
- [Wiring Guide](#wiring-guide)
- [License](#license)

---

## Features

- **Pure Hardware Generation**: Zero CPU cycles consumed during signal generation. Zero pulse jitter. No interrupts.
- **Multiple Protocols**: Out-of-the-box support for Standard PWM, Oneshot125, Oneshot42, Multishot, and custom pulse ranges.
- **Up to 8 Motors**: Drive quadcopters, hexacopters, octocopters, or custom robotic actuators concurrently.
- **Sub-Nanosecond Timer Resolution**: Auto-calculates optimal hardware clock dividers and 16-bit counter wrap (`TOP`) values to maximize timing precision.
- **Synchronous Phase Alignment**: Counters for all active motor slices are parked at `TOP` and enabled atomically on the exact same clock edge, ensuring pulses across all motors fire in perfect lockstep.
- **Hardware Glitch Protection**: Forces GPIO pins directly LOW via Software I/O (`SIO`) before reconfiguring PWM registers. Prevents frozen-HIGH transients (which an ESC would interpret as full throttle).
- **Built-in Watchdog Failsafe**: Automatic motor cut-off to minimum pulse (`minUs`) if no valid throttle command is received within a user-defined timeout.
- **Pulse Clamping**: Microsecond inputs are strictly constrained to protocol bounds; normalized throttle inputs (`0.0` – `1.0`) are clamped to prevent illegal frame widths.

---

## Supported Protocols

| Protocol | Pulse Width | Typical Refresh Rate | Max Supported Rate | Description |
| :--- | :---: | :---: | :---: | :--- |
| `ESCProtocol::STANDARD` | 1000 – 2000 µs | 50 – 400 Hz | ~400 Hz | Standard hobby PWM; compatible with virtually all ESCs and servos. |
| `ESCProtocol::ONESHOT125` | 125 – 250 µs | 1.0 – 3.0 kHz | ~3.6 kHz | 8× faster than standard PWM; widely supported by BLHeli / AM32 ESCs. |
| `ESCProtocol::ONESHOT42` | 42 – 84 µs | 4.0 – 8.0 kHz | ~10.0 kHz | Low-latency protocol for high-rate flight controllers. |
| `ESCProtocol::MULTISHOT` | 5 – 25 µs | 8.0 – 32.0 kHz | ~36.0 kHz | Ultra-fast analog protocol for racing drones and high-speed telemetry. |
| `ESCProtocol::CUSTOM` | Custom | Configurable | $f \le \frac{10^6}{\text{maxUs} \times 1.1}$ | Custom range set with `setPulseRange(minUs, maxUs)`. |

> [!NOTE]
> Every protocol requires at least a 10% inter-frame dead band gap between pulses (`(1e6 / rateHz) >= maxUs * 1.1`) so that the ESC microcontroller can reliably detect the falling edge of each pulse.

---

## Hardware Architecture & Pin Rules

The RP2350 PWM hardware consists of independent slices. Each slice has two output channels: **Channel A** and **Channel B**.

Each GPIO pin on the RP2350 maps to a specific slice and channel:
$$\text{Slice} = (\text{GPIO} / 2) \pmod{N_{\text{slices}}}$$
$$\text{Channel} = \text{GPIO} \pmod 2 \quad (\text{0 = Channel A, 1 = Channel B})$$

### Pin Selection Rules

1. **Two pins CAN share the same slice** if one is **Channel A** and the other is **Channel B** (e.g., `GPIO 0` and `GPIO 1`).
2. **Two pins CANNOT share the same channel on the same slice** (e.g., `GPIO 0` and `GPIO 16` both map to Slice 0 Channel A).
3. `begin()` automatically checks your pin array for channel conflicts and will reject invalid mappings before touching any hardware.

---

## Recommended Pin Mappings

Consecutive even/odd pin pairs always provide conflict-free Channel A / Channel B assignments:

| Motor Count | Recommended GPIOs | Slices Used | Channels Used |
| :--- | :--- | :--- | :--- |
| **Dual Motor (2)** | `0, 1` | Slice 0 | 0A, 0B |
| **Quadcopter (4)** | `0, 1, 2, 3` | Slices 0, 1 | 0A, 0B, 1A, 1B |
| **Hexacopter (6)** | `0, 1, 2, 3, 4, 5` | Slices 0, 1, 2 | 0A, 0B, 1A, 1B, 2A, 2B |
| **Octocopter (8)** | `0, 1, 2, 3, 4, 5, 6, 7` | Slices 0, 1, 2, 3 | 0A, 0B, 1A, 1B, 2A, 2B, 3A, 3B |

---

## Installation

### Requirements

- **Microcontroller**: Raspberry Pi RP2350 (Raspberry Pi Pico 2, SparkFun RP2350, Adafruit RP2350, etc.)
- **Arduino Core**: [Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower III](https://github.com/earlephilhower/arduino-pico) (v4.0.0+ recommended for full RP2350 support)

### Option 1: Drop into Sketch Folder (Simplest)
Copy `ESCDriver.h` and `ESCDriver.cpp` directly into your Arduino sketch directory alongside your `.ino` file:
```
YourSketch/
├── YourSketch.ino
├── ESCDriver.h
└── ESCDriver.cpp
```

### Option 2: Install as Arduino Library
1. Clone or download this repository:
   ```bash
   git clone https://github.com/jessmathews/ESCDriver.git
   ```
2. Place the folder into your Arduino libraries directory:
   - **Windows**: `Documents/Arduino/libraries/ESCDriver`
   - **macOS / Linux**: `~/Arduino/libraries/ESCDriver`
3. Restart the Arduino IDE.

---

## Quick Start

```cpp
#include "ESCDriver.h"

// Define GPIO pins for 4 motors (Quadcopter)
const uint8_t MOTOR_PINS[4] = {0, 1, 2, 3};

ESCDriver esc;

void setup() {
  Serial.begin(115200);

  // Initialize 4 motors with Standard PWM at 400 Hz
  if (!esc.begin(MOTOR_PINS, 4, ESCProtocol::STANDARD, 400.0f)) {
    Serial.println("ESC initialization failed!");
    while (true) delay(1000);
  }

  // Set a 250 ms failsafe timeout
  esc.setFailsafeTimeout(250);

  // Hold at minimum pulse for 3 seconds to allow ESCs to initialize and arm
  delay(3000);
}

void loop() {
  // Set all motors to 15% throttle (0.15 on a 0.0 - 1.0 scale)
  esc.writeAllThrottle(0.15f);

  // Service failsafe watchdog timer
  esc.service();

  delay(5);
}
```

---

## Bench Test Example

A complete interactive bench test sketch is provided in [`examples/BenchTest/BenchTest.ino`].

When installed as an Arduino library, you can open it directly from the Arduino IDE menu:
**File → Examples → ESCDriver → BenchTest**

> [!CAUTION]
> **SAFETY FIRST: ALWAYS REMOVE ALL PROPELLERS BEFORE BENCH TESTING!**
> Brushless motors can spin up instantly and cause severe injury.

### Interactive Serial Commands (115200 baud)

Open the Arduino Serial Monitor (with newline enabled at 115200 baud) to control your motors and inspect hardware telemetry:

| Command | Action | Example |
| :--- | :--- | :--- |
| `t<float>` | Set throttle for **all** motors (clamped to safety cap) | `t0.10` (10% throttle) |
| `m<index> <float>` | Set throttle for an **individual** motor | `m0 0.15` (15% throttle on motor 0) |
| `u<index> <us>` | Set exact microsecond pulse for a motor | `u0 1150` (1150 µs on motor 0) |
| `s` | **Emergency Stop** (all motors to minimum pulse immediately) | `s` |
| `r<float>` | Change refresh rate in Hz (while disarmed) | `r400` or `r1000` |
| `?` or `h` | Print telemetry (actual Hz, tick ns, pulse range, pins) | `?` |

---

## Repository Layout

```
ESCDriver/
├── examples/
│   └── BenchTest/
│       └── BenchTest.ino       # Interactive multi-motor bench testing utility
├── ESCDriver.h                 # Driver API definitions & protocol enums
├── ESCDriver.cpp               # RP2350 hardware PWM configuration & pulse generation
├── library.properties          # Arduino IDE Library 1.5 specification manifest
├── LICENSE                     # MIT License
└── README.md                   # Complete documentation
```

---

## API Reference

### Initialization & Lifecycle

#### `bool begin(const uint8_t *pins, uint8_t count, ESCProtocol proto = ESCProtocol::STANDARD, float rateHz = 400.0f)`
Initializes the driver, validates pin assignments, configures the hardware PWM slices, and sets initial output levels to `minUs`.
- **`pins`**: Pointer to an array of RP2350 GPIO pin numbers.
- **`count`**: Number of motors (`1` to `MAX_MOTORS`, where `MAX_MOTORS = 8`).
- **`proto`**: Protocol selection (`STANDARD`, `ONESHOT125`, `ONESHOT42`, `MULTISHOT`, `CUSTOM`).
- **`rateHz`**: Output refresh rate in Hertz.
- **Returns**: `true` if configuration succeeded; `false` if invalid pins, channel conflict, or incompatible frequency.

#### `void end()`
Disables all PWM slices and forces all configured GPIO pins LOW via software control (`GPIO_FUNC_SIO`).

---

### Throttle & Output Control

#### `void writeThrottle(uint8_t motor, float t)`
Sets the throttle for an individual motor using a normalized float.
- **`motor`**: Motor index (`0` to `count - 1`).
- **`t`**: Throttle value from `0.0f` (idle/stop) to `1.0f` (full throttle). Automatically constrained.

#### `void writeAllThrottle(float t)`
Sets all configured motors to the same normalized throttle value (`0.0f` to `1.0f`).

#### `void writeMicroseconds(uint8_t motor, float us)`
Sets the pulse width for an individual motor in microseconds.
- **`motor`**: Motor index (`0` to `count - 1`).
- **`us`**: Pulse width in microseconds. Constrained to `[minUs, maxUs]`.

#### `void writeAllMicroseconds(float us)`
Sets all configured motors to the same pulse width in microseconds.

#### `void stop()`
Immediately drives all motors to their minimum pulse width (`minUs`).

---

### Timing & Refresh Rate

#### `bool setRefreshRate(float hz)`
Dynamically reconfigures the PWM refresh rate.
- **`hz`**: Desired frequency in Hz.
- **Returns**: `true` if valid and applied; `false` if the rate cannot accommodate the pulse range.
- *Note: Call this while the craft is disarmed, as outputs are momentarily forced LOW during slice reconfiguration.*

#### `float getRefreshRate() const`
Returns the actual achieved refresh rate in Hz (calculated from clock divider and TOP wrap registers).

#### `float getTickNs() const`
Returns the timer resolution in nanoseconds per clock tick. (Typically $\approx 6.67\text{ ns}$ to $13.33\text{ ns}$ at standard 150 MHz sys clock).

#### `bool setPulseRange(float minUs, float maxUs)`
Configures custom pulse range limits for `ESCProtocol::CUSTOM`.
- **`minUs`**: Minimum pulse duration in microseconds (e.g. `1000.0f`).
- **`maxUs`**: Maximum pulse duration in microseconds (e.g. `2000.0f`).
- **Returns**: `true` if valid for current refresh rate; `false` otherwise.

#### `float minUs() const` / `float maxUs() const`
Return the current minimum and maximum pulse bounds in microseconds.

---

### Safety & Failsafe

#### `void setFailsafeTimeout(uint32_t timeoutMs)`
Enables an automatic failsafe cutoff timer.
- **`timeoutMs`**: Maximum permissible delay in milliseconds between calls to `writeThrottle()`, `writeMicroseconds()`, or `writeAll*()`. Set to `0` to disable failsafe.

#### `void service()`
Monitors the elapsed time since the last output command. If the failsafe timeout has expired, all motor outputs are immediately forced to `minUs`.
- *Call this method regularly in your `loop()` or flight controller task.*

---

## Safety Features

### 1. Zero-Throttle Boot & Reconfiguration Protection
When switching pin modes or configuring PWM registers, a pin left floating or in a high state can trigger an ESC to enter calibration mode or spin at full throttle. `ESCDriver` addresses this:
1. All pins are switched to SIO outputs and pulled **LOW** before any PWM registers are altered.
2. Initial compare registers are primed with `minUs` pulses.
3. Counters are parked at `TOP` so output lines transition cleanly on the first cycle.

### 2. Atomic Multi-Slice Startup
Different motors may reside on different PWM slices. Slices are initialized individually and then atomically started together by writing to the hardware enable register:
```c
hw_set_bits(&pwm_hw->en, _sliceMask);
```
This guarantees zero phase offset between motor signals.

### 3. Failsafe Watchdog
If a communication bus stall, sensor error, or infinite loop halts your flight control loop, `service()` detects the absence of updates and drives all motors to idle/stop.

---

## Wiring Guide

Connect your ESCs to the Raspberry Pi Pico 2 / RP2350 as shown below:

```
+------------------------+             +------------------------+
|      RP2350 Board      |             |          ESC           |
|                        |             |                        |
|  GPIO 0 (Motor 1 PWM)  +------------>+ Signal                 |
|  GPIO 1 (Motor 2 PWM)  +------------>+ Signal (Motor 2)       |
|  GND                   +-------------+ Ground (Common GND)    |
+------------------------+             +-----------+------------+
                                                   |
                                            +------+-------+
                                            |  LiPo Power  |
                                            |  (e.g. 4S)   |
                                            +--------------+
```

> [!IMPORTANT]
> **Ground Reference**: Always connect the ground wire from the ESC signal lead to a GND pin on the RP2350 board. Missing a common ground reference leads to signal noise, missed frames, and erratic motor behavior.
>
> **ESC BEC / 5V**: If your ESC has an onboard Battery Eliminator Circuit (BEC) providing 5V, **do not** connect it to the RP2350's 3.3V pin. Connect BEC 5V only to `VBUS` / `VSYS` if you intend to power the board from the ESC, or leave it disconnected if powering via USB.

---

## License

This library is released under the [MIT License](LICENSE). Feel free to use, modify, and integrate it into your hobby or commercial projects.
