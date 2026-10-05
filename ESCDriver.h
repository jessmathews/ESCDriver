// ESC.h - Hardware-PWM ESC output driver for RP2350 (arduino-pico core)
//
// - Pure hardware PWM: zero CPU load, zero jitter, no interrupts
// - Runtime-adjustable refresh rate (Standard PWM, Oneshot125/42, Multishot, custom)
// - Slice divider / wrap auto-computed for the finest possible pulse resolution
// - All motor slices phase-aligned and started on the same clock edge
// - Outputs forced LOW while reconfiguring (a frozen-high PWM pin = full throttle!)
// - Pulse clamping + optional failsafe timeout
#pragma once
#include <Arduino.h>

enum class ESCProtocol : uint8_t {
  STANDARD,    // 1000-2000 us, up to ~400 Hz
  ONESHOT125,  // 125-250 us,   up to ~3.6 kHz
  ONESHOT42,   // 42-84 us,     up to ~10 kHz
  MULTISHOT,   // 5-25 us,      up to ~36 kHz
  CUSTOM       // set range with setPulseRange()
};

class ESCDriver {
public:
  static constexpr uint8_t MAX_MOTORS = 8;

  // pins = GPIO numbers. Two pins may share a slice (A/B channel) but not the same channel.
  // Outputs start at the minimum pulse immediately so the ESC sees a valid "zero throttle".
  bool  begin(const uint8_t *pins, uint8_t count,
              ESCProtocol proto = ESCProtocol::STANDARD, float rateHz = 400.0f);
  void  end();                                  // outputs driven LOW, PWM slices stopped

  bool  setRefreshRate(float hz);               // do this while DISARMED (brief output gap)
  float getRefreshRate() const { return _actualHz; }   // actual achieved rate
  float getTickNs() const { return _ticksPerUs > 0 ? 1000.0f / _ticksPerUs : 0; }
  bool  setPulseRange(float minUs, float maxUs);

  void  writeMicroseconds(uint8_t motor, float us);
  void  writeThrottle(uint8_t motor, float t);  // t = 0.0 .. 1.0
  void  writeAllMicroseconds(float us);
  void  writeAllThrottle(float t);
  void  stop();                                 // all motors -> minimum pulse

  // Failsafe: if no write for timeoutMs, all motors go to minimum. 0 = disabled.
  void  setFailsafeTimeout(uint32_t timeoutMs) { _failsafeMs = timeoutMs; }
  void  service();                              // call every loop()

  float minUs() const { return _minUs; }
  float maxUs() const { return _maxUs; }

private:
  bool  configure(float hz);
  void  applyLevel(uint8_t m);
  void  markWrite();
  static void forceLow(uint8_t pin);

  uint8_t  _pins[MAX_MOTORS];
  uint8_t  _slice[MAX_MOTORS];
  uint8_t  _chan[MAX_MOTORS];
  float    _pulseUs[MAX_MOTORS];
  uint8_t  _count = 0;
  uint32_t _sliceMask = 0;

  float    _minUs = 1000, _maxUs = 2000;
  float    _ticksPerUs = 0;
  float    _actualHz = 0;
  uint16_t _top = 0;

  uint32_t _failsafeMs = 0;
  uint32_t _lastWriteMs = 0;
  bool     _failsafeActive = false;
};