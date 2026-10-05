#include "ESCDriver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/pwm.h"

void ESCDriver::forceLow(uint8_t pin) {
  gpio_put(pin, 0);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_set_function(pin, GPIO_FUNC_SIO);   // take pin away from PWM, drive low
}

bool ESCDriver::begin(const uint8_t *pins, uint8_t count, ESCProtocol proto, float rateHz) {
  if (count == 0 || count > MAX_MOTORS) return false;
  end();

  uint32_t usedChan = 0;   // bit (slice*2 + chan)
  for (uint8_t i = 0; i < count; i++) {
    if (pins[i] >= NUM_BANK0_GPIOS) return false;
    uint8_t s = pwm_gpio_to_slice_num(pins[i]);
    uint8_t c = pwm_gpio_to_channel(pins[i]);
    uint32_t bit = 1u << (s * 2 + c);
    if (usedChan & bit) return false;      // two motors on the same PWM channel
    usedChan |= bit;
    _pins[i] = pins[i]; _slice[i] = s; _chan[i] = c;
    _sliceMask |= 1u << s;
  }
  _count = count;

  switch (proto) {
    case ESCProtocol::STANDARD:   _minUs = 1000; _maxUs = 2000; break;
    case ESCProtocol::ONESHOT125: _minUs = 125;  _maxUs = 250;  break;
    case ESCProtocol::ONESHOT42:  _minUs = 42;   _maxUs = 84;   break;
    case ESCProtocol::MULTISHOT:  _minUs = 5;    _maxUs = 25;   break;
    case ESCProtocol::CUSTOM:     break;   // keeps previous / default 1000-2000
  }
  for (uint8_t i = 0; i < count; i++) _pulseUs[i] = _minUs;

  if (!configure(rateHz)) { end(); return false; }
  _lastWriteMs = millis();
  _failsafeActive = false;
  return true;
}

void ESCDriver::end() {
  if (_count) {
    hw_clear_bits(&pwm_hw->en, _sliceMask);
    for (uint8_t i = 0; i < _count; i++) forceLow(_pins[i]);
  }
  _count = 0; _sliceMask = 0; _actualHz = 0; _ticksPerUs = 0;
}

// Computes divider/wrap, validates, then (re)programs hardware.
// Nothing on the hardware is touched unless the rate is valid.
bool ESCDriver::configure(float hz) {
  if (!_count || hz < 10.0f) return false;
  // Frame must fit the longest pulse plus a 10% gap for the ESC to see an edge
  if ((1e6f / hz) < _maxUs * 1.1f) return false;

  const float clk = (float)clock_get_hz(clk_sys);

  // Smallest divider (best resolution) that keeps wrap within 16 bits.
  float div = clk / (hz * 65536.0f);
  if (div < 1.0f) div = 1.0f;
  div = ceilf(div * 16.0f) / 16.0f;            // divider has 4 fractional bits
  if (div > 255.9375f) return false;           // rate too low (< ~9 Hz at 150 MHz)

  uint32_t period = (uint32_t)(clk / (div * hz) + 0.5f);
  if (period > 65536) period = 65536;
  if (period < 2) return false;

  // --- validated; now touch hardware ---
  for (uint8_t i = 0; i < _count; i++) forceLow(_pins[i]);
  hw_clear_bits(&pwm_hw->en, _sliceMask);

  _top        = (uint16_t)(period - 1);
  _ticksPerUs = clk / div / 1e6f;
  _actualHz   = clk / (div * (float)period);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, div);
  pwm_config_set_wrap(&cfg, _top);
  for (uint s = 0; s < NUM_PWM_SLICES; s++)
    if (_sliceMask & (1u << s)) pwm_init(s, &cfg, false);

  for (uint8_t i = 0; i < _count; i++) applyLevel(i);

  // Park counters at TOP (>= any compare value) so the output is LOW when the
  // pin is handed to the PWM block; all slices then start from the same phase.
  for (uint s = 0; s < NUM_PWM_SLICES; s++)
    if (_sliceMask & (1u << s)) pwm_set_counter(s, _top);
  for (uint8_t i = 0; i < _count; i++) gpio_set_function(_pins[i], GPIO_FUNC_PWM);

  hw_set_bits(&pwm_hw->en, _sliceMask);        // atomic start, other slices untouched
  return true;
}

bool ESCDriver::setRefreshRate(float hz) { return configure(hz); }

bool ESCDriver::setPulseRange(float minUs, float maxUs) {
  if (minUs <= 0 || maxUs <= minUs) return false;
  if (_actualHz > 0 && (1e6f / _actualHz) < maxUs * 1.1f) return false;
  _minUs = minUs; _maxUs = maxUs;
  for (uint8_t i = 0; i < _count; i++) {
    _pulseUs[i] = constrain(_pulseUs[i], _minUs, _maxUs);
    applyLevel(i);
  }
  return true;
}

void ESCDriver::applyLevel(uint8_t m) {
  uint32_t lvl = (uint32_t)(_pulseUs[m] * _ticksPerUs + 0.5f);
  if (lvl > _top) lvl = _top;
  pwm_set_chan_level(_slice[m], _chan[m], (uint16_t)lvl);
}

void ESCDriver::markWrite() { _lastWriteMs = millis(); _failsafeActive = false; }

void ESCDriver::writeMicroseconds(uint8_t m, float us) {
  if (m >= _count) return;
  _pulseUs[m] = constrain(us, _minUs, _maxUs);
  applyLevel(m);
  markWrite();
}

void ESCDriver::writeThrottle(uint8_t m, float t) {
  t = constrain(t, 0.0f, 1.0f);
  writeMicroseconds(m, _minUs + t * (_maxUs - _minUs));
}

void ESCDriver::writeAllMicroseconds(float us) {
  us = constrain(us, _minUs, _maxUs);
  for (uint8_t i = 0; i < _count; i++) { _pulseUs[i] = us; applyLevel(i); }
  markWrite();
}

void ESCDriver::writeAllThrottle(float t) {
  t = constrain(t, 0.0f, 1.0f);
  writeAllMicroseconds(_minUs + t * (_maxUs - _minUs));
}

void ESCDriver::stop() { writeAllMicroseconds(_minUs); }

void ESCDriver::service() {
  if (_failsafeMs && !_failsafeActive && (millis() - _lastWriteMs) > _failsafeMs) {
    for (uint8_t i = 0; i < _count; i++) { _pulseUs[i] = _minUs; applyLevel(i); }
    _failsafeActive = true;
  }
}