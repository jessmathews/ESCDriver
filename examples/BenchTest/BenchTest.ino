/*
  BenchTest.ino - Interactive Bench Test & Telemetry for ESCDriver
  Part of ESC_PWM_RP2350 library

  Target: Raspberry Pi RP2350 (Raspberry Pi Pico 2 or compatible)
  Core:   Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower III (arduino-pico)

  =============================================================================
  !!! SAFETY WARNING !!!
  ALWAYS REMOVE PROPELLERS BEFORE BENCH TESTING MOTORS AND ESCS!
  Motors can spin up suddenly and cause severe physical injury or equipment damage.
  =============================================================================

  Serial Monitor (115200 baud, newline enabled):
    Commands:
      t<val>      - Set throttle for ALL motors (e.g., t0.10 for 10%)
      m<idx> <v>  - Set throttle for ONE motor (e.g., m0 0.15)
      u<idx> <us> - Set pulse width in microseconds (e.g., u0 1150)
      s           - Emergency Stop (throttle 0 / minimum pulse)
      r<hz>       - Change refresh rate (e.g., r400, r1000)
      ? or h      - Print telemetry and help menu
*/

#include <Arduino.h>
#include <ESCDriver.h>

// --- Configuration ---
constexpr ESCProtocol PROTOCOL    = ESCProtocol::STANDARD;  // STANDARD, ONESHOT125, ONESHOT42, MULTISHOT
constexpr float       REFRESH_HZ  = 400.0f;                 // Refresh frequency in Hz
constexpr float       SAFETY_CAP  = 0.50f;                  // Maximum allowed bench throttle (50%)
constexpr uint32_t    FAILSAFE_MS = 250;                    // Failsafe timeout in milliseconds

// Pin assignments: RP2350 GPIO 0, 1, 2, 3 (Slice 0 A/B, Slice 1 A/B)
// Modify this array to match your board wiring.
constexpr uint8_t MOTOR_COUNT = 4;
const uint8_t MOTOR_PINS[MOTOR_COUNT] = {0, 1, 2, 3};

ESCDriver esc;

// Print telemetry and status information
void printStatus() {
  Serial.println("\n--- ESC Telemetry & Status ---");
  Serial.printf("Configured Motors:  %d\n", MOTOR_COUNT);
  Serial.printf("Refresh Rate:       %.1f Hz (Actual: %.2f Hz)\n", REFRESH_HZ, esc.getRefreshRate());
  Serial.printf("Timer Resolution:   %.2f ns/tick\n", esc.getTickNs());
  Serial.printf("Pulse Range:        %.0f us to %.0f us\n", esc.minUs(), esc.maxUs());
  Serial.printf("Failsafe Timeout:   %lu ms\n", FAILSAFE_MS);
  Serial.printf("Safety Throttle Cap:%.0f%%\n", SAFETY_CAP * 100.0f);
  Serial.println("------------------------------");
  Serial.println("Available Commands:");
  Serial.println("  t<0.0-1.0>  - Set throttle for all motors (e.g. 't0.10')");
  Serial.println("  m<m> <val>  - Set throttle for motor m (e.g. 'm0 0.15')");
  Serial.println("  u<m> <us>   - Set microseconds for motor m (e.g. 'u0 1200')");
  Serial.println("  s           - Stop all motors immediately");
  Serial.println("  r<hz>       - Set new refresh rate in Hz (e.g. 'r400')");
  Serial.println("  ? or h      - Print this help menu\n");
}

void setup() {
  Serial.begin(115200);

  // Wait for USB serial connection if desired (timeout 3 seconds)
  uint32_t startMs = millis();
  while (!Serial && (millis() - startMs < 3000)) {
    delay(10);
  }

  Serial.println("\n========================================");
  Serial.println("   ESC_PWM_RP2350 - Bench Test Utility  ");
  Serial.println("========================================");
  Serial.println("WARNING: Ensure all propellers are REMOVED!");

  // Initialize ESC hardware PWM outputs
  if (!esc.begin(MOTOR_PINS, MOTOR_COUNT, PROTOCOL, REFRESH_HZ)) {
    Serial.println("[-] ERROR: ESC initialization failed!");
    Serial.println("    Check for pin channel conflicts or invalid refresh rate.");
    while (true) {
      delay(1000);
    }
  }

  // Set failsafe timeout (motors drop to min pulse if no updates occur)
  esc.setFailsafeTimeout(FAILSAFE_MS);

  printStatus();

  // Allow ESCs to complete initialization beeps at minimum pulse
  Serial.print("Arming ESCs at zero throttle");
  for (int i = 0; i < 6; i++) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" READY!");
  Serial.println("Type 't0.05' to test 5% throttle or 's' to stop.\n");
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char type = cmd[0];

  switch (type) {
    case 't': {
      // All motors throttle: t0.15
      float val = cmd.substring(1).toFloat();
      if (val > SAFETY_CAP) {
        Serial.printf("[!] Capping throttle to bench limit %.2f\n", SAFETY_CAP);
        val = SAFETY_CAP;
      }
      esc.writeAllThrottle(val);
      Serial.printf("[OK] All motors throttle: %.2f (%.1f%%)\n", val, val * 100.0f);
      break;
    }

    case 'm': {
      // Single motor throttle: m0 0.15
      int spaceIdx = cmd.indexOf(' ');
      if (spaceIdx > 1) {
        uint8_t m = cmd.substring(1, spaceIdx).toInt();
        float val = cmd.substring(spaceIdx + 1).toFloat();
        if (m < MOTOR_COUNT) {
          if (val > SAFETY_CAP) {
            Serial.printf("[!] Capping throttle to bench limit %.2f\n", SAFETY_CAP);
            val = SAFETY_CAP;
          }
          esc.writeThrottle(m, val);
          Serial.printf("[OK] Motor %d throttle: %.2f\n", m, val);
        } else {
          Serial.printf("[-] Invalid motor index: %d (max: %d)\n", m, MOTOR_COUNT - 1);
        }
      } else {
        Serial.println("[-] Usage: m<index> <throttle>, e.g. m0 0.10");
      }
      break;
    }

    case 'u': {
      // Single motor microsecond pulse: u0 1150
      int spaceIdx = cmd.indexOf(' ');
      if (spaceIdx > 1) {
        uint8_t m = cmd.substring(1, spaceIdx).toInt();
        float us = cmd.substring(spaceIdx + 1).toFloat();
        if (m < MOTOR_COUNT) {
          esc.writeMicroseconds(m, us);
          Serial.printf("[OK] Motor %d pulse: %.1f us\n", m, us);
        } else {
          Serial.printf("[-] Invalid motor index: %d\n", m, MOTOR_COUNT - 1);
        }
      } else {
        Serial.println("[-] Usage: u<index> <microseconds>, e.g. u0 1200");
      }
      break;
    }

    case 's': {
      // Emergency stop
      esc.stop();
      Serial.println("[STOP] All motors set to minimum pulse.");
      break;
    }

    case 'r': {
      // Change refresh rate
      float newHz = cmd.substring(1).toFloat();
      esc.stop(); // Safe practice: drop to idle before reconfiguring
      if (esc.setRefreshRate(newHz)) {
        Serial.printf("[OK] Refresh rate changed to %.1f Hz (Actual: %.2f Hz, %.2f ns/tick)\n",
                      newHz, esc.getRefreshRate(), esc.getTickNs());
      } else {
        Serial.printf("[-] Refresh rate %.1f Hz rejected (exceeds pulse envelope)\n", newHz);
      }
      break;
    }

    case '?':
    case 'h':
      printStatus();
      break;

    default:
      Serial.printf("[-] Unknown command '%c'. Type '?' for help.\n", type);
      break;
  }
}

void loop() {
  // Process incoming user commands from the Serial Monitor
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    handleSerialCommand(input);
  }

  // Service failsafe watchdog timer on each cycle
  esc.service();

  delay(2);
}
