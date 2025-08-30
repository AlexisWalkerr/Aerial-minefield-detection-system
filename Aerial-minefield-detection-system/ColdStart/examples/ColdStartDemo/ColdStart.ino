#include <ColdStart.h>

// ---------- USER WIRING (EDIT THESE) ----------
// Buzzer pin
const int BUZZER_PIN = 13; // Teensy onboard LED pin can drive a piezo via transistor; change to your buzzer pin

// Motor driver pins (through MOSFET/driver, NEVER direct to motor)
const int MOTOR_PINS[] = {2, 3, 4, 5}; // edit for your PCB
const uint8_t MOTOR_COUNT = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);

// ESP32 serial link (optional). Teensy 4.1 has Serial1/2/3/4/5/6/7.
HardwareSerial* ESP_PORT = &Serial1; // set nullptr to disable

// Battery monitor (optional)
// Example: 1S LiPo via divider 2:1 → vDivRatio = 2.0; threshold 3.5V per cell
const int BAT_ADC_PIN = A1;          // set -1 to disable
const float VDIV_RATIO = 2.0f;
const float VREF = 3.3f;             // Teensy analog ref
const float LOW_PER_CELL = 3.50f;    // per-cell low threshold

// ---------- LIB INSTANCE ----------
ColdStart cs(BUZZER_PIN, MOTOR_PINS, MOTOR_COUNT);

void setup() {
  // Serial for logs (optional)
  Serial.begin(115200);
  delay(50);
  Serial.println("ColdStart demo");

  // Configure optional modules
  cs.setImuAddress(0x68);                 // ICM-20948 expected on 0x68 (tries 0x69 if needed)
  if (ESP_PORT) cs.setEspSerial(ESP_PORT, 115200);
  cs.setBatteryMonitor(BAT_ADC_PIN, VDIV_RATIO, VREF, LOW_PER_CELL);

  cs.begin();

  // Run all checks (blocks with beep loop on failure)
  bool ok = cs.runStartupCheck();
  if (ok) Serial.println("All checks PASSED.");
}

void loop() {
  // normal flight code would continue here after a successful cold start
}
