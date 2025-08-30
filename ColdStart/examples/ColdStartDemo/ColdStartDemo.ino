#include <ColdStart.h>

#define BUZZER_PIN 5

ColdStart cold(BUZZER_PIN);

void setup() {
  Serial.begin(115200);

  // Simulate checks (later you’ll call actual module check functions)
  cold.checkIMU(false);    // IMU NOT connected
  cold.checkESP32(true);   // ESP32 OK
  cold.checkPower(true);   // Power OK

  cold.runStartupCheck();
}

void loop() {
  // Your main program here
}