#ifndef COLDSTART_H
#define COLDSTART_H

#include <Arduino.h>
#include <Wire.h>

// ---- Error codes for beep patterns ----
enum CS_Error : uint8_t {
  CS_OK         = 0,
  CS_IMU_FAIL   = 1,   // 1 beep
  CS_ESP_FAIL   = 2,   // 2 beeps
  CS_POWER_FAIL = 3,   // 3 beeps
  CS_MOTOR_FAIL = 4    // 4 beeps
};

class ColdStart {
public:
  // motors: array of pins, motorCount: length of array
  ColdStart(int buzzerPin, const int* motorPins, uint8_t motorCount);

  // Optional configs (call the ones you need)
  void setImuAddress(uint8_t addr0x68_or_0x69 = 0x68); // ICM-20948 on 0x68/0x69
  void setEspSerial(HardwareSerial* port, uint32_t baud = 115200);
  // Battery monitor: adcPin is Teensy analog pin, vDivRatio = Vbattery / Vadc (e.g., 2.0 if divider halves voltage)
  void setBatteryMonitor(int adcPin, float vDivRatio, float vRef = 3.3f, float lowVoltThresh = 3.50f);

  void begin();             // init pins, Wire, serial (if set)
  bool runStartupCheck();   // runs all checks; returns true if all OK
  CS_Error lastError() const { return _lastError; }

private:
  // Pins & hardware
  int _buzzerPin;
  const int* _motorPins;
  uint8_t _motorCount;

  // IMU
  bool _imuEnabled = true;
  uint8_t _imuAddr = 0x68;           // try 0x68, fallback 0x69 inside
  static const uint8_t ICM20948_WHO_AM_I_REG = 0x00; // should read 0xEA

  // ESP32 link
  bool _espEnabled = false;
  HardwareSerial* _espSerial = nullptr;
  uint32_t _espBaud = 115200;

  // Power/Battery
  bool _pwrEnabled = false;
  int _batAdcPin = -1;
  float _vDivRatio = 2.0f;   // divider ratio (Vbat / Vadc)
  float _vRef = 3.3f;        // Teensy analog reference voltage
  float _lowVolt = 3.50f;    // per-cell low threshold (V); adjust for your pack
  uint8_t _batteryCells = 1; // set 1 for 1S micro builds; change if using 2S/3S, etc.

  // State
  CS_Error _lastError = CS_OK;

  // Checks
  bool checkIMU();
  bool checkESP();
  bool checkPower();
  bool checkMotors();

  // Utilities
  void beep(uint16_t onMs, uint16_t offMs);
  void beepPatternOnce(uint8_t count, uint16_t gapMs);
  void beepPatternLoop(CS_Error err);
  void successChirp();

  // I2C helpers
  bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t& val);
};

#endif
