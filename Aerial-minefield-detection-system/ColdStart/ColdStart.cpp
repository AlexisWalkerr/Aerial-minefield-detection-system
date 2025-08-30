#include "ColdStart.h"

// ---------- Public API ----------
ColdStart::ColdStart(int buzzerPin, const int* motorPins, uint8_t motorCount)
: _buzzerPin(buzzerPin), _motorPins(motorPins), _motorCount(motorCount) {}

void ColdStart::setImuAddress(uint8_t addr0x68_or_0x69) {
  _imuAddr = addr0x68_or_0x69;
}

void ColdStart::setEspSerial(HardwareSerial* port, uint32_t baud) {
  _espSerial = port;
  _espBaud = baud;
  _espEnabled = (port != nullptr);
}

void ColdStart::setBatteryMonitor(int adcPin, float vDivRatio, float vRef, float lowVoltThresh) {
  _batAdcPin = adcPin;
  _vDivRatio = vDivRatio;
  _vRef = vRef;
  _lowVolt = lowVoltThresh;
  _pwrEnabled = (adcPin >= 0);
}

void ColdStart::begin() {
  pinMode(_buzzerPin, OUTPUT);
  digitalWrite(_buzzerPin, LOW);

  for (uint8_t i = 0; i < _motorCount; i++) {
    pinMode(_motorPins[i], OUTPUT);
    analogWrite(_motorPins[i], 0);
  }

  Wire.begin(); // Teensy default I2C pins

  if (_espEnabled && _espSerial) {
    _espSerial->begin(_espBaud);
    delay(50);
    while (_espSerial->available()) _espSerial->read(); // flush
  }

  if (_pwrEnabled && _batAdcPin >= 0) {
    pinMode(_batAdcPin, INPUT);
  }
}

bool ColdStart::runStartupCheck() {
  _lastError = CS_OK;

  if (_imuEnabled && !checkIMU()) {
    _lastError = CS_IMU_FAIL;
    beepPatternLoop(_lastError);
    return false;
  }

  if (_espEnabled && !checkESP()) {
    _lastError = CS_ESP_FAIL;
    beepPatternLoop(_lastError);
    return false;
  }

  if (_pwrEnabled && !checkPower()) {
    _lastError = CS_POWER_FAIL;
    beepPatternLoop(_lastError);
    return false;
  }

  if (!checkMotors()) {
    _lastError = CS_MOTOR_FAIL;
    beepPatternLoop(_lastError);
    return false;
  }

  successChirp();
  return true;
}

// ---------- Checks ----------
bool ColdStart::checkIMU() {
  // Try configured address; if not responding, try the alternate
  uint8_t who = 0;
  if (i2cReadReg(_imuAddr, ICM20948_WHO_AM_I_REG, who)) {
    if (who == 0xEA) return true; // ICM-20948 expected value
  }
  // Try the other common address
  uint8_t altAddr = (_imuAddr == 0x68) ? 0x69 : 0x68;
  if (i2cReadReg(altAddr, ICM20948_WHO_AM_I_REG, who)) {
    if (who == 0xEA) { _imuAddr = altAddr; return true; }
  }
  return false;
}

bool ColdStart::checkESP() {
  // Simple handshake: send "PING", expect "PONG" within timeout
  if (!_espSerial) return false;

  while (_espSerial->available()) _espSerial->read(); // flush
  _espSerial->print("PING\n");

  uint32_t start = millis();
  String rx;
  while (millis() - start < 500) { // 500 ms timeout
    if (_espSerial->available()) {
      char c = (char)_espSerial->read();
      rx += c;
      if (rx.indexOf("PONG") >= 0) return true;
    }
  }
  return false;
}

bool ColdStart::checkPower() {
  // Read ADC, convert to battery voltage using divider
  // NOTE: For micro builds you likely run 1S; set _batteryCells=1 and _lowVolt≈3.5V
  int raw = analogRead(_batAdcPin);
  // Teensy 4.x default ADC resolution is 10 bits (0..1023) unless changed.
  float vadc = (raw / 1023.0f) * _vRef;
  float vbat = vadc * _vDivRatio;

  // If you use >1S, set _batteryCells accordingly (not exposed here; change source or compare to absolute)
  float minV = _lowVolt * _batteryCells;
  return (vbat >= minV);
}

bool ColdStart::checkMotors() {
  // Lightweight spin test (no sensors): drive each motor briefly
  // CAUTION: 720 coreless motors MUST be driven through a proper MOSFET/driver, not directly from Teensy pins.
  const uint8_t testPwm = 128;   // 50% duty
  const uint16_t spinMs = 250;   // spin duration
  const uint16_t settleMs = 200; // gap between motors

  for (uint8_t i = 0; i < _motorCount; i++) {
    analogWrite(_motorPins[i], testPwm);
    delay(spinMs);
    analogWrite(_motorPins[i], 0);
    delay(settleMs);
  }

  // Without current/rotation feedback we assume OK
  return true;
}

// ---------- Utilities ----------
void ColdStart::beep(uint16_t onMs, uint16_t offMs) {
  digitalWrite(_buzzerPin, HIGH);
  delay(onMs);
  digitalWrite(_buzzerPin, LOW);
  delay(offMs);
}

void ColdStart::beepPatternOnce(uint8_t count, uint16_t gapMs) {
  for (uint8_t i = 0; i < count; i++) {
    beep(200, 200);
  }
  delay(gapMs);
}

void ColdStart::beepPatternLoop(CS_Error err) {
  uint8_t n = 0;
  uint16_t gap = 1000;
  switch (err) {
    case CS_IMU_FAIL:   n = 1; gap = 1000; break; // 1 short beep
    case CS_ESP_FAIL:   n = 2; gap = 800;  break; // 2 short beeps
    case CS_POWER_FAIL: n = 3; gap = 800;  break; // 3 short beeps
    case CS_MOTOR_FAIL: n = 4; gap = 800;  break; // 4 short beeps
    default: n = 5; gap = 600; break;
  }
  // Repeat forever until reset so operator immediately knows the fault
  while (true) {
    beepPatternOnce(n, gap);
  }
}

void ColdStart::successChirp() {
  // Three quick chirps
  for (uint8_t i = 0; i < 3; i++) beep(120, 120);
  delay(300);
}

// ---------- I2C helper ----------
bool ColdStart::i2cReadReg(uint8_t addr, uint8_t reg, uint8_t& val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  val = Wire.read();
  return true;
}
