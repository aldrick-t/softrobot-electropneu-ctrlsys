#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
#include <Wire.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Board connections retained from the original on/off controller.
const uint8_t PIN_PWM_PUMP = 2;
const uint8_t PIN_PWM_VALVE = 5;
const uint8_t PIN_I2C_SDA = 21;
const uint8_t PIN_I2C_SCL = 22;

const uint8_t ADS1115_GND_ADDRESS = 0x48;
// Temporary bench-test override only. When true, the firmware falls back to the
// other legal ADS1115 addresses if a floating ADDR pin does not answer at 0x48.
// Set false after wiring ADDR to GND for normal operation.
const bool ALLOW_FLOATING_ADS1115_ADDR_FOR_TESTING = true;
const uint8_t ADS1115_CHANNEL = 0;

const uint32_t CONTROL_PERIOD_MS = 50;
const uint32_t ZERO_SETTLE_MS = 1000;
const uint8_t ZERO_SAMPLE_COUNT = 32;
const uint8_t ADC_AVERAGE_COUNT = 4;
const float ZERO_MAX_STDDEV_KPA = 0.10f;

const float SENSOR_KPA_PER_VOLT = 50.0f;  // 100KPGPN: -100..100 kPa over 0.5..4.5 V.
const float MAX_VACUUM_TARGET_KPA = -90.0f;
const float ADC_MIN_VALID_VOLTS = 0.25f;
const float ADC_MAX_VALID_VOLTS = 4.75f;
const uint8_t MAX_INVALID_READINGS = 3;

const uint8_t PWM_OFF = 0;
const uint8_t PWM_MAX = 255;
const uint8_t PWM_SLEW_PER_CYCLE = 10;
const uint32_t VENT_SETTLE_MS = 100;
const float DERIVATIVE_FILTER_ALPHA = 0.25f;

struct Tuning {
  float kp;
  float ki;
  float kd;
  float bandKpa;
  uint16_t ventMs;
};

const Tuning DEFAULT_TUNING = {12.0f, 1.0f, 0.3f, 0.5f, 20};
const char *PREFERENCES_NAMESPACE = "vacuum-ctl";

enum class ControllerState {
  IDLE,
  CALIBRATING_SETTLE,
  CALIBRATING_SAMPLE,
  REGULATING,
  VENTING,
  SETTLING,
  FAULT,
};

Adafruit_ADS1115 ads;
Preferences preferences;
Tuning tuning = DEFAULT_TUNING;
ControllerState state = ControllerState::IDLE;

bool adsAvailable = false;
uint8_t adsAddressInUse = ADS1115_GND_ADDRESS;
bool zeroCalibrated = false;
float zeroVolts = NAN;
float measuredVolts = NAN;
float measuredPressureKpa = NAN;
float targetPressureKpa = 0.0f;
float integralTerm = 0.0f;
float previousVacuumKpa = NAN;
float filteredVacuumRate = 0.0f;
uint8_t pumpPwm = PWM_OFF;
bool valveOpen = false;
uint8_t invalidReadings = 0;
uint32_t lastControlMs = 0;
uint32_t phaseStartedMs = 0;
uint32_t lastPidMs = 0;
float zeroSum = 0.0f;
float zeroSumSquared = 0.0f;
uint8_t zeroSamples = 0;

char serialLine[96];
size_t serialLength = 0;

const char *stateName(ControllerState currentState) {
  switch (currentState) {
    case ControllerState::IDLE:
      return "IDLE";
    case ControllerState::CALIBRATING_SETTLE:
      return "ZERO_SETTLE";
    case ControllerState::CALIBRATING_SAMPLE:
      return "ZERO_SAMPLE";
    case ControllerState::REGULATING:
      return "REGULATING";
    case ControllerState::VENTING:
      return "VENTING";
    case ControllerState::SETTLING:
      return "SETTLING";
    case ControllerState::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

void setOutputs(uint8_t requestedPumpPwm, bool openValve) {
  pumpPwm = requestedPumpPwm;
  valveOpen = openValve;
  analogWrite(PIN_PWM_PUMP, pumpPwm);
  analogWrite(PIN_PWM_VALVE, valveOpen ? PWM_MAX : PWM_OFF);
}

void resetPid() {
  integralTerm = 0.0f;
  previousVacuumKpa = NAN;
  filteredVacuumRate = 0.0f;
  lastPidMs = 0;
}

void stopAndVent(const char *message) {
  targetPressureKpa = 0.0f;
  resetPid();
  state = ControllerState::IDLE;
  setOutputs(PWM_OFF, false);
  if (message != nullptr) {
    Serial.println(message);
  }
}

void enterFault(const char *message) {
  targetPressureKpa = 0.0f;
  zeroCalibrated = false;
  resetPid();
  state = ControllerState::FAULT;
  setOutputs(PWM_OFF, false);
  Serial.print("FAULT: ");
  Serial.println(message);
}

bool tuningIsValid(const Tuning &candidate) {
  return isfinite(candidate.kp) && candidate.kp >= 0.0f && candidate.kp <= 50.0f &&
         isfinite(candidate.ki) && candidate.ki >= 0.0f && candidate.ki <= 20.0f &&
         isfinite(candidate.kd) && candidate.kd >= 0.0f && candidate.kd <= 10.0f &&
         isfinite(candidate.bandKpa) && candidate.bandKpa >= 0.1f && candidate.bandKpa <= 5.0f &&
         candidate.ventMs >= 5 && candidate.ventMs <= 100;
}

void loadTuning() {
  Tuning loaded = DEFAULT_TUNING;
  if (preferences.begin(PREFERENCES_NAMESPACE, true)) {
    if (preferences.isKey("kp")) {
      loaded.kp = preferences.getFloat("kp", DEFAULT_TUNING.kp);
      loaded.ki = preferences.getFloat("ki", DEFAULT_TUNING.ki);
      loaded.kd = preferences.getFloat("kd", DEFAULT_TUNING.kd);
      loaded.bandKpa = preferences.getFloat("band", DEFAULT_TUNING.bandKpa);
      loaded.ventMs = preferences.getUShort("vent", DEFAULT_TUNING.ventMs);
    }
    preferences.end();
  }

  tuning = tuningIsValid(loaded) ? loaded : DEFAULT_TUNING;
}

bool saveTuning() {
  if (!tuningIsValid(tuning) || !preferences.begin(PREFERENCES_NAMESPACE, false)) {
    return false;
  }

  const bool saved = preferences.putFloat("kp", tuning.kp) > 0 &&
                     preferences.putFloat("ki", tuning.ki) > 0 &&
                     preferences.putFloat("kd", tuning.kd) > 0 &&
                     preferences.putFloat("band", tuning.bandKpa) > 0 &&
                     preferences.putUShort("vent", tuning.ventMs) > 0;
  preferences.end();
  return saved;
}

bool readAveragedVoltage(float &volts) {
  int32_t total = 0;
  for (uint8_t sample = 0; sample < ADC_AVERAGE_COUNT; ++sample) {
    total += ads.readADC_SingleEnded(ADS1115_CHANNEL);
  }

  // ADS1115 GAIN_TWOTHIRDS has a 0.1875 mV least-significant bit.
  volts = (static_cast<float>(total) / ADC_AVERAGE_COUNT) * 0.0001875f;
  return isfinite(volts);
}

bool initializeAds1115() {
  if (ads.begin(ADS1115_GND_ADDRESS, &Wire)) {
    adsAddressInUse = ADS1115_GND_ADDRESS;
    return true;
  }

  if (!ALLOW_FLOATING_ADS1115_ADDR_FOR_TESTING) {
    return false;
  }

  const uint8_t fallbackAddresses[] = {0x49, 0x4A, 0x4B};
  for (const uint8_t address : fallbackAddresses) {
    if (ads.begin(address, &Wire)) {
      adsAddressInUse = address;
      return true;
    }
  }
  return false;
}

void reportStatus() {
  Serial.print("STATE=");
  Serial.print(stateName(state));
  Serial.print(" V=");
  if (isfinite(measuredVolts)) {
    Serial.print(measuredVolts, 4);
  } else {
    Serial.print("NA");
  }
  Serial.print(" P_kPa=");
  if (isfinite(measuredPressureKpa)) {
    Serial.print(measuredPressureKpa, 2);
  } else {
    Serial.print("NA");
  }
  Serial.print(" TARGET=");
  Serial.print(targetPressureKpa, 2);
  Serial.print(" PWM=");
  Serial.print(pumpPwm);
  Serial.print(" VALVE=");
  Serial.print(valveOpen ? "OPEN" : "VENT");
  Serial.print(" ZERO=");
  Serial.print(zeroCalibrated ? "OK" : "REQUIRED");
  Serial.print(" KP=");
  Serial.print(tuning.kp, 3);
  Serial.print(" KI=");
  Serial.print(tuning.ki, 3);
  Serial.print(" KD=");
  Serial.print(tuning.kd, 3);
  Serial.print(" BAND=");
  Serial.print(tuning.bandKpa, 2);
  Serial.print(" VENT_MS=");
  Serial.print(tuning.ventMs);
  Serial.print(" ADS_ADDR=0x");
  Serial.println(adsAddressInUse, HEX);
}

void startZeroCalibration() {
  if (!adsAvailable || state == ControllerState::FAULT) {
    Serial.println("ERROR: ADS1115 unavailable; cannot ZERO");
    return;
  }

  targetPressureKpa = 0.0f;
  zeroCalibrated = false;
  resetPid();
  zeroSum = 0.0f;
  zeroSumSquared = 0.0f;
  zeroSamples = 0;
  state = ControllerState::CALIBRATING_SETTLE;
  phaseStartedMs = millis();
  setOutputs(PWM_OFF, false);
  Serial.println("ZERO: venting for 1000 ms; keep the pressure port at atmosphere");
}

void setTarget(float requestedTargetKpa) {
  if (!isfinite(requestedTargetKpa)) {
    stopAndVent("ERROR: target must be a finite number; venting");
    return;
  }

  if (state == ControllerState::FAULT) {
    Serial.println("ERROR: controller fault; reboot after correcting hardware");
    return;
  }

  if (requestedTargetKpa >= 0.0f) {
    stopAndVent(requestedTargetKpa > 0.0f ? "TARGET clamped to 0.00 kPa; venting" : "OFF: venting");
    return;
  }

  if (!zeroCalibrated) {
    stopAndVent("ERROR: run ZERO at atmosphere before a negative target; venting");
    return;
  }

  targetPressureKpa = requestedTargetKpa;
  if (targetPressureKpa < MAX_VACUUM_TARGET_KPA) {
    targetPressureKpa = MAX_VACUUM_TARGET_KPA;
    Serial.println("TARGET clamped to -90.00 kPa");
  }

  resetPid();
  state = ControllerState::REGULATING;
  phaseStartedMs = millis();
  Serial.print("TARGET set to ");
  Serial.print(targetPressureKpa, 2);
  Serial.println(" kPa");
}

bool parseFloatToken(const char *token, float &value) {
  if (token == nullptr) {
    return false;
  }

  char *end = nullptr;
  value = strtof(token, &end);
  return end != token && end != nullptr && *end == '\0' && isfinite(value);
}

void commandError(const char *message) {
  stopAndVent(message);
}

void handleTuningCommand(const char *name, const char *argument, const char *extra) {
  float value = 0.0f;
  if (argument == nullptr || extra != nullptr || !parseFloatToken(argument, value)) {
    commandError("ERROR: malformed tuning command; venting");
    return;
  }

  Tuning candidate = tuning;
  if (strcasecmp(name, "KP") == 0) {
    candidate.kp = value;
  } else if (strcasecmp(name, "KI") == 0) {
    candidate.ki = value;
  } else if (strcasecmp(name, "KD") == 0) {
    candidate.kd = value;
  } else if (strcasecmp(name, "BAND") == 0) {
    candidate.bandKpa = value;
  } else if (strcasecmp(name, "VENT") == 0) {
    const long rounded = lroundf(value);
    if (fabsf(value - rounded) > 0.001f || rounded < 0 || rounded > UINT16_MAX) {
      commandError("ERROR: VENT must be an integer number of milliseconds; venting");
      return;
    }
    candidate.ventMs = static_cast<uint16_t>(rounded);
  } else {
    commandError("ERROR: unknown command; venting");
    return;
  }

  if (!tuningIsValid(candidate)) {
    commandError("ERROR: tuning value is outside its safe range; venting");
    return;
  }

  tuning = candidate;
  resetPid();
  Serial.print(name);
  Serial.println(" updated in RAM; send SAVE to persist");
}

void processCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  char *tail = line + strlen(line);
  while (tail > line && (tail[-1] == ' ' || tail[-1] == '\t')) {
    *--tail = '\0';
  }
  if (*line == '\0') {
    return;
  }

  float numericTarget = 0.0f;
  if (parseFloatToken(line, numericTarget)) {
    setTarget(numericTarget);
    return;
  }

  char *savePointer = nullptr;
  char *command = strtok_r(line, " \t", &savePointer);
  char *argument = strtok_r(nullptr, " \t", &savePointer);
  char *extra = strtok_r(nullptr, " \t", &savePointer);

  if (strcasecmp(command, "ZERO") == 0 && argument == nullptr) {
    startZeroCalibration();
  } else if (strcasecmp(command, "STATUS") == 0 && argument == nullptr) {
    reportStatus();
  } else if (strcasecmp(command, "SAVE") == 0 && argument == nullptr) {
    Serial.println(saveTuning() ? "Tuning saved" : "ERROR: tuning could not be saved");
  } else if (strcasecmp(command, "DEFAULTS") == 0 && argument == nullptr) {
    tuning = DEFAULT_TUNING;
    resetPid();
    Serial.println("Safe defaults loaded in RAM; send SAVE to persist");
  } else if (strcasecmp(command, "KP") == 0 || strcasecmp(command, "KI") == 0 ||
             strcasecmp(command, "KD") == 0 || strcasecmp(command, "BAND") == 0 ||
             strcasecmp(command, "VENT") == 0) {
    handleTuningCommand(command, argument, extra);
  } else {
    commandError("ERROR: malformed command; venting");
  }
}

void serviceSerial() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      serialLine[serialLength] = '\0';
      processCommand(serialLine);
      serialLength = 0;
      continue;
    }
    if (serialLength >= sizeof(serialLine) - 1) {
      serialLength = 0;
      commandError("ERROR: command too long; venting");
      continue;
    }
    serialLine[serialLength++] = character;
  }
}

void updateCalibration(uint32_t now) {
  setOutputs(PWM_OFF, false);
  if (state == ControllerState::CALIBRATING_SETTLE) {
    if (now - phaseStartedMs >= ZERO_SETTLE_MS) {
      zeroSum = 0.0f;
      zeroSumSquared = 0.0f;
      zeroSamples = 0;
      state = ControllerState::CALIBRATING_SAMPLE;
      Serial.println("ZERO: sampling");
    }
    return;
  }

  zeroSum += measuredVolts;
  zeroSumSquared += measuredVolts * measuredVolts;
  ++zeroSamples;
  if (zeroSamples < ZERO_SAMPLE_COUNT) {
    return;
  }

  const float average = zeroSum / zeroSamples;
  const float variance = fmaxf(0.0f, (zeroSumSquared / zeroSamples) - average * average);
  const float stddevKpa = sqrtf(variance) * SENSOR_KPA_PER_VOLT;
  if (stddevKpa > ZERO_MAX_STDDEV_KPA) {
    zeroCalibrated = false;
    state = ControllerState::IDLE;
    setOutputs(PWM_OFF, false);
    Serial.print("ERROR: ZERO unstable (stddev ");
    Serial.print(stddevKpa, 3);
    Serial.println(" kPa); vent and try again");
    return;
  }

  zeroVolts = average;
  measuredPressureKpa = 0.0f;
  zeroCalibrated = true;
  state = ControllerState::IDLE;
  setOutputs(PWM_OFF, false);
  Serial.print("ZERO complete: ");
  Serial.print(zeroVolts, 4);
  Serial.println(" V = 0.00 kPa");
}

void updateRegulation(uint32_t now) {
  const float measuredVacuumKpa = -measuredPressureKpa;
  const float targetVacuumKpa = -targetPressureKpa;
  const float errorKpa = targetVacuumKpa - measuredVacuumKpa;

  if (state == ControllerState::VENTING) {
    setOutputs(PWM_OFF, false);
    if (now - phaseStartedMs >= tuning.ventMs) {
      state = ControllerState::SETTLING;
      phaseStartedMs = now;
      setOutputs(PWM_OFF, true);
    }
    return;
  }

  if (state == ControllerState::SETTLING) {
    setOutputs(PWM_OFF, true);
    if (now - phaseStartedMs >= VENT_SETTLE_MS) {
      state = ControllerState::REGULATING;
      resetPid();
    }
    return;
  }

  if (errorKpa < -tuning.bandKpa) {
    resetPid();
    state = ControllerState::VENTING;
    phaseStartedMs = now;
    setOutputs(PWM_OFF, false);
    return;
  }

  float dtSeconds = CONTROL_PERIOD_MS / 1000.0f;
  if (lastPidMs != 0) {
    dtSeconds = fmaxf(0.001f, (now - lastPidMs) / 1000.0f);
  }
  lastPidMs = now;

  float vacuumRate = 0.0f;
  if (isfinite(previousVacuumKpa)) {
    vacuumRate = (measuredVacuumKpa - previousVacuumKpa) / dtSeconds;
  }
  previousVacuumKpa = measuredVacuumKpa;
  filteredVacuumRate += DERIVATIVE_FILTER_ALPHA * (vacuumRate - filteredVacuumRate);

  if (fabsf(errorKpa) <= tuning.bandKpa) {
    integralTerm *= 0.98f;
  } else {
    integralTerm += errorKpa * dtSeconds;
    integralTerm = constrain(integralTerm, -PWM_MAX / fmaxf(tuning.ki, 0.001f),
                             PWM_MAX / fmaxf(tuning.ki, 0.001f));
  }

  const float requestedPwm = tuning.kp * errorKpa + tuning.ki * integralTerm - tuning.kd * filteredVacuumRate;
  uint8_t desiredPwm = requestedPwm <= 0.0f ? PWM_OFF : static_cast<uint8_t>(constrain(requestedPwm, 0.0f, static_cast<float>(PWM_MAX)));

  if (desiredPwm > pumpPwm + PWM_SLEW_PER_CYCLE) {
    desiredPwm = pumpPwm + PWM_SLEW_PER_CYCLE;
  } else if (pumpPwm > desiredPwm + PWM_SLEW_PER_CYCLE) {
    desiredPwm = pumpPwm - PWM_SLEW_PER_CYCLE;
  }

  setOutputs(desiredPwm, true);
}

void updateController(uint32_t now) {
  if (!adsAvailable || state == ControllerState::FAULT) {
    return;
  }

  float volts = NAN;
  if (!readAveragedVoltage(volts) || volts < ADC_MIN_VALID_VOLTS || volts > ADC_MAX_VALID_VOLTS) {
    // A questionable sensor value must never leave the previous pump command active.
    setOutputs(PWM_OFF, false);
    resetPid();
    ++invalidReadings;
    if (invalidReadings >= MAX_INVALID_READINGS) {
      enterFault("pressure signal outside 0.25..4.75 V");
    }
    return;
  }

  invalidReadings = 0;
  measuredVolts = volts;
  measuredPressureKpa = zeroCalibrated ? SENSOR_KPA_PER_VOLT * (measuredVolts - zeroVolts) : NAN;

  if (state == ControllerState::CALIBRATING_SETTLE || state == ControllerState::CALIBRATING_SAMPLE) {
    updateCalibration(now);
  } else if (state == ControllerState::REGULATING || state == ControllerState::VENTING || state == ControllerState::SETTLING) {
    updateRegulation(now);
  } else {
    setOutputs(PWM_OFF, false);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM_PUMP, OUTPUT);
  pinMode(PIN_PWM_VALVE, OUTPUT);
  setOutputs(PWM_OFF, false);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  adsAvailable = initializeAds1115();
  if (!adsAvailable) {
    enterFault(ALLOW_FLOATING_ADS1115_ADDR_FOR_TESTING
                   ? "ADS1115 not found at 0x48..0x4B; verify I2C wiring"
                   : "ADS1115 not found at 0x48; verify ADDR is tied to GND and I2C wiring");
    return;
  }

  ads.setGain(GAIN_TWOTHIRDS);
  ads.setDataRate(RATE_ADS1115_250SPS);
  loadTuning();
  if (adsAddressInUse != ADS1115_GND_ADDRESS) {
    Serial.print("WARNING: using floating-ADDR test override at 0x");
    Serial.println(adsAddressInUse, HEX);
  }
  Serial.println("Vacuum controller ready. Run ZERO with the port vented, then send a negative kPa target.");
}

void loop() {
  serviceSerial();

  const uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    updateController(now);
  }
}
