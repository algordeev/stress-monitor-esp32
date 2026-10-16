
/*
 * STRESS MONITOR v0.4.5
 * Gentle Motion Filtering
 *
 * ESP32 + Gravity Heart Rate + SSD1306 + GY-521
 *
 * Gravity OUT -> GPIO34
 * OLED + MPU SDA -> GPIO21
 * OLED + MPU SCL -> GPIO22
 *
 * Features:
 * - 100 Hz PPG sampling
 * - Median BPM using 5 valid intervals
 * - BPM update every 2 seconds
 * - Less sensitive motion detection
 * - Motion confirmation and hysteresis
 * - 700 ms motion recovery
 * - Automatic gyroscope calibration
 * - MPU reconnection
 * - OLED and Serial output
 *
 * Experimental device, not medical equipment.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ================= HARDWARE =================

constexpr uint8_t PULSE_PIN = 34;
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;

constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t OLED_ADDR = 0x3C;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ================= TIMING =================

constexpr uint32_t PPG_PERIOD_US = 10000;
constexpr uint32_t MOTION_PERIOD_MS = 20;
constexpr uint32_t OLED_PERIOD_MS = 100;
constexpr uint32_t SERIAL_PERIOD_MS = 200;
constexpr uint32_t BPM_PERIOD_MS = 2000;

// Less strict timing
constexpr uint32_t SETTLE_MS = 700;
constexpr uint32_t MOTION_HOLD_MS = 350;

constexpr uint32_t MOTION_TIMEOUT_MS = 1000;
constexpr uint32_t PULSE_TIMEOUT_MS = 3000;

uint32_t lastPPG = 0;
uint32_t lastMotion = 0;
uint32_t lastOLED = 0;
uint32_t lastSerial = 0;
uint32_t lastBPMUpdate = 0;
uint32_t lastRetry = 0;

// ================= PPG =================

float baseline = 0.0f;
float ppgSignal = 0.0f;
float previousSignal = 0.0f;
float envelope = 0.0f;
float pulseThreshold = 8.0f;

bool ppgInitialized = false;
bool beatArmed = true;

constexpr uint32_t MIN_IBI = 350;
constexpr uint32_t MAX_IBI = 1500;

uint32_t lastDetectedBeat = 0;
uint32_t cleanAnchor = 0;
uint32_t lastAcceptedBeat = 0;

bool haveCleanAnchor = false;

float bpm = 0.0f;
uint32_t ibi = 0;
bool pulseValid = false;

constexpr int HISTORY_SIZE = 5;
uint32_t ibiHistory[HISTORY_SIZE] = {0};

int ibiCount = 0;
int ibiIndex = 0;

// Waveform history
float waveform[128] = {0};
int graphIndex = 0;

// ================= MOTION =================

bool mpuAvailable = false;
bool motionReady = false;
bool moving = false;

uint32_t lastGoodMotion = 0;
uint32_t lastMovement = 0;
uint32_t i2cErrors = 0;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;

float accelMotion = 0;
float gyroMotion = 0;
float motionScore = 0;

// Base scaling references
constexpr float ACCEL_LIMIT = 0.7f;
constexpr float GYRO_LIMIT = 0.25f;

// New, less sensitive detection
constexpr float MOTION_TRIGGER = 2.0f;
constexpr float MOTION_RELEASE = 0.9f;

// Four consecutive 20 ms samples
constexpr uint8_t MOTION_CONFIRM_SAMPLES = 4;
uint8_t motionStreak = 0;

// MPU6050-compatible scaling assumptions.
// Our sensor returns WHO_AM_I = 0x72.
constexpr float ACCEL_SCALE =
    9.80665f / 16384.0f;

constexpr float GYRO_SCALE =
    (PI / 180.0f) / 131.0f;

// ================= I2C HELPERS =================

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *data, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(
      MPU_ADDR, (size_t)n
  );

  if (received != n) {
    return false;
  }

  for (uint8_t i = 0; i < n; i++) {
    data[i] = Wire.read();
  }

  return true;
}

int16_t decode16(uint8_t hi, uint8_t lo) {
  return (int16_t)(
      (uint16_t(hi) << 8) | lo
  );
}

bool readMotionRaw(
    int16_t &ax, int16_t &ay, int16_t &az,
    int16_t &gx, int16_t &gy, int16_t &gz
) {
  uint8_t b[14];

  if (!readRegs(0x3B, b, 14)) {
    i2cErrors++;
    return false;
  }

  ax = decode16(b[0], b[1]);
  ay = decode16(b[2], b[3]);
  az = decode16(b[4], b[5]);

  gx = decode16(b[8], b[9]);
  gy = decode16(b[10], b[11]);
  gz = decode16(b[12], b[13]);

  return true;
}

// ================= MOTION QUALITY =================

bool motionFresh() {
  return mpuAvailable &&
         motionReady &&
         (uint32_t)(millis() - lastGoodMotion)
             <= MOTION_TIMEOUT_MS;
}

bool stableNow() {
  if (!motionFresh() || moving) {
    return false;
  }

  return (uint32_t)(millis() - lastMovement)
      >= SETTLE_MS;
}

// ================= PULSE HISTORY =================

void clearPulseHistory() {
  bpm = 0;
  ibi = 0;
  pulseValid = false;

  ibiCount = 0;
  ibiIndex = 0;

  haveCleanAnchor = false;
  cleanAnchor = 0;
  lastAcceptedBeat = 0;
}

void resetBeatDetector() {
  lastDetectedBeat = 0;
  beatArmed = true;
}

uint32_t medianIBI() {
  if (ibiCount == 0) {
    return 0;
  }

  uint32_t sorted[HISTORY_SIZE];

  for (int i = 0; i < ibiCount; i++) {
    sorted[i] = ibiHistory[i];
  }

  for (int i = 1; i < ibiCount; i++) {
    uint32_t value = sorted[i];
    int j = i - 1;

    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      j--;
    }

    sorted[j + 1] = value;
  }

  return sorted[ibiCount / 2];
}

bool addInterval(uint32_t value, uint32_t timeMs) {
  if (value < MIN_IBI || value > MAX_IBI) {
    return false;
  }

  // Reject sudden large outliers
  if (ibiCount >= 3) {
    uint32_t med = medianIBI();

    if (fabsf((float)value - (float)med) >
        0.25f * med) {
      return false;
    }
  }

  ibiHistory[ibiIndex] = value;
  ibiIndex = (ibiIndex + 1) % HISTORY_SIZE;

  if (ibiCount < HISTORY_SIZE) {
    ibiCount++;
  }

  ibi = value;
  lastAcceptedBeat = timeMs;

  return true;
}

// ================= PPG PROCESSING =================

void processPPG() {
  int raw = analogRead(PULSE_PIN);

  if (!ppgInitialized) {
    baseline = raw;
    ppgInitialized = true;
  }

  baseline += 0.005f * (raw - baseline);

  float centered = raw - baseline;

  ppgSignal +=
      0.25f * (centered - ppgSignal);

  envelope +=
      0.01f * (fabsf(ppgSignal) - envelope);

  pulseThreshold =
      max(8.0f, envelope * 0.65f);

  uint32_t now = millis();

  bool crossing =
      beatArmed &&
      ppgSignal > pulseThreshold &&
      previousSignal <= pulseThreshold;

  if (crossing) {
    beatArmed = false;

    if (lastDetectedBeat == 0 ||
        (uint32_t)(now - lastDetectedBeat) >= MIN_IBI) {

      lastDetectedBeat = now;

      if (stableNow()) {
        if (haveCleanAnchor) {
          uint32_t interval = now - cleanAnchor;

          if (interval >= MIN_IBI &&
              interval <= MAX_IBI) {

            addInterval(interval, now);

          } else {
            clearPulseHistory();
          }
        }

        cleanAnchor = now;
        haveCleanAnchor = true;

      } else {
        haveCleanAnchor = false;
      }
    }
  }

  if (ppgSignal < pulseThreshold * 0.4f) {
    beatArmed = true;
  }

  previousSignal = ppgSignal;

  // Ignore beat intervals during confirmed motion
  if (!stableNow()) {
    if (ibiCount > 0 || pulseValid ||
        haveCleanAnchor) {
      clearPulseHistory();
    }
  }

  // Remove stale pulse values
  if (lastAcceptedBeat != 0 &&
      (uint32_t)(now - lastAcceptedBeat) >
          PULSE_TIMEOUT_MS) {
    clearPulseHistory();
  }

  if (lastDetectedBeat != 0 &&
      (uint32_t)(now - lastDetectedBeat) >
          PULSE_TIMEOUT_MS) {
    clearPulseHistory();
    resetBeatDetector();
  }

  // Always update waveform
  waveform[graphIndex] = ppgSignal;
  graphIndex = (graphIndex + 1) % 128;
}

// ================= BPM UPDATE =================

void updateDisplayedBPM() {
  if (!stableNow() || ibiCount < 3 ||
      lastAcceptedBeat == 0) {
    bpm = 0;
    pulseValid = false;
    return;
  }

  if ((uint32_t)(millis() - lastAcceptedBeat) >
      PULSE_TIMEOUT_MS) {
    clearPulseHistory();
    return;
  }

  uint32_t med = medianIBI();

  if (med > 0) {
    bpm = 60000.0f / med;
    pulseValid = true;
  }
}

// ================= GYRO CALIBRATION =================

bool calibrateGyro() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 12);
  display.println("GYRO CALIBRATION");

  display.setCursor(0, 30);
  display.println("Keep MPU still!");

  display.display();

  Serial.println("Calibrating gyro for 3 seconds");

  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;

  int count = 0;

  uint32_t start = millis();
  uint32_t lastRead = start;

  while (millis() - start < 3000) {
    uint32_t nowUs = micros();

    if ((uint32_t)(nowUs - lastPPG) >=
        PPG_PERIOD_US) {
      lastPPG += PPG_PERIOD_US;
      processPPG();
    }

    uint32_t now = millis();

    if ((uint32_t)(now - lastRead) < 20) {
      delay(1);
      continue;
    }

    lastRead = now;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (readMotionRaw(
        ax, ay, az, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      count++;
    }
  }

  if (count < 100) {
    Serial.println("Gyro calibration failed");
    return false;
  }

  gyroBiasX = (sumX / count) * GYRO_SCALE;
  gyroBiasY = (sumY / count) * GYRO_SCALE;
  gyroBiasZ = (sumZ / count) * GYRO_SCALE;

  Serial.println("Gyro calibration successful");

  motionReady = false;
  moving = false;
  motionStreak = 0;

  accelMotion = 0;
  gyroMotion = 0;
  motionScore = 0;

  lastGoodMotion = millis();
  lastMovement = millis();
  lastMotion = millis();

  clearPulseHistory();

  return true;
}

// ================= MPU INITIALIZATION =================

bool initializeMPU() {
  Serial.println("Initializing motion sensor");

  uint8_t id = 0;

  if (!readRegs(0x75, &id, 1)) {
    Serial.println("WHO_AM_I read failed");
    return false;
  }

  Serial.print("WHO_AM_I: 0x");
  Serial.println(id, HEX);

  if (id != 0x68) {
    Serial.println("Using compatibility mode");
  }

  if (!writeReg(0x6B, 0x00)) {
    return false;
  }

  delay(50);

  if (!writeReg(0x1C, 0x00)) {
    return false;
  }

  if (!writeReg(0x1B, 0x00)) {
    return false;
  }

  uint8_t test[14];

  if (!readRegs(0x3B, test, 14)) {
    return false;
  }

  return calibrateGyro();
}

// ================= MOTION PROCESSING =================

void processMotion() {
  int16_t axr, ayr, azr;
  int16_t gxr, gyr, gzr;

  if (!readMotionRaw(
      axr, ayr, azr,
      gxr, gyr, gzr)) {
    return;
  }

  lastGoodMotion = millis();

  float ax = axr * ACCEL_SCALE;
  float ay = ayr * ACCEL_SCALE;
  float az = azr * ACCEL_SCALE;

  float gx = gxr * GYRO_SCALE - gyroBiasX;
  float gy = gyr * GYRO_SCALE - gyroBiasY;
  float gz = gzr * GYRO_SCALE - gyroBiasZ;

  if (!motionReady) {
    gravityX = ax;
    gravityY = ay;
    gravityZ = az;

    motionReady = true;
    lastMovement = millis();
    return;
  }

  constexpr float alpha = 0.05f;

  gravityX += alpha * (ax - gravityX);
  gravityY += alpha * (ay - gravityY);
  gravityZ += alpha * (az - gravityZ);

  float dx = ax - gravityX;
  float dy = ay - gravityY;
  float dz = az - gravityZ;

  accelMotion = sqrtf(
      dx * dx + dy * dy + dz * dz
  );

  gyroMotion = sqrtf(
      gx * gx + gy * gy + gz * gz
  );

  motionScore = max(
      accelMotion / ACCEL_LIMIT,
      gyroMotion / GYRO_LIMIT
  );

  uint32_t now = millis();

  // ------- GENTLE MOTION DETECTION -------

  if (!moving) {

    // A single vibration is not enough.
    if (motionScore >= MOTION_TRIGGER) {
      if (motionStreak < MOTION_CONFIRM_SAMPLES) {
        motionStreak++;
      }
    } else {
      motionStreak = 0;
    }

    // Confirm sustained movement.
    if (motionStreak >= MOTION_CONFIRM_SAMPLES) {
      moving = true;
      lastMovement = now;
      motionStreak = 0;
    }

  } else {

    // Lower threshold for keeping MOVING active.
    if (motionScore >= MOTION_RELEASE) {
      lastMovement = now;
    }

    // Return to STILL after motion settles.
    if ((uint32_t)(now - lastMovement) >=
        MOTION_HOLD_MS) {
      moving = false;
      motionStreak = 0;
    }
  }
}

// ================= OLED =================

void drawOLED() {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("STRESS MONITOR");

  display.setCursor(0, 12);

  if (!motionFresh()) {
    display.print("MOTION SENSOR ERROR");
  } else if (moving) {
    display.print("MOVING - PAUSED");
  } else if (!stableNow()) {
    display.print("STABILIZING...");
  } else if (!pulseValid) {
    display.print("ACQUIRING PULSE");
  } else {
    display.print("STILL - VALID");
  }

  display.setTextSize(2);
  display.setCursor(0, 25);

  if (pulseValid) {
    display.print((int)roundf(bpm));
  } else {
    display.print("--");
  }

  display.setTextSize(1);
  display.setCursor(50, 31);
  display.print("BPM");

  display.setCursor(0, 45);
  display.print("IBI: ");

  if (pulseValid && ibi > 0) {
    display.print(ibi);
    display.print(" ms");
  } else {
    display.print("-- ms");
  }

  float maxAbs = 10.0f;

  for (int i = 0; i < 128; i++) {
    maxAbs = max(maxAbs, fabsf(waveform[i]));
  }

  for (int x = 1; x < 128; x++) {
    int i1 = (graphIndex + x - 1) % 128;
    int i2 = (graphIndex + x) % 128;

    int y1 = constrain(
        58 - int(waveform[i1] * 5 / maxAbs),
        53, 63
    );

    int y2 = constrain(
        58 - int(waveform[i2] * 5 / maxAbs),
        53, 63
    );

    display.drawLine(
        x - 1, y1,
        x, y2,
        SSD1306_WHITE
    );
  }

  display.display();
}

// ================= SERIAL =================

void printSerial() {
  Serial.print("Signal:");
  Serial.print(ppgSignal, 1);

  Serial.print("\tThreshold:");
  Serial.print(pulseThreshold, 1);

  Serial.print("\tBPM:");
  Serial.print(pulseValid ? bpm : 0, 1);

  Serial.print("\tIBI:");
  Serial.print(pulseValid ? ibi : 0);

  Serial.print("\tMotion:");
  Serial.print(motionScore, 2);

  Serial.print("\tMoving:");
  Serial.print(moving ? 1 : 0);

  Serial.print("\tStable:");
  Serial.print(stableNow() ? 1 : 0);

  Serial.print("\tPulseValid:");
  Serial.print(pulseValid ? 1 : 0);

  Serial.print("\tMotionOK:");
  Serial.print(motionFresh() ? 1 : 0);

  Serial.print("\tI2CErrors:");
  Serial.println(i2cErrors);
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(
      PULSE_PIN, ADC_11db
  );

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(30);

  if (!display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR)) {
    Serial.println("OLED initialization failed");
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println("STRESS MONITOR");

  display.setCursor(42, 35);
  display.println("v0.4.5");

  display.display();

  lastPPG = micros();

  mpuAvailable = initializeMPU();

  lastMotion = millis();
  lastRetry = millis();
  lastBPMUpdate = millis();

  Serial.println("Stress Monitor v0.4.5 started");
}

// ================= MAIN LOOP =================

void loop() {
  uint32_t now = millis();

  // Read motion at 50 Hz.
  if (mpuAvailable &&
      (uint32_t)(now - lastMotion) >=
          MOTION_PERIOD_MS) {
    lastMotion = now;
    processMotion();
  }

  // Motion sensor disconnection
  if (mpuAvailable &&
      (uint32_t)(millis() - lastGoodMotion) >
          MOTION_TIMEOUT_MS) {

    mpuAvailable = false;
    motionReady = false;
    moving = false;
    motionStreak = 0;

    clearPulseHistory();

    lastRetry = millis();
    Serial.println("Motion sensor disconnected");
  }

  // Try reconnecting every 2 seconds.
  if (!mpuAvailable &&
      (uint32_t)(millis() - lastRetry) >= 2000) {

    lastRetry = millis();
    mpuAvailable = initializeMPU();
    lastMotion = millis();
  }

  // Read PPG at 100 Hz.
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastPPG) >=
      PPG_PERIOD_US) {

    lastPPG += PPG_PERIOD_US;
    processPPG();
  }

  now = millis();

  // Update BPM every 2 seconds.
  if ((uint32_t)(now - lastBPMUpdate) >=
      BPM_PERIOD_MS) {

    lastBPMUpdate = now;
    updateDisplayedBPM();
  }

  // Refresh OLED.
  if ((uint32_t)(now - lastOLED) >=
      OLED_PERIOD_MS) {

    lastOLED = now;
    drawOLED();
  }

  // Debug output.
  if ((uint32_t)(now - lastSerial) >=
      SERIAL_PERIOD_MS) {

    lastSerial = now;
    printSerial();
  }
}
