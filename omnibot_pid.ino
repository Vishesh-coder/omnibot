#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <PS4Controller.h>
#include <math.h>

#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

struct euler_t {
  float yaw;
} ypr;

bool imuReady = false;
bool globalFrame = true;
float yawOffset = 0.0f;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quatToYaw(float qr, float qi, float qj, float qk, euler_t* out) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  out->yaw = atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
}

void updateIMU() {
  if (!imuReady) return;

  if (bno08x.wasReset()) {
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      auto rv = sensorValue.un.arvrStabilizedRV;
      quatToYaw(rv.real, rv.i, rv.j, rv.k, &ypr);
    }
  }
}

void worldToRobot(float vxWorld, float vyWorld, float yawDeg, float &vxOut, float &vyOut) {
  float yawRad = radians(yawDeg);
  float c = cosf(yawRad);
  float s = sinf(yawRad);

  vxOut =  vxWorld * c + vyWorld * s;
  vyOut = -vxWorld * s + vyWorld * c;
}

void resetHeading() {
  yawOffset = ypr.yaw;
}

float wrapDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

#define M1_PWM 13
#define M1_DIR 12
#define M2_PWM 26
#define M2_DIR 25
#define M3_PWM 33
#define M3_DIR 32

int pwmMax = 120;
int speedLevel = 0;
const int PWM_STEP = 20;
const int MAX_SPEED_LEVEL = 5;

const float wheelAngles[3] = { 0.0f, 120.0f, 240.0f };
const float botRadius = 0.15f;

const int joyDeadzone = 15;
const int triggerDeadzone = 5;
const float dpadSpeed = 1.0f;
const float spinRate = 1.0f;

void setMotor(uint8_t pwmPin, uint8_t dirPin, float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  digitalWrite(dirPin, speed >= 0.0f ? HIGH : LOW);
  analogWrite(pwmPin, (int)(fabsf(speed) * pwmMax));
}

void stopBot() {
  setMotor(M1_PWM, M1_DIR, 0);
  setMotor(M2_PWM, M2_DIR, 0);
  setMotor(M3_PWM, M3_DIR, 0);
}

void inverseKinematics(float vx, float vy, float w, float &w1, float &w2, float &w3) {
  float t1 = radians(wheelAngles[0]);
  float t2 = radians(wheelAngles[1]);
  float t3 = radians(wheelAngles[2]);

  w1 = -sinf(t1) * vx + cosf(t1) * vy + botRadius * w;
  w2 = -sinf(t2) * vx + cosf(t2) * vy + botRadius * w;
  w3 = -sinf(t3) * vx + cosf(t3) * vy + botRadius * w;

  float maxMag = fmaxf(fabsf(w1), fmaxf(fabsf(w2), fabsf(w3)));
  if (maxMag > 1.0f) {
    w1 /= maxMag;
    w2 /= maxMag;
    w3 /= maxMag;
  }
}

void driveWheels(float w1, float w2, float w3) {
  setMotor(M1_PWM, M1_DIR, w1);
  setMotor(M2_PWM, M2_DIR, w2);
  setMotor(M3_PWM, M3_DIR, w3);
}

float readStick(int8_t raw) {
  if (abs(raw) < joyDeadzone) return 0.0f;
  return constrain((float)raw / 127.0f, -1.0f, 1.0f);
}

float readTriggerSpin() {
  int r2 = PS4.R2Value();
  int l2 = PS4.L2Value();
  float rNorm = (r2 > triggerDeadzone) ? (float)r2 / 255.0f : 0.0f;
  float lNorm = (l2 > triggerDeadzone) ? (float)l2 / 255.0f : 0.0f;
  return constrain((rNorm - lNorm) * spinRate, -1.0f, 1.0f);
}

bool justPressed(bool now, bool &prevState) {
  bool pressed = now && !prevState;
  prevState = now;
  return pressed;
}

struct PID {
  float kp = 0, ki = 0, kd = 0;
  float integral = 0;
  float prevError = 0;
  float outMin = -1.0f, outMax = 1.0f;
  float integralMax = 1.0f;
  unsigned long prevTimeUs = 0;

  void reset() {
    integral = 0;
    prevError = 0;
    prevTimeUs = micros();
  }

  float update(float error) {
    unsigned long nowUs = micros();
    float dt = (nowUs - prevTimeUs) / 1000000.0f;
    prevTimeUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

    integral += error * dt;
    integral = constrain(integral, -integralMax, integralMax);

    float derivative = (error - prevError) / dt;
    prevError = error;

    float out = kp * error + ki * integral + kd * derivative;
    return constrain(out, outMin, outMax);
  }
};

PID headingPID;
float headingTarget = 0.0f;
bool headingHoldActive = false;

enum TuneState { TUNE_IDLE, TUNE_RUNNING, TUNE_DONE, TUNE_FAILED };
TuneState tuneState = TUNE_IDLE;

const float RELAY_AMPLITUDE = 0.35f;
const float RELAY_HYSTERESIS_DEG = 1.5f;
const int RELAY_CYCLES_NEEDED = 6;
const unsigned long TUNE_TIMEOUT_MS = 15000;

float tuneStartYaw = 0.0f;
bool relayHigh = true;
unsigned long lastCrossingUs = 0;
int cycleCount = 0;
float periodSumUs = 0.0f;
float peakHigh = -1000.0f;
float peakLow = 1000.0f;
unsigned long tuneStartMs = 0;

void startAutotune() {
  if (!imuReady) {
    Serial.println("AUTOTUNE: skipped, IMU not ready");
    tuneState = TUNE_FAILED;
    return;
  }
  Serial.println("AUTOTUNE: starting relay-feedback Ziegler-Nichols tune");
  tuneStartYaw = ypr.yaw;
  relayHigh = true;
  cycleCount = 0;
  periodSumUs = 0.0f;
  peakHigh = -1000.0f;
  peakLow = 1000.0f;
  lastCrossingUs = micros();
  tuneStartMs = millis();
  tuneState = TUNE_RUNNING;
}

void applyZieglerNichols(float Ku, float Tu) {
  headingPID.kp = 0.6f * Ku;
  headingPID.ki = 1.2f * Ku / Tu;
  headingPID.kd = 0.075f * Ku * Tu;
  headingPID.integralMax = 1.0f / max(headingPID.ki, 0.0001f);

  Serial.printf("AUTOTUNE DONE: Ku=%.4f Tu=%.4fs -> Kp=%.4f Ki=%.4f Kd=%.4f\n",
                Ku, Tu, headingPID.kp, headingPID.ki, headingPID.kd);
}

void runAutotuneStep() {
  float error = wrapDeg(tuneStartYaw - ypr.yaw);

  if (relayHigh && error < -RELAY_HYSTERESIS_DEG) {
    relayHigh = false;
  } else if (!relayHigh && error > RELAY_HYSTERESIS_DEG) {
    relayHigh = true;
  }

  float w1, w2, w3;
  float spin = relayHigh ? RELAY_AMPLITUDE : -RELAY_AMPLITUDE;
  inverseKinematics(0, 0, spin, w1, w2, w3);
  driveWheels(w1, w2, w3);

  if (ypr.yaw > peakHigh) peakHigh = ypr.yaw;
  if (ypr.yaw < peakLow) peakLow = ypr.yaw;

  static bool lastRelayHigh = true;
  if (relayHigh != lastRelayHigh) {
    unsigned long nowUs = micros();
    if (relayHigh && !lastRelayHigh) {
      unsigned long periodUs = nowUs - lastCrossingUs;
      lastCrossingUs = nowUs;
      cycleCount++;
      if (cycleCount > 1) {
        periodSumUs += (float)periodUs;
      }
    }
    lastRelayHigh = relayHigh;
  }

  if (cycleCount > RELAY_CYCLES_NEEDED) {
    stopBot();
    float Tu = (periodSumUs / (cycleCount - 2)) / 1000000.0f;
    float amplitudeDeg = (peakHigh - peakLow) / 2.0f;
    if (amplitudeDeg < 0.5f) amplitudeDeg = 0.5f;
    float Ku = (4.0f * RELAY_AMPLITUDE) / (float)(M_PI * amplitudeDeg / 57.2958f);

    applyZieglerNichols(Ku, Tu);
    headingPID.reset();
    tuneState = TUNE_DONE;
    return;
  }

  if (millis() - tuneStartMs > TUNE_TIMEOUT_MS) {
    stopBot();
    Serial.println("AUTOTUNE: timed out, falling back to default gains");
    headingPID.kp = 0.02f;
    headingPID.ki = 0.01f;
    headingPID.kd = 0.001f;
    headingPID.integralMax = 50.0f;
    headingPID.reset();
    tuneState = TUNE_FAILED;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  stopBot();
  Wire.begin();

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    imuReady = false;
  } else {
    Serial.println("BNO08x Found!");
    setReports(reportType, reportIntervalUs);
    imuReady = true;
  }

  PS4.begin();
  Serial.println("Waiting for PS4 controller...");

  headingPID.outMin = -1.0f;
  headingPID.outMax = 1.0f;
}

void loop() {
  updateIMU();

  if (tuneState == TUNE_IDLE && imuReady) {
    delay(300);
    startAutotune();
  }
  if (tuneState == TUNE_RUNNING) {
    runAutotuneStep();
    return;
  }

  if (!PS4.isConnected()) {
    stopBot();
    Serial.println("PS4: disconnected");
    delay(200);
    return;
  }

  float lx = readStick(PS4.LStickX());
  float ly = readStick(PS4.LStickY());
  float vx = 0.0f, vy = 0.0f;
  bool dpadActive = true;

  if (PS4.Up())
    vx = -dpadSpeed;
  else if (PS4.Down())
    vx = dpadSpeed;
  else if (PS4.Left())
    vy = -dpadSpeed;
  else if (PS4.Right())
    vy =  dpadSpeed;
  else if (PS4.UpRight()){
    vx = -dpadSpeed;
    vy = dpadSpeed;
  }
  else if (PS4.DownRight()){
    vx = dpadSpeed;
    vy = dpadSpeed;
  }
  else if (PS4.UpLeft()){
    vx = -dpadSpeed;
    vy = -dpadSpeed;
  }
  else if (PS4.DownLeft()){
    vx = dpadSpeed;
    vy = -dpadSpeed;
  }
  else dpadActive = false;

  static bool prevTriangle = false, prevCircle = false;
  if (justPressed(PS4.Triangle(), prevTriangle)) {
    globalFrame = false;
    Serial.println("MODE:BOT FRAME");
  }
  if (justPressed(PS4.Circle(), prevCircle)) {
    globalFrame = true;
    Serial.println("MODE:GLOBAL FRAME");
  }
  digitalWrite(LED_BUILTIN, globalFrame ? HIGH : LOW);

  static bool prevR1 = false, prevL1 = false;
  if (justPressed(PS4.R1(), prevR1) && speedLevel < MAX_SPEED_LEVEL)
    speedLevel++;
  if (justPressed(PS4.L1(), prevL1) && speedLevel > 0)
    speedLevel--;

  pwmMax = 120 + (speedLevel * PWM_STEP);

  if (!dpadActive) {
    vx = -ly;
    vy = lx;
  }

  float manualSpin = readTriggerSpin();
  float w;

  if (imuReady && tuneState != TUNE_RUNNING) {
    bool spinIdle = fabsf(manualSpin) < 0.02f;

    if (spinIdle) {
      if (!headingHoldActive) {
        headingTarget = ypr.yaw;
        headingPID.reset();
        headingHoldActive = true;
      }
      float error = wrapDeg(headingTarget - ypr.yaw);
      w = headingPID.update(error);
    } else {
      headingHoldActive = false;
      w = manualSpin;
    }
  } else {
    w = manualSpin;
  }

  if (globalFrame && imuReady) {
    float heading = ypr.yaw - yawOffset;
    float vxRobot, vyRobot;
    worldToRobot(vx, vy, heading, vxRobot, vyRobot);
    vx = vxRobot;
    vy = vyRobot;
  }

  static bool prevOptions = false;
  if (justPressed(PS4.Options(), prevOptions)) {
    resetHeading();
    headingTarget = ypr.yaw;
    Serial.println("Heading zeroed");
  }

  static bool prevShare = false;
  if (justPressed(PS4.Share(), prevShare)) {
    startAutotune();
  }

  float w1, w2, w3;
  inverseKinematics(vx, vy, w, w1, w2, w3);
  driveWheels(w1, w2, w3);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    if (imuReady) {
      Serial.printf(
        "PS4: connected=1 battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwm=%d | w1=%.2f w2=%.2f w3=%.2f | yaw=%.1f target=%.1f hold=%d bno_status=%d\n",
        PS4.Battery(), vx, vy, w, pwmMax, w1, w2, w3, ypr.yaw, headingTarget, headingHoldActive, sensorValue.status);
    } else {
      Serial.printf(
        "PS4: connected=1 battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwm=%d | w1=%.2f w2=%.2f w3=%.2f | BNO not detected\n",
        PS4.Battery(), vx, vy, w, pwmMax, w1, w2, w3);
    }
  }
}
