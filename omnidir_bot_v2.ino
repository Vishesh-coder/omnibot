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

void setup() {
  Serial.begin(115200);
  while (!Serial) 
  delay(10);

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
}

void loop() {
  updateIMU();

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
  y = -dpadSpeed;
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

  float w = readTriggerSpin();

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
    Serial.println("Heading zeroed");
  }

  float w1, w2, w3;
  inverseKinematics(vx, vy, w, w1, w2, w3);
  driveWheels(w1, w2, w3);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    if (imuReady) {
      Serial.printf(
        "PS4: connected=1 battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwm=%d | w1=%.2f w2=%.2f w3=%.2f | yaw=%.1f bno_status=%d\n",
        PS4.Battery(), vx, vy, w, pwmMax, w1, w2, w3, ypr.yaw, sensorValue.status);
    } else {
      Serial.printf(
        "PS4: connected=1 battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwm=%d | w1=%.2f w2=%.2f w3=%.2f | BNO not detected\n",
        PS4.Battery(), vx, vy, w, pwmMax, w1, w2, w3);
    }
  }
}
