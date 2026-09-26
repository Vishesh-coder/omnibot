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

bool imuReady = false;
float yaw = 0.0f;
float yawTarget = 0.0f;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quatToYaw(float qr, float qi, float qj, float qk) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  yaw = atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
}

void updateIMU() {
  if (!imuReady) return;

  if (bno08x.wasReset()) {
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      auto rv = sensorValue.un.arvrStabilizedRV;
      quatToYaw(rv.real, rv.i, rv.j, rv.k);
    }
  }
}

#define M1_PWM 13
#define M1_DIR 12
#define M2_PWM 26
#define M2_DIR 25
#define M3_PWM 33
#define M3_DIR 32
#define LED    2

int pwmMax = 120;
int speedLevel = 0;
const int PWM_STEP = 20;
const int MAX_SPEED_LEVEL = 5;

const int joyDeadzone = 10;
const int triggerDeadzone = 10;

bool emergencyStop = false;

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

float Kp = 9.5;
float Ki = 1.2;
float Kd = 0.75;
float Yi = 0.0;
float E  = 0.0;
unsigned long thenPID = 0;

void resetPID() {
  Yi = 0.0;
  E = 0.0;
  thenPID = millis();
}

float yawPID(float Ty, float Cy) {
  unsigned long now = millis();
  float dt = (now - thenPID) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  thenPID = now;

  float e = Ty - Cy;

  while (e > 180.0f)  e -= 360.0f;
  while (e < -180.0f) e += 360.0f;

  Yi += e * dt;
  Yi = constrain(Yi, -100.0f, 100.0f);

  float d = (e - E) / dt;
  E = e;

  float output = (Kp * e) + (Ki * Yi) + (Kd * d);
  output = constrain(output, -100.0f, 100.0f);
  return output / 100.0f;
}

const float wheelAngles[3] = { 0.0f, 120.0f, 240.0f };
const float botRadius = 0.15f;

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

bool justPressed(bool now, bool &prevState) {
  bool pressed = now && !prevState;
  prevState = now;
  return pressed;
}

void ledStatus() {
  if (PS4.Square()) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
  }
  if (PS4.Triangle()) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  stopBot();
  Wire.begin();

  Serial.println("Starting BNO08x...");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    imuReady = false;
  } else {
    Serial.println("BNO08x Found!");
    setReports(reportType, reportIntervalUs);
    imuReady = true;
  }

  Serial.println("PS4 Controller connecting, just a minute");
  PS4.begin();

  updateIMU();
  yawTarget = yaw;
  resetPID();
}

void loop() {
  updateIMU();

  if (!PS4.isConnected() || PS4.PSButton()) {
    emergencyStop = true;
  } else {
    emergencyStop = false;
  }

  if (PS4.Share()) {
    emergencyStop = false;
  }

  if (emergencyStop) {
    stopBot();
    return;
  }

  static bool prevR1 = false, prevL1 = false;
  if (justPressed(PS4.R1(), prevR1) && speedLevel < MAX_SPEED_LEVEL) speedLevel++;
  if (justPressed(PS4.L1(), prevL1) && speedLevel > 0) speedLevel--;
  pwmMax = 120 + (speedLevel * PWM_STEP);
  pwmMax = constrain(pwmMax, 0, 255);

  ledStatus();

  float lx = readStick(PS4.LStickX());
  float ly = readStick(PS4.LStickY());
  float vx = 0.0f, vy = 0.0f;
  bool dpadActive = true;

  if (PS4.Up())            vx = -1.0f;
  else if (PS4.Down())     vx =  1.0f;
  else if (PS4.Left())     vy = -1.0f;
  else if (PS4.Right())    vy =  1.0f;
  else if (PS4.UpRight())   { vx = -1.0f; vy =  1.0f; }
  else if (PS4.DownRight()) { vx =  1.0f; vy =  1.0f; }
  else if (PS4.UpLeft())    { vx = -1.0f; vy = -1.0f; }
  else if (PS4.DownLeft())  { vx =  1.0f; vy = -1.0f; }
  else dpadActive = false;

  if (!dpadActive) {
    vx = -ly;
    vy = lx;
  }

  static bool globalFrame = true;
  static bool prevTriangleMode = false, prevCircleMode = false;
  if (justPressed(PS4.Triangle(), prevTriangleMode)) globalFrame = false;
  if (justPressed(PS4.Circle(), prevCircleMode))     globalFrame = true;
  digitalWrite(LED, globalFrame ? HIGH : LOW);

  if (globalFrame && imuReady) {
    float heading = yaw - yawTarget;
    float headingRad = radians(heading);
    float c = cosf(headingRad);
    float s = sinf(headingRad);
    float vxRobot =  vx * c + vy * s;
    float vyRobot = -vx * s + vy * c;
    vx = vxRobot;
    vy = vyRobot;
  }

  static bool prevOptions = false;
  if (justPressed(PS4.Options(), prevOptions)) {
    yawTarget = yaw;
  }

  int l2 = PS4.L2Value();
  int r2 = PS4.R2Value();
  if (abs(r2) < triggerDeadzone) r2 = 0;
  if (abs(l2) < triggerDeadzone) l2 = 0;

  float w;
  if (abs(l2 - r2) > triggerDeadzone) {
    w = (r2 - l2) / 255.0f;
    yawTarget = yaw;
    resetPID();
  } else {
    w = yawPID(yawTarget, yaw);
  }

  float w1, w2, w3;
  inverseKinematics(vx, vy, w, w1, w2, w3);
  driveWheels(w1, w2, w3);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf(
      "PS4 battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwmMax=%d | w1=%.2f w2=%.2f w3=%.2f | yaw=%.1f target=%.1f frame=%s\n",
      PS4.Battery(), vx, vy, w, pwmMax, w1, w2, w3, yaw, yawTarget, globalFrame ? "GLOBAL" : "ROBOT");
  }
}
