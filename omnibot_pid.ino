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

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void updateIMU() {
  if (!imuReady) return;

  if (bno08x.wasReset()) {
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
    }
  }
}

void globalFrameTransform(float vx_world, float vy_world, float yawDeg,
                             float &vx_robot, float &vy_robot) {
  float yawRad = radians(yawDeg);
  float cosYaw = cosf(yawRad);
  float sinYaw = sinf(yawRad);

  vx_robot =  vx_world * cosYaw + vy_world * sinYaw;
  vy_robot = -vx_world * sinYaw + vy_world * cosYaw;
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

int PWM_MAX = 120;
int PWM_COUNTER = 0;
int yaw=0;

const float WHEEL_ANGLES_DEG[3] = { 0.0f, 120.0f, 240.0f };

const float BOT_RADIUS = 0.15f;

const int JOY_OFFSET = 10;
const int TRIGGER_OFFSET = 5;

const float DPAD_SPEED_LIM = 1.0f;
const float MAX_SPIN_RATE = 1.0f;

void setMotor(uint8_t pwmPin, uint8_t dirPin, float normSpeed) {
  normSpeed = constrain(normSpeed, -1.0f, 1.0f);
  digitalWrite(dirPin, normSpeed >= 0.0f ? HIGH : LOW);
  analogWrite(pwmPin, abs(normSpeed) * PWM_MAX);
}

void stop_bot() {
  setMotor(M1_PWM, M1_DIR, 0);
  setMotor(M2_PWM, M2_DIR, 0);
  setMotor(M3_PWM, M3_DIR, 0);
}

void inverseKinematics(float vx, float vy, float w,
                        float &w1, float &w2, float &w3) {
  float t1 = radians(WHEEL_ANGLES_DEG[0]);
  float t2 = radians(WHEEL_ANGLES_DEG[1]);
  float t3 = radians(WHEEL_ANGLES_DEG[2]);

  w1 = -sinf(t1) * vx + cosf(t1) * vy + BOT_RADIUS * w;
  w2 = -sinf(t2) * vx + cosf(t2) * vy + BOT_RADIUS * w;
  w3 = -sinf(t3) * vx + cosf(t3) * vy + BOT_RADIUS * w;

  float maxMag = max(abs(w1), max(abs(w2), abs(w3)));
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

float readJoystick(int8_t joystick) {
  if (abs(joystick) < JOY_OFFSET) return 0.0f;
  return constrain((float)joystick / 127.0f, -1.0f, 1.0f);
}

float readTriggerRotation() {
  int r2 = PS4.R2Value();
  int l2 = PS4.L2Value();
  float rNorm = (r2 > TRIGGER_OFFSET) ? (float)r2 / 255.0f : 0.0f;
  float lNorm = (l2 > TRIGGER_OFFSET) ? (float)l2 / 255.0f : 0.0f;
  return constrain((rNorm - lNorm) * MAX_SPIN_RATE, -1.0f, 1.0f);
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

  stop_bot();

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

  bool connected = PS4.isConnected();
  if (!connected) {
    stop_bot();
    Serial.println("PS4: disconnected");
    delay(200);
    return;
  }



  float lx = readJoystick(PS4.LStickX());
  float ly = readJoystick(PS4.LStickY());
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  bool dpadActive = false;

  if (PS4.Up()) {
    vx = DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.Down()) {
    vx = -DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.Left()) {
    vy = -DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.Right()) {
    vy = DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.UpRight()) {
    vx = DPAD_SPEED_LIM;
    vy = DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.DownRight()) {
    vx = -DPAD_SPEED_LIM;
    vy = DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.UpLeft()) {
    vx = DPAD_SPEED_LIM;
    vy = -DPAD_SPEED_LIM;
    dpadActive = true;
  } else if (PS4.DownLeft()) {
    vx = -DPAD_SPEED_LIM;
    vy = -DPAD_SPEED_LIM;
    dpadActive = true;
  }

  static bool lastTriangleState = false;
  static bool lastCircleState = false;
  bool currentTriangleState = PS4.Triangle();
  bool currentCircleState = PS4.Circle();

  if (currentTriangleState && !lastTriangleState) {
    globalFrame = false;
    Serial.println("MODE:BOT FRAME");
  }
  if (currentCircleState && !lastCircleState) {
    globalFrame = true;
    Serial.println("MODE:GLOBAL FRAME");
  }
  lastTriangleState = currentTriangleState;
  lastCircleState = currentCircleState;

  if(globalFrame){
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
  }

  static bool lastR1State = false;
  static bool lastL1State = false;
  bool currentR1State = PS4.R1();
  bool currentL1State = PS4.L1();

  if (currentR1State && !lastR1State) {
    if (PWM_COUNTER < 5) {
      PWM_COUNTER++;
      PWM_MAX = 120 + (PWM_COUNTER * 20);
    }
  }
  lastR1State = currentR1State;

  if (currentL1State && !lastL1State) {
    if (PWM_COUNTER > 0) {
      PWM_COUNTER--;
      PWM_MAX = 120 + (PWM_COUNTER * 20);
    }
  }
  lastL1State = currentL1State;

  if (!dpadActive) {
    vx = ly;
    vy = lx;
  }

  w = readTriggerRotation();

  if (globalFrame && imuReady) {
    float correctedYaw = ypr.yaw - yawOffset;
    float vx_fo, vy_fo;
    globalFrameTransform(vx, vy, correctedYaw, vx_fo, vy_fo);
    vx = vx_fo;
    vy = vy_fo;
  }

  static bool lastOptionsState = false;
  bool optionsState = PS4.Options();
  if (optionsState && !lastOptionsState) {
    resetHeading();
    Serial.println("Heading zeroed");
  }
  lastOptionsState = optionsState;

  float w1, w2, w3;
    //pid
  float vxerror;
  float vyerror;
  vxerror=math.sqrt(vx*vx+vy*vy)*(cos(ypr.yaw-yawOffset)-cos(yaw-yawOffset));
  vyerror=math.sqrt(vx*vx+vy*vy)*(sin(ypr.yaw-yawOffset)-sin(yaw-yawOffset));
  inverseKinematics(vx-vxerror, vy-vyerror, w, w1, w2, w3);
  if(!w){
    inverseKinematics(vx, vy, w, w1, w2, w3);
  }
  driveWheels(w1, w2, w3);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    if (imuReady) {
      Serial.printf(
        "PS4: connected=%d battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwm=%d | w1=%.2f w2=%.2f w3=%.2f | yaw=%.1f bno_status=%d\n",
        connected, PS4.Battery(), vx, vy, w, PWM_MAX, w1, w2, w3, ypr.yaw, sensorValue.status);
    } else {
      Serial.printf(
        "PS4: connected=%d battery=%d/5 | vx=%.2f vy=%.2f w=%.2f pwm=%d | w1=%.2f w2=%.2f w3=%.2f | BNO not detected\n",
        connected, PS4.Battery(), vx, vy, w, PWM_MAX, w1, w2, w3);
    }
  }
  yaw=ypr.yaw-yawOffset
}
