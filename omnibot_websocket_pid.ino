#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char* AP_SSID     = "OmniBot";
const char* AP_PASSWORD = "omnibot123";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile float wsVx = 0.0f, wsVy = 0.0f, wsW = 0.0f;
volatile unsigned long lastWsMsgMs = 0;
const unsigned long WS_TIMEOUT_MS = 300;

enum ControlSource { SRC_NONE, SRC_WEB };
ControlSource activeSource = SRC_NONE;

float lastW1 = 0.0f, lastW2 = 0.0f, lastW3 = 0.0f;
float lastCmdVx = 0.0f, lastCmdVy = 0.0f, lastCmdW = 0.0f;

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

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
float yawTarget = 0.0f;

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

float spinGain = 0.35f - (17.0f / 120.0f);

void updateSpinGain() {
  spinGain = 0.35f - (17.0f / (float)pwmMax);
}

const int triggerDeadzone = 5;

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

  w1 = -sinf(t1) * vx + cosf(t1) * vy + spinGain * w;
  w2 = -sinf(t2) * vx + cosf(t2) * vy + spinGain * w;
  w3 = -sinf(t3) * vx + cosf(t3) * vy + spinGain * w;

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

void handleWsMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) {
    return;
  }

  StaticJsonDocument<192> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    Serial.printf("WS JSON parse failed: %s\n", err.c_str());
    return;
  }

  const char* type = doc["type"] | "";

  if (strcmp(type, "control") == 0) {
    int rawVx = doc["vx"] | 0;
    int rawVy = doc["vy"] | 0;
    int rawW  = doc["w"]  | 0;

    wsVx = constrain((float)rawVx / 127.0f, -1.0f, 1.0f);
    wsVy = constrain((float)rawVy / 127.0f, -1.0f, 1.0f);
    wsW  = constrain((float)rawW  / 127.0f, -1.0f, 1.0f);
    lastWsMsgMs = millis();

    Serial.printf("WS RX raw: vx=%d vy=%d w=%d -> scaled vx=%.2f vy=%.2f w=%.2f\n",
                  rawVx, rawVy, rawW, wsVx, wsVy, wsW);
    return;
  }

  if (strcmp(type, "frame") == 0) {
    bool g = doc["global"] | true;
    globalFrame = g;
    digitalWrite(LED_BUILTIN, globalFrame ? HIGH : LOW);
    Serial.printf("WS: frame set to %s\n", globalFrame ? "GLOBAL" : "ROBOT");
    return;
  }

  if (strcmp(type, "speed") == 0) {
    int delta = doc["delta"] | 0;
    speedLevel = (int)constrain(speedLevel + delta, 0, MAX_SPEED_LEVEL);
    pwmMax = 120 + (speedLevel * PWM_STEP);
    updateSpinGain();
    Serial.printf("WS: speed delta=%d -> speedLevel=%d pwmMax=%d spinGain=%.4f\n", delta, speedLevel, pwmMax, spinGain);
    return;
  }

  if (strcmp(type, "reset_heading") == 0) {
    resetHeading();
    yawTarget = ypr.yaw;
    resetPID();
    Serial.println("WS: heading zeroed");
    return;
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      wsVx = 0; wsVy = 0; wsW = 0;
      lastWsMsgMs = 0;
      break;
    case WS_EVT_DATA:
      handleWsMessage(arg, data, len);
      break;
    default:
      break;
  }
}

void broadcastTelemetry() {
  if (ws.count() == 0) return;

  StaticJsonDocument<384> doc;
  doc["yaw"]  = imuReady ? ypr.yaw : 0.0f;
  doc["yawTarget"] = yawTarget;

  doc["w1"] = roundf(lastW1 * 100.0f) / 100.0f;
  doc["w2"] = roundf(lastW2 * 100.0f) / 100.0f;
  doc["w3"] = roundf(lastW3 * 100.0f) / 100.0f;

  doc["pwm1"] = (int)(fabsf(lastW1) * pwmMax);
  doc["pwm2"] = (int)(fabsf(lastW2) * pwmMax);
  doc["pwm3"] = (int)(fabsf(lastW3) * pwmMax);

  doc["vx"] = roundf(lastCmdVx * 100.0f) / 100.0f;
  doc["vy"] = roundf(lastCmdVy * 100.0f) / 100.0f;
  doc["w"]  = roundf(lastCmdW  * 100.0f) / 100.0f;

  doc["globalFrame"] = globalFrame;

  doc["speedLevel"] = speedLevel;
  doc["pwmMax"] = pwmMax;

  String modeStr = (activeSource == SRC_WEB) ? "WEB" : "IDLE";
  modeStr += globalFrame ? "-GLOBAL" : "-ROBOT";
  doc["mode"] = modeStr;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
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

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP started. IP address: ");
  Serial.println(apIP);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("WebSocket server started at /ws");

  updateIMU();
  yawTarget = ypr.yaw;
  resetPID();
}

void loop() {
  updateIMU();
  ws.cleanupClients();

  bool webActive = (millis() - lastWsMsgMs) < WS_TIMEOUT_MS;

  float vx = 0.0f, vy = 0.0f, w = 0.0f;

  if (webActive) {
    activeSource = SRC_WEB;
    vx = wsVx;
    vy = wsVy;

    if (fabsf(wsW) > (triggerDeadzone / 255.0f)) {
      w = wsW;
      yawTarget = imuReady ? ypr.yaw : yawTarget;
      resetPID();
    } else if (imuReady) {
      w = yawPID(yawTarget, ypr.yaw);
    } else {
      w = 0.0f;
    }

  } else {
    activeSource = SRC_NONE;
    vx = 0; vy = 0; w = 0;
  }

  if (globalFrame && imuReady) {
    float heading = ypr.yaw - yawOffset;
    float vxRobot, vyRobot;
    worldToRobot(vx, vy, heading, vxRobot, vyRobot);
    vx = vxRobot;
    vy = vyRobot;
  }

  lastCmdVx = vx;
  lastCmdVy = vy;
  lastCmdW  = w;

  float w1, w2, w3;
  inverseKinematics(vx, vy, w, w1, w2, w3);
  lastW1 = w1; lastW2 = w2; lastW3 = w3;

  if (activeSource == SRC_NONE) {
    stopBot();
  } else {
    driveWheels(w1, w2, w3);
  }

  static unsigned long lastPrint = 0;
  static unsigned long lastTelemetry = 0;
  unsigned long now = millis();

  if (now - lastTelemetry > 100) {
    lastTelemetry = now;
    broadcastTelemetry();
  }

  if (now - lastPrint > 200) {
    lastPrint = now;
    const char* srcStr = (activeSource == SRC_WEB) ? "WEB" : "NONE";
    if (imuReady) {
      Serial.printf(
        "src=%s webActive=%d wsRaw(vx=%.2f vy=%.2f w=%.2f) | cmd(vx=%.2f vy=%.2f w=%.2f) pwm=%d spinGain=%.4f | wheels(w1=%.2f w2=%.2f w3=%.2f) | yaw=%.1f target=%.1f bno_status=%d | wsClients=%d\n",
        srcStr, webActive, (float)wsVx, (float)wsVy, (float)wsW,
        lastCmdVx, lastCmdVy, lastCmdW, pwmMax, spinGain, w1, w2, w3, ypr.yaw, yawTarget, sensorValue.status, ws.count());
    } else {
      Serial.printf(
        "src=%s webActive=%d wsRaw(vx=%.2f vy=%.2f w=%.2f) | cmd(vx=%.2f vy=%.2f w=%.2f) pwm=%d spinGain=%.4f | wheels(w1=%.2f w2=%.2f w3=%.2f) | BNO not detected | wsClients=%d\n",
        srcStr, webActive, (float)wsVx, (float)wsVy, (float)wsW,
        lastCmdVx, lastCmdVy, lastCmdW, pwmMax, spinGain, w1, w2, w3, ws.count());
    }
  }
}
