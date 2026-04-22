/*
  Project: ESP32 Brushed Quadcopter Controller
  File: TX_Final.cpp
  Author: Ferr Manil
  Role: First Year Computer Engineering Student at McMaster University

  Description:
  This file implements the handheld transmitter for a custom low-cost brushed
  quadcopter. It reads two joysticks, handles arm/disarm and level-calibration
  commands, sends control packets to the drone over ESP-NOW, and prints flight
  telemetry received back from the receiver.
*/

#include <WiFi.h>
#include <esp_now.h>

#define PRINT_TEL_MONITOR_ON_USB 1
#define PRINT_TEL_PLOTTER_ON_USB 0

static const float PLOT_MAX_ROLL_DEG  = 30.0f;
static const float PLOT_MAX_PITCH_DEG = 30.0f;
static const float PLOT_MAX_YAW_DPS   = 90.0f;

#define THROTTLE_PIN 33
#define YAW_PIN      32
#define PITCH_PIN    35
#define ROLL_PIN     34

#define L_SW_PIN     26
#define R_SW_PIN     27

#define LED_PIN       2

uint8_t receiverMac[] = { 0x00, 0x70, 0x07, 0x82, 0xE0, 0xBC };

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  uint8_t  armed;
  uint8_t  calibrate_level;
  int16_t  thr;
  int16_t  yaw;
  int16_t  pitch;
  int16_t  roll;
} ControlPacket;

ControlPacket pkt;

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  uint8_t  seq;
  uint8_t  rx_ok;
  uint16_t rx_age_ms;

  uint8_t  armed_cmd;
  uint8_t  armedFly;
  uint8_t  stby;
  uint8_t  reserved0;

  int16_t  thr, yaw, pitch, roll;

  int16_t  roll_imu_cdeg;
  int16_t  pitch_imu_cdeg;

  int16_t  roll_pid_milli;
  int16_t  pitch_pid_milli;
  int16_t  yaw_pid_milli;

  uint8_t  m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
  uint16_t acc_mg;

} TelemetryPacket;

static TelemetryPacket lastTel;
static volatile uint32_t lastTelRxMs = 0;
static volatile bool haveTel = false;
static portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(TelemetryPacket)) return;

  portENTER_CRITICAL(&telMux);
  memcpy(&lastTel, data, sizeof(TelemetryPacket));
  lastTelRxMs = millis();
  haveTel = true;
  portEXIT_CRITICAL(&telMux);
}

static bool getTelemetry(TelemetryPacket &out, uint32_t &ageMs) {
  if (!haveTel) return false;

  portENTER_CRITICAL(&telMux);
  memcpy(&out, &lastTel, sizeof(TelemetryPacket));
  ageMs = millis() - lastTelRxMs;
  portEXIT_CRITICAL(&telMux);

  return true;
}

int cThr, cYaw, cPitch, cRoll;

bool armed = false;
bool lastL = true;
bool lastR = true;
uint32_t lastToggleMs = 0;

uint32_t armGraceUntilMs = 0;

uint32_t lastSendMs = 0;

uint32_t lastPrintMs = 0;
static const uint32_t PRINT_PERIOD_MS = 100;

static const int CAL_GESTURE_THRESHOLD_RAW = 900;
static const uint32_t LEVEL_CAL_HOLD_MS = 3000;
uint32_t levelCalHoldStartMs = 0;
bool levelCalTriggeredThisHold = false;

static inline int readAnalog(int pin) { return analogRead(pin); }

static inline int16_t mapAxis(int raw, int center, bool invert=false) {
  int v = raw - center;

  if (abs(v) < 40) v = 0;

  v = constrain(v, -1800, 1800);
  long out = (long)v * 500L / 1800L;
  if (invert) out = -out;
  return (int16_t)constrain(out, -500, 500);
}

static inline int16_t mapThrottle(int raw, int center) {
  int v = raw - center;

  v = constrain(v, -1800, 1800);

  long out = (long)(v + 1800) * 1000L / 3600L;
  out = constrain(out, 0, 1000);

  if (raw < center - 120) out = 0;

  return (int16_t)out;
}

static inline float cmdToSetpointDeg(int16_t cmd, float maxAbs) {
  return constrain((float)cmd, -500.0f, 500.0f) * (maxAbs / 500.0f);
}

void calibrateCenters() {
  long sThr=0, sYaw=0, sPitch=0, sRoll=0;
  const int N = 200;

  for (int i=0; i<N; i++) {
    sThr   += readAnalog(THROTTLE_PIN);
    sYaw   += readAnalog(YAW_PIN);
    sPitch += readAnalog(PITCH_PIN);
    sRoll  += readAnalog(ROLL_PIN);
    delay(5);
  }

  cThr   = sThr / N;
  cYaw   = sYaw / N;
  cPitch = sPitch / N;
  cRoll  = sRoll / N;
}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Controller MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.begin(115200);
  delay(200);
  Serial.println("\nController starting... keep sticks centered for calibration.");

  pinMode(L_SW_PIN, INPUT_PULLUP);
  pinMode(R_SW_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while(true) delay(1000);
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add peer FAILED");
    while(true) delay(1000);
  }

  esp_now_register_recv_cb(onEspNowRecv);

  calibrateCenters();
  Serial.print("Centers: thr=");   Serial.print(cThr);
  Serial.print(" yaw=");           Serial.print(cYaw);
  Serial.print(" pitch=");         Serial.print(cPitch);
  Serial.print(" roll=");          Serial.println(cRoll);

  Serial.println("Ready. Click either joystick to ARM/DISARM.");
}

void loop() {
  uint32_t now = millis();

  bool l = digitalRead(L_SW_PIN);
  bool r = digitalRead(R_SW_PIN);

  bool pressedEdge =
    (lastL == true && l == false) ||
    (lastR == true && r == false);

  if (pressedEdge && (now - lastToggleMs > 250)) {
    armed = !armed;
    lastToggleMs = now;

    if (armed) {
      armGraceUntilMs = now + 300;
      Serial.println("ARMED (grace throttle=0 for 300ms)");
    } else {
      armGraceUntilMs = 0;
      Serial.println("DISARMED");
    }
  }

  lastL = l;
  lastR = r;

  digitalWrite(LED_PIN, armed ? HIGH : LOW);

  int aThr   = readAnalog(THROTTLE_PIN);
  int aYaw   = readAnalog(YAW_PIN);
  int aPitch = readAnalog(PITCH_PIN);
  int aRoll  = readAnalog(ROLL_PIN);

  const bool levelGestureActive =
    !armed &&
    (aThr   < (cThr   - CAL_GESTURE_THRESHOLD_RAW)) &&
    (aYaw   > (cYaw   + CAL_GESTURE_THRESHOLD_RAW)) &&
    (aPitch < (cPitch - CAL_GESTURE_THRESHOLD_RAW)) &&
    (aRoll  < (cRoll  - CAL_GESTURE_THRESHOLD_RAW));

  bool calibrateLevelCmd = false;
  if (levelGestureActive) {
    if (levelCalHoldStartMs == 0) {
      levelCalHoldStartMs = now;
      levelCalTriggeredThisHold = false;
      Serial.println("Still IMU + level calibration gesture detected. Hold 3 seconds...");
    }

    if (!levelCalTriggeredThisHold && (now - levelCalHoldStartMs >= LEVEL_CAL_HOLD_MS)) {
      levelCalTriggeredThisHold = true;
      Serial.println("Still IMU + level calibration command sent.");
    }

    calibrateLevelCmd = levelCalTriggeredThisHold;
  } else {
    if (levelCalHoldStartMs != 0 && !levelCalTriggeredThisHold) {
      Serial.println("Still IMU + level calibration gesture cancelled.");
    }
    levelCalHoldStartMs = 0;
    levelCalTriggeredThisHold = false;
  }

  pkt.t_ms  = now;
  pkt.armed = armed ? 1 : 0;
  pkt.calibrate_level = calibrateLevelCmd ? 1 : 0;

  int16_t thr = mapThrottle(aThr, cThr);

  if (!armed || (armGraceUntilMs != 0 && now < armGraceUntilMs)) {
    thr = 0;
  }

  pkt.thr   = thr;
  pkt.yaw   = mapAxis(aYaw, cYaw, false);
  pkt.pitch = mapAxis(aPitch, cPitch, true);
  pkt.roll  = mapAxis(aRoll, cRoll, false);

  if (now - lastSendMs >= 20) {
    lastSendMs = now;
    esp_now_send(receiverMac, (uint8_t*)&pkt, sizeof(pkt));
  }

  
  TelemetryPacket t{};
  uint32_t age = 0;
  const bool haveTelNow = getTelemetry(t, age);

#if PRINT_TEL_MONITOR_ON_USB
  static uint32_t lastTelMonitorMs = 0;
  if (millis() - lastTelMonitorMs >= 50) {
    lastTelMonitorMs = millis();
    if (!haveTelNow) {
      Serial.println("TEL: none yet");
    } else {
      const float rollMad = t.roll_imu_cdeg / 100.0f;
      const float pitchMad = t.pitch_imu_cdeg / 100.0f;

      Serial.printf(
        "%lu -> TEL age=%lums ok=%d seq=%u | armedCmd=%d armedFly=%d STBY=%d | thr=%d yaw=%d pitch=%d roll=%d | IMU r=%.2f p=%.2f | PID r=%.3f p=%.3f y=%.3f | duty %3u %3u %3u %3u acc|=%.3f g aRej=%u%%\n",
        (unsigned long)millis(),
        (unsigned long)age,
        (int)t.rx_ok,
        (unsigned)t.seq,
        (int)t.armed_cmd,
        (int)t.armedFly,
        (int)t.stby,
        (int)t.thr, (int)t.yaw, (int)t.pitch, (int)t.roll,
        rollMad, pitchMad,
        t.roll_pid_milli / 1000.0f,
        t.pitch_pid_milli / 1000.0f,
        t.yaw_pid_milli / 1000.0f,
        t.m1_command_PWM, t.m2_command_PWM, t.m3_command_PWM, t.m4_command_PWM,
        t.acc_mg / 1000.0f,
        (unsigned)t.reserved0
      );
    }
  }
#endif

#if PRINT_TEL_PLOTTER_ON_USB && !PRINT_TEL_MONITOR_ON_USB
  static uint32_t lastTelPlotMs = 0;
  if (haveTelNow && (millis() - lastTelPlotMs >= 25)) {
    lastTelPlotMs = millis();

    const float rSet = cmdToSetpointDeg(t.roll,  PLOT_MAX_ROLL_DEG);
    const float pSet = cmdToSetpointDeg(t.pitch, PLOT_MAX_PITCH_DEG);
    const float ySet = cmdToSetpointDeg(t.yaw,   PLOT_MAX_YAW_DPS);

    const float rImu = t.roll_imu_cdeg / 100.0f;
    const float pImu = t.pitch_imu_cdeg / 100.0f;

    const float rPid = t.roll_pid_milli / 1000.0f;
    const float pPid = t.pitch_pid_milli / 1000.0f;
    const float yPid = t.yaw_pid_milli / 1000.0f;

    const float m1 = t.m1_command_PWM / 255.0f;
    const float m2 = t.m2_command_PWM / 255.0f;
    const float m3 = t.m3_command_PWM / 255.0f;
    const float m4 = t.m4_command_PWM / 255.0f;
    const float mMin = min(min(m1, m2), min(m3, m4));
    const float mMax = max(max(m1, m2), max(m3, m4));
    const float mSpan = mMax - mMin;

    const float thr = constrain(t.thr / 1000.0f, 0.0f, 1.0f);

    Serial.printf(
      "rSet:%.2f,rIMU:%.2f,rPid:%.3f,pSet:%.2f,pIMU:%.2f,pPid:%.3f,ySet:%.1f,yPid:%.3f,aLpf:%.3f,aRej:%.1f,thr:%.3f,mMin:%.3f,mMax:%.3f,mSpan:%.3f\n",
      rSet,
      rImu,
      rPid,
      pSet,
      pImu,
      pPid,
      ySet,
      yPid,
      t.acc_mg / 1000.0f,
      (float)t.reserved0,
      thr,
      mMin,
      mMax,
      mSpan
    );
  }
#endif

}

