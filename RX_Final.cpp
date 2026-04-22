/*
  Project: ESP32 Brushed Quadcopter Flight Controller
  File: RX_Final.cpp
  Author: Ferr Manil
  Role: First Year Computer Engineering Student at McMaster University

  Description:
  This file implements the drone-side flight controller for a custom low-cost
  brushed quadcopter. It receives pilot commands over ESP-NOW, reads and filters
  MPU6050 IMU data, estimates attitude with a 6DOF Madgwick-based pipeline,
  applies roll/pitch angle PID and yaw rate PID control, mixes commands for four
  brushed motors, and returns telemetry to the transmitter.
*/


  #include <Arduino.h>
  #include <Wire.h>
  #include <WiFi.h>
  #include <esp_now.h>

  #define MOTOR_ISOLATION_MODE 0

  #define ALL_MOTORS_SAME_THRUST_MODE 0

  #define PRINT_TEL_ON_USB 1

  static const int LOOP_HZ = 500;

  static const int PWM_FREQ = 20000;
  static const int PWM_RES  = 8;

  static const int M1_PIN = 25;
  static const int M2_PIN = 26;
  static const int M3_PIN = 27;
  static const int M4_PIN = 14;

  static const int STBY_PIN = 33;

  static const int I2C_SDA_PIN = 22;
  static const int I2C_SCL_PIN = 21;

  enum MotorSpinDirection : int8_t {
    MOTOR_SPIN_CW  = -1,
    MOTOR_SPIN_CCW =  1,
  };
  static inline const char* motorSpinName(MotorSpinDirection dir) {
    return (dir == MOTOR_SPIN_CCW) ? "CCW" : "CW";
  }
  static const MotorSpinDirection M1_SPIN = MOTOR_SPIN_CW;
  static const MotorSpinDirection M2_SPIN = MOTOR_SPIN_CCW;
  static const MotorSpinDirection M3_SPIN = MOTOR_SPIN_CW;
  static const MotorSpinDirection M4_SPIN = MOTOR_SPIN_CCW;

  static const uint32_t FAILSAFE_MS = 250;

  float i_limit   = 25.0f;
  float maxRoll   = 30.0f;
  float maxPitch  = 30.0f;
  float maxYaw    = 160.0f;

  float Kp_roll_angle  = 0.33f;    
  float Ki_roll_angle  = 0.18f;   
  float Kd_roll_angle  = 0.06f;    
  float Kp_pitch_angle = 0.33f;    
  float Ki_pitch_angle = 0.18f;    
  float Kd_pitch_angle = 0.06f;    

  float Kp_yaw = 0.005;
  float Ki_yaw = 0.0f;
  float Kd_yaw = 0.00f;
  float yaw_pid_limit = 0.08f;

  float B_madgwick_armed    = 0.06f;
  float B_madgwick_disarmed = 0.50f;
  float B_accel    = 0.27f;
  float B_gyro     = 0.10f;

  float B_madgwick_recovery  = 0.20f;
  float recovery_threshold_deg = 5.0f;
  float recovery_ramp_rate   = 0.01f;

  int MIN_DUTY_WHEN_RUNNING = 0;

  float accMag_g = 1.0f;
  float accTrustWeight = 1.0f;
  float accRejectPct = 0.0f;
  float currentBeta = 0.02f;

  static const float ACC_TRUST_HARD_LOW_G  = 0.70f;
  static const float ACC_TRUST_HARD_HIGH_G = 1.30f;
  static const float ACC_TRUST_SOFT_ERR_G  = 0.25f;

  static const int STARTUP_IMU_CAL_DISCARD_SAMPLES = 100;
  static const int STARTUP_IMU_CAL_SAMPLES         = 800;
  static const int STARTUP_IMU_CAL_DELAY_MS        = 2;
  static const int LEVEL_CAPTURE_DISCARD_SAMPLES   = 30;
  static const int LEVEL_CAPTURE_SAMPLES           = 80;

  static uint8_t CONTROLLER_MAC[6] = {0x4C,0xC3,0x82,0xDA,0xF8,0x38};

  struct __attribute__((packed)) ControlPacket {
    uint32_t t_ms;
    uint8_t  armed;
    uint8_t  calibrate_level;
    int16_t  thr;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;
  };

  struct __attribute__((packed)) TelemetryPacket {
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
  };

  static portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
  static ControlPacket lastPacket{};
  static uint32_t lastPacketRxMs = 0;
  static bool havePacket = false;

  static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    

    if (len != (int)sizeof(ControlPacket)) return;
    

    portENTER_CRITICAL(&packetMux);
    memcpy(&lastPacket, data, sizeof(ControlPacket));
    lastPacketRxMs = millis();
    havePacket = true;
    portEXIT_CRITICAL(&packetMux);
  }

  static bool getLatestPacket(void *outBuf, size_t outSize, uint32_t &ageMs) {
    uint32_t rxMs;
    bool ok;
    

    portENTER_CRITICAL(&packetMux);
    if (outBuf != nullptr && outSize >= sizeof(ControlPacket)) {
      memcpy(outBuf, &lastPacket, sizeof(ControlPacket));
    }
    rxMs = lastPacketRxMs;
    ok = havePacket;
    portEXIT_CRITICAL(&packetMux);
    

    uint32_t now = millis();
    ageMs = ok ? (now - rxMs) : 0xFFFFFFFFu;
    return ok;
  }

  static const uint8_t MPU_ADDR       = 0x68;
  static const uint8_t REG_WHOAMI     = 0x75;
  static const uint8_t REG_PWR_MGMT_1 = 0x6B;
  static const uint8_t REG_SMPLRT_DIV = 0x19;
  static const uint8_t REG_CONFIG     = 0x1A;
  static const uint8_t REG_GYRO_CFG   = 0x1B;
  static const uint8_t REG_ACCEL_CFG  = 0x1C;
  static const uint8_t REG_DATA       = 0x3B;

  static const float GYRO_SCALE_FACTOR  = 131.0f;
  static const float ACCEL_SCALE_FACTOR = 16384.0f;

  static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    

    size_t got = Wire.requestFrom((int)addr, (int)n, (int)true);
    if (got != n) return false;
    

    for (size_t i=0;i<n;i++) buf[i] = Wire.read();
    return true;
  }

  static bool mpuInit() {
    uint8_t who=0;
    

    if (!i2cReadBytes(MPU_ADDR, REG_WHOAMI, &who, 1)) return false;
    if (who != 0x68) return false;
    

    if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false;
    delay(50);
    

    i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 0x00);
    

    i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03);
    

    i2cWriteByte(MPU_ADDR, REG_GYRO_CFG, 0x00);
    

    i2cWriteByte(MPU_ADDR, REG_ACCEL_CFG, 0x00);
    
    delay(10);
    return true;
  }

  static bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    uint8_t b[14];
    

    if (!i2cReadBytes(MPU_ADDR, REG_DATA, b, 14)) return false;
    

    ax = (int16_t)((b[0]<<8)  | b[1]);
    ay = (int16_t)((b[2]<<8)  | b[3]);
    az = (int16_t)((b[4]<<8)  | b[5]);
    gx = (int16_t)((b[8]<<8)  | b[9]);
    gy = (int16_t)((b[10]<<8) | b[11]);
    gz = (int16_t)((b[12]<<8) | b[13]);
    return true;
  }

  static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
  static void getIMUdata();

  float dt = 0.0f;
  uint32_t prev_time = 0;

  float channel_1_pwm = 1000.0f;
  float channel_2_pwm = 1500.0f;
  float channel_3_pwm = 1500.0f;
  float channel_4_pwm = 1500.0f;
  float channel_5_pwm = 2000.0f;
  const float channel_1_fs = 1000.0f;
  const float channel_2_fs = 1500.0f;
  const float channel_3_fs = 1500.0f;
  const float channel_4_fs = 1500.0f;
  const float channel_5_fs = 2000.0f;

  float AccX=0, AccY=0, AccZ=0;
  float GyroX=0, GyroY=0, GyroZ=0;

  float AccX_prev=0, AccY_prev=0, AccZ_prev=0;
  float GyroX_prev=0, GyroY_prev=0, GyroZ_prev=0;

  float AccErrorX = 0.0f, AccErrorY = 0.0f, AccErrorZ = 0.0f;
  float GyroErrorX = 0.0f, GyroErrorY = 0.0f, GyroErrorZ = 0.0f;

  float roll_IMU=0, pitch_IMU=0, yaw_IMU=0;
  float q0=1, q1=0, q2=0, q3=0;
  float levelRollTrimDeg = 0.0f;
  float levelPitchTrimDeg = 0.0f;
  bool calibrateLevelCmd = false;
  bool prevCalibrateLevelCmd = false;

  float thro_des=0;
  float roll_des=0;
  float pitch_des=0;
  float yaw_des=0;

  float error_roll=0;
  float integral_roll=0;
  float derivative_roll=0;
  float roll_PID=0;

  float error_pitch=0;
  float integral_pitch=0;
  float derivative_pitch=0;
  float pitch_PID=0;

  float error_yaw=0;
  float error_yaw_prev=0;
  float integral_yaw=0;
  float derivative_yaw=0;
  float yaw_PID=0;

  float m1_command_scaled=0, m2_command_scaled=0, m4_command_scaled=0, m3_command_scaled=0;
  int m1_command_PWM=0, m2_command_PWM=0, m4_command_PWM=0, m3_command_PWM=0;

  bool armedFly = false;

  static bool calibrateImuStill() {
    float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
    float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < STARTUP_IMU_CAL_DISCARD_SAMPLES + STARTUP_IMU_CAL_SAMPLES; ++i) {
      int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
      if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
        if (i >= STARTUP_IMU_CAL_DISCARD_SAMPLES) {
          sumAx += (float)axRaw / ACCEL_SCALE_FACTOR;
          sumAy += (float)ayRaw / ACCEL_SCALE_FACTOR;
          sumAz += (float)azRaw / ACCEL_SCALE_FACTOR;
          sumGx += (float)gxRaw / GYRO_SCALE_FACTOR;
          sumGy += (float)gyRaw / GYRO_SCALE_FACTOR;
          sumGz += (float)gzRaw / GYRO_SCALE_FACTOR;
          ++validSamples;
        }
      }
      delay(STARTUP_IMU_CAL_DELAY_MS);
    }

    if (validSamples < (STARTUP_IMU_CAL_SAMPLES / 2)) {
      return false;
    }

    const float invCount = 1.0f / (float)validSamples;
    const float avgAx = sumAx * invCount;
    const float avgAy = sumAy * invCount;
    const float avgAz = sumAz * invCount;
    const float avgGx = sumGx * invCount;
    const float avgGy = sumGy * invCount;
    const float avgGz = sumGz * invCount;
    const float avgAccNorm = sqrtf(avgAx*avgAx + avgAy*avgAy + avgAz*avgAz);

    GyroErrorX = avgGx;
    GyroErrorY = avgGy;
    GyroErrorZ = avgGz;

    if (avgAccNorm > 0.80f && avgAccNorm < 1.20f) {
      const float gravityBias = avgAccNorm - 1.0f;
      const float gravityDirX = avgAx / avgAccNorm;
      const float gravityDirY = avgAy / avgAccNorm;
      const float gravityDirZ = avgAz / avgAccNorm;

      AccErrorX = gravityBias * gravityDirX;
      AccErrorY = gravityBias * gravityDirY;
      AccErrorZ = gravityBias * gravityDirZ;
    } else {
      AccErrorX = 0.0f;
      AccErrorY = 0.0f;
      AccErrorZ = 0.0f;
    }

    AccX_prev = avgAx - AccErrorX;
    AccY_prev = avgAy - AccErrorY;
    AccZ_prev = avgAz - AccErrorZ;
    GyroX_prev = 0.0f;
    GyroY_prev = 0.0f;
    GyroZ_prev = 0.0f;
    accMag_g = sqrtf(AccX_prev*AccX_prev + AccY_prev*AccY_prev + AccZ_prev*AccZ_prev);

    Serial.printf(
      "Still IMU calibration: accNormRaw=%.3f g accNormCorrected=%.3f g | accBias=%.4f,%.4f,%.4f g | gyroBias=%.3f,%.3f,%.3f dps\n",
      avgAccNorm,
      accMag_g,
      AccErrorX, AccErrorY, AccErrorZ,
      GyroErrorX, GyroErrorY, GyroErrorZ
    );

    return true;
  }

  static float invSqrt(float x) { return 1.0f / sqrtf(x); }

  static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    float accMag = sqrtf(ax*ax + ay*ay + az*az);

    float accW = 1.0f;
    if (accMag < ACC_TRUST_HARD_LOW_G || accMag > ACC_TRUST_HARD_HIGH_G) {
      accW = 0.0f;
    } else {

      float err = fabsf(accMag - 1.0f);
      accW = 1.0f - (err / ACC_TRUST_SOFT_ERR_G);
      accW = constrain(accW, 0.0f, 1.0f);
    }

    accTrustWeight = accW;
    accRejectPct = (1.0f - accW) * 100.0f;

    

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

      recipNorm = invSqrt(ax*ax + ay*ay + az*az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      _2q0 = 2.0f*q0;
      _2q1 = 2.0f*q1;
      _2q2 = 2.0f*q2;
      _2q3 = 2.0f*q3;
      _4q0 = 4.0f*q0;
      _4q1 = 4.0f*q1;
      _4q2 = 4.0f*q2;
      _8q1 = 8.0f*q1;
      _8q2 = 8.0f*q2;
      q0q0 = q0*q0;
      q1q1 = q1*q1;
      q2q2 = q2*q2;
      q3q3 = q3*q3;

      s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
      s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
      s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
      s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

      recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      float betaBase;
      if (!armedFly) {
        betaBase = B_madgwick_disarmed;
        currentBeta = B_madgwick_armed;
      } else {

        float accelRoll  = atan2f(ay, az) * 57.29577951f;
        float accelPitch = -atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29577951f;
        float errDeg = max(fabsf(roll_IMU - accelRoll), fabsf(pitch_IMU - accelPitch));

        if (accW > 0.8f && errDeg > recovery_threshold_deg) {
          currentBeta = B_madgwick_recovery;
        } else {
          currentBeta -= recovery_ramp_rate;
          if (currentBeta < B_madgwick_armed) currentBeta = B_madgwick_armed;
        }
        betaBase = currentBeta;
      }
      float betaEff = betaBase * accW;
      qDot1 -= betaEff*s0;
      qDot2 -= betaEff*s1;
      qDot3 -= betaEff*s2;
      qDot4 -= betaEff*s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    roll_IMU  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29577951f;
    pitch_IMU = asinf(constrain(-2.0f*(q1*q3 - q0*q2), -0.999999f, 0.999999f)) * 57.29577951f;
    yaw_IMU   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29577951f;
  }

  static void getIMUdata() {
    int16_t ax,ay,az,gx,gy,gz;
    if (!mpuReadRaw(ax,ay,az,gx,gy,gz)) return;

    AccX = (float)ax  / ACCEL_SCALE_FACTOR;
    AccY = (float)ay  / ACCEL_SCALE_FACTOR;
    AccZ = (float)az  / ACCEL_SCALE_FACTOR;
    GyroX = (float)gx / GYRO_SCALE_FACTOR;
    GyroY = (float)gy / GYRO_SCALE_FACTOR;
    GyroZ = (float)gz / GYRO_SCALE_FACTOR;

    AccX -= AccErrorX; AccY -= AccErrorY; AccZ -= AccErrorZ;
    GyroX -= GyroErrorX; GyroY -= GyroErrorY; GyroZ -= GyroErrorZ;

    AccX = (1.0f - B_accel)*AccX_prev + B_accel*AccX;
    AccY = (1.0f - B_accel)*AccY_prev + B_accel*AccY;
    AccZ = (1.0f - B_accel)*AccZ_prev + B_accel*AccZ;
    AccX_prev = AccX; AccY_prev = AccY; AccZ_prev = AccZ;

    GyroX = (1.0f - B_gyro)*GyroX_prev + B_gyro*GyroX;
    GyroY = (1.0f - B_gyro)*GyroY_prev + B_gyro*GyroY;
    GyroZ = (1.0f - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
    GyroX_prev = GyroX; GyroY_prev = GyroY; GyroZ_prev = GyroZ;

    accMag_g = sqrtf(AccX*AccX + AccY*AccY + AccZ*AccZ);

  }

  static void getCommandsFromEspNow() {
    ControlPacket p{};
    uint32_t age;
    bool ok = getLatestPacket(&p, sizeof(p), age);

    if (!ok || age > FAILSAFE_MS) {

      channel_1_pwm = channel_1_fs;
      channel_2_pwm = channel_2_fs;
      channel_3_pwm = channel_3_fs;
      channel_4_pwm = channel_4_fs;
      channel_5_pwm = channel_5_fs;
      armedFly = false;
      calibrateLevelCmd = false;
      return;
    }

    int16_t thr   = constrain((int)p.thr,   0, 1000);
    int16_t roll  = constrain((int)p.roll,  -500, 500);
    int16_t pitch = constrain((int)p.pitch, -500, 500);
    int16_t yaw   = constrain((int)p.yaw,   -500, 500);

    channel_1_pwm = 1000.0f + (float)thr;
    channel_2_pwm = 1500.0f + (float)roll;
    channel_3_pwm = 1500.0f + (float)pitch;
    channel_4_pwm = 1500.0f + (float)yaw;

    bool pktArmed = (p.armed != 0);
    channel_5_pwm = pktArmed ? 1000.0f : 2000.0f;
    calibrateLevelCmd = (!pktArmed) && (p.calibrate_level != 0);

    if (!pktArmed) {
      armedFly = false;
    } else if (!armedFly) {
      if (channel_1_pwm < 1100.0f) armedFly = true;
    }
  }

  static void applyLevelTrimToAttitude() {
    roll_IMU -= levelRollTrimDeg;
    pitch_IMU -= levelPitchTrimDeg;
  }

  static inline float bodyRollRateDps()  { return -GyroY; }
  static inline float bodyPitchRateDps() { return  GyroX; }
  static inline float bodyYawRateDps()   { return -GyroZ; }

  static inline void getBodyAccelG(float &axBody, float &ayBody, float &azBody) {
    axBody = -AccY;
    ayBody =  AccX;
    azBody =  AccZ;
  }

  static inline void getBodyGyroDps(float &gxBody, float &gyBody, float &gzBody) {
    gxBody = bodyRollRateDps();
    gyBody = bodyPitchRateDps();
    gzBody = bodyYawRateDps();
  }

  static void seedAttitudeFromCurrentAccel() {

    float ax, ay, az;
    getBodyAccelG(ax, ay, az);

    const float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.5f) {
      q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
      roll_IMU = 0.0f;
      pitch_IMU = 0.0f;
      yaw_IMU = 0.0f;
      return;
    }

    ax /= norm;
    ay /= norm;
    az /= norm;

    const float rollRad = atan2f(ay, az);
    const float pitchRad = atan2f(-ax, sqrtf(ay*ay + az*az));
    const float yawRad = 0.0f;

    const float cr = cosf(rollRad * 0.5f);
    const float sr = sinf(rollRad * 0.5f);
    const float cp = cosf(pitchRad * 0.5f);
    const float sp = sinf(pitchRad * 0.5f);
    const float cy = cosf(yawRad * 0.5f);
    const float sy = sinf(yawRad * 0.5f);

    q0 = cr*cp*cy + sr*sp*sy;
    q1 = sr*cp*cy - cr*sp*sy;
    q2 = cr*sp*cy + sr*cp*sy;
    q3 = cr*cp*sy - sr*sp*cy;

    roll_IMU  = rollRad  * 57.29577951f;
    pitch_IMU = pitchRad * 57.29577951f;
    yaw_IMU   = 0.0f;
  }

  static void refreshAttitudeEstimateFromCurrentPose() {
    getIMUdata();
    seedAttitudeFromCurrentAccel();
    applyLevelTrimToAttitude();
  }

  static bool captureAverageLevelTrim(float &rollTrimOut, float &pitchTrimOut) {
    const float settleDt = max(STARTUP_IMU_CAL_DELAY_MS, 1) * 0.001f;
    float sumRoll = 0.0f;
    float sumPitch = 0.0f;
    int validSamples = 0;

    getIMUdata();
    seedAttitudeFromCurrentAccel();

    for (int i = 0; i < LEVEL_CAPTURE_DISCARD_SAMPLES + LEVEL_CAPTURE_SAMPLES; ++i) {
      delay(STARTUP_IMU_CAL_DELAY_MS);
      getIMUdata();
      float gxBody, gyBody, gzBody;
      float axBody, ayBody, azBody;
      getBodyGyroDps(gxBody, gyBody, gzBody);
      getBodyAccelG(axBody, ayBody, azBody);
      Madgwick6DOF(gxBody, gyBody, gzBody, axBody, ayBody, azBody, settleDt);

      if (i >= LEVEL_CAPTURE_DISCARD_SAMPLES) {
        sumRoll += roll_IMU;
        sumPitch += pitch_IMU;
        ++validSamples;
      }
    }

    if (validSamples < (LEVEL_CAPTURE_SAMPLES / 2)) {
      return false;
    }

    rollTrimOut = sumRoll / (float)validSamples;
    pitchTrimOut = sumPitch / (float)validSamples;
    return true;
  }

  static void forceOutputsSafeAndResetControllers() {
    armedFly = false;
    channel_1_pwm = 1000.0f;
    channel_2_pwm = 1500.0f;
    channel_3_pwm = 1500.0f;
    channel_4_pwm = 1500.0f;
    channel_5_pwm = 2000.0f;
    thro_des = 0.0f;
    roll_des = 0.0f;
    pitch_des = 0.0f;
    yaw_des = 0.0f;
    m1_command_scaled = m2_command_scaled = m3_command_scaled = m4_command_scaled = 0.0f;
    m1_command_PWM = m2_command_PWM = m3_command_PWM = m4_command_PWM = 0;
    error_roll = 0.0f;
    error_pitch = 0.0f;
    error_yaw = 0.0f;
    error_yaw_prev = 0.0f;
    integral_roll = 0.0f;
    integral_pitch = 0.0f;
    integral_yaw = 0.0f;
    derivative_roll = 0.0f;
    derivative_pitch = 0.0f;
    derivative_yaw = 0.0f;
    roll_PID = 0.0f;
    pitch_PID = 0.0f;
    yaw_PID = 0.0f;
    currentBeta = B_madgwick_armed;

    digitalWrite(STBY_PIN, LOW);
    ledcWrite(M1_PIN, 0);
    ledcWrite(M2_PIN, 0);
    ledcWrite(M3_PIN, 0);
    ledcWrite(M4_PIN, 0);
  }

  static void maybeApplyLevelCalibration() {
    const bool canCalibrateNow = calibrateLevelCmd && !armedFly && (channel_1_pwm < 1050.0f);

    if (canCalibrateNow && !prevCalibrateLevelCmd) {
      forceOutputsSafeAndResetControllers();

  #if PRINT_TEL_ON_USB
      Serial.println("Keep the drone still: running joystick IMU + level calibration...");
  #endif

      if (calibrateImuStill()) {
        float measuredRollTrim = 0.0f;
        float measuredPitchTrim = 0.0f;
        const bool haveLevelTrim = captureAverageLevelTrim(measuredRollTrim, measuredPitchTrim);

        if (haveLevelTrim) {
          levelRollTrimDeg = measuredRollTrim;
          levelPitchTrimDeg = measuredPitchTrim;
        } else {
          levelRollTrimDeg = roll_IMU;
          levelPitchTrimDeg = pitch_IMU;
        }
        refreshAttitudeEstimateFromCurrentPose();
        if (fabsf(roll_IMU) < 0.25f) roll_IMU = 0.0f;
        if (fabsf(pitch_IMU) < 0.25f) pitch_IMU = 0.0f;
        yaw_IMU = 0.0f;

  #if PRINT_TEL_ON_USB
        Serial.printf(
          "Joystick IMU + level calibration applied: rollTrim=%.2f deg pitchTrim=%.2f deg acc|=%.3f g\n",
          levelRollTrimDeg,
          levelPitchTrimDeg,
          accMag_g
        );
  #endif
      } else {
  #if PRINT_TEL_ON_USB
        Serial.println("Joystick IMU calibration failed. Keeping previous offsets.");
  #endif
      }
    }

    prevCalibrateLevelCmd = canCalibrateNow;
  }

  static void getDesState() {

    float thr_norm  = (channel_1_pwm - 1000.0f)/1000.0f;
    float roll_norm = (channel_2_pwm - 1500.0f)/500.0f;
    float pit_norm  = (channel_3_pwm - 1500.0f)/500.0f;
    float yaw_norm  = (channel_4_pwm - 1500.0f)/500.0f;

    thro_des  = constrain(thr_norm, 0.0f, 1.0f);
    roll_des  = constrain(roll_norm, -1.0f, 1.0f) * maxRoll;
    pitch_des = constrain(pit_norm,  -1.0f, 1.0f) * maxPitch;
    yaw_des   = constrain(yaw_norm,  -1.0f, 1.0f) * maxYaw;
  }

  static void controlANGLE() {

    

    error_roll = roll_des - roll_IMU;
    

    integral_roll += error_roll * dt;
    

    if (channel_1_pwm < 1060) integral_roll = 0;
    

    integral_roll = constrain(integral_roll, -i_limit, i_limit);
    

    derivative_roll = bodyRollRateDps();
    

    roll_PID = 0.01f * (Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll);

    
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch += error_pitch * dt;
    if (channel_1_pwm < 1060) integral_pitch = 0;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);

    derivative_pitch = bodyPitchRateDps();
    pitch_PID = 0.01f * (Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch);

    

    error_yaw = yaw_des - bodyYawRateDps();
    

    integral_yaw += error_yaw * dt;
    if (channel_1_pwm < 1060) integral_yaw = 0;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    

    derivative_yaw = (error_yaw - error_yaw_prev) / max(dt, 1e-6f);
    

    yaw_PID = 0.01f * (Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);

    

    error_yaw_prev = error_yaw;
  }

  static void mixerQuad() {

    m1_command_scaled = thro_des - pitch_PID + roll_PID + ((float)M1_SPIN * yaw_PID);
    
    m2_command_scaled = thro_des - pitch_PID - roll_PID + ((float)M2_SPIN * yaw_PID);

    m3_command_scaled = thro_des + pitch_PID - roll_PID + ((float)M3_SPIN * yaw_PID);
    
    m4_command_scaled = thro_des + pitch_PID + roll_PID + ((float)M4_SPIN * yaw_PID);

    m1_command_scaled = constrain(m1_command_scaled, 0.0f, 1.0f);
    m2_command_scaled = constrain(m2_command_scaled, 0.0f, 1.0f);
    m4_command_scaled = constrain(m4_command_scaled, 0.0f, 1.0f);
    m3_command_scaled = constrain(m3_command_scaled, 0.0f, 1.0f);
  }

  static void scaleToDuty() {

    m1_command_PWM = (int)lroundf(m1_command_scaled * 255.0f);
    m2_command_PWM = (int)lroundf(m2_command_scaled * 255.0f);
    m4_command_PWM = (int)lroundf(m4_command_scaled * 255.0f);
    m3_command_PWM = (int)lroundf(m3_command_scaled * 255.0f);

    if (MIN_DUTY_WHEN_RUNNING > 0 && thro_des > 0.05f) {
      if (m1_command_PWM>0 && m1_command_PWM<MIN_DUTY_WHEN_RUNNING) m1_command_PWM = MIN_DUTY_WHEN_RUNNING;
      if (m2_command_PWM>0 && m2_command_PWM<MIN_DUTY_WHEN_RUNNING) m2_command_PWM = MIN_DUTY_WHEN_RUNNING;
      if (m4_command_PWM>0 && m4_command_PWM<MIN_DUTY_WHEN_RUNNING) m4_command_PWM = MIN_DUTY_WHEN_RUNNING;
      if (m3_command_PWM>0 && m3_command_PWM<MIN_DUTY_WHEN_RUNNING) m3_command_PWM = MIN_DUTY_WHEN_RUNNING;
    }

    m1_command_PWM = constrain(m1_command_PWM, 0, 255);
    m2_command_PWM = constrain(m2_command_PWM, 0, 255);
    m4_command_PWM = constrain(m4_command_PWM, 0, 255);
    m3_command_PWM = constrain(m3_command_PWM, 0, 255);
  }

  static void throttleCutAndCommand() {

    const bool cut = (channel_5_pwm > 1500) || (!armedFly);
    
    if (cut) {

      armedFly = false;
      digitalWrite(STBY_PIN, LOW);
      ledcWrite(M1_PIN, 0);
      ledcWrite(M2_PIN, 0);
      ledcWrite(M4_PIN, 0);
      ledcWrite(M3_PIN, 0);
      return;
    }
    

    digitalWrite(STBY_PIN, HIGH);
    ledcWrite(M1_PIN, m1_command_PWM);
    ledcWrite(M2_PIN, m2_command_PWM);
    ledcWrite(M4_PIN, m4_command_PWM);
    ledcWrite(M3_PIN, m3_command_PWM);
  }

  static void loopRate(int hz) {
    static uint32_t tNext = 0;
    if (tNext == 0) tNext = micros();
    

    uint32_t period = 1000000UL / (uint32_t)hz;
    

    while ((int32_t)(micros() - tNext) < 0) { }
    

    tNext += period;
  }

  static uint32_t lastTelTxMs = 0;
  static uint8_t  telSeq = 0;

  static void sendTelemetry() {
    if (millis() - lastTelTxMs < 50) return;
    lastTelTxMs = millis();

    ControlPacket p; 
    uint32_t age = 0;
    bool ok = getLatestPacket(&p, sizeof(p), age);

    TelemetryPacket t{};
    t.t_ms = millis();
    t.seq  = telSeq++;
    t.rx_ok = ok ? 1 : 0;
    t.rx_age_ms = (uint16_t)min(age, (uint32_t)65535);

    t.armed_cmd = (uint8_t)p.armed;
    t.armedFly  = armedFly ? 1 : 0;
    t.stby      = digitalRead(STBY_PIN) ? 1 : 0;
    t.reserved0 = (uint8_t)constrain((int)lroundf(accRejectPct), 0, 100);

    t.thr = p.thr; t.yaw = p.yaw; t.pitch = p.pitch; t.roll = p.roll;

    t.roll_imu_cdeg  = (int16_t)lroundf(roll_IMU  * 100.0f);
    t.pitch_imu_cdeg = (int16_t)lroundf(pitch_IMU * 100.0f);

    t.roll_pid_milli  = (int16_t)lroundf(roll_PID  * 1000.0f);
    t.pitch_pid_milli = (int16_t)lroundf(pitch_PID * 1000.0f);
    t.yaw_pid_milli   = (int16_t)lroundf(yaw_PID   * 1000.0f);

    t.m1_command_PWM = (uint8_t)constrain(m1_command_PWM, 0, 255);
    t.m2_command_PWM = (uint8_t)constrain(m2_command_PWM, 0, 255);
    t.m4_command_PWM = (uint8_t)constrain(m4_command_PWM, 0, 255);
    t.m3_command_PWM = (uint8_t)constrain(m3_command_PWM, 0, 255);

    t.acc_mg = (uint16_t)constrain((int)(accMag_g * 1000.0f), 0, 4000);

    esp_now_send(CONTROLLER_MAC, (uint8_t*)&t, sizeof(t));

  #if PRINT_TEL_ON_USB
  const float yawRateBody = bodyYawRateDps();

  static uint32_t lastTelPrintMs = 0;
  if (millis() - lastTelPrintMs >= 50) {
    lastTelPrintMs = millis();
    Serial.printf(
      "%lu -> TEL age=%lums ok=%d seq=%u | armedCmd=%d armedFly=%d STBY=%d | thr=%d yaw=%d pitch=%d roll=%d | IMU r=%.2f p=%.2f yRate=%.1f | PID r=%.3f p=%.3f y=%.3f | duty %3d %3d %3d %3d acc|=%.3f g aRej=%.1f%%\n",
      (unsigned long)millis(),
      (unsigned long)age,
      ok ? 1 : 0,
      (unsigned int)t.seq,
      (int)t.armed_cmd,
      (int)t.armedFly,
      (int)t.stby,
      (int)t.thr, (int)t.yaw, (int)t.pitch, (int)t.roll,
      roll_IMU, pitch_IMU,
      yawRateBody,
      roll_PID, pitch_PID, yaw_PID,
      (int)constrain(m1_command_PWM, 0, 255),
      (int)constrain(m2_command_PWM, 0, 255),
      (int)constrain(m3_command_PWM, 0, 255),
      (int)constrain(m4_command_PWM, 0, 255),
      accMag_g,
      accRejectPct
    );
  }
  #endif
  }

  void calculate_IMU_error() {

    forceOutputsSafeAndResetControllers();
    (void)calibrateImuStill();
  }

  void calibrateAttitude() {

    forceOutputsSafeAndResetControllers();

    float measuredRollTrim = 0.0f;
    float measuredPitchTrim = 0.0f;
    const bool haveLevelTrim = captureAverageLevelTrim(measuredRollTrim, measuredPitchTrim);

    if (haveLevelTrim) {
      levelRollTrimDeg = measuredRollTrim;
      levelPitchTrimDeg = measuredPitchTrim;
    } else {
      getIMUdata();
      seedAttitudeFromCurrentAccel();
      levelRollTrimDeg = roll_IMU;
      levelPitchTrimDeg = pitch_IMU;
    }

    refreshAttitudeEstimateFromCurrentPose();
    if (fabsf(roll_IMU) < 0.25f) roll_IMU = 0.0f;
    if (fabsf(pitch_IMU) < 0.25f) pitch_IMU = 0.0f;
    yaw_IMU = 0.0f;
  }

  static void motorIsolationTest() {

    const float thr_norm  = constrain((channel_1_pwm - 1000.0f)/1000.0f, 0.0f, 1.0f);
    const float roll_norm = constrain((channel_2_pwm - 1500.0f)/500.0f, -1.0f, 1.0f);
    const float pit_norm  = constrain((channel_3_pwm - 1500.0f)/500.0f, -1.0f, 1.0f);
    const float yaw_norm  = constrain((channel_4_pwm - 1500.0f)/500.0f, -1.0f, 1.0f);

    const float aRoll = fabsf(roll_norm);
    const float aPit  = fabsf(pit_norm);
    const float aYaw  = fabsf(yaw_norm);

    const float DB = 0.12f;

    const int TEST_MAX_DUTY = 200;

    auto map01ToDuty = [&](float x)->int {
      x = constrain(x, 0.0f, 1.0f);
      int d = (int)lroundf(x * (float)TEST_MAX_DUTY);
      return constrain(d, 0, TEST_MAX_DUTY);
    };

    m1_command_PWM = m2_command_PWM = m3_command_PWM = m4_command_PWM = 0;

    int which = 0;

    const bool otherCentered = (aRoll < DB && aPit < DB && aYaw < DB);

    if (thr_norm > 0.95f && otherCentered) {
      which = 1;
      m1_command_PWM = map01ToDuty(thr_norm);
    } 
    else {

      float best = 0.0f;

      if (aYaw > DB && aYaw >= best) { best = aYaw; which = 2; }
      if (aRoll > DB && aRoll >  best) { best = aRoll; which = 3; }
      if (aPit > DB && aPit >  best) { best = aPit;  which = 4; }

      if (which == 2) m2_command_PWM = map01ToDuty(aYaw);
      if (which == 3) m3_command_PWM = map01ToDuty(aRoll);
      if (which == 4) m4_command_PWM = map01ToDuty(aPit);
    }

    const int TEST_MIN_DUTY = 0;
    if (TEST_MIN_DUTY > 0) {
      if (m1_command_PWM > 0 && m1_command_PWM < TEST_MIN_DUTY) m1_command_PWM = TEST_MIN_DUTY;
      if (m2_command_PWM > 0 && m2_command_PWM < TEST_MIN_DUTY) m2_command_PWM = TEST_MIN_DUTY;
      if (m3_command_PWM > 0 && m3_command_PWM < TEST_MIN_DUTY) m3_command_PWM = TEST_MIN_DUTY;
      if (m4_command_PWM > 0 && m4_command_PWM < TEST_MIN_DUTY) m4_command_PWM = TEST_MIN_DUTY;
    }

    m1_command_scaled = (float)m1_command_PWM / 255.0f;
    m2_command_scaled = (float)m2_command_PWM / 255.0f;
    m3_command_scaled = (float)m3_command_PWM / 255.0f;
    m4_command_scaled = (float)m4_command_PWM / 255.0f;

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("[MOTOR TEST] which=%d duty=%d,%d,%d,%d acc|=%.3f g gyro=%.1f,%.1f,%.1f\n",
        which, m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, accMag_g, GyroX, GyroY, GyroZ);
    }
  }

  static void allMotorsSameThrust() {

    const float thr_norm = constrain((channel_1_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);

    int thrust_duty = (int)lroundf(thr_norm * 255.0f);

    const int TEST_MIN_DUTY = 0;
    if (TEST_MIN_DUTY > 0 && thrust_duty > 0 && thrust_duty < TEST_MIN_DUTY) {
      thrust_duty = TEST_MIN_DUTY;
    }

    m1_command_PWM = thrust_duty;
    m2_command_PWM = thrust_duty;
    m3_command_PWM = thrust_duty;
    m4_command_PWM = thrust_duty;

    m1_command_scaled = (float)m1_command_PWM / 255.0f;
    m2_command_scaled = (float)m2_command_PWM / 255.0f;
    m3_command_scaled = (float)m3_command_PWM / 255.0f;
    m4_command_scaled = (float)m4_command_PWM / 255.0f;

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf(
        "[ALL MOTORS SAME] thr=%.3f duty=%d acc|=%.3f g gyro=%.1f,%.1f,%.1f\n",
        thr_norm,
        thrust_duty,
        accMag_g,
        GyroX,
        GyroY,
        GyroZ
      );
    }
  }

  void setup() {

    Serial.begin(115200);
    delay(300);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, LOW);

    ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
    

    ledcWrite(M1_PIN, 0);
    ledcWrite(M2_PIN, 0);
    ledcWrite(M4_PIN, 0);
    ledcWrite(M3_PIN, 0);

    if (!mpuInit()) {

      Serial.println("MPU6050 not found at 0x68. Check SDA/SCL/3V3/GND.");
      while(true) { delay(1000); }
    }
    Serial.printf(
      "Axis remap: roll=-GyroY pitch=GyroX yaw=-GyroZ | spins M1=%s M2=%s M3=%s M4=%s | yawPidLimit=%.2f\n",
      motorSpinName(M1_SPIN),
      motorSpinName(M2_SPIN),
      motorSpinName(M3_SPIN),
      motorSpinName(M4_SPIN),
      yaw_pid_limit
    );

    WiFi.mode(WIFI_STA);
    Serial.print("Drone MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
      Serial.println("ESP‑NOW init FAILED");
      while(true) delay(1000);
    }
    esp_now_register_recv_cb(onEspNowRecv);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("ESP-NOW: failed to add controller peer");
    }

    for (int i = 0; i < 3; i++) {
      digitalWrite(STBY_PIN, HIGH);
      delay(200);
      digitalWrite(STBY_PIN, LOW);
      delay(200);
    }

    delay(3000);

  #if PRINT_TEL_ON_USB
    Serial.println("Startup calibration: keep the drone still.");
  #endif
    calculate_IMU_error();
    calibrateAttitude();

    prev_time = micros();
    

    Serial.println("Ready. PROPS OFF. Startup calibration finished. You can still recalibrate with the joystick gesture.");
  }

  void loop() {

    uint32_t current_time = micros();
    dt = (current_time - prev_time) / 1000000.0f;
    prev_time = current_time;

    getCommandsFromEspNow();

    getIMUdata();
    

    if (!armedFly) {
      seedAttitudeFromCurrentAccel();
    } else {
      float gxBody, gyBody, gzBody;
      float axBody, ayBody, azBody;
      getBodyGyroDps(gxBody, gyBody, gzBody);
      getBodyAccelG(axBody, ayBody, azBody);
      Madgwick6DOF(gxBody, gyBody, gzBody, axBody, ayBody, azBody, dt);
    }
    applyLevelTrimToAttitude();
    maybeApplyLevelCalibration();

  #if MOTOR_ISOLATION_MODE

    motorIsolationTest();
    throttleCutAndCommand();
    sendTelemetry();
    loopRate(LOOP_HZ);
    return;
  #endif

  #if ALL_MOTORS_SAME_THRUST_MODE

    allMotorsSameThrust();
    throttleCutAndCommand();
    sendTelemetry();
    loopRate(LOOP_HZ);
    return;
  #endif

    getDesState();
    

    controlANGLE();
    

    mixerQuad();
    

    scaleToDuty();
    

    throttleCutAndCommand();

    sendTelemetry();
    

    loopRate(LOOP_HZ);
  }
