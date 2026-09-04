#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

namespace Config {
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t BT_RX_PIN = 4;
constexpr uint8_t BT_TX_PIN = 3;
constexpr uint8_t LEFT_ESC_PIN = 5;
constexpr uint8_t RIGHT_ESC_PIN = 6;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t BLUETOOTH_BAUD = 115200;
constexpr uint32_t CONTROL_PERIOD_US = 10000;   // 100 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;  // 10 Hz
constexpr uint32_t REMOTE_TIMEOUT_MS = 250;
constexpr uint32_t REARM_HOLD_MS = 500;

// Geometry: nominal 16 mm rocker with about 100 mm half-track -> ~9.2 deg.
constexpr float EDGE_ANGLE_DEG = 9.0f;
constexpr float EDGE_HYSTERESIS_DEG = 1.0f;
constexpr float FULL_EDGE_ANGLE_DEG = 13.0f;

// Second-order complementary observer.
constexpr float FILTER_KP = 2.0f;
constexpr float FILTER_KI = 1.0f;
constexpr float FILTER_INTEGRAL_LIMIT = 30.0f;

// Spin envelope. Free spin below soft limit; propulsion fades to zero at hard limit.
constexpr float SPIN_SOFT_DPS = 150.0f;
constexpr float SPIN_HARD_DPS = 200.0f;
constexpr float SPIN_REARM_DPS = 80.0f;

constexpr float ARM_THROTTLE = 0.05f;
constexpr float UNLOADED_WHEEL_BIAS = 0.15f;

// ESC command range. Keep stop below active range; do not auto-calibrate ESCs at boot.
constexpr int ESC_STOP_US = 1190;
constexpr int ESC_MAX_US = 1500;
constexpr int SERVO_ATTACH_MIN_US = 1000;
constexpr int SERVO_ATTACH_MAX_US = 2000;
}

static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static float smoothstep01(float x) {
  x = clamp01(x);
  return x * x * (3.0f - 2.0f * x);
}

class ImuSensor {
public:
  bool begin() {
    Wire.begin();
    Wire.beginTransmission(Config::MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // wake up
    return Wire.endTransmission(true) == 0;
  }

  bool calibrate(uint16_t samples = 200) {
    long sumAx = 0, sumAy = 0, sumAz = 0;
    long sumGx = 0, sumGy = 0, sumGz = 0;

    for (uint16_t i = 0; i < samples; ++i) {
      if (!readRaw()) return false;
      sumAx += ax_; sumAy += ay_; sumAz += az_;
      sumGx += gx_; sumGy += gy_; sumGz += gz_;
      delay(5);
    }

    baseAx_ = static_cast<float>(sumAx) / samples;
    baseAy_ = static_cast<float>(sumAy) / samples;
    baseAz_ = static_cast<float>(sumAz) / samples;
    baseGx_ = static_cast<float>(sumGx) / samples;
    baseGy_ = static_cast<float>(sumGy) / samples;
    baseGz_ = static_cast<float>(sumGz) / samples;
    return true;
  }

  bool update() {
    return readRaw();
  }

  // Axis convention for the revised sketch:
  //   +X forward, +Y left, +Z up.
  // Roll is rotation about X, yaw is rotation about Z.
  // Verify sensor mounting and signs on the actual board before riding.
  float accelRollDeg() const {
    const float ay = static_cast<float>(ay_) - baseAy_;
    const float az = static_cast<float>(az_) + (16384.0f - baseAz_);
    return atan2(ay, az) * 180.0f / PI;
  }

  float rollRateDps() const {
    return (static_cast<float>(gx_) - baseGx_) / 131.0f;
  }

  float yawRateDps() const {
    return (static_cast<float>(gz_) - baseGz_) / 131.0f;
  }

private:
  bool readRaw() {
    Wire.beginTransmission(Config::MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;

    const uint8_t requested = 14;
    const uint8_t received = Wire.requestFrom(Config::MPU_ADDR, requested, true);
    if (received != requested) return false;

    ax_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    ay_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    az_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    temp_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    gx_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    gy_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    gz_ = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    return true;
  }

  int16_t ax_ = 0, ay_ = 0, az_ = 0, temp_ = 0;
  int16_t gx_ = 0, gy_ = 0, gz_ = 0;
  float baseAx_ = 0.0f, baseAy_ = 0.0f, baseAz_ = 0.0f;
  float baseGx_ = 0.0f, baseGy_ = 0.0f, baseGz_ = 0.0f;
};

class AttitudeEstimator {
public:
  void reset(float rollDeg = 0.0f) {
    rollDeg_ = rollDeg;
    errorIntegral_ = 0.0f;
  }

  void update(float accelRollDeg, float gyroRollRateDps, float dt) {
    if (dt <= 0.0f || dt > 0.1f) return;

    const float error = accelRollDeg - rollDeg_;
    errorIntegral_ += error * dt;
    if (errorIntegral_ > Config::FILTER_INTEGRAL_LIMIT)
      errorIntegral_ = Config::FILTER_INTEGRAL_LIMIT;
    else if (errorIntegral_ < -Config::FILTER_INTEGRAL_LIMIT)
      errorIntegral_ = -Config::FILTER_INTEGRAL_LIMIT;

    const float correctedRate = gyroRollRateDps
                              + Config::FILTER_KP * error
                              + Config::FILTER_KI * errorIntegral_;
    rollDeg_ += correctedRate * dt;
    rollRateDps_ = gyroRollRateDps;
  }

  float rollDeg() const { return rollDeg_; }
  float rollRateDps() const { return rollRateDps_; }

private:
  float rollDeg_ = 0.0f;
  float rollRateDps_ = 0.0f;
  float errorIntegral_ = 0.0f;
};

class RemoteReceiver {
public:
  explicit RemoteReceiver(SoftwareSerial& serial) : serial_(serial) {}

  void begin() {
    serial_.begin(Config::BLUETOOTH_BAUD);
  }

  void update() {
    while (serial_.available() > 0) {
      const char c = static_cast<char>(serial_.read());

      if (!receiving_) {
        if (c == 'a') {
          receiving_ = true;
          index_ = 0;
        }
        continue;
      }

      if (c == 'z') {
        buffer_[index_] = '\0';
        parseFrame();
        receiving_ = false;
        index_ = 0;
        continue;
      }

      if (c < '0' || c > '9' || index_ >= sizeof(buffer_) - 1) {
        receiving_ = false;
        index_ = 0;
        continue;
      }

      buffer_[index_++] = c;
    }
  }

  bool fresh(uint32_t nowMs) const {
    return hasValidFrame_ && (nowMs - lastValidPacketMs_ <= Config::REMOTE_TIMEOUT_MS);
  }

  float throttle() const {
    return static_cast<float>(rawThrottle_) / 255.0f;
  }

  uint8_t rawThrottle() const { return rawThrottle_; }
  uint32_t lastValidPacketMs() const { return lastValidPacketMs_; }

private:
  void parseFrame() {
    if (index_ == 0) return;
    const int value = atoi(buffer_);
    if (value < 0 || value > 255) return;

    rawThrottle_ = static_cast<uint8_t>(value);
    lastValidPacketMs_ = millis();
    hasValidFrame_ = true;
    serial_.write('i'); // one-byte acknowledgement for revised controller link indication
  }

  SoftwareSerial& serial_;
  char buffer_[4] = {0};
  uint8_t index_ = 0;
  uint8_t rawThrottle_ = 0;
  uint32_t lastValidPacketMs_ = 0;
  bool receiving_ = false;
  bool hasValidFrame_ = false;
};

enum class ContactMode : uint8_t {
  LEFT_EDGE,
  CENTER,
  RIGHT_EDGE
};

class ContactEstimator {
public:
  ContactMode update(float rollDeg) {
    const float on = Config::EDGE_ANGLE_DEG + Config::EDGE_HYSTERESIS_DEG;
    const float off = Config::EDGE_ANGLE_DEG - Config::EDGE_HYSTERESIS_DEG;

    switch (mode_) {
      case ContactMode::CENTER:
        if (rollDeg >= on) mode_ = ContactMode::RIGHT_EDGE;
        else if (rollDeg <= -on) mode_ = ContactMode::LEFT_EDGE;
        break;
      case ContactMode::RIGHT_EDGE:
        if (rollDeg <= off) mode_ = ContactMode::CENTER;
        break;
      case ContactMode::LEFT_EDGE:
        if (rollDeg >= -off) mode_ = ContactMode::CENTER;
        break;
    }
    return mode_;
  }

  float engagement(float rollDeg) const {
    if (mode_ == ContactMode::CENTER) return 0.0f;
    // Start the blend at the same angle that enters an EDGE state. This avoids
    // a command jump when hysteresis changes CENTER -> EDGE.
    const float start = Config::EDGE_ANGLE_DEG + Config::EDGE_HYSTERESIS_DEG;
    const float span = Config::FULL_EDGE_ANGLE_DEG - start;
    if (span <= 0.0f) return 1.0f;
    const float z = (fabs(rollDeg) - start) / span;
    return smoothstep01(z);
  }

  ContactMode mode() const { return mode_; }

private:
  ContactMode mode_ = ContactMode::CENTER;
};

struct MotorCommand {
  float left;
  float right;
};

class TractionController {
public:
  MotorCommand compute(float throttle, ContactMode mode, float engagement) const {
    throttle = clamp01(throttle);
    engagement = clamp01(engagement);

    MotorCommand cmd = {0.0f, 0.0f};
    if (mode == ContactMode::CENTER || engagement <= 0.0f) return cmd;

    const float loaded = throttle * engagement;
    const float unloaded = loaded * Config::UNLOADED_WHEEL_BIAS;

    if (mode == ContactMode::LEFT_EDGE) {
      cmd.left = loaded;
      cmd.right = unloaded;
    } else {
      cmd.left = unloaded;
      cmd.right = loaded;
    }
    return cmd;
  }
};

class SpinEnvelope {
public:
  float gain(float yawRateDps) const {
    const float r = fabs(yawRateDps);
    if (r <= Config::SPIN_SOFT_DPS) return 1.0f;
    if (r >= Config::SPIN_HARD_DPS) return 0.0f;

    const float x = (r - Config::SPIN_SOFT_DPS)
                  / (Config::SPIN_HARD_DPS - Config::SPIN_SOFT_DPS);
    return 1.0f - smoothstep01(x);
  }

  bool hardLimitExceeded(float yawRateDps) const {
    return fabs(yawRateDps) >= Config::SPIN_HARD_DPS;
  }
};

enum class RideState : uint8_t {
  DISARMED,
  RUNNING,
  SPIN_CUTOFF,
  FAILSAFE
};

class SafetyManager {
public:
  void update(bool imuHealthy, bool remoteFresh, float throttle,
              float yawRateDps, bool hardSpin, uint32_t nowMs) {
    if (!imuHealthy || !remoteFresh) {
      state_ = RideState::FAILSAFE;
      safeSinceMs_ = 0;
      return;
    }

    if (state_ == RideState::RUNNING && hardSpin) {
      state_ = RideState::SPIN_CUTOFF;
      safeSinceMs_ = 0;
      return;
    }

    if (state_ == RideState::SPIN_CUTOFF) {
      const bool safe = fabs(yawRateDps) <= Config::SPIN_REARM_DPS
                     && throttle <= Config::ARM_THROTTLE;
      holdAndRearm(safe, nowMs);
      return;
    }

    if (state_ == RideState::FAILSAFE || state_ == RideState::DISARMED) {
      // A radio/IMU fault can occur during a spin. Do not let FAILSAFE bypass
      // the spin re-arm condition after communication or sensor recovery.
      const bool safe = fabs(yawRateDps) <= Config::SPIN_REARM_DPS
                     && throttle <= Config::ARM_THROTTLE;
      holdAndRearm(safe, nowMs);
    }
  }

  bool canDrive() const { return state_ == RideState::RUNNING; }
  RideState state() const { return state_; }

private:
  void holdAndRearm(bool safe, uint32_t nowMs) {
    if (!safe) {
      safeSinceMs_ = 0;
      return;
    }

    if (safeSinceMs_ == 0) safeSinceMs_ = nowMs;
    if (nowMs - safeSinceMs_ >= Config::REARM_HOLD_MS) {
      state_ = RideState::RUNNING;
      safeSinceMs_ = 0;
    }
  }

  RideState state_ = RideState::DISARMED;
  uint32_t safeSinceMs_ = 0;
};

class EscPair {
public:
  void begin() {
    left_.attach(Config::LEFT_ESC_PIN,
                 Config::SERVO_ATTACH_MIN_US,
                 Config::SERVO_ATTACH_MAX_US);
    right_.attach(Config::RIGHT_ESC_PIN,
                  Config::SERVO_ATTACH_MIN_US,
                  Config::SERVO_ATTACH_MAX_US);
    stop();
    delay(1000);
  }

  void write(const MotorCommand& cmd) {
    left_.writeMicroseconds(toPulse(cmd.left));
    right_.writeMicroseconds(toPulse(cmd.right));
  }

  void stop() {
    left_.writeMicroseconds(Config::ESC_STOP_US);
    right_.writeMicroseconds(Config::ESC_STOP_US);
  }

private:
  int toPulse(float command) const {
    command = clamp01(command);
    const float span = static_cast<float>(Config::ESC_MAX_US - Config::ESC_STOP_US);
    return Config::ESC_STOP_US + static_cast<int>(span * command + 0.5f);
  }

  Servo left_;
  Servo right_;
};

SoftwareSerial bluetooth(Config::BT_RX_PIN, Config::BT_TX_PIN);
ImuSensor imu;
AttitudeEstimator attitude;
RemoteReceiver remote(bluetooth);
ContactEstimator contactEstimator;
TractionController tractionController;
SpinEnvelope spinEnvelope;
SafetyManager safety;
EscPair motors;

uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
bool imuReady = false;

static const __FlashStringHelper* contactName(ContactMode mode) {
  switch (mode) {
    case ContactMode::LEFT_EDGE: return F("LEFT");
    case ContactMode::RIGHT_EDGE: return F("RIGHT");
    default: return F("CENTER");
  }
}

static const __FlashStringHelper* rideStateName(RideState state) {
  switch (state) {
    case RideState::RUNNING: return F("RUNNING");
    case RideState::SPIN_CUTOFF: return F("SPIN_CUTOFF");
    case RideState::FAILSAFE: return F("FAILSAFE");
    default: return F("DISARMED");
  }
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  remote.begin();
  motors.begin();

  Serial.println(F("BNE 2017 Revised: initializing MPU-6050"));
  if (!imu.begin()) {
    Serial.println(F("MPU init failed; motors remain stopped until reset"));
  } else if (!imu.calibrate()) {
    Serial.println(F("MPU calibration failed; motors remain stopped until reset"));
  } else {
    attitude.reset(imu.accelRollDeg());
    imuReady = true;
    Serial.println(F("MPU ready"));
  }

  lastControlUs = micros();
  lastTelemetryMs = millis();
}

void loop() {
  // Parse remote bytes as often as possible so the 100 Hz control scheduler
  // never blocks on SoftwareSerial framing.
  remote.update();

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  if (elapsedUs < Config::CONTROL_PERIOD_US) return;

  lastControlUs = nowUs;
  const float dt = static_cast<float>(elapsedUs) * 1.0e-6f;
  const uint32_t nowMs = millis();

  const bool imuHealthy = imuReady && imu.update();
  if (imuHealthy) {
    attitude.update(imu.accelRollDeg(), imu.rollRateDps(), dt);
  }

  const float rollDeg = attitude.rollDeg();
  const float yawRateDps = imuHealthy ? imu.yawRateDps() : 0.0f;
  const bool remoteFresh = remote.fresh(nowMs);
  const float throttle = remote.throttle();

  const ContactMode mode = contactEstimator.update(rollDeg);
  const float edgeEngagement = contactEstimator.engagement(rollDeg);
  const bool hardSpin = spinEnvelope.hardLimitExceeded(yawRateDps);

  safety.update(imuHealthy, remoteFresh, throttle,
                yawRateDps, hardSpin, nowMs);

  MotorCommand cmd = {0.0f, 0.0f};
  float spinGain = 0.0f;

  if (safety.canDrive()) {
    cmd = tractionController.compute(throttle, mode, edgeEngagement);
    spinGain = spinEnvelope.gain(yawRateDps);
    cmd.left *= spinGain;
    cmd.right *= spinGain;
    motors.write(cmd);
  } else {
    motors.stop();
  }

  if (nowMs - lastTelemetryMs >= Config::TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    Serial.print(F("state=")); Serial.print(rideStateName(safety.state()));
    Serial.print(F(" mode=")); Serial.print(contactName(mode));
    Serial.print(F(" roll=")); Serial.print(rollDeg, 2);
    Serial.print(F(" yaw_dps=")); Serial.print(yawRateDps, 1);
    Serial.print(F(" edge=")); Serial.print(edgeEngagement, 2);
    Serial.print(F(" throttle=")); Serial.print(throttle, 2);
    Serial.print(F(" spinGain=")); Serial.print(spinGain, 2);
    Serial.print(F(" L=")); Serial.print(cmd.left, 2);
    Serial.print(F(" R=")); Serial.println(cmd.right, 2);
  }
}
