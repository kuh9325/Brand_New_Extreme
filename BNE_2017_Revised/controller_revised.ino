#include <SoftwareSerial.h>

namespace ControllerConfig {
constexpr uint8_t ROTARY_ANGLE_SENSOR = A0;
constexpr uint8_t BT_RX_PIN = 2;
constexpr uint8_t BT_TX_PIN = 3;
constexpr uint8_t LINK_RED_PIN = 4;
constexpr uint8_t LINK_GREEN_PIN = 5;
constexpr uint8_t LINK_UNUSED_PIN = 6;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t BLUETOOTH_BAUD = 115200;
constexpr uint32_t TX_PERIOD_MS = 20;       // 50 Hz, vs. archived 5 Hz (delay(200))
constexpr uint32_t ACK_TIMEOUT_MS = 250;
}

SoftwareSerial bluetooth(ControllerConfig::BT_RX_PIN, ControllerConfig::BT_TX_PIN);
uint32_t lastTxMs = 0;
uint32_t lastAckMs = 0;
bool hasAck = false;

static uint8_t readThrottle() {
  const int raw = analogRead(ControllerConfig::ROTARY_ANGLE_SENSOR);
  return static_cast<uint8_t>(map(raw, 0, 1023, 0, 255));
}

static void updateLinkIndicator(uint32_t nowMs) {
  const bool linked = hasAck && (nowMs - lastAckMs <= ControllerConfig::ACK_TIMEOUT_MS);
  digitalWrite(ControllerConfig::LINK_RED_PIN, linked ? LOW : HIGH);
  digitalWrite(ControllerConfig::LINK_GREEN_PIN, linked ? HIGH : LOW);
  digitalWrite(ControllerConfig::LINK_UNUSED_PIN, LOW);
}

void setup() {
  Serial.begin(ControllerConfig::SERIAL_BAUD);
  bluetooth.begin(ControllerConfig::BLUETOOTH_BAUD);

  pinMode(ControllerConfig::LINK_RED_PIN, OUTPUT);
  pinMode(ControllerConfig::LINK_GREEN_PIN, OUTPUT);
  pinMode(ControllerConfig::LINK_UNUSED_PIN, OUTPUT);

  updateLinkIndicator(millis());
}

void loop() {
  const uint32_t nowMs = millis();

  while (bluetooth.available() > 0) {
    if (bluetooth.read() == 'i') {
      lastAckMs = nowMs;
      hasAck = true;
    }
  }

  if (nowMs - lastTxMs >= ControllerConfig::TX_PERIOD_MS) {
    lastTxMs = nowMs;
    const uint8_t throttle = readThrottle();

    bluetooth.print('a');
    bluetooth.print(throttle);
    bluetooth.print('z');

    Serial.println(throttle);
  }

  updateLinkIndicator(nowMs);
}
