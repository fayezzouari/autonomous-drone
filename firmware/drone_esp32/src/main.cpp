// Singlecopter actuator + IMU node — ESP32-S3 (N8R2).
//
// Role (the PC runs the PID; this board is deliberately "dumb"):
//   * SUB  drone/hw  : {"throttle":0..1, "s1":deg, "s2":deg, "s3":deg, "s4":deg}
//                      -> per-pin trim calibration -> clamp to [40,160] -> 4 servos
//                      -> ESC (brushless) on pin 14
//   * PUB  drone/imu : {"t":sec,"yaw":deg,"pitch":deg,"roll":deg,"gz":deg/s} @ 50 Hz
//   * Fail-safe: if no command arrives for FAILSAFE_MS, cut the ESC to idle and
//     re-centre the vanes. Control depends on the Wi-Fi/MQTT link, so this is the
//     safety net for a dropped PC / broker.
//
// IMU: MPU6050 on I2C  SDA=GPIO5  SCL=GPIO4.
// Servos: 36, 37, 38, 39.   Brushless ESC: 14.

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <MPU6050_light.h>

#include "secrets.h"

// ── Topics (must match config/config.yaml) ──────────────────────────────────────
static const char* TOPIC_HW     = "drone/hw";
static const char* TOPIC_IMU    = "drone/imu";
static const char* TOPIC_STATUS = "drone/status";

// ── Pins ────────────────────────────────────────────────────────────────────────
static const int SDA_PIN = 5;
static const int SCL_PIN = 4;
static const int ESC_PIN = 14;
static const int SERVO_PINS[4] = {36, 37, 38, 39};

// ── Vane rotation limit (hard clamp on the physical servo write) ─────────────────
static const float SERVO_MIN_DEG = 40.0f;
static const float SERVO_MAX_DEG = 160.0f;
static const float SERVO_NEUTRAL_DEG = 90.0f;   // logical neutral fed by the PC

// Per-pin trim: logical angle (from PC) -> physical write, via two calibration
// points {in -> out}. Index order matches SERVO_PINS / s1..s4.
struct ServoCal { float in0, out0, in90, out90; };
static const ServoCal CAL[4] = {
    {0.0f,  5.0f, 90.0f, 100.0f},   // servo 36 (s1)
    {0.0f,  5.0f, 90.0f, 100.0f},   // servo 37 (s2)
    {0.0f, 10.0f, 90.0f, 105.0f},   // servo 38 (s3)
    {0.0f, 10.0f, 90.0f, 100.0f},   // servo 39 (s4)
};

// ── ESC (brushless) PWM range ────────────────────────────────────────────────────
static const int ESC_MIN_US = 1000;   // idle / armed-zero
static const int ESC_MAX_US = 2000;   // full throttle

// ── LEDC PWM (direct core API, replaces ESP32Servo) ──────────────────────────────
// We drive the 4 servos + ESC straight off the LEDC peripheral with EXPLICIT,
// hand-picked channels. ESP32Servo's auto channel allocation collided two servos
// onto one channel on this S3; assigning channels ourselves makes that impossible.
static const int LEDC_FREQ_HZ  = 50;            // servo / ESC frame rate
// 14 bits is the ESP32-S3 LEDC hardware MAX (the original ESP32 allows 20). Using
// 16 here makes ledcSetup() fail at runtime and emit NO pwm — servos stay dead.
static const int LEDC_RES_BITS = 14;            // duty resolution (S3 max)
static const int SERVO_CH[4]   = {0, 1, 2, 3};  // one distinct channel per vane
static const int ESC_CH        = 4;             // ESC on its own channel
static const int SERVO_US_MIN  = 500;           // pulse width at 0°
static const int SERVO_US_MAX  = 2500;          // pulse width at 180°

// ── Timing ────────────────────────────────────────────────────────────────────────
static const unsigned long FAILSAFE_MS = 400;    // no command for this long -> safe
static const unsigned long IMU_PERIOD_MS = 20;   // 50 Hz telemetry

// ── Globals ───────────────────────────────────────────────────────────────────────
WiFiClient    wifiClient;
PubSubClient  mqtt(wifiClient);
MPU6050       imu(Wire);

unsigned long lastCmdMs = 0;
unsigned long lastImuMs = 0;
unsigned long lastHbMs  = 0;
bool          failsafeActive = false;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ── LEDC PWM helpers (work on both Arduino-ESP32 core 2.x and 3.x) ────────────────
// Pulse width [us] -> LEDC duty for the configured frame rate & resolution.
static uint32_t usToDuty(float us) {
    const float full    = (float)(1UL << LEDC_RES_BITS);   // duty for a full frame
    const float frameUs = 1000000.0f / LEDC_FREQ_HZ;       // 20000 us @ 50 Hz
    return (uint32_t)(us / frameUs * full + 0.5f);
}

static void pwmAttach(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pin, LEDC_FREQ_HZ, LEDC_RES_BITS, ch);   // explicit channel
#else
    ledcSetup(ch, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(pin, ch);
#endif
}

static void pwmWriteUs(int pin, int ch, float us) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ch;  ledcWrite(pin, usToDuty(us));     // core 3.x: address by pin
#else
    (void)pin; ledcWrite(ch, usToDuty(us));      // core 2.x: address by channel
#endif
}

// Logical servo angle (deg) -> calibrated, clamped physical angle (deg).
static int calibrate(int i, float logical) {
    const ServoCal& c = CAL[i];
    float slope = (c.out90 - c.out0) / (c.in90 - c.in0);
    float physical = c.out0 + (logical - c.in0) * slope;
    return (int)lroundf(clampf(physical, SERVO_MIN_DEG, SERVO_MAX_DEG));
}

static void writeVane(int i, float logicalDeg) {
    int phys = calibrate(i, logicalDeg);   // calibrated, clamped physical degrees
    float us = SERVO_US_MIN +
               (float)phys * (SERVO_US_MAX - SERVO_US_MIN) / 180.0f;
    pwmWriteUs(SERVO_PINS[i], SERVO_CH[i], us);
}

static void writeEsc(float throttleFrac) {
    float us = ESC_MIN_US + clampf(throttleFrac, 0.0f, 1.0f) *
                            (ESC_MAX_US - ESC_MIN_US);
    pwmWriteUs(ESC_PIN, ESC_CH, us);
}

static void goSafe() {
    writeEsc(0.0f);                                  // motor to idle
    for (int i = 0; i < 4; i++) writeVane(i, SERVO_NEUTRAL_DEG);  // vanes neutral
}

// ── MQTT ──────────────────────────────────────────────────────────────────────────
static void onMessage(char* topic, byte* payload, unsigned int len) {
    if (strcmp(topic, TOPIC_HW) != 0) return;

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload, len)) return;   // bad JSON -> ignore

    float thr = doc["throttle"] | 0.0f;
    float s1  = doc["s1"] | SERVO_NEUTRAL_DEG;
    float s2  = doc["s2"] | SERVO_NEUTRAL_DEG;
    float s3  = doc["s3"] | SERVO_NEUTRAL_DEG;
    float s4  = doc["s4"] | SERVO_NEUTRAL_DEG;

    writeVane(0, s1);
    writeVane(1, s2);
    writeVane(2, s3);
    writeVane(3, s4);
    writeEsc(thr);

    lastCmdMs = millis();
    if (failsafeActive) {
        failsafeActive = false;
        mqtt.publish(TOPIC_STATUS, "link_restored");
    }
}

// Non-blocking: attempt a connection, throttled, and ALWAYS return so loop()
// keeps running (heartbeat + failsafe) even when the network is down.
static void ensureWifi() {
    static unsigned long lastTry = 0;
    if (WiFi.status() == WL_CONNECTED) return;
    unsigned long now = millis();
    if (lastTry != 0 && now - lastTry < 5000) return;   // retry at most every 5 s
    lastTry = now;
    Serial.printf("[wifi] connecting to \"%s\" ...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
        delay(250);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[wifi] CONNECTED ip=%s rssi=%d\n",
                      WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    } else {
        Serial.printf("\n[wifi] FAILED (status=%d) — check SSID/password and that "
                      "the network is 2.4 GHz\n", (int)WiFi.status());
    }
}

static void ensureMqtt() {
    static unsigned long lastTry = 0;
    if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;
    unsigned long now = millis();
    if (lastTry != 0 && now - lastTry < 2000) return;   // retry at most every 2 s
    lastTry = now;
    String cid = "drone-esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.printf("[mqtt] connecting to %s:%d as %s ...\n", MQTT_HOST, MQTT_PORT,
                  cid.c_str());
    bool ok;
#if defined(MQTT_USER)
    ok = mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS);
#else
    ok = mqtt.connect(cid.c_str());
#endif
    if (ok) {
        mqtt.subscribe(TOPIC_HW);
        mqtt.publish(TOPIC_STATUS, "esp32_online");
        Serial.println("[mqtt] CONNECTED, subscribed to drone/hw");
    } else {
        Serial.printf("[mqtt] connect failed rc=%d — broker reachable on the LAN? "
                      "anonymous allowed?\n", mqtt.state());
    }
}

// ── Setup / loop ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== singlecopter ESP32-S3 actuator/IMU node ===");

    // Servos + ESC on explicit LEDC channels (0..3 = vanes, 4 = ESC). No library,
    // no auto channel allocation, so two outputs can never share a channel.
    for (int i = 0; i < 4; i++) pwmAttach(SERVO_PINS[i], SERVO_CH[i]);
    pwmAttach(ESC_PIN, ESC_CH);

    // Arm the ESC at idle and centre the vanes before anything else.
    goSafe();
    delay(2000);
    Serial.println("[esc] armed at idle");

    // IMU.
    Wire.begin(SDA_PIN, SCL_PIN);
    byte status = imu.begin();
    if (status != 0) {
        Serial.printf("[imu] MPU6050 not found (status %d) — check SDA=5/SCL=4\n", status);
    } else {
        Serial.println("[imu] calibrating, keep the drone still...");
        delay(1000);
        imu.calcOffsets();           // gyro + accel zero
        Serial.println("[imu] ready");
    }

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(onMessage);
    mqtt.setKeepAlive(15);
    ensureWifi();      // non-blocking — loop() keeps retrying if this fails
    ensureMqtt();
    lastCmdMs = millis();
    Serial.println("[setup] done — entering loop");
}

void loop() {
    ensureWifi();
    ensureMqtt();
    if (mqtt.connected()) mqtt.loop();

    imu.update();
    unsigned long now = millis();

    // Heartbeat: the answer to "is Wi-Fi working?" — printed every 2 s no matter
    // what, so you can diagnose even with no broker.
    if (now - lastHbMs >= 2000) {
        lastHbMs = now;
        Serial.printf("[hb] wifi=%s ip=%s rssi=%d mqtt=%s yaw=%.1f\n",
                      WiFi.status() == WL_CONNECTED ? "OK" : "--",
                      WiFi.localIP().toString().c_str(),
                      (int)WiFi.RSSI(),
                      mqtt.connected() ? "OK" : "--",
                      imu.getAngleZ());
    }

    // Fail-safe: lost the PC / broker -> idle the motor, centre the vanes.
    if (now - lastCmdMs > FAILSAFE_MS && !failsafeActive) {
        goSafe();
        failsafeActive = true;
        mqtt.publish(TOPIC_STATUS, "failsafe_no_command");
        Serial.println("[failsafe] no command — motor idle, vanes neutral");
    }

    // Publish orientation at 50 Hz (only when the broker link is up).
    // DISABLED: an Android phone now feeds drone/imu via the `phone-imu` bridge
    // (drone_nav/phone_imu.py). The MPU6050 is still read above (calibration +
    // heartbeat yaw) but must NOT publish, or it would fight the phone on the
    // same topic. Re-enable this block to go back to the onboard IMU.
    if (mqtt.connected() && now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs = now;
        StaticJsonDocument<192> doc;
        doc["t"]     = now / 1000.0f;
        doc["yaw"]   = imu.getAngleZ();   // degrees
        doc["pitch"] = imu.getAngleY();
        doc["roll"]  = imu.getAngleX();
        doc["gz"]    = imu.getGyroZ();    // deg/s
        char buf[192];
        size_t n = serializeJson(doc, buf);
        mqtt.publish(TOPIC_IMU, buf, n);
    }
}
