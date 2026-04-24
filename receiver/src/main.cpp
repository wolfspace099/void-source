#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

namespace {
constexpr uint8_t PIN_BAT_READ = 2;
constexpr uint8_t PIN_ENCODER = 4;
constexpr uint8_t PIN_ENGINE_PWM = 21;
constexpr uint8_t PIN_SERVO = 20;
constexpr uint8_t PIN_FAN = 8;
constexpr uint8_t PIN_LED2 = 9;
constexpr uint8_t PIN_LED1 = 10;

constexpr int SERVO_CTR_US = 1500;
constexpr int SERVO_RANGE_US = 400;

constexpr int ESC_MIN = 1000;
constexpr int ESC_NEUTRAL = 1500;
constexpr int ESC_MAX = 2000;

constexpr uint32_t SPEED_WINDOW_MS = 100;
constexpr float KPH_FACTOR = 2.139f;

constexpr uint32_t FAILSAFE_MS = 500;
constexpr uint32_t TEL_PERIOD_MS = 100;
constexpr uint32_t LOG_PERIOD_MS = 200;
constexpr uint32_t FAILSAFE_LOG_PERIOD_MS = 1000;

constexpr uint8_t ADC_SAMPLES = 16;
constexpr float BAT_SCALE = (3.3f * 4.0f) / (4095.0f * ADC_SAMPLES);

volatile uint32_t encPulses = 0;
volatile bool newData = false;

struct __attribute__((packed)) RCPacket {
    int16_t steering;
    int16_t throttle;
    int8_t trim;
    bool led1;
    bool led2;
    bool fan;
    uint8_t crc;
};

struct __attribute__((packed)) TelPacket {
    float kph;
    float kphMax;
    float vbat;
    uint8_t crc;
};

volatile RCPacket rxPkt = {};
uint32_t lastRxMs = 0;
float currentKph = 0.0f;
float maxKph = 0.0f;
float lastVbat = 0.0f;

Servo steerServo;
Servo esc;

uint8_t VOID_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}

template <typename T>
uint8_t calcCRC(const T& packet) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&packet);
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(T) - 1; ++i) {
        crc ^= bytes[i];
    }
    return crc;
}

void IRAM_ATTR encoderISR() { ++encPulses; }

float readBattVoltage() {
    uint32_t acc = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
        acc += analogRead(PIN_BAT_READ);
    }
    return static_cast<float>(acc) * BAT_SCALE;
}

void applyFailsafe() {
    esc.writeMicroseconds(ESC_NEUTRAL);
    steerServo.writeMicroseconds(SERVO_CTR_US);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_FAN, LOW);
}

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
    if (len != static_cast<int>(sizeof(RCPacket))) {
        return;
    }

    RCPacket incoming;
    std::memcpy(&incoming, data, sizeof(incoming));
    if (calcCRC(incoming) != incoming.crc) {
        return;
    }

    std::memcpy((void*)&rxPkt, &incoming, sizeof(incoming));
    newData = true;
    lastRxMs = millis();
}

void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    (void)mac;
    (void)status;
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_FAN, OUTPUT);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_FAN, LOW);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    steerServo.setPeriodHertz(50);
    steerServo.attach(PIN_SERVO, 600, 2400);
    steerServo.writeMicroseconds(SERVO_CTR_US);

    esc.setPeriodHertz(50);
    esc.attach(PIN_ENGINE_PWM, ESC_MIN, ESC_MAX);
    esc.writeMicroseconds(ESC_NEUTRAL);

    pinMode(PIN_ENCODER, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), encoderISR, RISING);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.print("Abyss MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED - halting");
        while (true) {
            delay(1000);
        }
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    std::memcpy(peer.peer_addr, VOID_MAC, sizeof(VOID_MAC));
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    lastRxMs = millis();
    Serial.println("Abyss ready - waiting for VOID...");
}

void loop() {
    const uint32_t now = millis();

    static uint32_t lastSpeedMs = 0;
    if (now - lastSpeedMs >= SPEED_WINDOW_MS) {
        lastSpeedMs = now;

        uint32_t pulses;
        noInterrupts();
        pulses = encPulses;
        encPulses = 0;
        interrupts();

        currentKph = static_cast<float>(pulses) * KPH_FACTOR;
        if (currentKph > maxKph) {
            maxKph = currentKph;
        }
    }

    if (now - lastRxMs > FAILSAFE_MS) {
        applyFailsafe();

        static uint32_t lastFailsafeLogMs = 0;
        if (now - lastFailsafeLogMs > FAILSAFE_LOG_PERIOD_MS) {
            lastFailsafeLogMs = now;
            Serial.println("FAILSAFE - no signal");
        }
        return;
    }

    RCPacket packet;
    bool hasPacket = false;
    noInterrupts();
    if (newData) {
        std::memcpy(&packet, (const void*)&rxPkt, sizeof(packet));
        newData = false;
        hasPacket = true;
    }
    interrupts();

    if (hasPacket) {
        int steerUs = SERVO_CTR_US + (packet.steering * SERVO_RANGE_US / 100);
        steerUs = constrain(steerUs, SERVO_CTR_US - SERVO_RANGE_US, SERVO_CTR_US + SERVO_RANGE_US);
        steerServo.writeMicroseconds(steerUs);

        int escUs = ESC_NEUTRAL + (packet.throttle * (ESC_MAX - ESC_NEUTRAL) / 100);
        escUs = constrain(escUs, ESC_MIN, ESC_MAX);
        esc.writeMicroseconds(escUs);

        digitalWrite(PIN_LED1, packet.led1 ? HIGH : LOW);
        digitalWrite(PIN_LED2, packet.led2 ? HIGH : LOW);
        digitalWrite(PIN_FAN, packet.fan ? HIGH : LOW);
    }

    static uint32_t lastTelMs = 0;
    if (now - lastTelMs >= TEL_PERIOD_MS) {
        lastTelMs = now;
        lastVbat = readBattVoltage();

        TelPacket tel = {currentKph, maxKph, lastVbat, 0};
        tel.crc = calcCRC(tel);
        esp_now_send(VOID_MAC, reinterpret_cast<uint8_t*>(&tel), sizeof(tel));
    }

    static uint32_t lastLogMs = 0;
    if (now - lastLogMs >= LOG_PERIOD_MS) {
        lastLogMs = now;
        Serial.printf("SPD:%5.2f km/h  MAX:%5.2f km/h  BAT:%.2fV\n", currentKph, maxKph, lastVbat);
    }
}
