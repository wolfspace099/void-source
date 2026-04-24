#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Helpers.h"
#include "Globals.h"
#include "Inputs.h"
#include "Screen.h"
#include "Page.h"
#include "Now.h"

namespace {
constexpr uint32_t kScreenUpdateIntervalMs = 50;
constexpr uint32_t kStatsPersistIntervalMs = 30000UL;
constexpr uint16_t kNeutralPwm = 1500;
constexpr int kFullPwmSpanUs = 500;
constexpr int kSteerRangeUs = 500;
constexpr float kMicrosToSeconds = 1.0f / 1000000.0f;
constexpr float kMinGyroSampleDt = 0.005f;
constexpr float kRadToDeg = 57.2957795f;
constexpr uint8_t kMpuCandidates[] = {0x68, 0x69};
}

static Adafruit_MPU6050 mpu;
static uint32_t mpuLastSampleUs = 0;

struct __attribute__((packed)) LegacyRCPacket {
    int16_t steering;
    int16_t throttle;
    int8_t  trim;
    bool    led1;
    bool    led2;
    bool    fan;
    uint8_t crc;
};

static void refreshPacketState(bool includeLiveSticks) {
    if (includeLiveSticks && armed) {
        packet.throttle = getThrottlePWM();
        packet.steering = getSteerPWM();
    } else {
        packet.throttle = kNeutralPwm;
        packet.steering = steerCtrPwm;
    }

    packet.version = RC_PROTOCOL_VERSION;
    packet.trim = steeringTrim;
    packet.driveMode = currentMode;
    packet.frontLightCmd = frontLightCmd;
    packet.rearLightCmd = rearLightCmd;
    packet.fanPctCmd = fanPctCmd;
    packet.armed = armed;
    packet.crc = packetCRC(packet);
}

static int16_t pwmToPercent(uint16_t pwm, uint16_t lo, uint16_t ctr, uint16_t hi) {
    if (pwm >= ctr) return (int16_t)constrain(map((int)pwm, (int)ctr, (int)hi, 0, 100), 0, 100);
    return (int16_t)constrain(map((int)pwm, (int)lo, (int)ctr, -100, 0), -100, 0);
}

static int16_t applyModeToThrottlePercent(int16_t throttlePct) {
    DriveMode& mode = driveModes[currentMode];
    const int posSpanUs = constrain((int)mode.maxPWM - (int)mode.midPWM, 0, kFullPwmSpanUs);
    const int negSpanUs = constrain((int)mode.midPWM - (int)mode.minPWM, 0, kFullPwmSpanUs);

    if (throttlePct > 0) {
        throttlePct = (int16_t)((int32_t)throttlePct * posSpanUs / kFullPwmSpanUs);
    } else if (throttlePct < 0) {
        throttlePct = (int16_t)((int32_t)throttlePct * negSpanUs / kFullPwmSpanUs);
    }
    return constrain(throttlePct, -100, 100);
}

static LegacyRCPacket makeLegacyPacket(int16_t steerPct, int16_t throttlePct) {
    LegacyRCPacket tx = {};
    tx.steering = constrain((int)steerPct + (int)steeringTrim, -100, 100);
    tx.throttle = applyModeToThrottlePercent(throttlePct);
    tx.trim = steeringTrim;
    tx.led1 = (frontLightCmd != 0);
    tx.led2 = (rearLightCmd != 0);
    tx.fan = (fanPctCmd > 0);
    tx.crc = packetCRC(tx);
    return tx;
}

static void setupMPU6050() {
    mpuAvailable = false;

    for (uint8_t i = 0; i < sizeof(kMpuCandidates); ++i) {
        if (mpu.begin(kMpuCandidates[i], &Wire)) {
            mpuAvailable = true;
            break;
        }
    }

    if (!mpuAvailable) {
        Wire.begin();
        Wire.setClock(100000);
        Wire.setTimeOut(20);
        u8g2.begin();
        u8g2.setContrast(255);
        return;
    }

    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpuHeadingDeg = 0.0f;
    mpuLastSampleUs = micros();
}

static void tickMPU6050() {
    if (!mpuAvailable) return;

    uint32_t nowUs = micros();
    if (mpuLastSampleUs == 0) {
        mpuLastSampleUs = nowUs;
        return;
    }

    float dt = (nowUs - mpuLastSampleUs) * kMicrosToSeconds;
    if (dt < kMinGyroSampleDt) return;
    mpuLastSampleUs = nowUs;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    mpu.getEvent(&accel, &gyro, &temp);

    float zDps = gyro.gyro.z * kRadToDeg;
    float deadband = ((float)mpuGyroDeadbandX10) / 10.0f;
    if (fabsf(zDps) < deadband) zDps = 0.0f;

    mpuHeadingDeg += zDps * dt;
    while (mpuHeadingDeg >= 360.0f) mpuHeadingDeg -= 360.0f;
    while (mpuHeadingDeg < 0.0f)    mpuHeadingDeg += 360.0f;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);
    Wire.setTimeOut(20);

    setupScreen();
    delay(120);
    bootStatus("Setting up inputs", 2, 7);
    setupInputs();
    bootStatus("Loading settings", 3, 7);
    prefs.begin("voidOS", false);
    loadProfiles();
    armed = false;
    bootStatus("Initializing MPU6050", 4, 7);
    setupMPU6050();

    bootStatus("Starting WiFi services", 5, 7);
    bootStatus("Starting ESP-NOW", 6, 7);
    nowInit();
    bootStatus("Boot complete", 7, 7);

    packet.throttle = kNeutralPwm;
    packet.steering = steerCtrPwm;
    packet.seq = 0;
    packet.trim = steeringTrim;
    refreshPacketState(false);

    currentPage = hudPage;
    previousPage = nullptr;
    bootMillis = millis();
    lastInputActivityMs = millis();
    displaySleeping = false;
    finishBootScreen();
}

void loop() {
    inputsTick();
    if (consumeUserActivity()) {
        lastInputActivityMs = millis();
        if (displaySleeping) {
            u8g2.setPowerSave(0);
            displaySleeping = false;
        }
    }

    if (!displaySleeping && (millis() - lastInputActivityMs >= screenSleepDelayMs)) {
        u8g2.setPowerSave(1);
        displaySleeping = true;
    }

    tickMPU6050();
    hudTick++;

    nowProcessTelemetry();
    nowApplyTelemetryTimeout(telemetryTimeoutMs);

    if (tele.connected) {
        float liveMax = max(tele.speedKmh, telePacketRx.kphMax);
        if (liveMax > statsTopSpeedKmh) statsTopSpeedKmh = liveMax;
        if (liveMax > modeTopSpeedKmh[currentMode]) modeTopSpeedKmh[currentMode] = liveMax;
    }

    static uint32_t lastStatsPersistMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastStatsPersistMs >= kStatsPersistIntervalMs) {
        uint32_t sessionSec = (nowMs - bootMillis) / 1000UL;
        statsRunSeconds += sessionSec;
        bootMillis = nowMs;
        saveProfiles();
        lastStatsPersistMs = nowMs;
    }

    every(kScreenUpdateIntervalMs) {
        if (!displaySleeping) {
            if (previousPage != currentPage) {
                currentPage->init();
                previousPage = currentPage;
            }

            u8g2.clearBuffer();
            currentPage->loop();
            u8g2.sendBuffer();
        }
    }

    static uint32_t lastRadioSendMs = 0;
    uint32_t radioNow = millis();
    if (radioNow - lastRadioSendMs >= txIntervalMs) {
        lastRadioSendMs = radioNow;

        if (currentPage == hudPage) {
            refreshPacketState(true);
            packet.seq++;

            int16_t steerPct = getSteerPercentFiltered();
            int16_t throttlePct = getThrottlePercentFiltered();
            if (!armed) {
                steerPct = 0;
                throttlePct = 0;
            }

            LegacyRCPacket tx = makeLegacyPacket(steerPct, throttlePct);
            nowSendPacket((const uint8_t*)&tx, sizeof(tx));
        } else if (calibratePage->calTestActive) {
            packet.seq++;
            uint16_t lo = (uint16_t)constrain((int)steerCtrPwm - kSteerRangeUs, 1000, 2200);
            uint16_t hi = (uint16_t)constrain((int)steerCtrPwm + kSteerRangeUs, 1000, 2200);
            int16_t steerPct = pwmToPercent(calibratePage->calTestSteerPwm, lo, steerCtrPwm, hi);
            LegacyRCPacket tx = makeLegacyPacket(steerPct, 0);
            nowSendPacket((const uint8_t*)&tx, sizeof(tx));
        }
    }
}
