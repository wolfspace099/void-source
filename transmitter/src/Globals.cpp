#include <Arduino.h>
#include "Globals.h"
#include "Page.h"

namespace {
constexpr uint8_t kDefaultMode = 1;
constexpr uint32_t kDefaultScreenSleepDelayMs = 2000;
constexpr uint16_t kDefaultAxisMin = 100;
constexpr uint16_t kDefaultAxisCenter = 2048;
constexpr uint16_t kDefaultAxisMax = 3995;
constexpr uint16_t kDefaultAxisDeadzone = 40;
constexpr uint16_t kDefaultSteerDeadzone = 80;
constexpr uint16_t kDefaultThrottleDeadzone = 24;
constexpr uint32_t kDefaultTxIntervalMs = 40;
constexpr uint32_t kDefaultTelemetryTimeoutMs = 1000;
constexpr uint16_t kDefaultArmHoldMs = 600;
constexpr uint16_t kDefaultMpuGyroDeadbandX10 = 8;
}

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

RCPacket packet = {RC_PROTOCOL_VERSION, 0, 1500, 1500, DEFAULT_STEER_TRIM, 1, 0, 0, 0, 0, 0};
TelemetryPacket telePacketRx = {0};
Telemetry tele = {0, 0, false, 0, 0, 0, 0, 0, 0, 0};

DriveMode driveModes[MODE_COUNT] = {
    {"ECO", 1300, 1700, 1500},
    {"Normal", 1200, 1800, 1500},
    {"Turbo", 1000, 2000, 1500},
};
uint8_t currentMode = kDefaultMode;
Preferences prefs;

bool    armed = false;
float   ctrlBattVolt = 0;
uint8_t ctrlBattPct = 0;
uint8_t hudTick = 0;
int8_t  steeringTrim = DEFAULT_STEER_TRIM;
uint8_t frontLightCmd = 0;
uint8_t rearLightCmd = 0;
uint8_t fanPctCmd = 0;
bool    invertThrottle = false;
bool    invertSteer = false;
uint16_t axisDeadzone = kDefaultAxisDeadzone;
uint16_t steerDeadzone = kDefaultSteerDeadzone;
uint16_t throttleDeadzone = kDefaultThrottleDeadzone;
uint16_t throttleCalMin = kDefaultAxisMin;
uint16_t throttleCalCenter = kDefaultAxisCenter;
uint16_t throttleCalMax = kDefaultAxisMax;
uint16_t steerCalMin = kDefaultAxisMin;
uint16_t steerCalCenter = kDefaultAxisCenter;
uint16_t steerCalMax = kDefaultAxisMax;
uint16_t steerCtrPwm = 1700;
bool     mpuAvailable = false;
float    mpuHeadingDeg = 0.0f;
float    statsTopSpeedKmh = 0.0f;
float    modeTopSpeedKmh[MODE_COUNT] = {0.0f, 0.0f, 0.0f};
uint32_t txIntervalMs = kDefaultTxIntervalMs;
uint32_t telemetryTimeoutMs = kDefaultTelemetryTimeoutMs;
uint16_t armHoldMs = kDefaultArmHoldMs;
uint16_t mpuGyroDeadbandX10 = kDefaultMpuGyroDeadbandX10;
uint32_t statsRunSeconds = 0;
uint32_t bootMillis = 0;
uint32_t screenSleepDelayMs = kDefaultScreenSleepDelayMs;
uint32_t lastInputActivityMs = 0;
bool     displaySleeping = false;

volatile int encRawCount = 0;

HudPage*      hudPage = new HudPage();
MenuPage*     menuPage = new MenuPage();
SettingsPage* settingsPage = new SettingsPage();
ControlsPage* controlsPage = new ControlsPage();
ModesPage*    modesPage = new ModesPage();
StatsPage*     statsPage     = new StatsPage();
CalibratePage* calibratePage = new CalibratePage();

Page* currentPage = nullptr;
Page* previousPage = nullptr;

void saveProfiles() {
    prefs.putUChar("mode", currentMode);
    prefs.putChar("trim", steeringTrim);
    prefs.putUChar("frontL", frontLightCmd);
    prefs.putUChar("rearL", rearLightCmd);
    prefs.putUChar("fanPct", fanPctCmd);
    prefs.putBool("invT", invertThrottle);
    prefs.putBool("invS", invertSteer);
    prefs.putUShort("deadz", axisDeadzone);
    prefs.putUShort("sDeadz", steerDeadzone);
    prefs.putUShort("tDeadz", throttleDeadzone);
    prefs.putUShort("tMin", throttleCalMin);
    prefs.putUShort("tCtr", throttleCalCenter);
    prefs.putUShort("tMax", throttleCalMax);
    prefs.putUShort("sMin", steerCalMin);
    prefs.putUShort("sCtr", steerCalCenter);
    prefs.putUShort("sMax", steerCalMax);
    prefs.putUShort("sCtrPwm", steerCtrPwm);
    prefs.putFloat("topKmh", statsTopSpeedKmh);
    prefs.putFloat("m0Top", modeTopSpeedKmh[0]);
    prefs.putFloat("m1Top", modeTopSpeedKmh[1]);
    prefs.putFloat("m2Top", modeTopSpeedKmh[2]);
    prefs.putULong("txInt", txIntervalMs);
    prefs.putULong("telTO", telemetryTimeoutMs);
    prefs.putUShort("armHold", armHoldMs);
    prefs.putUShort("gyroDb", mpuGyroDeadbandX10);
    prefs.putULong("runSec", statsRunSeconds);
    prefs.putULong("scrSleep", screenSleepDelayMs);
}

void loadProfiles() {
    currentMode = prefs.getUChar("mode", kDefaultMode);
    steeringTrim = prefs.getChar("trim", DEFAULT_STEER_TRIM);
    frontLightCmd = prefs.getUChar("frontL", 0);
    rearLightCmd = prefs.getUChar("rearL", 0);
    fanPctCmd = prefs.getUChar("fanPct", 0);
    invertThrottle = prefs.getBool("invT", false);
    invertSteer = prefs.getBool("invS", false);
    axisDeadzone = prefs.getUShort("deadz", kDefaultAxisDeadzone);
    steerDeadzone = prefs.getUShort("sDeadz", kDefaultSteerDeadzone);
    throttleDeadzone = prefs.getUShort("tDeadz", kDefaultThrottleDeadzone);
    throttleCalMin = prefs.getUShort("tMin", kDefaultAxisMin);
    throttleCalCenter = prefs.getUShort("tCtr", kDefaultAxisCenter);
    throttleCalMax = prefs.getUShort("tMax", kDefaultAxisMax);
    steerCalMin = prefs.getUShort("sMin", kDefaultAxisMin);
    steerCalCenter = prefs.getUShort("sCtr", kDefaultAxisCenter);
    steerCalMax = prefs.getUShort("sMax", kDefaultAxisMax);
    steerCtrPwm = prefs.getUShort("sCtrPwm", 1700);
    statsTopSpeedKmh = prefs.getFloat("topKmh", 0.0f);
    modeTopSpeedKmh[0] = prefs.getFloat("m0Top", 0.0f);
    modeTopSpeedKmh[1] = prefs.getFloat("m1Top", 0.0f);
    modeTopSpeedKmh[2] = prefs.getFloat("m2Top", 0.0f);
    txIntervalMs = prefs.getULong("txInt", kDefaultTxIntervalMs);
    telemetryTimeoutMs = prefs.getULong("telTO", kDefaultTelemetryTimeoutMs);
    armHoldMs = prefs.getUShort("armHold", kDefaultArmHoldMs);
    mpuGyroDeadbandX10 = prefs.getUShort("gyroDb", kDefaultMpuGyroDeadbandX10);
    statsRunSeconds = prefs.getULong("runSec", 0);
    screenSleepDelayMs = prefs.getULong("scrSleep", kDefaultScreenSleepDelayMs);

    currentMode = constrain(currentMode, 0U, (uint8_t)(MODE_COUNT - 1));
    steeringTrim = constrain((int)steeringTrim, -20, 20);
    axisDeadzone = constrain(axisDeadzone, 0U, 300U);
    steerDeadzone = constrain(steerDeadzone, 0U, 300U);
    throttleDeadzone = constrain(throttleDeadzone, 0U, 300U);
    txIntervalMs = constrain(txIntervalMs, 20UL, 200UL);
    telemetryTimeoutMs = constrain(telemetryTimeoutMs, 300UL, 3000UL);
    armHoldMs = constrain(armHoldMs, 300U, 2000U);
    mpuGyroDeadbandX10 = constrain(mpuGyroDeadbandX10, 0U, 50U);
    screenSleepDelayMs = constrain(screenSleepDelayMs, 500UL, 120000UL);
}

void resetAll() {
    currentMode = kDefaultMode;
    steeringTrim = DEFAULT_STEER_TRIM;
    frontLightCmd = 0;
    rearLightCmd = 0;
    fanPctCmd = 0;
    invertThrottle = false;
    invertSteer = false;
    axisDeadzone = kDefaultAxisDeadzone;
    steerDeadzone = kDefaultSteerDeadzone;
    throttleDeadzone = kDefaultThrottleDeadzone;
    throttleCalMin = kDefaultAxisMin;
    throttleCalCenter = kDefaultAxisCenter;
    throttleCalMax = kDefaultAxisMax;
    steerCalMin = kDefaultAxisMin;
    steerCalCenter = kDefaultAxisCenter;
    steerCalMax = kDefaultAxisMax;
    steerCtrPwm = 1700;
    statsTopSpeedKmh = 0.0f;
    modeTopSpeedKmh[0] = 0.0f;
    modeTopSpeedKmh[1] = 0.0f;
    modeTopSpeedKmh[2] = 0.0f;
    txIntervalMs = kDefaultTxIntervalMs;
    telemetryTimeoutMs = kDefaultTelemetryTimeoutMs;
    armHoldMs = kDefaultArmHoldMs;
    mpuGyroDeadbandX10 = kDefaultMpuGyroDeadbandX10;
    statsRunSeconds = 0;
    screenSleepDelayMs = kDefaultScreenSleepDelayMs;
    saveProfiles();
    currentPage = hudPage;
}
