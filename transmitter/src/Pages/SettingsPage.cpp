#include <Arduino.h>
#include "Page.h"
#include "Screen.h"
#include "Inputs.h"
#include "Globals.h"
#include "Popup.h"
#include "Helpers.h"

namespace {
enum SettingType {
    BOOLEAN,
    INTEGER,
    ACTION
};

enum SettingId {
    SET_SLEEP_DELAY,
    SET_DEADZONE,
    SET_STEER_TRIM,
    SET_TX_INTERVAL,
    SET_TELEM_TIMEOUT,
    SET_ARM_HOLD,
    SET_GYRO_DEADBAND,
    SET_INVERT_THROTTLE,
    SET_INVERT_STEER,
    SET_LIGHTS,
    SET_FAN_PRESET,
    SET_RESET_ALL,
};

struct Setting {
    SettingId id;
    const char* name;
    SettingType type;
    void* value;
    long minVal;
    long maxVal;
};

long sSleepDelay;
long sDeadzone;
long sSteerTrim;
long sTxInterval;
long sTelemTimeout;
long sArmHold;
long sGyroDeadband;

Setting settings[] = {
    {SET_SLEEP_DELAY,     "Sleep Delay",     INTEGER, &sSleepDelay,   500, 120000},
    {SET_DEADZONE,        "Deadzone",        INTEGER, &sDeadzone,       0,    300},
    {SET_STEER_TRIM,      "Steer Trim",      INTEGER, &sSteerTrim,    -20,     20},
    {SET_TX_INTERVAL,     "TX Interval",     INTEGER, &sTxInterval,    20,    200},
    {SET_TELEM_TIMEOUT,   "Telem Timeout",   INTEGER, &sTelemTimeout, 300,   3000},
    {SET_ARM_HOLD,        "Arm Hold",        INTEGER, &sArmHold,      300,   2000},
    {SET_GYRO_DEADBAND,   "Gyro Deadband",   INTEGER, &sGyroDeadband,   0,     50},
    {SET_INVERT_THROTTLE, "Invert Throttle", BOOLEAN, &invertThrottle,  0,      0},
    {SET_INVERT_STEER,    "Invert Steer",    BOOLEAN, &invertSteer,     0,      0},
    {SET_LIGHTS,          "Lights",          ACTION,  nullptr,          0,      0},
    {SET_FAN_PRESET,      "Fan Preset",      ACTION,  nullptr,          0,      0},
    {SET_RESET_ALL,       "Reset All",       ACTION,  nullptr,          0,      0},
};

constexpr int kNumSettings = (int)(sizeof(settings) / sizeof(settings[0]));

bool isMillisecondSetting(SettingId id) {
    return id == SET_SLEEP_DELAY ||
           id == SET_TX_INTERVAL ||
           id == SET_TELEM_TIMEOUT ||
           id == SET_ARM_HOLD;
}

void syncFromGlobals() {
    sSleepDelay = (long)screenSleepDelayMs;
    sDeadzone = (long)axisDeadzone;
    sSteerTrim = (long)steeringTrim;
    sTxInterval = (long)txIntervalMs;
    sTelemTimeout = (long)telemetryTimeoutMs;
    sArmHold = (long)armHoldMs;
    sGyroDeadband = (long)mpuGyroDeadbandX10;
}

void syncToGlobals() {
    screenSleepDelayMs = (uint32_t)constrain(sSleepDelay, 500L, 120000L);
    axisDeadzone = (uint16_t)constrain(sDeadzone, 0L, 300L);
    steeringTrim = (int8_t)constrain(sSteerTrim, -20L, 20L);
    txIntervalMs = (uint32_t)constrain(sTxInterval, 20L, 200L);
    telemetryTimeoutMs = (uint32_t)constrain(sTelemTimeout, 300L, 3000L);
    armHoldMs = (uint16_t)constrain(sArmHold, 300L, 2000L);
    mpuGyroDeadbandX10 = (uint16_t)constrain(sGyroDeadband, 0L, 50L);
    saveProfiles();
}

void formatSettingText(const Setting& setting, char* out, size_t outSize) {
    if (setting.type == INTEGER) {
        long value = *(long*)setting.value;
        if (isMillisecondSetting(setting.id)) {
            snprintf(out, outSize, "%s: %ld ms", setting.name, value);
            return;
        }
        if (setting.id == SET_STEER_TRIM) {
            snprintf(out, outSize, "%s: %+ld", setting.name, value);
            return;
        }
        if (setting.id == SET_GYRO_DEADBAND) {
            long fraction = value % 10L;
            if (fraction < 0) fraction = -fraction;
            snprintf(out, outSize, "%s: %ld.%ld dps", setting.name, value / 10L, fraction);
            return;
        }

        snprintf(out, outSize, "%s: %ld", setting.name, value);
        return;
    }

    if (setting.type == BOOLEAN) {
        snprintf(out, outSize, "%s: %s", setting.name, (*(bool*)setting.value) ? "On" : "Off");
        return;
    }

    if (setting.id == SET_LIGHTS) {
        if (frontLightCmd && rearLightCmd) snprintf(out, outSize, "Lights: Both");
        else if (frontLightCmd) snprintf(out, outSize, "Lights: Front");
        else if (rearLightCmd) snprintf(out, outSize, "Lights: Rear");
        else snprintf(out, outSize, "Lights: Off");
        return;
    }

    if (setting.id == SET_FAN_PRESET) {
        snprintf(out, outSize, "Fan: %u%%", (unsigned)fanPctCmd);
        return;
    }

    snprintf(out, outSize, "%s", setting.name);
}

void runAction(SettingId id) {
    if (id == SET_RESET_ALL) {
        String opts[] = {"Cancel", "RESET"};
        int choice = openPopupMultiChoice("Reset All?", opts, 2, 0);
        if (choice == 1) resetAll();
        return;
    }

    if (id == SET_LIGHTS) {
        String opts[] = {"Off", "Front", "Rear", "Both"};
        int current = 0;
        if (frontLightCmd && rearLightCmd) current = 3;
        else if (frontLightCmd) current = 1;
        else if (rearLightCmd) current = 2;

        int choice = openPopupMultiChoice("Lights", opts, 4, current);
        if (choice < 0) return;

        frontLightCmd = (choice == 1 || choice == 3) ? 1 : 0;
        rearLightCmd = (choice == 2 || choice == 3) ? 1 : 0;
        saveProfiles();
        syncFromGlobals();
        return;
    }

    if (id == SET_FAN_PRESET) {
        String opts[] = {"0%", "25%", "50%", "75%", "100%"};
        int current = 0;
        if (fanPctCmd >= 88) current = 4;
        else if (fanPctCmd >= 63) current = 3;
        else if (fanPctCmd >= 38) current = 2;
        else if (fanPctCmd >= 13) current = 1;

        int choice = openPopupMultiChoice("Fan", opts, 5, current);
        if (choice < 0) return;

        static const uint8_t kPreset[] = {0, 25, 50, 75, 100};
        fanPctCmd = kPreset[choice];
        saveProfiles();
        syncFromGlobals();
    }
}
}

void SettingsPage::init() {
    rotaryEncoderButtonReady = false;
    rotaryEncoderSwitchValue = UNPRESSED;
    syncFromGlobals();
}

void SettingsPage::loop() {
    static bool aReady = true;
    if (getButtonValue(BTN_A) == UNPRESSED) aReady = true;
    if (getButtonValue(BTN_A) == PRESSED && aReady) {
        aReady = false;
        currentPage = menuPage;
        return;
    }

    rotaryEncoderSwitchValue = getRotaryEncoderSwitchValue();
    if (rotaryEncoderSwitchValue == UNPRESSED) rotaryEncoderButtonReady = true;

    int spins = getRotaryEncoderSpins();
    if (spins > 0 && hovered < kNumSettings - 1) hovered++;
    if (spins < 0 && hovered > 0) hovered--;

    drawPageHeader("< Home < Menu < ", "Settings");

    int listYStart = 24;
    const int rowSpacing = 14;
    const int listLeftSpacing = 7;
    if (hovered >= 2) listYStart -= rowSpacing * (hovered - 2);

    u8g2.setFont(FONT_TEXT);
    char buffer[48];
    for (int i = 0; i < kNumSettings; ++i) {
        const Setting& setting = settings[i];
        formatSettingText(setting, buffer, sizeof(buffer));

        if (hovered < i + 3) {
            u8g2.drawStr(listLeftSpacing, listYStart, buffer);
            if (hovered == i) {
                int w = u8g2.getStrWidth(buffer) + 8;
                u8g2.drawRFrame(listLeftSpacing - 4, listYStart - 10, w, 13, 5);
            }
        }
        listYStart += rowSpacing;
    }

    if (rotaryEncoderSwitchValue == PRESSED && rotaryEncoderButtonReady) {
        rotaryEncoderButtonReady = false;
        const Setting& setting = settings[hovered];

        if (setting.type == ACTION) {
            runAction(setting.id);
            return;
        }

        if (setting.type == BOOLEAN) {
            String opts[] = {"Off", "On"};
            bool value = *(bool*)setting.value;
            int choice = openPopupMultiChoice(setting.name, opts, 2, value ? 1 : 0);
            if (choice < 0) return;
            *(bool*)setting.value = (choice == 1);
            syncToGlobals();
            return;
        }

        long current = *(long*)setting.value;
        long value = openPopupNumber(
            setting.name,
            constrain(current, setting.minVal, setting.maxVal),
            setting.minVal,
            setting.maxVal
        );
        *(long*)setting.value = value;
        syncToGlobals();
    }

    drawScrollBar(kNumSettings, hovered);
}
