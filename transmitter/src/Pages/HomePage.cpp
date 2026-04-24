#include <Arduino.h>
#include <math.h>
#include "Page.h"
#include "Screen.h"
#include "Inputs.h"
#include "Globals.h"
#include "Helpers.h"
#include "Popup.h"

namespace {
constexpr int kSpeedPanelX = 38;
constexpr int kSpeedPanelY = 12;
constexpr int kSpeedPanelW = 52;
constexpr int kSpeedPanelH = 52;

void syncPressState(bool& pressInProgress, bool& holdTriggered) {
    bool down = (digitalRead(PIN_ENC_SW) == LOW);
    pressInProgress = down;
    holdTriggered = down;
}

void openQuickOutputPopup() {
    String targetOpts[] = {"Front", "Rear", "Fan"};
    int target = openPopupMultiChoice("Quick Output", targetOpts, 3, 0);
    if (target < 0) return;

    bool currentOn = false;
    if (target == 0) currentOn = (frontLightCmd != 0);
    else if (target == 1) currentOn = (rearLightCmd != 0);
    else currentOn = (fanPctCmd > 0);

    String stateOpts[] = {"Off", "On"};
    int state = openPopupMultiChoice(targetOpts[target], stateOpts, 2, currentOn ? 1 : 0);
    if (state < 0) return;

    bool on = (state == 1);
    if (target == 0) frontLightCmd = on ? 1 : 0;
    else if (target == 1) rearLightCmd = on ? 1 : 0;
    else fanPctCmd = on ? 100 : 0;

    saveProfiles();
}
}

void HudPage::init() {
    rotaryEncoderButtonReady = false;
    rotaryEncoderSwitchValue = UNPRESSED;
    syncPressState(pressInProgress, holdTriggered);
}

void HudPage::loop() {
    startTime = millis();

    static bool bReady = true;
    static bool cReady = true;
    static bool dReady = true;

    int btnB = getButtonValue(BTN_B);
    int btnC = getButtonValue(BTN_C);
    int btnD = getButtonValue(BTN_D);

    if (btnB == UNPRESSED) bReady = true;
    if (btnC == UNPRESSED) cReady = true;
    if (btnD == UNPRESSED) dReady = true;

    if (btnD == PRESSED && dReady) {
        dReady = false;
        currentPage = menuPage;
        return;
    }

    if (btnB == PRESSED && bReady) {
        bReady = false;
        String modeOpts[] = {"Eco", "Drive", "Turbo"};
        int choice = openPopupMultiChoice("Select a Mode", modeOpts, 3, currentMode);
        if (choice >= 0) {
            currentMode = (uint8_t)constrain(choice, 0, MODE_COUNT - 1);
            packet.driveMode = currentMode;
            prefs.putUChar("mode", currentMode);
        }
        syncPressState(pressInProgress, holdTriggered);
        return;
    }

    if (btnC == PRESSED && cReady) {
        cReady = false;
        openQuickOutputPopup();
        syncPressState(pressInProgress, holdTriggered);
        return;
    }

    bool encDown = (digitalRead(PIN_ENC_SW) == LOW);
    if (encDown && !pressInProgress) {
        pressInProgress = true;
        holdTriggered = false;
    }
    if (pressInProgress && getRotaryEncoderHeld(armHoldMs)) {
        holdTriggered = true;
        armed = !armed;
        if (!armed) {
            packet.throttle = 1500;
            packet.steering = 1500;
            resetThrottleRamp();
        }
    }
    if (!encDown && pressInProgress) {
        pressInProgress = false;
        if (!holdTriggered) {
            currentPage = menuPage;
            return;
        }
    }

    u8g2.setFont(FONT_TEXT_MONOSPACE);
    u8g2.drawStr(0, 8, "VOID");

    u8g2.setFont(u8g2_font_siji_t_6x10);
    u8g2.drawGlyph(110, 10, tele.connected ? 0xe21a : 0xe217);

    drawStringButton(4, 25, "B", "Mode", FONT_TEXT);
    u8g2.setFont(FONT_BOLD_HEADER);
    u8g2.setFontMode(1);
    const char* modeCompact = "Drive";
    if (currentMode == 0) modeCompact = "Eco";
    else if (currentMode == 2) modeCompact = "Turbo";
    u8g2.drawStr(1, 41, modeCompact);
    u8g2.setFontMode(0);

    drawStringButton(4, 59, "D", "Menu", FONT_TEXT);

    u8g2.drawRFrame(kSpeedPanelX, kSpeedPanelY, kSpeedPanelW, kSpeedPanelH, 5);

    int compassCx = kSpeedPanelX + kSpeedPanelW - 9;
    int compassCy = kSpeedPanelY + 8;
    if (mpuAvailable) {
        float ang = ((-mpuHeadingDeg) - 90.0f) * (PI / 180.0f);
        int px = compassCx + (int)(cosf(ang) * 4.0f);
        int py = compassCy + (int)(sinf(ang) * 4.0f);
        u8g2.drawLine(compassCx, compassCy, px, py);
        u8g2.drawDisc(px, py, 1);
    } else {
        u8g2.drawLine(compassCx - 2, compassCy, compassCx + 2, compassCy);
    }

    if (!armed || !tele.connected) {
        u8g2.setFont(FONT_BOLD_HEADER);
        u8g2.drawStr(kSpeedPanelX + 4, kSpeedPanelY + 13, "!");
    }

    char speedBuf[12];
    if (tele.connected) snprintf(speedBuf, sizeof(speedBuf), "%d", (int)tele.speedKmh);
    else snprintf(speedBuf, sizeof(speedBuf), "--");

    u8g2.setFont(FONT_BOLD_HEADER);
    int speedW = u8g2.getStrWidth(speedBuf);
    u8g2.drawStr(kSpeedPanelX + (kSpeedPanelW - speedW) / 2, kSpeedPanelY + 34, speedBuf);

    const char* unit = "km/h";
    u8g2.setFont(FONT_TEXT_MONOSPACE);
    int unitW = u8g2.getStrWidth(unit);
    u8g2.drawStr(kSpeedPanelX + (kSpeedPanelW - unitW) / 2, kSpeedPanelY + 46, unit);

    const int textY = 24;
    const int offset = 13;
    const int iconColX = 96;
    uint8_t battPct = tele.connected ? tele.battPct : 0;

    const int battX = iconColX + 1;
    const int battY = textY - 6;
    const int battW = 10;
    const int battH = 6;
    const int rightColCenterX = battX + (battW / 2);
    u8g2.drawFrame(battX, battY, battW, battH);
    u8g2.drawBox(battX + battW, battY + 2, 1, 2);

    int fillW = constrain(map((int)battPct, 0, 100, 0, battW - 2), 0, battW - 2);
    if (tele.connected && fillW > 0) {
        u8g2.drawBox(battX + 1, battY + 1, fillW, battH - 2);
    }

    drawStringButton(rightColCenterX, textY + offset * 2 - 1, "C", "", FONT_TEXT);

    totalDrawTime = millis() - startTime;
}
