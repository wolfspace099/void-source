#include <Arduino.h>
#include "Page.h"
#include "Screen.h"
#include "Inputs.h"
#include "Globals.h"
#include "Helpers.h"

namespace {
const char* modeLabel(uint8_t mode) {
    if (mode == 0) return "Eco";
    if (mode == 2) return "Turbo";
    return "Drive";
}
}

void ModesPage::init() {
    rotaryEncoderButtonReady = false;
    rotaryEncoderSwitchValue = UNPRESSED;
    hovered = currentMode;
    getRotaryEncoderSpins();
}

void ModesPage::loop() {
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
    if (spins > 0 && hovered < MODE_COUNT - 1) hovered++;
    if (spins < 0 && hovered > 0) hovered--;

    drawPageHeader("< Home < Menu < ", "Modes");

    int listY = 24;
    const int rowSpacing = 14;
    const int listX = 8;

    u8g2.setFont(FONT_TEXT);
    for (int i = 0; i < MODE_COUNT; ++i) {
        const char* label = modeLabel(i);
        if (i == (int)currentMode) u8g2.drawStr(listX, listY, ">");
        u8g2.drawStr(listX + 10, listY, label);
        if (hovered == i) {
            int w = u8g2.getStrWidth(label);
            u8g2.drawRFrame(listX + 8, listY - 10, w + 6, 13, 5);
        }
        listY += rowSpacing;
    }

    const int panelX = 72;
    const int panelY = 14;
    const int panelW = 54;

    float topKmh = modeTopSpeedKmh[hovered];
    char speedBuf[12];
    if (topKmh <= 0.05f) snprintf(speedBuf, sizeof(speedBuf), "--");
    else snprintf(speedBuf, sizeof(speedBuf), "%.1f", topKmh);

    u8g2.setFont(FONT_BOLD_HEADER);
    int speedW = u8g2.getStrWidth(speedBuf);
    u8g2.drawStr(panelX + (panelW - speedW) / 2, panelY + 24, speedBuf);

    const char* unit = "km/h";
    u8g2.setFont(FONT_TINY_TEXT);
    int unitW = u8g2.getStrWidth(unit);
    u8g2.drawStr(panelX + (panelW - unitW) / 2, panelY + 31, unit);

    const int barX = panelX + 7;
    const int barY = panelY + 37;
    const int barW = panelW - 14;
    const int barH = 8;
    u8g2.drawFrame(barX, barY, barW, barH);

    int fillW = 0;
    if (topKmh > 0.05f) {
        fillW = map((int)(topKmh + 0.5f), 0, 60, 1, barW - 2);
        fillW = constrain(fillW, 0, barW - 2);
    }
    if (fillW > 0) u8g2.drawBox(barX + 1, barY + 1, fillW, barH - 2);

    if (rotaryEncoderSwitchValue == PRESSED && rotaryEncoderButtonReady) {
        rotaryEncoderButtonReady = false;
        currentMode = hovered;
        packet.driveMode = currentMode;
        prefs.putUChar("mode", currentMode);
        currentPage = menuPage;
    }
}
