#include <Arduino.h>
#include "Page.h"
#include "Screen.h"
#include "Inputs.h"
#include "Globals.h"
#include "Helpers.h"

static void waitEncRelease() {
    uint32_t t = millis();
    while (digitalRead(PIN_ENC_SW) == LOW) {
        if (millis() - t > 700) break;
        delay(1);
    }
}

void CalibratePage::resetSweep() {
    sweepSteerMin = 4095;
    sweepSteerMax = 0;
    sweepThrotMin = 4095;
    sweepThrotMax = 0;
    centerSteer = 2048;
    centerThrot = 2048;
    calValid = false;
}

void CalibratePage::init() {
    state = CAL_MENU;
    menuHovered = 0;
    encReady = false;
    aReady = false;
    calTestActive = false;
    calTestSteerPwm = steerCtrPwm;
    resetSweep();
    getRotaryEncoderSpins();
}

void CalibratePage::loopMenu() {
    static const char* opts[] = { "Wizard", "Test" };
    const int N = 2;

    int spins = getRotaryEncoderSpins();
    if (spins > 0) menuHovered = (menuHovered + 1) % N;
    if (spins < 0) menuHovered = (menuHovered - 1 + N) % N;

    drawPageHeader("< Home < Menu < ", "Calibrate");

    u8g2.setFont(FONT_TEXT);
    int y = 30;
    for (int i = 0; i < N; i++) {
        u8g2.drawStr(14, y, opts[i]);
        if (i == menuHovered) {
            int tw = u8g2.getStrWidth(opts[i]);
            u8g2.drawRFrame(10, y - 10, tw + 8, 13, 4);
        }
        y += 14;
    }

    u8g2.setFont(FONT_TINY_TEXT);
    u8g2.drawStr(4, 63, "ENC=Select  A=Back");

    int encSw = getRotaryEncoderSwitchValue();
    if (encSw == UNPRESSED) encReady = true;
    if (encSw == PRESSED && encReady) {
        encReady = false;
        waitEncRelease();
        getRotaryEncoderSpins();
        if (menuHovered == 0) {
            resetSweep();
            state = CAL_WIZ_SWEEP;
        } else {
            state = CAL_TEST;
        }
    }
}

void CalibratePage::loopWizardSweep() {
    uint16_t rawS = (uint16_t)analogRead(PIN_STEER);
    uint16_t rawT = (uint16_t)analogRead(PIN_THROTTLE);

    if (rawS < sweepSteerMin) sweepSteerMin = rawS;
    if (rawS > sweepSteerMax) sweepSteerMax = rawS;
    if (rawT < sweepThrotMin) sweepThrotMin = rawT;
    if (rawT > sweepThrotMax) sweepThrotMax = rawT;

    drawPageHeader("< Calibrate < ", "Wizard");

    u8g2.setFont(FONT_TEXT);
    u8g2.drawStr(4, 22, "1. Sweep BOTH sticks");
    u8g2.drawStr(4, 32, "to all 4 corners");

    char buf[32];
    u8g2.setFont(FONT_TEXT_MONOSPACE);
    snprintf(buf, sizeof(buf), "S:%4u-%4u", sweepSteerMin, sweepSteerMax);
    u8g2.drawStr(4, 46, buf);
    snprintf(buf, sizeof(buf), "T:%4u-%4u", sweepThrotMin, sweepThrotMax);
    u8g2.drawStr(4, 56, buf);

    u8g2.setFont(FONT_TINY_TEXT);
    u8g2.drawStr(4, 63, "ENC=Next  A=Cancel");

    int encSw = getRotaryEncoderSwitchValue();
    if (encSw == UNPRESSED) encReady = true;
    if (encSw == PRESSED && encReady) {
        encReady = false;
        waitEncRelease();
        getRotaryEncoderSpins();
        state = CAL_WIZ_CENTER;
    }
}

void CalibratePage::loopWizardCenter() {
    drawPageHeader("< Calibrate < ", "Wizard");

    uint16_t rawS = (uint16_t)analogRead(PIN_STEER);
    uint16_t rawT = (uint16_t)analogRead(PIN_THROTTLE);

    u8g2.setFont(FONT_TEXT);
    u8g2.drawStr(4, 22, "2. Center BOTH sticks");
    u8g2.drawStr(4, 32, "hold still, press ENC");

    char buf[32];
    u8g2.setFont(FONT_TEXT_MONOSPACE);
    snprintf(buf, sizeof(buf), "S now:%4u", rawS);
    u8g2.drawStr(4, 46, buf);
    snprintf(buf, sizeof(buf), "T now:%4u", rawT);
    u8g2.drawStr(4, 56, buf);

    u8g2.setFont(FONT_TINY_TEXT);
    u8g2.drawStr(4, 63, "ENC=Capture  A=Cancel");

    int encSw = getRotaryEncoderSwitchValue();
    if (encSw == UNPRESSED) encReady = true;
    if (encSw == PRESSED && encReady) {
        encReady = false;

        uint32_t sAcc = 0;
        uint32_t tAcc = 0;
        for (int i = 0; i < 32; i++) {
            sAcc += analogRead(PIN_STEER);
            tAcc += analogRead(PIN_THROTTLE);
            delay(8);
        }
        centerSteer = (uint16_t)(sAcc / 32);
        centerThrot = (uint16_t)(tAcc / 32);

        calValid =
            (sweepSteerMax - sweepSteerMin > 400) &&
            (sweepThrotMax - sweepThrotMin > 400) &&
            (centerSteer > sweepSteerMin + 100) &&
            (centerSteer < sweepSteerMax - 100) &&
            (centerThrot > sweepThrotMin + 100) &&
            (centerThrot < sweepThrotMax - 100);

        if (calValid) {
            steerCalMin = sweepSteerMin;
            steerCalCenter = centerSteer;
            steerCalMax = sweepSteerMax;
            throttleCalMin = sweepThrotMin;
            throttleCalCenter = centerThrot;
            throttleCalMax = sweepThrotMax;
            saveProfiles();
            resetThrottleRamp();
        }

        waitEncRelease();
        getRotaryEncoderSpins();
        state = CAL_WIZ_RESULT;
    }
}

void CalibratePage::loopWizardResult() {
    drawPageHeader("< Calibrate < ", "Result");

    u8g2.setFont(FONT_BOLD_HEADER);
    if (calValid) {
        u8g2.drawStr(4, 22, "Saved");
    } else {
        u8g2.drawStr(4, 22, "Error");
    }

    char buf[32];
    u8g2.setFont(FONT_TEXT_MONOSPACE);
    snprintf(buf, sizeof(buf), "S:%4u %4u %4u", sweepSteerMin, centerSteer, sweepSteerMax);
    u8g2.drawStr(4, 36, buf);
    snprintf(buf, sizeof(buf), "T:%4u %4u %4u", sweepThrotMin, centerThrot, sweepThrotMax);
    u8g2.drawStr(4, 46, buf);

    u8g2.setFont(FONT_TINY_TEXT);
    if (calValid) {
        u8g2.drawStr(4, 56, "Calibration updated.");
    } else {
        u8g2.drawStr(4, 56, "Range/center invalid.");
    }
    u8g2.drawStr(4, 63, "ENC=Menu  A=Cancel");

    int encSw = getRotaryEncoderSwitchValue();
    if (encSw == UNPRESSED) encReady = true;
    if (encSw == PRESSED && encReady) {
        encReady = false;
        waitEncRelease();
        getRotaryEncoderSpins();
        state = CAL_MENU;
    }
}

void CalibratePage::loopTest() {
    drawPageHeader("< Calibrate < ", "Test");

    uint16_t rawS = (uint16_t)analogRead(PIN_STEER);
    uint16_t rawT = (uint16_t)analogRead(PIN_THROTTLE);
    int16_t steerPct = getSteerPercentFiltered();
    int16_t throtPct = getThrottlePercentFiltered();

    char buf[32];
    u8g2.setFont(FONT_TEXT_MONOSPACE);
    snprintf(buf, sizeof(buf), "S raw:%4u %4d", rawS, steerPct);
    u8g2.drawStr(4, 22, buf);
    snprintf(buf, sizeof(buf), "T raw:%4u %4d", rawT, throtPct);
    u8g2.drawStr(4, 32, buf);
    snprintf(buf, sizeof(buf), "S cal:%u/%u/%u", steerCalMin, steerCalCenter, steerCalMax);
    u8g2.drawStr(4, 46, buf);
    snprintf(buf, sizeof(buf), "T cal:%u/%u/%u", throttleCalMin, throttleCalCenter, throttleCalMax);
    u8g2.drawStr(4, 56, buf);

    u8g2.setFont(FONT_TINY_TEXT);
    u8g2.drawStr(4, 63, "A=Back");

    calTestActive = false;
}

void CalibratePage::loop() {
    if (getButtonValue(BTN_A) == UNPRESSED) aReady = true;
    if (getButtonValue(BTN_A) == PRESSED && aReady) {
        aReady = false;
        calTestActive = false;
        if (state == CAL_MENU) {
            currentPage = menuPage;
            return;
        }
        state = CAL_MENU;
        encReady = false;
        getRotaryEncoderSpins();
        return;
    }

    switch (state) {
        case CAL_MENU:       loopMenu(); break;
        case CAL_WIZ_SWEEP:  loopWizardSweep(); break;
        case CAL_WIZ_CENTER: loopWizardCenter(); break;
        case CAL_WIZ_RESULT: loopWizardResult(); break;
        case CAL_TEST:       loopTest(); break;
    }
}
