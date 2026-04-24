#pragma once

#include <Arduino.h>
#include "Helpers.h"
#include "Globals.h"

class Page {
public:
    virtual void init() = 0;
    virtual void loop() = 0;
    virtual ~Page() = default;

    bool rotaryEncoderButtonReady = false;
    int  rotaryEncoderSwitchValue = UNPRESSED;
};

class HudPage : public Page {
public:
    void init() override;
    void loop() override;
private:
    long startTime = 0;
    long totalDrawTime = 0;
    bool pressInProgress = false;
    bool holdTriggered = false;
};

class MenuPage : public Page {
public:
    void init() override;
    void loop() override;
private:
    int hovered = 0;
    bool backButtonReady = false;
};

class SettingsPage : public Page {
public:
    void init() override;
    void loop() override;
private:
    int hovered = 0;
};

class ControlsPage : public Page {
public:
    void init() override;
    void loop() override;
};

class ModesPage : public Page {
public:
    void init() override;
    void loop() override;
private:
    int hovered = 0;
};

class StatsPage : public Page {
public:
    void init() override;
    void loop() override;
private:
    int hovered = 0;
};

enum CalibState {
    CAL_MENU,
    CAL_WIZ_SWEEP,
    CAL_WIZ_CENTER,
    CAL_WIZ_RESULT,
    CAL_TEST,
};

class CalibratePage : public Page {
public:
    void init() override;
    void loop() override;

    bool     calTestActive  = false;
    uint16_t calTestSteerPwm = 1700;

private:
    CalibState state       = CAL_MENU;
    int        menuHovered = 0;
    bool       encReady    = false;
    bool       aReady      = false;
    bool       calValid    = false;
    uint16_t sweepSteerMin = 4095, sweepSteerMax = 0;
    uint16_t sweepThrotMin = 4095, sweepThrotMax = 0;
    uint16_t centerSteer = 2048, centerThrot = 2048;

    void resetSweep();
    void loopMenu();
    void loopWizardSweep();
    void loopWizardCenter();
    void loopWizardResult();
    void loopTest();
};
