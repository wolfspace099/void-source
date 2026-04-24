#include "Inputs.h"
#include "Globals.h"

namespace {
volatile uint8_t encPrevState = 0;
int previousDetent = 0;
bool switchPressed = false;
uint32_t releasedAtMs = 0;
uint32_t holdStartMs = 0;
bool holdFired = false;
bool userActivity = false;
bool activityInit = false;
int lastEncRawCount = 0;
int lastThrottleRaw = 0;
int lastSteerRaw = 0;
uint8_t lastEncSwRaw = UNPRESSED;
uint8_t lastBtnRaw[4] = {UNPRESSED, UNPRESSED, UNPRESSED, UNPRESSED};

constexpr uint32_t kDebounceMs = 4;
constexpr int kEncoderDirection = 1;
constexpr int kActivityAxisThreshold = 40;
constexpr uint16_t kPwmDeadbandUs = 10;
constexpr uint8_t kSteerAvgSamples = 4;
constexpr uint8_t kThrottleAvgSamples = 8;
constexpr float kThrottleSmoothAlpha = 0.05f;
constexpr float kThrottleSlewPerSec = 180.0f;
constexpr float kSteerSmoothAlpha = 0.10f;
constexpr float kSteerSlewPerSec = 300.0f;

const ButtonId kButtons[] = {BTN_A, BTN_B, BTN_C, BTN_D};

struct AxisFilterState {
    float filtered = 0.0f;
    float command = 0.0f;
    bool initialized = false;
    uint32_t lastMs = 0;
};

AxisFilterState steerState;
AxisFilterState throttleState;
}

static int buttonPin(ButtonId button) {
    switch (button) {
        case BTN_A: return PIN_BTN_A;
        case BTN_B: return PIN_BTN_B;
        case BTN_C: return PIN_BTN_C;
        case BTN_D: return PIN_BTN_D;
        default: return -1;
    }
}

static uint16_t pwmDeadband(uint16_t v, uint16_t lo, uint16_t ctr, uint16_t hi, uint16_t band) {
    if (v <= lo + band) return lo;
    if (v >= hi - band) return hi;
    if (abs((int)v - (int)ctr) <= (int)band) return ctr;
    return v;
}

static int16_t mapJoyCal(uint16_t raw,
                         uint16_t mn,
                         uint16_t mid,
                         uint16_t mx,
                         bool invert,
                         uint16_t deadzone) {
    int32_t centered = (int32_t)raw - (int32_t)mid;
    if (abs(centered) < (int32_t)deadzone) return 0;

    int16_t out = 0;
    if (centered > 0) {
        int32_t span = (int32_t)mx - (int32_t)mid - deadzone;
        out = (span <= 0)
            ? 100
            : (int16_t)constrain(map(centered - deadzone, 0, span, 0, 100), 0, 100);
    } else {
        int32_t span = (int32_t)mid - (int32_t)mn - deadzone;
        out = -((span <= 0)
            ? 100
            : (int16_t)constrain(map(-centered - deadzone, 0, span, 0, 100), 0, 100));
    }

    return invert ? -out : out;
}

static uint16_t readAveragedAnalog(uint8_t pin, uint8_t samples) {
    uint32_t acc = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        acc += analogRead(pin);
    }
    return (uint16_t)(acc / samples);
}

static int16_t updateFilteredPercent(AxisFilterState& state,
                                     int16_t mapped,
                                     float smoothAlpha,
                                     float slewPerSec,
                                     uint32_t nowMs) {
    if (!state.initialized) {
        state.filtered = (float)mapped;
        state.command = (float)mapped;
        state.initialized = true;
        state.lastMs = nowMs;
    } else {
        state.filtered += smoothAlpha * ((float)mapped - state.filtered);
        float dt = (float)(nowMs - state.lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        float maxStep = slewPerSec * dt;
        float delta = state.filtered - state.command;
        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;
        state.command += delta;
        state.lastMs = nowMs;
    }

    float v = state.command;
    int16_t rounded = (v >= 0.0f) ? (int16_t)(v + 0.5f) : (int16_t)(v - 0.5f);
    return constrain(rounded, -100, 100);
}

static int16_t readSteerPercent() {
    uint16_t raw = readAveragedAnalog(PIN_STEER, kSteerAvgSamples);
    int16_t mapped = mapJoyCal(raw,
                               steerCalMin,
                               steerCalCenter,
                               steerCalMax,
                               invertSteer,
                               steerDeadzone);
    return updateFilteredPercent(steerState, mapped, kSteerSmoothAlpha, kSteerSlewPerSec, millis());
}

static int16_t readThrottlePercent() {
    uint16_t raw = readAveragedAnalog(PIN_THROTTLE, kThrottleAvgSamples);
    int16_t mapped = mapJoyCal(raw,
                               throttleCalMin,
                               throttleCalCenter,
                               throttleCalMax,
                               invertThrottle,
                               throttleDeadzone);
    return updateFilteredPercent(throttleState, mapped, kThrottleSmoothAlpha, kThrottleSlewPerSec, millis());
}

void IRAM_ATTR encISR() {
    static const int8_t transitionLut[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    uint8_t a = digitalRead(PIN_ENC_CLK);
    uint8_t b = digitalRead(PIN_ENC_DT);
    uint8_t state = (a << 1) | b;
    uint8_t idx = (encPrevState << 2) | state;
    encPrevState = state;
    encRawCount += transitionLut[idx];
}

void setupInputs() {
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);
    pinMode(PIN_THROTTLE, INPUT);
    pinMode(PIN_STEER, INPUT);

    for (uint8_t i = 0; i < 4; ++i) {
        int pin = buttonPin(kButtons[i]);
        if (pin >= 0) pinMode(pin, INPUT_PULLUP);
    }

    analogReadResolution(12);
#if defined(ESP32)
    analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);
    analogSetPinAttenuation(PIN_STEER, ADC_11db);
#endif

    uint8_t a = digitalRead(PIN_ENC_CLK);
    uint8_t b = digitalRead(PIN_ENC_DT);
    encPrevState = (a << 1) | b;

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), encISR, CHANGE);
}

void inputsTick() {
    bool encPressed = (digitalRead(PIN_ENC_SW) == LOW);
    if (encPressed && holdStartMs == 0) {
        holdStartMs = millis();
        holdFired = false;
    }
    if (!encPressed) holdStartMs = 0;

    every(2000) { readControllerBatt(); }

    int encNow;
    noInterrupts();
    encNow = encRawCount;
    interrupts();

    uint8_t encSwNow = digitalRead(PIN_ENC_SW);
    uint8_t btnNow[4];
    for (uint8_t i = 0; i < 4; ++i) {
        btnNow[i] = getButtonValue(kButtons[i]);
    }

    int throttleNow = analogRead(PIN_THROTTLE);
    int steerNow = analogRead(PIN_STEER);

    if (!activityInit) {
        lastEncRawCount = encNow;
        lastEncSwRaw = encSwNow;
        for (uint8_t i = 0; i < 4; ++i) lastBtnRaw[i] = btnNow[i];
        lastThrottleRaw = throttleNow;
        lastSteerRaw = steerNow;
        activityInit = true;
        return;
    }

    bool movedAxis = (abs(throttleNow - lastThrottleRaw) > kActivityAxisThreshold) ||
                     (abs(steerNow - lastSteerRaw) > kActivityAxisThreshold);
    bool rotatedEncoder = (encNow != lastEncRawCount);
    bool pressedEncoder = (encSwNow != lastEncSwRaw);
    bool pressedButtons = false;
    for (uint8_t i = 0; i < 4; ++i) {
        if (btnNow[i] != lastBtnRaw[i]) {
            pressedButtons = true;
            break;
        }
    }

    if (movedAxis || rotatedEncoder || pressedEncoder || pressedButtons) {
        userActivity = true;
    }

    lastEncRawCount = encNow;
    lastEncSwRaw = encSwNow;
    for (uint8_t i = 0; i < 4; ++i) lastBtnRaw[i] = btnNow[i];
    lastThrottleRaw = throttleNow;
    lastSteerRaw = steerNow;
}

int getRotaryEncoderSpins() {
    int detents;
    noInterrupts();
    detents = encRawCount / 4;
    interrupts();

    int spins = detents - previousDetent;
    previousDetent = detents;
    return spins * kEncoderDirection;
}

int getRotaryEncoderTotalSpins() {
    int detents;
    noInterrupts();
    detents = encRawCount / 4;
    interrupts();
    return detents * kEncoderDirection;
}

int getRotaryEncoderSwitchValue() {
    uint8_t val = digitalRead(PIN_ENC_SW);
    if (val == PRESSED && !switchPressed) {
        if (millis() - releasedAtMs > kDebounceMs) {
            switchPressed = true;
            return PRESSED;
        }
    }
    if (val == UNPRESSED && switchPressed) {
        switchPressed = false;
        releasedAtMs = millis();
        return UNPRESSED;
    }
    return UNPRESSED;
}

bool getRotaryEncoderHeld(uint32_t ms) {
    bool pressed = (digitalRead(PIN_ENC_SW) == LOW);
    if (pressed && !holdFired && holdStartMs > 0 && (millis() - holdStartMs >= ms)) {
        holdFired = true;
        return true;
    }
    return false;
}

bool consumeUserActivity() {
    bool active = userActivity;
    userActivity = false;
    return active;
}

int getButtonValue(ButtonId button) {
    int pin = buttonPin(button);
    if (pin < 0) return UNPRESSED;
    return (digitalRead(pin) == LOW) ? PRESSED : UNPRESSED;
}

int16_t getThrottlePercentFiltered() {
    return readThrottlePercent();
}

int16_t getSteerPercentFiltered() {
    return readSteerPercent();
}

uint16_t getThrottlePWM() {
    DriveMode& mode = driveModes[currentMode];
    int16_t pct = readThrottlePercent();
    uint16_t out = (pct >= 0)
        ? (uint16_t)map(pct, 0, 100, mode.midPWM, mode.maxPWM)
        : (uint16_t)map(pct, -100, 0, mode.minPWM, mode.midPWM);
    out = constrain(out, mode.minPWM, mode.maxPWM);
    return pwmDeadband(out, mode.minPWM, mode.midPWM, mode.maxPWM, kPwmDeadbandUs);
}

uint16_t getSteerPWM() {
    int16_t pct = readSteerPercent();
    int16_t finalPct = constrain((int)pct + (int)steeringTrim, -100, 100);
    uint16_t lo = (uint16_t)constrain((int)steerCtrPwm - 500, 1000, 2200);
    uint16_t hi = (uint16_t)constrain((int)steerCtrPwm + 500, 1000, 2200);
    uint16_t out = (finalPct >= 0)
        ? (uint16_t)map(finalPct, 0, 100, steerCtrPwm, hi)
        : (uint16_t)map(finalPct, -100, 0, lo, steerCtrPwm);
    out = constrain(out, 1000, 2200);
    return pwmDeadband(out, lo, steerCtrPwm, hi, kPwmDeadbandUs);
}

void resetThrottleRamp() {
    throttleState = AxisFilterState{};
    steerState = AxisFilterState{};
}

void readControllerBatt() {
    ctrlBattVolt = 0.0f;
    ctrlBattPct = 0;
}
