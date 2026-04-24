#pragma once

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Preferences.h>

#define FONT_BOLD_HEADER  u8g2_font_prospero_bold_nbp_tf
#define FONT_HEADER       u8g2_font_prospero_nbp_tf
#define FONT_TEXT         u8g2_font_heisans_tr
#define FONT_TEXT_MONOSPACE u8g2_font_spleen5x8_mf
#define FONT_TINY_TEXT    u8g2_font_4x6_mf
#define FONT_TINY_NUMBERS u8g2_font_micro_mn

#define OFF       0x1
#define ON        0x0
#define UNPRESSED 0x1
#define PRESSED   0x0

#define PIN_ENC_CLK 5
#define PIN_ENC_DT 21
#define PIN_ENC_SW 10
#define PIN_THROTTLE 2
#define PIN_STEER 3
#define PIN_BTN_A 4
#define PIN_BTN_B 8
#define PIN_BTN_C 9
#define PIN_BTN_D 20

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

#define RC_PROTOCOL_VERSION 1
#define ESPNOW_CHANNEL 1
#define TX_INTERVAL_MS 40
#define DEFAULT_STEER_TRIM -20

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  seq;
    uint16_t throttle;
    uint16_t steering;
    int8_t   trim;
    uint8_t  driveMode;
    uint8_t  frontLightCmd;
    uint8_t  rearLightCmd;
    uint8_t  fanPctCmd;
    uint8_t  armed;
    uint8_t  crc;
} RCPacket;

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  seq;
    float    kph;
    float    kphMax;
    float    vbat;
    uint16_t rpm;
    uint16_t speedKmhX100;
    uint16_t battmV;
    uint8_t  battPct;
    uint8_t  frontLightState;
    uint8_t  rearLightState;
    uint8_t  fanPctState;
    uint8_t  crc;
} TelemetryPacket;

typedef struct {
    float    speedKmh;
    uint8_t  battPct;
    bool     connected;
    uint32_t lastRx;
    uint16_t rpm;
    uint16_t battmV;
    uint8_t  frontLightState;
    uint8_t  rearLightState;
    uint8_t  fanPctState;
    uint8_t  lastSeq;
} Telemetry;

extern RCPacket           packet;
extern TelemetryPacket    telePacketRx;
extern Telemetry          tele;

#define MODE_COUNT 3

struct DriveMode {
    const char* name;
    uint16_t    minPWM;
    uint16_t    maxPWM;
    uint16_t    midPWM;
};

extern DriveMode driveModes[MODE_COUNT];
extern uint8_t   currentMode;
extern Preferences prefs;

extern bool    armed;
extern float   ctrlBattVolt;
extern uint8_t ctrlBattPct;
extern uint8_t hudTick;
extern int8_t  steeringTrim;
extern uint8_t frontLightCmd;
extern uint8_t rearLightCmd;
extern uint8_t fanPctCmd;
extern bool    invertThrottle;
extern bool    invertSteer;
extern uint16_t axisDeadzone;
extern uint16_t steerDeadzone;
extern uint16_t throttleDeadzone;
extern uint16_t throttleCalMin;
extern uint16_t throttleCalCenter;
extern uint16_t throttleCalMax;
extern uint16_t steerCalMin;
extern uint16_t steerCalCenter;
extern uint16_t steerCalMax;
extern uint16_t steerCtrPwm;
extern bool    mpuAvailable;
extern float   mpuHeadingDeg;
extern float   statsTopSpeedKmh;
extern float   modeTopSpeedKmh[MODE_COUNT];
extern uint32_t txIntervalMs;
extern uint32_t telemetryTimeoutMs;
extern uint16_t armHoldMs;
extern uint16_t mpuGyroDeadbandX10;
extern uint32_t statsRunSeconds;
extern uint32_t bootMillis;
extern uint32_t screenSleepDelayMs;
extern uint32_t lastInputActivityMs;
extern bool     displaySleeping;

extern volatile int encRawCount;

class Page;
class HudPage;
class MenuPage;
class SettingsPage;
class ControlsPage;
class ModesPage;
class StatsPage;
class CalibratePage;

extern Page*        currentPage;
extern Page*        previousPage;

extern HudPage*       hudPage;
extern MenuPage*      menuPage;
extern SettingsPage*  settingsPage;
extern ControlsPage*  controlsPage;
extern ModesPage*     modesPage;
extern StatsPage*     statsPage;
extern CalibratePage* calibratePage;

void saveProfiles();
void loadProfiles();
void resetAll();

template<typename T>
inline uint8_t packetCRC(const T& p) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(T) - 1; ++i) {
        crc ^= bytes[i];
    }
    return crc;
}
