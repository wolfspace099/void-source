// ============================================================
//  VOID — Remote Controller Firmware  v1.5
//  Board  : Seeed Studio XIAO ESP32C3
//  Proto  : ESP-NOW  (TX control → Abyss, RX telemetry ← Abyss)
//
//  ── PIN MAP (J6 / J9 → XIAO) ────────────────────────────
//  J6-1  JoyR   (throttle pot)  → GPIO2  / A0
//  J6-2  JoyL   (steering pot)  → GPIO3  / A1
//  J6-3  SW4    (button 4)      → GPIO4
//  J6-4  CLK    (enc clock)     → GPIO5
//  J6-5  SDA                   → GPIO6
//  J6-6  SCL                   → GPIO7
//  J6-7  BT     (enc DT)        → GPIO21
//
//  J9-1  SW3    (button 3)      → GPIO20
//  J9-2  SW2    (button 2)      → GPIO8
//  J9-3  SW1    (button 1)      → GPIO9
//  J9-4  SW     (enc btn)       → GPIO10
//  J9-5  +3V3   (power)
//  J9-6  GND    (power)
//  J9-7  NC
//
//  ── BUTTON FUNCTIONS ────────────────────────────────────
//  SW1  → Hold 2 s anytime = enter joystick calibration wizard
//  SW2  → Toggle front lights (LED1)
//  SW3  → Toggle rear  lights (LED2)
//  SW4  → Toggle fan
//  ENC rotate → steering trim (-20 … +20 steps)
//  ENC press  → reset trim to 0  /  reset max speed
//
//  ── OLED LAYOUT ─────────────────────────────────────────
//  ┌──────────────────────────┐
//  │   VOID RC CONTROLLER    │  ← title bar (inverted)
//  │ STR [========|====    ] │  ← steering bar
//  │ THR [=====|            ]│  ← throttle bar
//  │ TRIM +3   F:on R:off   ●│  ← trim + lights + link dot
//  │  12.4 km/h  MAX 18.3   │  ← speed / max speed
//  │ BAT 7.82V     FAN:on   │  ← battery + fan (or CAL bar)
//  └──────────────────────────┘
//
//  ── RADIO CONFIG ────────────────────────────────────────
//  Channel  : 1  (locked — avoids WiFi channel overlap)
//  Protocol : LR (ESP32 long-range mode, ~3x obstacle penetration)
//  TX power : 21 dBm (max for C3)
//  TX rate  : 25 Hz (reduces airtime, more room for retries)
//  Failsafe : 1000 ms (tolerates indoor packet bursts)
//
//  ── JOYSTICK CALIBRATION ────────────────────────────────
//  Hold SW1 for 2 seconds anytime → calibration wizard
//  Step 1: sweep sticks to all corners → press ENC button
//  Step 2: centre both sticks         → press ENC button
//  Saves min/mid/max to NVS (flash), survives reboot
//
//  ── NOTE: ESP32Encoder NOT used ─────────────────────────
//  That library relies on the Xtensa PCNT hardware peripheral
//  which does not exist on the RISC-V ESP32-C3.
//  Encoder is handled with a plain GPIO rising-edge ISR.
// ============================================================

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// ── Pins ─────────────────────────────────────────────────────
#define PIN_JOY_STEER     3   // JoyL → GPIO3 / A1
#define PIN_JOY_THROTTLE  2   // JoyR → GPIO2 / A0
#define PIN_SW4           4
#define PIN_ENC_CLK       5
#define PIN_ENC_DT       21
#define PIN_ENC_BTN      10
#define PIN_SW1           9
#define PIN_SW2           8
#define PIN_SW3          20
#define PIN_SDA           6
#define PIN_SCL           7

// ── Radio config ─────────────────────────────────────────────
// Channel 1 is locked on both Void and Abyss.
// LR protocol: ESP32 proprietary 512 kbps mode — same API,
// ~3× wall/obstacle penetration vs standard 802.11b.
// Both sides MUST use the same channel and protocol or they
// will not hear each other.
#define ESPNOW_CHANNEL   1
#define TX_INTERVAL_MS  40    // 25 Hz — halved from 50 Hz to cut airtime
#define DEFAULT_STEER_TRIM   -20
#define THROTTLE_SMOOTH_ALPHA 0.05f
#define THROTTLE_SLEW_PER_SEC 180.0f
#define STEER_SMOOTH_ALPHA    0.10f
#define STEER_SLEW_PER_SEC    300.0f
#define STEER_DEADZONE        80
#define THROTTLE_DEADZONE     24

// ── Calibration hold time ────────────────────────────────────
#define CAL_HOLD_MS  2000

// ── Software rotary encoder ──────────────────────────────────
volatile int32_t  encCount  = 0;
volatile uint8_t  encState  = 0;
volatile uint32_t encLastUs = 0;

static const int8_t ENC_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

void IRAM_ATTR encISR() {
    uint32_t now = micros();
    if (now - encLastUs < 2000) return;
    uint8_t cur = (digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT);
    int8_t  dir = ENC_TABLE[(encState << 2) | cur];
    encState = cur;
    if (dir != 0) { encCount += dir; encLastUs = now; }
}

// ── OLED ─────────────────────────────────────────────────────
#define OLED_ADDR  0x3C
#define SCREEN_W   128
#define SCREEN_H    64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ── NVS ──────────────────────────────────────────────────────
Preferences prefs;

// ── Joystick calibration ──────────────────────────────────────
struct JoyCal {
    uint16_t steerMin,  steerMid,  steerMax;
    uint16_t throttMin, throttMid, throttMax;
};
JoyCal cal = {100, 2048, 3995, 100, 2048, 3995};

void saveCal() {
    prefs.begin("joycal", false);
    prefs.putBytes("cal", &cal, sizeof(cal));
    prefs.end();
}
void loadCal() {
    prefs.begin("joycal", true);
    if (prefs.isKey("cal")) prefs.getBytes("cal", &cal, sizeof(cal));
    prefs.end();
}

// ── Control packet (VOID → Abyss) ────────────────────────────
typedef struct __attribute__((packed)) {
    int16_t  steering;
    int16_t  throttle;
    int8_t   trim;
    bool     led1;
    bool     led2;
    bool     fan;
    uint8_t  crc;
} RCPacket;

// ── Telemetry packet (Abyss → VOID) ──────────────────────────
typedef struct __attribute__((packed)) {
    float    kph;
    float    kphMax;
    float    vbat;
    uint8_t  crc;
} TelPacket;

RCPacket pkt = {};
float steerFiltered = 0.0f;
float steerCommand = 0.0f;
bool  steerFilterInit = false;
uint32_t steerLastMs = 0;
float throttleFiltered = 0.0f;
float throttleCommand = 0.0f;
bool  throttleFilterInit = false;
uint32_t throttleLastMs = 0;

// ── Telemetry state ───────────────────────────────────────────
volatile bool      newTel  = false;
volatile TelPacket rxTel   = {};
float    dispKph    = 0.0f;
float    dispKphMax = 0.0f;
float    dispVbat   = 0.0f;

// ── Runtime state ─────────────────────────────────────────────
bool     lastSW2 = HIGH, lastSW3 = HIGH, lastSW4 = HIGH, lastEncBtn = HIGH;
int32_t  lastEncVal = 0;
bool     espNowOK   = false;
uint32_t lastTx     = 0;
uint32_t lastDisp   = 0;
bool     txAck      = false;

// ── SW1 hold-for-calibration state ───────────────────────────
uint32_t sw1HoldStart = 0;
bool     sw1Holding   = false;
bool     sw1CalArmed  = false;

// ── Target MAC ── paste Abyss MAC here ───────────────────────
uint8_t ABYSS_MAC[] = {0x94, 0xA9, 0x90, 0x6B, 0x77, 0x14};

// ── CRC ───────────────────────────────────────────────────────
template<typename T>
uint8_t calcCRC(const T &p) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&p);
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(T) - 1; i++) {
        crc ^= bytes[i];
    }
    return crc;
}

// ── Joystick mapping ─────────────────────────────────────────
int16_t mapJoyCal(uint16_t raw, uint16_t mn, uint16_t mid, uint16_t mx,
                  bool invert = false, uint16_t dead = STEER_DEADZONE)
{
    int32_t v = (int32_t)raw - (int32_t)mid;
    if (abs(v) < (int32_t)dead) return 0;
    int16_t out;
    if (v > 0) {
        int32_t range = (int32_t)mx - (int32_t)mid - dead;
        out = (range <= 0) ? 100
            : (int16_t)constrain(map(v - dead, 0, range, 0, 100), 0, 100);
    } else {
        int32_t range = (int32_t)mid - (int32_t)mn - dead;
        out = -((range <= 0) ? 100
              : (int16_t)constrain(map(-v - dead, 0, range, 0, 100), 0, 100));
    }
    return invert ? -out : out;
}

uint16_t readAveragedAnalog(uint8_t pin, uint8_t samples = 8) {
    uint32_t acc = 0;
    for (uint8_t i = 0; i < samples; ++i) acc += analogRead(pin);
    return (uint16_t)(acc / samples);
}

// ── ESP-NOW callbacks ─────────────────────────────────────────
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    txAck = (status == ESP_NOW_SEND_SUCCESS);
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(TelPacket)) return;
    TelPacket tmp;
    memcpy(&tmp, data, sizeof(TelPacket));
    if (calcCRC(tmp) != tmp.crc) return;
    memcpy((void *)&rxTel, &tmp, sizeof(TelPacket));
    newTel = true;
}

// ── OLED helpers ─────────────────────────────────────────────
void drawCentred(const char *s, int y, uint8_t size = 1) {
    display.setTextSize(size);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_W - w) / 2, y);
    display.print(s);
}

void drawBar(int16_t val, int y) {
    display.drawRect(28, y, 92, 7, SSD1306_WHITE);
    display.drawFastVLine(74, y, 7, SSD1306_WHITE);
    if (val > 0) {
        int w = max(1, (int)(val * 45 / 100));
        display.fillRect(75, y + 1, w, 5, SSD1306_WHITE);
    } else if (val < 0) {
        int w = max(1, (int)(-val * 45 / 100));
        display.fillRect(74 - w, y + 1, w, 5, SSD1306_WHITE);
    }
}

// ── Calibration wizard ───────────────────────────────────────
void runCalibration() {
    uint16_t sMin = 4095, sMax = 0, tMin = 4095, tMax = 0;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawCentred("= CALIBRATION =", 0);
    drawCentred("Sweep BOTH sticks", 14);
    drawCentred("to all 4 corners,", 24);
    drawCentred("then press ENC BTN", 34);
    display.display();

    while (digitalRead(PIN_ENC_BTN) == LOW) delay(10);
    delay(300);

    while (digitalRead(PIN_ENC_BTN) == HIGH) {
        uint16_t rs = analogRead(PIN_JOY_STEER);
        uint16_t rt = analogRead(PIN_JOY_THROTTLE);
        sMin = min(sMin, rs);  sMax = max(sMax, rs);
        tMin = min(tMin, rt);  tMax = max(tMax, rt);
        display.fillRect(0, 54, 128, 10, SSD1306_BLACK);
        display.setCursor(0, 55);
        display.setTextSize(1);
        display.printf("S:%4d-%4d T:%4d-%4d", sMin, sMax, tMin, tMax);
        display.display();
        delay(15);
    }
    while (digitalRead(PIN_ENC_BTN) == LOW) delay(10);
    delay(300);

    display.clearDisplay();
    drawCentred("= CALIBRATION =", 0);
    drawCentred("Centre BOTH sticks", 16);
    drawCentred("then press ENC BTN", 30);
    display.display();

    while (digitalRead(PIN_ENC_BTN) == HIGH) delay(10);
    while (digitalRead(PIN_ENC_BTN) == LOW)  delay(10);
    delay(100);

    uint32_t sAcc = 0, tAcc = 0;
    for (int i = 0; i < 32; i++) {
        sAcc += analogRead(PIN_JOY_STEER);
        tAcc += analogRead(PIN_JOY_THROTTLE);
        delay(8);
    }
    uint16_t sMid = (uint16_t)(sAcc / 32);
    uint16_t tMid = (uint16_t)(tAcc / 32);

    bool ok = (sMax - sMin > 400) && (tMax - tMin > 400)
           && (sMid > sMin + 100)  && (sMid < sMax - 100)
           && (tMid > tMin + 100)  && (tMid < tMax - 100);

    display.clearDisplay();
    if (ok) {
        cal = { sMin, sMid, sMax, tMin, tMid, tMax };
        saveCal();
        drawCentred("SAVED!", 18, 2);
        drawCentred("Rebooting...", 44, 1);
    } else {
        drawCentred("ERROR", 10, 2);
        drawCentred("Range too small.", 36, 1);
        drawCentred("Try again.", 47, 1);
    }
    display.display();
    delay(2000);
    ESP.restart();
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(PIN_SW1,     INPUT_PULLUP);
    pinMode(PIN_SW2,     INPUT_PULLUP);
    pinMode(PIN_SW3,     INPUT_PULLUP);
    pinMode(PIN_SW4,     INPUT_PULLUP);
    pinMode(PIN_ENC_BTN, INPUT_PULLUP);
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);

    encState = (digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  encISR, CHANGE);

    Wire.begin(PIN_SDA, PIN_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 init failed");
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    loadCal();
    pkt.trim = DEFAULT_STEER_TRIM;
    steerLastMs = millis();
    throttleLastMs = millis();

    // ── WiFi + ESP-NOW + range optimisations ─────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Lock to channel 1 and enable LR (long-range) protocol.
    // LR is an Espressif proprietary mode that uses a narrower
    // bandwidth and stronger FEC — same frequency, same API,
    // but penetrates walls ~3× better than standard 802.11b.
    // Both sides must use WIFI_PROTOCOL_LR or they go deaf.
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
        WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

    // Max TX power — 84 units = 21 dBm on C3
    esp_wifi_set_max_tx_power(84);

    if (esp_now_init() == ESP_OK) {
        espNowOK = true;
        esp_now_register_send_cb(onSent);
        esp_now_register_recv_cb(onReceive);

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, ABYSS_MAC, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
        Serial.println("ESP-NOW ready  ch=1  LR+21dBm");
    } else {
        Serial.println("ESP-NOW init FAILED");
    }

    drawCentred("VOID v1.5", 10, 2);
    drawCentred("RC CONTROLLER", 35, 1);
    display.display();
    delay(800);
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // ── Joysticks ────────────────────────────────────────────
    uint16_t rawS = readAveragedAnalog(PIN_JOY_STEER, 4);
    uint16_t rawT = readAveragedAnalog(PIN_JOY_THROTTLE, 8);
    int16_t steerMapped = mapJoyCal(rawS, cal.steerMin,  cal.steerMid,  cal.steerMax,  false, STEER_DEADZONE);
    int16_t throttleMapped = mapJoyCal(rawT, cal.throttMin, cal.throttMid, cal.throttMax, false, THROTTLE_DEADZONE);
    if (!steerFilterInit) {
        steerFiltered = (float)steerMapped;
        steerCommand = (float)steerMapped;
        steerFilterInit = true;
    } else {
        steerFiltered += STEER_SMOOTH_ALPHA * ((float)steerMapped - steerFiltered);
        float dt = (float)(now - steerLastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        float maxStep = STEER_SLEW_PER_SEC * dt;
        float delta = steerFiltered - steerCommand;
        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;
        steerCommand += delta;
    }
    steerLastMs = now;

    if (!throttleFilterInit) {
        throttleFiltered = (float)throttleMapped;
        throttleCommand = (float)throttleMapped;
        throttleFilterInit = true;
    } else {
        throttleFiltered += THROTTLE_SMOOTH_ALPHA * ((float)throttleMapped - throttleFiltered);
        float dt = (float)(now - throttleLastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        float maxStep = THROTTLE_SLEW_PER_SEC * dt;
        float delta = throttleFiltered - throttleCommand;
        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;
        throttleCommand += delta;
    }
    throttleLastMs = now;
    int16_t throttleRounded = (throttleCommand >= 0.0f)
        ? (int16_t)(throttleCommand + 0.5f)
        : (int16_t)(throttleCommand - 0.5f);
    int16_t steerRounded = (steerCommand >= 0.0f)
        ? (int16_t)(steerCommand + 0.5f)
        : (int16_t)(steerCommand - 0.5f);
    pkt.steering = constrain(steerRounded, -100, 100);
    pkt.throttle = constrain(throttleRounded, -100, 100);
    int16_t steerFinal = constrain((int16_t)pkt.steering + pkt.trim, -100, 100);

    // ── Encoder → trim ───────────────────────────────────────
    int32_t encNow;
    noInterrupts();
    encNow = encCount;
    interrupts();

    if (encNow != lastEncVal) {
        pkt.trim = (int8_t)constrain((int32_t)pkt.trim + (encNow - lastEncVal), -20, 20);
        lastEncVal = encNow;
    }

    // Encoder press → reset trim + reset max speed
    bool encBtn = digitalRead(PIN_ENC_BTN);
    if (lastEncBtn == HIGH && encBtn == LOW) {
        pkt.trim = DEFAULT_STEER_TRIM;
        noInterrupts();
        encCount = 0;
        interrupts();
        lastEncVal = 0;
        dispKphMax = 0.0f;
    }
    lastEncBtn = encBtn;

    // ── SW1 — hold 2 s for calibration ───────────────────────
    {
        bool s1 = digitalRead(PIN_SW1);
        if (s1 == HIGH) {
            sw1Holding  = false;
            sw1CalArmed = false;
        } else {
            if (!sw1Holding) {
                sw1Holding   = true;
                sw1HoldStart = now;
                sw1CalArmed  = false;
            } else {
                if (now - sw1HoldStart >= CAL_HOLD_MS) runCalibration();
                sw1CalArmed = true;
            }
        }
    }

    // ── SW2 → front lights ───────────────────────────────────
    bool s2 = digitalRead(PIN_SW2);
    if (lastSW2 == HIGH && s2 == LOW) pkt.led1 = !pkt.led1;
    lastSW2 = s2;

    // ── SW3 → rear lights ────────────────────────────────────
    bool s3 = digitalRead(PIN_SW3);
    if (lastSW3 == HIGH && s3 == LOW) pkt.led2 = !pkt.led2;
    lastSW3 = s3;

    // ── SW4 → fan ────────────────────────────────────────────
    bool s4 = digitalRead(PIN_SW4);
    if (lastSW4 == HIGH && s4 == LOW) pkt.fan = !pkt.fan;
    lastSW4 = s4;

    // ── Ingest telemetry from Abyss ──────────────────────────
    if (newTel) {
        newTel = false;
        TelPacket tel;
        noInterrupts();
        memcpy(&tel, (const void *)&rxTel, sizeof(TelPacket));
        interrupts();
        dispKph  = tel.kph;
        dispVbat = tel.vbat;
        if (tel.kphMax > dispKphMax) dispKphMax = tel.kphMax;
    }

    // ── TX @ 25 Hz ───────────────────────────────────────────
    if (now - lastTx >= TX_INTERVAL_MS) {
        lastTx = now;
        RCPacket tx = pkt;
        tx.steering = steerFinal;
        tx.crc      = calcCRC(tx);
        if (espNowOK) esp_now_send(ABYSS_MAC, (uint8_t *)&tx, sizeof(tx));
    }

    // ── OLED @ 10 Hz ─────────────────────────────────────────
    if (now - lastDisp >= 100) {
        lastDisp = now;
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);

        // Title bar
        display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(14, 1);
        display.setTextSize(1);
        display.print("VOID RC CONTROLLER");
        display.setTextColor(SSD1306_WHITE);

        // STR / THR bars
        display.setCursor(0, 12); display.print("STR");
        drawBar(steerFinal, 12);
        display.setCursor(0, 22); display.print("THR");
        drawBar(pkt.throttle, 22);

        // Trim + lights + link dot
        display.setCursor(0, 33);
        display.printf("T%+d F:%s R:%s",
            pkt.trim,
            pkt.led1 ? "on" : "--",
            pkt.led2 ? "on" : "--");

        bool isBroadcast = (ABYSS_MAC[0] == 0xFF && ABYSS_MAC[1] == 0xFF);
        if (isBroadcast) {
            display.setCursor(110, 33); display.print("BC");
        } else if (txAck) {
            display.fillCircle(123, 36, 4, SSD1306_WHITE);
        } else {
            display.drawCircle(123, 36, 4, SSD1306_WHITE);
        }

        // Speed row
        display.setCursor(0, 44);
        if (dispKph == 0.0f && dispKphMax == 0.0f && dispVbat == 0.0f) {
            display.print("-- km/h  MAX --");
        } else {
            display.printf("%.1f km/h MAX%.1f", dispKph, dispKphMax);
        }

        // Bottom row: CAL bar while holding SW1, else battery + fan
        display.setCursor(0, 55);
        if (sw1CalArmed) {
            uint32_t held = now - sw1HoldStart;
            uint8_t  barW = (uint8_t)(min(held, (uint32_t)CAL_HOLD_MS) * 76 / CAL_HOLD_MS);
            display.print("CAL");
            display.drawRect(26, 56, 78, 6, SSD1306_WHITE);
            if (barW > 0) display.fillRect(27, 57, barW, 4, SSD1306_WHITE);
        } else {
            display.printf("BAT %.2fV  FAN:%s",
                dispVbat,
                pkt.fan ? "on" : "--");
        }

        display.display();
    }
}
