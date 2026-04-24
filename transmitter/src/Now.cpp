#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "Globals.h"
#include "Now.h"

static uint8_t receiverMac[6] = {0x94, 0xA9, 0x90, 0x6B, 0x77, 0x14};
static esp_now_peer_info_t peerInfo = {};
static bool nowReady = false;

typedef struct __attribute__((packed)) {
    float   kph;
    float   kphMax;
    float   vbat;
    uint8_t crc;
} LegacyTelPacket;

static portMUX_TYPE teleMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool telePending = false;
static TelemetryPacket telePendingPacket = {0};

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onSent(const wifi_tx_info_t* txInfo, esp_now_send_status_t status) {
    (void)txInfo;
    (void)status;
}
#else
static void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    (void)mac;
    (void)status;
}
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onReceive(const esp_now_recv_info_t* recvInfo, const uint8_t* data, int len) {
    (void)recvInfo;
#else
static void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
#endif
    TelemetryPacket in = {};

    if (len == (int)sizeof(LegacyTelPacket)) {
        LegacyTelPacket legacy = {};
        memcpy(&legacy, data, sizeof(LegacyTelPacket));
        if (packetCRC(legacy) != legacy.crc) return;

        in.version = RC_PROTOCOL_VERSION;
        in.kph = legacy.kph;
        in.kphMax = legacy.kphMax;
        in.vbat = legacy.vbat;
        in.speedKmhX100 = (uint16_t)constrain((int)(legacy.kph * 100.0f), 0, 65535);
        in.battmV = (uint16_t)constrain((int)(legacy.vbat * 1000.0f), 0, 65535);
        float pct = ((legacy.vbat - 6.6f) / (8.4f - 6.6f)) * 100.0f;
        in.battPct = (uint8_t)constrain((int)pct, 0, 100);
        in.crc = packetCRC(in);
    } else if (len == (int)sizeof(TelemetryPacket)) {
        memcpy(&in, data, sizeof(TelemetryPacket));
        if (in.version != RC_PROTOCOL_VERSION) return;
        if (packetCRC(in) != in.crc) return;
    } else {
        return;
    }

    portENTER_CRITICAL_ISR(&teleMux);
    memcpy(&telePendingPacket, &in, sizeof(TelemetryPacket));
    telePending = true;
    portEXIT_CRITICAL_ISR(&teleMux);
}

static bool configurePeer() {
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(receiverMac)) esp_now_del_peer(receiverMac);
    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

bool nowInit() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_protocol(
        WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR
    );
    esp_wifi_set_max_tx_power(84);

    if (esp_now_init() != ESP_OK) return false;

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);
    nowReady = configurePeer();
    return nowReady;
}

void nowProcessTelemetry() {
    if (!telePending) return;

    TelemetryPacket pkt = {};
    portENTER_CRITICAL(&teleMux);
    memcpy(&pkt, &telePendingPacket, sizeof(TelemetryPacket));
    telePending = false;
    portEXIT_CRITICAL(&teleMux);

    telePacketRx = pkt;
    tele.speedKmh = (pkt.speedKmhX100 > 0) ? ((float)pkt.speedKmhX100 / 100.0f) : pkt.kph;
    tele.battmV = (pkt.battmV > 0) ? pkt.battmV : (uint16_t)constrain((int)(pkt.vbat * 1000.0f), 0, 65535);
    tele.rpm = pkt.rpm;

    uint8_t battPct = pkt.battPct;
    if (battPct == 0 && pkt.vbat > 0.0f) {
        float pct = ((pkt.vbat - 6.6f) / (8.4f - 6.6f)) * 100.0f;
        battPct = (uint8_t)constrain((int)pct, 0, 100);
    }

    tele.battPct = battPct;
    tele.frontLightState = pkt.frontLightState;
    tele.rearLightState = pkt.rearLightState;
    tele.fanPctState = pkt.fanPctState;
    tele.lastSeq = pkt.seq;
    tele.connected = true;
    tele.lastRx = millis();
}

void nowApplyTelemetryTimeout(uint32_t timeoutMs) {
    if (!tele.connected) return;
    if (millis() - tele.lastRx <= timeoutMs) return;

    tele.connected = false;
    tele.speedKmh = 0;
    tele.rpm = 0;
}

bool nowSendPacket(const uint8_t* data, size_t len) {
    if (!nowReady) return false;
    if (len == 0 || data == nullptr) return false;
    return esp_now_send(receiverMac, data, len) == ESP_OK;
}

void nowSetReceiverMac(const uint8_t mac[6]) {
    if (!mac) return;
    memcpy(receiverMac, mac, 6);
    if (nowReady) configurePeer();
}

const uint8_t* nowGetReceiverMac() {
    return receiverMac;
}
