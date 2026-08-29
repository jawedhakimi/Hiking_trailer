#include "vesc_can.h"
#include <string.h>
#include <Arduino.h>
#include <esp_err.h>
#include "config.h"

namespace VescCan {

namespace {

Status1 g_status1;
StatusExtra g_statusExtra;

// Reused TX message buffer: the structural parameters (extd/rtr — VESC CAN
// commands are always 29-bit extended, never remote-frame) are set once in
// begin() and never touched again. Each transmit() call only updates what
// actually changes per message: identifier (packet id + controller id),
// data_length_code, and the payload bytes. Avoids re-zeroing/rebuilding the
// whole struct on the stack every single send (20Hz by default).
twai_message_t g_txMsg = {};

CanLogEntry g_log[CAN_LOG_RING_SIZE];
uint32_t g_logHead = 0;  // next write index
uint32_t g_logTotal = 0; // total entries ever written (== next seq)

void pushLog(uint32_t identifier, uint8_t dlc, const uint8_t *data, bool tx, bool ok) {
    CanLogEntry &e = g_log[g_logHead];
    e.seq = g_logTotal;
    e.millisAt = millis();
    e.identifier = identifier;
    e.dlc = dlc > 8 ? 8 : dlc;
    memset(e.data, 0, sizeof(e.data));
    memcpy(e.data, data, e.dlc);
    e.tx = tx;
    e.ok = ok;
    g_logHead = (g_logHead + 1) % CAN_LOG_RING_SIZE;
    g_logTotal++;
}

// VESC firmware's buffer_append_int32() writes big-endian (MSB first).
void appendInt32BE(uint8_t *buf, int32_t value) {
    buf[0] = (uint8_t)(((uint32_t)value >> 24) & 0xFF);
    buf[1] = (uint8_t)(((uint32_t)value >> 16) & 0xFF);
    buf[2] = (uint8_t)(((uint32_t)value >> 8) & 0xFF);
    buf[3] = (uint8_t)((uint32_t)value & 0xFF);
}

int32_t readInt32BE(const uint8_t *buf) {
    return (int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                      ((uint32_t)buf[2] << 8) | (uint32_t)buf[3]);
}

int16_t readInt16BE(const uint8_t *buf) {
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

bool transmit(uint8_t controllerId, CanPacketId packetId, const uint8_t *data, uint8_t len) {
    // g_txMsg's extd/rtr were set once in begin() and are left alone here —
    // only the identifier and payload (the actual CAN message) change per
    // send.
    g_txMsg.identifier = ((uint32_t)packetId << 8) | controllerId;
    g_txMsg.data_length_code = len;
    memcpy(g_txMsg.data, data, len);

    esp_err_t err = twai_transmit(&g_txMsg, pdMS_TO_TICKS(10));
    bool ok = (err == ESP_OK);
    pushLog(g_txMsg.identifier, len, data, /*tx=*/true, ok);

#if DEBUG_MODE == DEBUG_MODE_CAN
    Serial.printf("CAN TX id=0x%08lX dlc=%u data=[%02X %02X %02X %02X] %s\n",
                   (unsigned long)g_txMsg.identifier, g_txMsg.data_length_code,
                   len > 0 ? data[0] : 0, len > 1 ? data[1] : 0,
                   len > 2 ? data[2] : 0, len > 3 ? data[3] : 0,
                   ok ? "OK" : esp_err_to_name(err));
#endif

    return ok;
}

} // namespace

void begin() {
    // Set the TX message's structural parameters exactly once, here —
    // every subsequent transmit() call reuses g_txMsg and only updates the
    // identifier/data_length_code/data fields that actually vary per send.
    g_txMsg.extd = 1; // VESC CAN commands use 29-bit extended IDs
    g_txMsg.rtr = 0;  // never a remote-frame request
}

bool sendSetRpm(uint8_t controllerId, int32_t erpm) {
    uint8_t data[4];
    appendInt32BE(data, erpm);
    return transmit(controllerId, CAN_PACKET_SET_RPM, data, sizeof(data));
}

bool sendSetCurrent(uint8_t controllerId, float currentAmps) {
    // Scaling factor 1000 per VESC comm_can protocol (milliamps as int32).
    int32_t scaled = (int32_t)(currentAmps * 1000.0f);
    uint8_t data[4];
    appendInt32BE(data, scaled);
    return transmit(controllerId, CAN_PACKET_SET_CURRENT, data, sizeof(data));
}

bool sendSetCurrentBrake(uint8_t controllerId, float currentAmps) {
    if (currentAmps < 0.0f) currentAmps = -currentAmps;
    int32_t scaled = (int32_t)(currentAmps * 1000.0f);
    uint8_t data[4];
    appendInt32BE(data, scaled);
    return transmit(controllerId, CAN_PACKET_SET_CURRENT_BRAKE, data, sizeof(data));
}

bool sendSetCurrentHandbrake(uint8_t controllerId, float currentAmps) {
    if (currentAmps < 0.0f) currentAmps = -currentAmps;
    int32_t scaled = (int32_t)(currentAmps * 1000.0f);
    uint8_t data[4];
    appendInt32BE(data, scaled);
    return transmit(controllerId, CAN_PACKET_SET_CURRENT_HANDBRAKE, data, sizeof(data));
}

bool sendSetDuty(uint8_t controllerId, float duty) {
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    // Scaling factor 100000 per VESC comm_can protocol.
    int32_t scaled = (int32_t)(duty * 100000.0f);
    uint8_t data[4];
    appendInt32BE(data, scaled);
    return transmit(controllerId, CAN_PACKET_SET_DUTY, data, sizeof(data));
}

void poll(uint8_t controllerId) {
    twai_message_t msg;
    // Non-blocking: drain everything currently queued, since several other
    // packet types (status 2/3/4/5, pings, etc.) may also be on the bus and
    // would otherwise back up the RX queue.
    while (twai_receive(&msg, 0) == ESP_OK) {
#if DEBUG_MODE == DEBUG_MODE_CAN
        Serial.printf(
            "CAN RX id=0x%08lX ext=%d dlc=%u data=[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
            (unsigned long)msg.identifier, msg.extd, msg.data_length_code,
            msg.data[0], msg.data[1], msg.data[2], msg.data[3],
            msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
#endif

        // Log every frame we see, regardless of whether it's one we
        // understand/care about below — that's what makes the web CAN
        // Monitor tab useful for debugging things this firmware doesn't
        // otherwise decode.
        pushLog(msg.identifier, msg.data_length_code, msg.data, /*tx=*/false, /*ok=*/true);

        if (!msg.extd || msg.rtr) continue;

        uint8_t packetId = (uint8_t)((msg.identifier >> 8) & 0xFF);
        uint8_t rxControllerId = (uint8_t)(msg.identifier & 0xFF);
        if (rxControllerId != controllerId) continue;

        switch (packetId) {
            case CAN_PACKET_STATUS:
                if (msg.data_length_code < 6) break;
                g_status1.erpm = readInt32BE(&msg.data[0]);
                g_status1.currentA = readInt16BE(&msg.data[4]) / 10.0f;
                if (msg.data_length_code >= 8) {
                    g_status1.dutyCycle = readInt16BE(&msg.data[6]) / 1000.0f;
                }
                g_status1.lastRxMillis = millis();
                g_status1.everReceived = true;
                break;

            case CAN_PACKET_STATUS_2:
                if (msg.data_length_code < 8) break;
                g_statusExtra.ampHoursConsumed = readInt32BE(&msg.data[0]) / 10000.0f;
                g_statusExtra.ampHoursCharged = readInt32BE(&msg.data[4]) / 10000.0f;
                g_statusExtra.status2LastRxMillis = millis();
                g_statusExtra.status2EverReceived = true;
                g_statusExtra.lastRxMillis = millis();
                g_statusExtra.everReceived = true;
                break;

            case CAN_PACKET_STATUS_3:
                if (msg.data_length_code < 8) break;
                g_statusExtra.wattHoursConsumed = readInt32BE(&msg.data[0]) / 10000.0f;
                g_statusExtra.wattHoursCharged = readInt32BE(&msg.data[4]) / 10000.0f;
                g_statusExtra.status3LastRxMillis = millis();
                g_statusExtra.status3EverReceived = true;
                g_statusExtra.lastRxMillis = millis();
                g_statusExtra.everReceived = true;
                break;

            case CAN_PACKET_STATUS_4:
                if (msg.data_length_code < 6) break;
                g_statusExtra.fetTempC = readInt16BE(&msg.data[0]) / 10.0f;
                g_statusExtra.motorTempC = readInt16BE(&msg.data[2]) / 10.0f;
                g_statusExtra.currentInA = readInt16BE(&msg.data[4]) / 10.0f;
                if (msg.data_length_code >= 8) {
                    g_statusExtra.pidPositionDeg = readInt16BE(&msg.data[6]) / 50.0f;
                }
                g_statusExtra.status4LastRxMillis = millis();
                g_statusExtra.status4EverReceived = true;
                g_statusExtra.lastRxMillis = millis();
                g_statusExtra.everReceived = true;
                break;

            case CAN_PACKET_STATUS_5:
                if (msg.data_length_code < 6) break;
                g_statusExtra.tachometerRaw = readInt32BE(&msg.data[0]);
                g_statusExtra.tachometerEverReceived = true;
                g_statusExtra.inputVoltageV = readInt16BE(&msg.data[4]) / 10.0f;
                g_statusExtra.status5LastRxMillis = millis();
                g_statusExtra.status5EverReceived = true;
                g_statusExtra.lastRxMillis = millis();
                g_statusExtra.everReceived = true;
                break;

            case CAN_PACKET_STATUS_6:
                if (msg.data_length_code < 8) break;
                g_statusExtra.adc1V = readInt16BE(&msg.data[0]) / 1000.0f;
                g_statusExtra.adc2V = readInt16BE(&msg.data[2]) / 1000.0f;
                g_statusExtra.adc3V = readInt16BE(&msg.data[4]) / 1000.0f;
                g_statusExtra.ppm = readInt16BE(&msg.data[6]) / 1000.0f;
                g_statusExtra.status6LastRxMillis = millis();
                g_statusExtra.status6EverReceived = true;
                g_statusExtra.lastRxMillis = millis();
                g_statusExtra.everReceived = true;
                break;

            default:
                break; // Unknown frames remain available as raw bytes in the CAN monitor.
        }
    }
}

const Status1 &getStatus1() { return g_status1; }
const StatusExtra &getStatusExtra() { return g_statusExtra; }

size_t getLogEntriesAfter(uint32_t afterSeq, CanLogEntry *out, size_t maxCount) {
    uint32_t available = g_logTotal < CAN_LOG_RING_SIZE ? g_logTotal : CAN_LOG_RING_SIZE;
    uint32_t oldestSeq = g_logTotal - available;
    uint32_t startSeq = afterSeq + 1;
    if (afterSeq == 0 || startSeq < oldestSeq) startSeq = oldestSeq;

    size_t n = 0;
    for (uint32_t s = startSeq; s < g_logTotal && n < maxCount; s++) {
        out[n++] = g_log[s % CAN_LOG_RING_SIZE];
    }
    return n;
}

uint32_t getLogLatestSeq() { return g_logTotal == 0 ? 0 : g_logTotal - 1; }

} // namespace VescCan
