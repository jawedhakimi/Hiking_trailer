/*
 * ESP32-S3 (N16R8) + TJA1051 CAN transceiver — hardware bring-up test
 * ---------------------------------------------------------------------
 * Wiring assumed:
 *   ESP32-S3 GPIO17 (TX) -> TJA1051 TXD
 *   ESP32-S3 GPIO16 (RX) <- TJA1051 RXD
 *   TJA1051 CANH/CANL -> CAN bus (with 120R termination resistors at
 *   BOTH physical ends of the bus if you have a real multi-drop bus)
 *
 * What this sketch does:
 *   - Brings up the ESP32's built-in TWAI (CAN) controller on the pins above.
 *   - Every 500 ms it transmits a dummy CAN frame (ID 0x123, 8 bytes,
 *     counting pattern) so you can probe CANH/CANL with a scope/analyzer
 *     or see it received on another CAN node.
 *   - Continuously polls for received frames and prints them to Serial.
 *   - Prints TWAI driver alerts (bus errors, TX failures, etc.) so you
 *     can immediately see wiring / bit-rate / termination problems.
 *
 * You can disable either half (TX or RX) below if you only want to test
 * one direction at a time.
 *
 * IMPORTANT — single-node testing:
 *   A real CAN bus requires another node to ACK every frame, otherwise
 *   the transmitter sees an error and keeps retrying (you'll see
 *   "[ALERT] TX failed" spam). If you are testing this board alone
 *   (nothing else on the bus yet), leave TWAI_TEST_MODE set to
 *   TWAI_MODE_NO_ACK below. Once you connect a real second node
 *   (e.g. your VESC), switch it to TWAI_MODE_NORMAL.
 */

#include <Arduino.h>
#include "driver/twai.h"

// ----------------------------- User config -----------------------------

#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_18

// Set to 1/0 to enable or disable each half of the test
#define ENABLE_TX 1
#define ENABLE_RX 0

// TWAI_MODE_NO_ACK  -> use for solo bring-up (no other node on the bus)
// TWAI_MODE_NORMAL  -> use once a real second CAN node is present
#define TWAI_TEST_MODE TWAI_MODE_NORMAL

// Bus bit rate in kbit/s. VESC's default CAN bit rate is 500 kbit/s.
static const uint32_t CAN_BITRATE_KBPS = 500;

static const uint32_t TX_INTERVAL_MS = 500;

// -------------------------------------------------------------------------

static unsigned long lastTxTime = 0;
static uint8_t txCounter = 0;

static twai_timing_config_t getTimingConfig(uint32_t kbps) {
    switch (kbps) {
        case 1000: return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        case 800:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        case 500:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        case 250:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        case 125:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        case 100:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        case 50:   return (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
        case 25:   return (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
        default:   return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    }
}

static void printAlerts() {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts != 0) {
        if (alerts & TWAI_ALERT_TX_FAILED)   Serial.println("[ALERT] TX failed (no ACK / bus error) - is anything else on the bus? termination resistors present?");
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) Serial.println("[ALERT] RX queue full - messages are arriving faster than they're being read");
        if (alerts & TWAI_ALERT_ERR_PASS)    Serial.println("[ALERT] Controller entered ERROR PASSIVE state - check wiring/bit-rate/termination");
        if (alerts & TWAI_ALERT_BUS_OFF)     Serial.println("[ALERT] Bus-off! Too many errors. Check CANH/CANL wiring, termination, and bit rate.");
        if (alerts & TWAI_ALERT_BUS_ERROR)   Serial.println("[ALERT] Bus error detected (bit/stuff/CRC/form/ACK error)");
    }
}

void setup() {
    Serial.begin(115200);
    // On native-USB boards the CDC port takes a moment to enumerate after
    // reset; wait (with a timeout) so early prints aren't lost if the
    // Serial Monitor is opened right after flashing.
    unsigned long serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < 3000) {
        delay(10);
    }
    delay(200);

    Serial.println();
    Serial.println("=== ESP32-S3 TWAI/CAN Hardware Test (TJA1051) ===");
    Serial.printf("TX pin: GPIO%d, RX pin: GPIO%d, bit rate: %lu kbit/s\n",
                   (int)CAN_TX_PIN, (int)CAN_RX_PIN, (unsigned long)CAN_BITRATE_KBPS);
    Serial.printf("TX test: %s, RX test: %s, mode: %s\n",
                   ENABLE_TX ? "ON" : "OFF",
                   ENABLE_RX ? "ON" : "OFF",
                   (TWAI_TEST_MODE == TWAI_MODE_NO_ACK) ? "NO_ACK (solo bring-up)" : "NORMAL");

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_TEST_MODE);
    twai_timing_config_t t_config = getTimingConfig(CAN_BITRATE_KBPS);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("FATAL: Failed to install TWAI driver. Halting.");
        while (1) delay(1000);
    }
    Serial.println("TWAI driver installed OK.");

    if (twai_start() != ESP_OK) {
        Serial.println("FATAL: Failed to start TWAI driver. Halting.");
        while (1) delay(1000);
    }
    Serial.println("TWAI driver started OK.");

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED |
                                 TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS |
                                 TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR;
    twai_reconfigure_alerts(alerts_to_enable, NULL);

    Serial.println("Setup complete. Entering loop...\n");
}

void loop() {
#if ENABLE_TX
    if (millis() - lastTxTime >= TX_INTERVAL_MS) {
        lastTxTime = millis();

        twai_message_t message = {};
        message.identifier = 0x123;
        message.extd = 0;   // standard 11-bit ID
        message.rtr = 0;
        message.data_length_code = 8;
        for (int i = 0; i < 8; i++) {
            message.data[i] = (uint8_t)(txCounter + i);
        }
        txCounter++;

        if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
            Serial.printf("[TX] ID=0x%03X DLC=%d Data=", message.identifier, message.data_length_code);
            for (int i = 0; i < message.data_length_code; i++) {
                Serial.printf("%02X ", message.data[i]);
            }
            Serial.println();
        } else {
            Serial.println("[TX] Failed to queue message (TX queue full?)");
        }
    }
#endif

#if ENABLE_RX
    twai_message_t rxMessage;
    while (twai_receive(&rxMessage, 0) == ESP_OK) {
        Serial.printf("[RX] ID=0x%03X %s DLC=%d Data=",
                       rxMessage.identifier,
                       rxMessage.extd ? "EXT" : "STD",
                       rxMessage.data_length_code);
        if (!rxMessage.rtr) {
            for (int i = 0; i < rxMessage.data_length_code; i++) {
                Serial.printf("%02X ", rxMessage.data[i]);
            }
        } else {
            Serial.print("(RTR frame)");
        }
        Serial.println();
    }
#endif

    printAlerts();
}
