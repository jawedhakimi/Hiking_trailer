#pragma once
//
// vesc_can.h — minimal helper for sending VESC "set value" commands over
// CAN using the ESP-IDF TWAI driver (talking to the bus through a TJA1051
// transceiver), decoding the VESC's periodic status broadcasts, and
// keeping a small ring-buffer log of every frame sent/seen for the web
// UI's CAN Monitor tab. See
// https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
// for the full protocol if you want to extend this further.
//

#include <stdint.h>
#include "driver/twai.h"

namespace VescCan {

// CAN_PACKET_ID values from the VESC firmware (datatypes.h), extended ID =
// (packet_id << 8) | controller_id.
enum CanPacketId : uint8_t {
    CAN_PACKET_SET_DUTY          = 0,
    CAN_PACKET_SET_CURRENT       = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM           = 3,
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
    CAN_PACKET_STATUS            = 9,   // ERPM / current / duty
    CAN_PACKET_STATUS_2          = 14,  // amp-hours consumed/charged
    CAN_PACKET_STATUS_3          = 15,  // watt-hours consumed/charged
    CAN_PACKET_STATUS_4          = 16,  // FET/motor temp, input current, PID pos
    CAN_PACKET_STATUS_5          = 27,  // tachometer, input voltage
    CAN_PACKET_STATUS_6          = 58,  // ADC1, ADC2, ADC3, PPM
};

// Most recent decoded "Status Message 1" from the VESC (actual measured
// values, as opposed to what we last commanded).
struct Status1 {
    int32_t erpm = 0;          // measured electrical RPM
    float currentA = 0.0f;     // measured motor current, amps
    float dutyCycle = 0.0f;    // measured duty cycle, -1.0 .. 1.0
    uint32_t lastRxMillis = 0; // millis() timestamp of the last update
    bool everReceived = false;
};

// Everything else useful that trickles in from Status 2/3/4/5/6, if you've
// enabled those broadcasts in VESC Tool (App Settings -> General -> CAN
// Status Message). Any message you haven't enabled just leaves its fields
// at their last-known (or default) value — check the per-status timestamp if
// you need to know freshness; the simple *_TIMEOUT_MS staleness check in
// main.cpp only applies to Status1/ERPM.
struct StatusExtra {
    float fetTempC = 0.0f;
    float motorTempC = 0.0f;
    float currentInA = 0.0f;
    float inputVoltageV = 0.0f;
    float ampHoursConsumed = 0.0f;
    float ampHoursCharged = 0.0f;
    float wattHoursConsumed = 0.0f;
    float wattHoursCharged = 0.0f;
    float pidPositionDeg = 0.0f;
    float adc1V = 0.0f;
    float adc2V = 0.0f;
    float adc3V = 0.0f;
    float ppm = 0.0f;
    // Raw tachometer, in "electrical revolutions x6" steps (VESC's native
    // unit) — divide by 6 for electrical revs, then by pole pairs and gear
    // ratio for wheel revs. Monotonic-ish odometer value (not reset by us
    // unless commanded), used for trip distance instead of integrating
    // instantaneous speed.
    int32_t tachometerRaw = 0;
    bool tachometerEverReceived = false;
    uint32_t status2LastRxMillis = 0;
    uint32_t status3LastRxMillis = 0;
    uint32_t status4LastRxMillis = 0;
    uint32_t status5LastRxMillis = 0;
    uint32_t status6LastRxMillis = 0;
    bool status2EverReceived = false;
    bool status3EverReceived = false;
    bool status4EverReceived = false;
    bool status5EverReceived = false;
    bool status6EverReceived = false;
    uint32_t lastRxMillis = 0;
    bool everReceived = false;
};

// One entry in the CAN Monitor ring buffer.
struct CanLogEntry {
    uint32_t seq = 0;
    uint32_t millisAt = 0;
    uint32_t identifier = 0;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
    bool tx = false; // true = we sent it, false = we received it
    bool ok = true;  // for tx entries: whether twai_transmit() succeeded
};

// Must be called once after twai_driver_install()/twai_start() succeed.
void begin();

// Sends CAN_PACKET_SET_RPM: commands electrical RPM (ERPM) directly.
// Returns true if the frame was handed to the TWAI driver successfully.
bool sendSetRpm(uint8_t controllerId, int32_t erpm);

// Sends CAN_PACKET_SET_CURRENT: commands motor current in amps.
bool sendSetCurrent(uint8_t controllerId, float currentAmps);

// Sends CAN_PACKET_SET_CURRENT_BRAKE: commands a braking current in amps
// (magnitude only — negative values are treated as their absolute value,
// since brake current isn't directional the way drive current is; the VESC
// applies it to resist whatever direction the motor is currently spinning).
bool sendSetCurrentBrake(uint8_t controllerId, float currentAmps);

// Holds the motor's present position using the VESC handbrake controller.
// Current is a positive magnitude in amps and should be set conservatively
// because it can produce continuous motor/VESC heating while parked.
bool sendSetCurrentHandbrake(uint8_t controllerId, float currentAmps);

// Sends CAN_PACKET_SET_DUTY: commands duty cycle, range -1.0 .. 1.0.
bool sendSetDuty(uint8_t controllerId, float duty);

// Non-blocking: drains every CAN frame currently sitting in the TWAI RX
// queue, updates Status1/StatusExtra for controllerId, and logs every
// frame (any ID, any controller) into the CAN Monitor ring buffer. Call
// this once per loop().
void poll(uint8_t controllerId);

// Most recently decoded status (call poll() first each loop to keep it
// current). everReceived is false until the first relevant frame arrives.
const Status1 &getStatus1();
const StatusExtra &getStatusExtra();

// --- CAN Monitor ring buffer ----------------------------------------------
// Copies up to maxCount entries with seq > afterSeq into out (oldest
// first), returning how many were written. Pass afterSeq=0 the first time
// to get "whatever's currently in the buffer".  If the caller has fallen
// behind far enough that afterSeq predates everything still in the ring,
// this silently skips ahead to the oldest available entry instead of
// erroring.
size_t getLogEntriesAfter(uint32_t afterSeq, CanLogEntry *out, size_t maxCount);
uint32_t getLogLatestSeq();

} // namespace VescCan
