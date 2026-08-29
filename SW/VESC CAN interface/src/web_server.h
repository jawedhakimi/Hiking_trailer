#pragma once
//
// web_server.h — hosts the configuration/telemetry web UI: WiFi Access
// Point, LittleFS-served single-page dashboard, REST API for settings and
// calibration actions, and a WebSocket pushing live telemetry + a CAN
// frame log to any connected browser.
//
// main.cpp owns the actual control loop (pot read, CAN send, IMU update,
// fall-safety cutoff); this module only reports what's happening and lets
// the browser change settings / trigger calibration actions. It talks
// directly to settings.h, potcal.h, imu.h, vesc_can.h and odometry.h for
// that, rather than routing everything back through main.cpp.
//

#include <stdint.h>

namespace WebServerApp {

// One frame of "what's happening right now", filled in by main.cpp every
// loop() and handed over with setTelemetry() — this module only formats
// and broadcasts it (at WEB_TELEMETRY_INTERVAL_MS, not every call).
struct TelemetrySnapshot {
    float potPositionPercent = 0.0f;
    int32_t potRaw = 0;         // moving-average value used for command mapping
    int32_t potRawInstant = 0;  // single unfiltered ADC sample, for comparing against potRaw while tuning
    uint8_t controlMode = 0; // VESC_CONTROL_MODE_RPM (0) or _CURRENT (1)
    int32_t targetErpm = 0;      // meaningful in RPM mode
    float targetCurrentA = 0.0f; // meaningful in Current mode
    int32_t actualErpm = 0;
    bool vescStatusFresh = false;
    float speedMps = 0.0f;
    bool odometryAvailable = false;
    bool odometryUsingTachometer = false;
    float vBatt = 0.0f;
    float vBattEsp32 = 0.0f;
    float vBattVesc = 0.0f;
    uint8_t batteryVoltageSource = 0;
    bool batteryVoltageFresh = false;
    float motorCurrentA = 0.0f;
    float dutyCycle = 0.0f;
    float fetTempC = 0.0f;
    float motorTempC = 0.0f;
    float inputCurrentA = 0.0f;
    float ampHoursConsumed = 0.0f;
    float ampHoursCharged = 0.0f;
    float wattHoursConsumed = 0.0f;
    float wattHoursCharged = 0.0f;
    float pidPositionDeg = 0.0f;
    int32_t tachometerRaw = 0;
    float adc1V = 0.0f;
    float adc2V = 0.0f;
    float adc3V = 0.0f;
    float ppm = 0.0f;
    bool status2Fresh = false;
    bool status3Fresh = false;
    bool status4Fresh = false;
    bool status5Fresh = false;
    bool status6Fresh = false;
    bool status1Ever = false;
    bool status2Ever = false;
    bool status3Ever = false;
    bool status4Ever = false;
    bool status5Ever = false;
    bool status6Ever = false;
    uint32_t status1AgeMs = 0;
    uint32_t status2AgeMs = 0;
    uint32_t status3AgeMs = 0;
    uint32_t status4AgeMs = 0;
    uint32_t status5AgeMs = 0;
    uint32_t status6AgeMs = 0;

    bool pidActive = false;       // speed PID enabled AND actually trimming (not just the setting)
    int32_t pidTrimErpm = 0;

    bool brakingActive = false;   // downhill braking is currently engaged (SET_CURRENT_BRAKE being sent)
    bool parkEnabled = false;     // dashboard Park request is latched in firmware
    bool parkHolding = false;     // handbrake command is currently being transmitted
    bool shaftPowerEnabled = false; // runtime permission; always false after boot

    // Safety resume gating. The postFallLockout wire name is retained for UI
    // compatibility, but it also covers calibration and direction changes.
    bool postFallLockout = false;
    bool resumeWarnActive = false;
    bool calibrationReady = false; // false means no motor CAN frames are transmitted
    bool calibrationActive = false;
    bool calibrationWaitingForCenter = false;
    bool directionChangeLockout = false;

    const char *canState = "?";
    uint32_t canTxErrorCount = 0;
    uint32_t canRxErrorCount = 0;
};

// Starts WiFi AP + mDNS + LittleFS + the HTTP/WebSocket server. Returns
// false only on a hard failure (e.g. LittleFS won't mount) — WiFi AP
// failures are logged but non-fatal since the rest of the controller
// should keep working without the web UI.
bool begin();

// Call every loop() iteration. Cheap when there's nothing to send yet;
// internally rate-limits telemetry/CAN-log broadcasts and services any
// pending restart request.
void loop();

void setTelemetry(const TelemetrySnapshot &snap);

// Live Dashboard Park state. It is intentionally runtime-only (starts off
// after reboot), but remains latched if the browser disconnects.
bool isParkEnabled();
bool isShaftPowerEnabled();

} // namespace WebServerApp
