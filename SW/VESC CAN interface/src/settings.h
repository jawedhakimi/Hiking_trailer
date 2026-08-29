#pragma once
//
// settings.h — every user-tunable value that used to be a compile-time
// #define in config.h and now needs to be readable/writable from the web
// UI at runtime. Backed by NVS (Preferences) so changes survive reboots.
//
// config.h still supplies the *first-boot defaults* (and the pins, which
// aren't runtime-tunable) — this module is only the mutable layer on top.
//
// Pot min/max calibration is intentionally NOT in here: it already has its
// own working NVS-backed flow in potcal.h/.cpp (buzzer-guided at boot, or
// captured live from the web UI) and there was no reason to disturb that.
//

#include <Arduino.h>
#include <ArduinoJson.h>

struct Settings {
    // --- Pot -> VESC command mapping ----------------------------------------
    float deadbandPercent;        // POT_DEADBAND_PERCENT default
    uint16_t potFilterSamples;     // moving-average window length
    uint8_t controlMode;            // VESC_CONTROL_MODE_RPM or _CURRENT
    int32_t maxErpm;               // VESC_MAX_ERPM default (RPM mode, FORWARD)
    float maxCurrentA;             // VESC_MAX_CURRENT_A default (Current mode, FORWARD)
    uint32_t canSendIntervalMs;    // CAN_SEND_INTERVAL_MS default

    // --- Speed limit unit: ERPM (maxErpm/maxErpmBackward above) or km/h ---
    uint8_t speedLimitUnit;         // SPEED_LIMIT_UNIT_ERPM or _KMH
    float maxSpeedKmh;              // forward, converted to ERPM at runtime when active
    float maxSpeedKmhBackward;      // backward, converted to ERPM at runtime when active

    // --- Forward/backward asymmetric limits + reverse disable --------------
    int32_t maxErpmBackward;        // RPM mode
    float maxCurrentBackwardA;      // Current mode
    bool reverseEnabled;            // false = zero torque in reverse, always
    bool invertMotorDirection;      // flips VESC command and feedback signs

    // --- Downhill braking (SET_CURRENT_BRAKE when overspeeding) ------------
    bool downhillBrakingEnabled;
    float brakeCurrentA;
    int32_t brakeEngageErpmMargin;
    float parkCurrentA;             // handbrake holding current while parked

    // --- VESC / CAN bus (changing either REQUIRES A RESTART to take effect)
    uint8_t vescControllerId;
    uint32_t canBaudrateBps;

    // --- Speed / distance conversion ----------------------------------------
    uint16_t motorPolePairs;
    float wheelDiameterMm;
    float gearRatio;

    // --- Battery ---------------------------------------------------------
    uint8_t batteryVoltageSource; // ESP32 divider or VESC Status 5
    float vbatCalibrationScale;

    // --- Closed-loop speed PID (trim on top of pot feedforward) ----------
    bool speedPidEnabled;
    float pidKp;
    float pidKi;
    float pidKd;
    int32_t pidMaxTrimErpm;

    // --- IMU: fall detection ----------------------------------------------
    float fallAngleThresholdDeg;
    float fallRecoverMarginDeg;
    uint32_t fallConfirmMs;
    uint32_t fallRecoverStableMs;
    bool impactDetectEnabled;
    float impactAccelThresholdG;

    // --- Fall-proximity warning buzzer --------------------------------------
    bool fallWarningBuzzerEnabled;
    float fallWarningStartPercent;

    // --- Post-fall resume warning buzzer (gating itself is not optional —
    // see config.h's RESUME_WARNING_* notes) --------------------------------
    bool resumeWarningEnabled;

    // --- IMU: compass -------------------------------------------------------
    float magDeclinationDeg;

    // --- WiFi Access Point (changing REQUIRES A RESTART to take effect) ---
    char wifiSsid[33];
    char wifiPassword[65];

    void setDefaults();

    // Loads every key from NVS, leaving setDefaults() values in place for
    // any key that isn't present yet (e.g. first boot, or a field added by
    // a firmware update since the last save).
    void load();

    // Writes every field to NVS.
    void save();

    // Serializes the whole struct into a JSON object for the web UI.
    void toJson(JsonObject obj) const;

    // Applies any fields present in obj (partial update — fields not
    // present are left untouched), clamping to sane ranges. Returns true
    // if a field that needs a restart to take effect was changed.
    //
    // Changing the WiFi password additionally requires a correct
    // "wifiPasswordCurrent" field matching the password actually stored
    // right now — a new "wifiPassword" without it (or with a wrong one) is
    // silently NOT applied. If passwordRejected is non-null, it's set to
    // true in that case so the caller (the web server) can report it back
    // to the user instead of the change just quietly not happening.
    bool fromJson(JsonObjectConst obj, bool *passwordRejected = nullptr);
};

extern Settings g_settings;
