#include "settings.h"
#include <Preferences.h>
#include <string.h>
#include "config.h"

Settings g_settings;

namespace {
Preferences prefs;

// Small helpers so load() reads back exactly what setDefaults() would have
// produced when a key is missing, without repeating the default table
// twice.
float getF(const char *key, float def) { return prefs.getFloat(key, def); }
uint32_t getU(const char *key, uint32_t def) { return prefs.getUInt(key, def); }
int32_t getI(const char *key, int32_t def) { return prefs.getInt(key, def); }
bool getB(const char *key, bool def) { return prefs.getBool(key, def); }
} // namespace

void Settings::setDefaults() {
    deadbandPercent = POT_DEADBAND_PERCENT;
    potFilterSamples = POT_MOVING_AVERAGE_SAMPLES_DEFAULT;
    controlMode = VESC_CONTROL_MODE_DEFAULT;
    maxErpm = VESC_MAX_ERPM;
    maxCurrentA = VESC_MAX_CURRENT_A;
    canSendIntervalMs = CAN_SEND_INTERVAL_MS;

    speedLimitUnit = SPEED_LIMIT_UNIT_DEFAULT;
    maxSpeedKmh = VESC_MAX_SPEED_KMH_DEFAULT;
    maxSpeedKmhBackward = VESC_MAX_SPEED_KMH_BACKWARD_DEFAULT;

    maxErpmBackward = VESC_MAX_ERPM_BACKWARD_DEFAULT;
    maxCurrentBackwardA = VESC_MAX_CURRENT_BACKWARD_A_DEFAULT;
    reverseEnabled = VESC_REVERSE_ENABLED_DEFAULT;
    invertMotorDirection = VESC_INVERT_MOTOR_DIRECTION_DEFAULT;

    downhillBrakingEnabled = DOWNHILL_BRAKING_ENABLED_DEFAULT;
    brakeCurrentA = DOWNHILL_BRAKE_CURRENT_A_DEFAULT;
    brakeEngageErpmMargin = DOWNHILL_BRAKE_ENGAGE_ERPM_MARGIN_DEFAULT;
    parkCurrentA = PARK_HANDBRAKE_CURRENT_A_DEFAULT;

    vescControllerId = VESC_CONTROLLER_ID;
    canBaudrateBps = CAN_BAUDRATE_BPS;

    motorPolePairs = MOTOR_POLE_PAIRS;
    wheelDiameterMm = WHEEL_DIAMETER_MM;
    gearRatio = GEAR_RATIO;

    batteryVoltageSource = BATTERY_VOLTAGE_SOURCE_DEFAULT;
    vbatCalibrationScale = VBAT_CALIBRATION_SCALE;

    speedPidEnabled = SPEED_PID_ENABLED_DEFAULT;
    pidKp = SPEED_PID_KP_DEFAULT;
    pidKi = SPEED_PID_KI_DEFAULT;
    pidKd = SPEED_PID_KD_DEFAULT;
    pidMaxTrimErpm = SPEED_PID_MAX_TRIM_ERPM;

    fallAngleThresholdDeg = FALL_ANGLE_THRESHOLD_DEG;
    fallRecoverMarginDeg = FALL_RECOVER_MARGIN_DEG;
    fallConfirmMs = FALL_CONFIRM_MS;
    fallRecoverStableMs = FALL_RECOVER_STABLE_MS;
    impactDetectEnabled = IMPACT_DETECT_ENABLED_DEFAULT;
    impactAccelThresholdG = IMPACT_ACCEL_THRESHOLD_G;

    fallWarningBuzzerEnabled = FALL_WARNING_BUZZER_ENABLED_DEFAULT;
    fallWarningStartPercent = FALL_WARNING_START_PERCENT_DEFAULT;

    resumeWarningEnabled = RESUME_WARNING_ENABLED_DEFAULT;

    magDeclinationDeg = MAG_DECLINATION_DEG;

    strncpy(wifiSsid, WIFI_AP_SSID_DEFAULT, sizeof(wifiSsid) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = '\0';
    strncpy(wifiPassword, WIFI_AP_PASSWORD_DEFAULT, sizeof(wifiPassword) - 1);
    wifiPassword[sizeof(wifiPassword) - 1] = '\0';
}

void Settings::load() {
    setDefaults(); // seed every field first so missing keys keep sane values

    prefs.begin(SETTINGS_NVS_NAMESPACE, true); // read-only

    deadbandPercent = getF("deadband", deadbandPercent);
    potFilterSamples = (uint16_t)getI("potSamples", potFilterSamples);
    potFilterSamples = (uint16_t)constrain((int)potFilterSamples, 1,
                                           POT_MOVING_AVERAGE_SAMPLES_MAX);
    controlMode = (uint8_t)getI("ctrlMode", controlMode);
    maxErpm = getI("maxErpm", maxErpm);
    maxCurrentA = getF("maxCurA", maxCurrentA);
    canSendIntervalMs = getU("sendMs", canSendIntervalMs);

    speedLimitUnit = (uint8_t)getI("spdLimitUnit", speedLimitUnit);
    maxSpeedKmh = getF("maxSpdKmh", maxSpeedKmh);
    maxSpeedKmhBackward = getF("maxSpdKmhBk", maxSpeedKmhBackward);

    maxErpmBackward = getI("maxErpmBk", maxErpmBackward);
    maxCurrentBackwardA = getF("maxCurBkA", maxCurrentBackwardA);
    reverseEnabled = getB("revEn", reverseEnabled);
    invertMotorDirection = getB("invMotor", invertMotorDirection);

    downhillBrakingEnabled = getB("brakeEn", downhillBrakingEnabled);
    brakeCurrentA = getF("brakeA", brakeCurrentA);
    brakeEngageErpmMargin = getI("brakeMargin", brakeEngageErpmMargin);
    parkCurrentA = getF("parkA", parkCurrentA);

    vescControllerId = (uint8_t)getI("ctrlId", vescControllerId);
    canBaudrateBps = getU("canBaud", canBaudrateBps);

    motorPolePairs = (uint16_t)getI("poles", motorPolePairs);
    wheelDiameterMm = getF("wheelMm", wheelDiameterMm);
    gearRatio = getF("gearRatio", gearRatio);

    batteryVoltageSource = (uint8_t)getI("vbatSource", batteryVoltageSource);
    batteryVoltageSource = batteryVoltageSource == BATTERY_VOLTAGE_SOURCE_VESC
        ? BATTERY_VOLTAGE_SOURCE_VESC : BATTERY_VOLTAGE_SOURCE_ESP32;
    vbatCalibrationScale = getF("vbatScale", vbatCalibrationScale);

    speedPidEnabled = getB("pidEn", speedPidEnabled);
    pidKp = getF("pidKp", pidKp);
    pidKi = getF("pidKi", pidKi);
    pidKd = getF("pidKd", pidKd);
    pidMaxTrimErpm = getI("pidMaxTrim", pidMaxTrimErpm);

    fallAngleThresholdDeg = getF("fallDeg", fallAngleThresholdDeg);
    fallRecoverMarginDeg = getF("fallMargin", fallRecoverMarginDeg);
    fallConfirmMs = getU("fallConfMs", fallConfirmMs);
    fallRecoverStableMs = getU("fallRecMs", fallRecoverStableMs);
    impactDetectEnabled = getB("impactEn", impactDetectEnabled);
    impactAccelThresholdG = getF("impactG", impactAccelThresholdG);

    fallWarningBuzzerEnabled = getB("fwBuzzEn", fallWarningBuzzerEnabled);
    fallWarningStartPercent = getF("fwBuzzPct", fallWarningStartPercent);

    resumeWarningEnabled = getB("resWarnEn", resumeWarningEnabled);

    magDeclinationDeg = getF("declDeg", magDeclinationDeg);

    String ssid = prefs.getString("wifiSsid", wifiSsid);
    String pass = prefs.getString("wifiPass", wifiPassword);
    strncpy(wifiSsid, ssid.c_str(), sizeof(wifiSsid) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = '\0';
    strncpy(wifiPassword, pass.c_str(), sizeof(wifiPassword) - 1);
    wifiPassword[sizeof(wifiPassword) - 1] = '\0';

    prefs.end();
}

void Settings::save() {
    prefs.begin(SETTINGS_NVS_NAMESPACE, false);

    prefs.putFloat("deadband", deadbandPercent);
    prefs.putInt("potSamples", potFilterSamples);
    prefs.putInt("ctrlMode", controlMode);
    prefs.putInt("maxErpm", maxErpm);
    prefs.putFloat("maxCurA", maxCurrentA);
    prefs.putUInt("sendMs", canSendIntervalMs);

    prefs.putInt("spdLimitUnit", speedLimitUnit);
    prefs.putFloat("maxSpdKmh", maxSpeedKmh);
    prefs.putFloat("maxSpdKmhBk", maxSpeedKmhBackward);

    prefs.putInt("maxErpmBk", maxErpmBackward);
    prefs.putFloat("maxCurBkA", maxCurrentBackwardA);
    prefs.putBool("revEn", reverseEnabled);
    prefs.putBool("invMotor", invertMotorDirection);

    prefs.putBool("brakeEn", downhillBrakingEnabled);
    prefs.putFloat("brakeA", brakeCurrentA);
    prefs.putInt("brakeMargin", brakeEngageErpmMargin);
    prefs.putFloat("parkA", parkCurrentA);

    prefs.putInt("ctrlId", vescControllerId);
    prefs.putUInt("canBaud", canBaudrateBps);

    prefs.putInt("poles", motorPolePairs);
    prefs.putFloat("wheelMm", wheelDiameterMm);
    prefs.putFloat("gearRatio", gearRatio);

    prefs.putInt("vbatSource", batteryVoltageSource);
    prefs.putFloat("vbatScale", vbatCalibrationScale);

    prefs.putBool("pidEn", speedPidEnabled);
    prefs.putFloat("pidKp", pidKp);
    prefs.putFloat("pidKi", pidKi);
    prefs.putFloat("pidKd", pidKd);
    prefs.putInt("pidMaxTrim", pidMaxTrimErpm);

    prefs.putFloat("fallDeg", fallAngleThresholdDeg);
    prefs.putFloat("fallMargin", fallRecoverMarginDeg);
    prefs.putUInt("fallConfMs", fallConfirmMs);
    prefs.putUInt("fallRecMs", fallRecoverStableMs);
    prefs.putBool("impactEn", impactDetectEnabled);
    prefs.putFloat("impactG", impactAccelThresholdG);

    prefs.putBool("fwBuzzEn", fallWarningBuzzerEnabled);
    prefs.putFloat("fwBuzzPct", fallWarningStartPercent);

    prefs.putBool("resWarnEn", resumeWarningEnabled);

    prefs.putFloat("declDeg", magDeclinationDeg);

    prefs.putString("wifiSsid", wifiSsid);
    prefs.putString("wifiPass", wifiPassword);

    prefs.end();
}

void Settings::toJson(JsonObject obj) const {
    obj["deadbandPercent"] = deadbandPercent;
    obj["potFilterSamples"] = potFilterSamples;
    obj["controlMode"] = controlMode;
    obj["maxErpm"] = maxErpm;
    obj["maxCurrentA"] = maxCurrentA;
    obj["canSendIntervalMs"] = canSendIntervalMs;

    obj["speedLimitUnit"] = speedLimitUnit;
    obj["maxSpeedKmh"] = maxSpeedKmh;
    obj["maxSpeedKmhBackward"] = maxSpeedKmhBackward;

    obj["maxErpmBackward"] = maxErpmBackward;
    obj["maxCurrentBackwardA"] = maxCurrentBackwardA;
    obj["reverseEnabled"] = reverseEnabled;
    obj["invertMotorDirection"] = invertMotorDirection;

    obj["downhillBrakingEnabled"] = downhillBrakingEnabled;
    obj["brakeCurrentA"] = brakeCurrentA;
    obj["brakeEngageErpmMargin"] = brakeEngageErpmMargin;
    obj["parkCurrentA"] = parkCurrentA;

    obj["vescControllerId"] = vescControllerId;
    obj["canBaudrateBps"] = canBaudrateBps;

    obj["motorPolePairs"] = motorPolePairs;
    obj["wheelDiameterMm"] = wheelDiameterMm;
    obj["gearRatio"] = gearRatio;

    obj["batteryVoltageSource"] = batteryVoltageSource;
    obj["vbatCalibrationScale"] = vbatCalibrationScale;

    obj["speedPidEnabled"] = speedPidEnabled;
    obj["pidKp"] = pidKp;
    obj["pidKi"] = pidKi;
    obj["pidKd"] = pidKd;
    obj["pidMaxTrimErpm"] = pidMaxTrimErpm;

    obj["fallAngleThresholdDeg"] = fallAngleThresholdDeg;
    obj["fallRecoverMarginDeg"] = fallRecoverMarginDeg;
    obj["fallConfirmMs"] = fallConfirmMs;
    obj["fallRecoverStableMs"] = fallRecoverStableMs;
    obj["impactDetectEnabled"] = impactDetectEnabled;
    obj["impactAccelThresholdG"] = impactAccelThresholdG;

    obj["fallWarningBuzzerEnabled"] = fallWarningBuzzerEnabled;
    obj["fallWarningStartPercent"] = fallWarningStartPercent;

    obj["resumeWarningEnabled"] = resumeWarningEnabled;

    obj["magDeclinationDeg"] = magDeclinationDeg;

    obj["wifiSsid"] = wifiSsid;
    // Deliberately NOT included: the actual AP password is never sent back
    // to the browser. Changing it now requires submitting the correct
    // current password (see fromJson()) — sending the real value back here
    // would make that pointless, since the web UI could just read it
    // straight out of this response instead of the user having to know it.
}

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
} // namespace

bool Settings::fromJson(JsonObjectConst obj, bool *passwordRejected) {
    if (passwordRejected) *passwordRejected = false;

    bool restartRequired = false;

    if (obj["deadbandPercent"].is<float>()) deadbandPercent = clampf(obj["deadbandPercent"], 0.0f, 45.0f);
    if (obj["potFilterSamples"].is<int>()) {
        potFilterSamples = (uint16_t)constrain((int)obj["potFilterSamples"], 1,
                                               POT_MOVING_AVERAGE_SAMPLES_MAX);
    }
    if (obj["controlMode"].is<int>()) {
        int v = obj["controlMode"];
        controlMode = (v == VESC_CONTROL_MODE_CURRENT) ? VESC_CONTROL_MODE_CURRENT : VESC_CONTROL_MODE_RPM;
    }
    if (obj["maxErpm"].is<int32_t>()) maxErpm = constrain((int32_t)obj["maxErpm"], 100, 200000);
    if (obj["maxCurrentA"].is<float>()) maxCurrentA = clampf(obj["maxCurrentA"], 0.5f, 300.0f);
    if (obj["canSendIntervalMs"].is<uint32_t>()) canSendIntervalMs = constrain((uint32_t)obj["canSendIntervalMs"], 10u, 500u);

    if (obj["speedLimitUnit"].is<int>()) {
        int v = obj["speedLimitUnit"];
        speedLimitUnit = (v == SPEED_LIMIT_UNIT_ERPM) ? SPEED_LIMIT_UNIT_ERPM : SPEED_LIMIT_UNIT_KMH;
    }
    if (obj["maxSpeedKmh"].is<float>()) maxSpeedKmh = clampf(obj["maxSpeedKmh"], 0.1f, 120.0f);
    if (obj["maxSpeedKmhBackward"].is<float>()) maxSpeedKmhBackward = clampf(obj["maxSpeedKmhBackward"], 0.0f, 120.0f);

    // Backward limits: 0 is a valid (if blunt) way to zero out reverse
    // without touching reverseEnabled — the actual "always exactly zero,
    // no matter what" guarantee still comes from reverseEnabled itself.
    if (obj["maxErpmBackward"].is<int32_t>()) maxErpmBackward = constrain((int32_t)obj["maxErpmBackward"], 0, 200000);
    if (obj["maxCurrentBackwardA"].is<float>()) maxCurrentBackwardA = clampf(obj["maxCurrentBackwardA"], 0.0f, 300.0f);
    if (obj["reverseEnabled"].is<bool>()) reverseEnabled = obj["reverseEnabled"];
    if (obj["invertMotorDirection"].is<bool>()) invertMotorDirection = obj["invertMotorDirection"];

    if (obj["downhillBrakingEnabled"].is<bool>()) downhillBrakingEnabled = obj["downhillBrakingEnabled"];
    if (obj["brakeCurrentA"].is<float>()) brakeCurrentA = clampf(obj["brakeCurrentA"], 0.1f, 300.0f);
    if (obj["brakeEngageErpmMargin"].is<int32_t>()) brakeEngageErpmMargin = constrain((int32_t)obj["brakeEngageErpmMargin"], 0, 50000);
    if (obj["parkCurrentA"].is<float>()) parkCurrentA = clampf(obj["parkCurrentA"], 0.1f, 300.0f);

    if (obj["vescControllerId"].is<int>()) {
        uint8_t v = (uint8_t)constrain((int)obj["vescControllerId"], 0, 255);
        if (v != vescControllerId) restartRequired = true;
        vescControllerId = v;
    }
    if (obj["canBaudrateBps"].is<uint32_t>()) {
        uint32_t v = obj["canBaudrateBps"];
        if (v != canBaudrateBps) restartRequired = true;
        canBaudrateBps = v;
    }

    if (obj["motorPolePairs"].is<int>()) motorPolePairs = (uint16_t)constrain((int)obj["motorPolePairs"], 1, 100);
    if (obj["wheelDiameterMm"].is<float>()) wheelDiameterMm = clampf(obj["wheelDiameterMm"], 10.0f, 2000.0f);
    if (obj["gearRatio"].is<float>()) gearRatio = clampf(obj["gearRatio"], 0.01f, 100.0f);

    if (obj["batteryVoltageSource"].is<int>()) {
        batteryVoltageSource = (int)obj["batteryVoltageSource"] == BATTERY_VOLTAGE_SOURCE_VESC
            ? BATTERY_VOLTAGE_SOURCE_VESC : BATTERY_VOLTAGE_SOURCE_ESP32;
    }
    if (obj["vbatCalibrationScale"].is<float>()) vbatCalibrationScale = clampf(obj["vbatCalibrationScale"], 0.5f, 1.5f);

    if (obj["speedPidEnabled"].is<bool>()) speedPidEnabled = obj["speedPidEnabled"];
    if (obj["pidKp"].is<float>()) pidKp = clampf(obj["pidKp"], 0.0f, 100.0f);
    if (obj["pidKi"].is<float>()) pidKi = clampf(obj["pidKi"], 0.0f, 100.0f);
    if (obj["pidKd"].is<float>()) pidKd = clampf(obj["pidKd"], 0.0f, 10.0f);
    if (obj["pidMaxTrimErpm"].is<int32_t>()) pidMaxTrimErpm = constrain((int32_t)obj["pidMaxTrimErpm"], 0, 50000);

    if (obj["fallAngleThresholdDeg"].is<float>()) fallAngleThresholdDeg = clampf(obj["fallAngleThresholdDeg"], 10.0f, 90.0f);
    if (obj["fallRecoverMarginDeg"].is<float>()) fallRecoverMarginDeg = clampf(obj["fallRecoverMarginDeg"], 2.0f, 60.0f);
    if (obj["fallConfirmMs"].is<uint32_t>()) fallConfirmMs = constrain((uint32_t)obj["fallConfirmMs"], 0u, 5000u);
    if (obj["fallRecoverStableMs"].is<uint32_t>()) fallRecoverStableMs = constrain((uint32_t)obj["fallRecoverStableMs"], 0u, 20000u);
    if (obj["impactDetectEnabled"].is<bool>()) impactDetectEnabled = obj["impactDetectEnabled"];
    if (obj["impactAccelThresholdG"].is<float>()) impactAccelThresholdG = clampf(obj["impactAccelThresholdG"], 1.2f, 10.0f);

    if (obj["fallWarningBuzzerEnabled"].is<bool>()) fallWarningBuzzerEnabled = obj["fallWarningBuzzerEnabled"];
    if (obj["fallWarningStartPercent"].is<float>()) fallWarningStartPercent = clampf(obj["fallWarningStartPercent"], 0.0f, 99.0f);

    if (obj["resumeWarningEnabled"].is<bool>()) resumeWarningEnabled = obj["resumeWarningEnabled"];

    if (obj["magDeclinationDeg"].is<float>()) magDeclinationDeg = clampf(obj["magDeclinationDeg"], -180.0f, 180.0f);

    if (obj["wifiSsid"].is<const char *>()) {
        const char *s = obj["wifiSsid"];
        if (strlen(s) > 0 && strcmp(s, wifiSsid) != 0) {
            strncpy(wifiSsid, s, sizeof(wifiSsid) - 1);
            wifiSsid[sizeof(wifiSsid) - 1] = '\0';
            restartRequired = true;
        }
    }
    if (obj["wifiPassword"].is<const char *>()) {
        const char *newPass = obj["wifiPassword"];
        // Changing the password requires proving you know the current one
        // — otherwise anyone who can reach the settings API (e.g. anyone
        // already connected to the AP) could silently lock everyone else
        // out. The current password is never sent back to the browser
        // (see toJson()), so the only way to supply a correct match here is
        // to actually know it.
        const char *currentGiven = obj["wifiPasswordCurrent"].is<const char *>()
                                        ? (const char *)obj["wifiPasswordCurrent"]
                                        : "";
        bool currentMatches = (strcmp(currentGiven, wifiPassword) == 0);
        if (!currentMatches) {
            if (passwordRejected) *passwordRejected = true;
        } else if (strlen(newPass) >= 8 && strlen(newPass) <= 63 && strcmp(newPass, wifiPassword) != 0) {
            // WPA2-PSK requires 8-63 chars; reject silently (not a
            // passwordRejected case) rather than bricking the AP.
            strncpy(wifiPassword, newPass, sizeof(wifiPassword) - 1);
            wifiPassword[sizeof(wifiPassword) - 1] = '\0';
            restartRequired = true;
        }
    }

    return restartRequired;
}
