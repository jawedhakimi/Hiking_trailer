// ESP32-S3 (N16R8) — linear-potentiometer follow controller for a VESC
// motor controller, communicating directly over CAN through a TJA1051
// transceiver, with a 9-axis IMU (ICM20948) for compass heading + fall
// detection, and a WiFi-hosted web dashboard for live telemetry, CAN
// monitoring, and calibrating/tuning everything.
//
// Pot ------ ADS1115 (I2C: SDA=IO7 SCL=IO6, channel A0) --- reads the
//            trailer's handle/linear-pot position.
// ESP32 ---- buzzer (IO39) guides the user through a two-point endpoint
//            calibration at boot if none is saved yet (the web UI's Pot
//            Calibration tab can also (re)calibrate at any time).
// ESP32 ---- ICM20948 (FSPI: SCK=12 MOSI=11 MISO=13 CS=8 INT=42) --- tilt-
//            compensated compass heading + fall detection (cuts motor
//            output immediately on a fall, auto-resumes once upright).
// ESP32 ---- TJA1051 (TX=IO17 RX=IO18) --- CAN bus --- VESC, sending
//            CAN_PACKET_SET_RPM and reading back the VESC's own status
//            broadcasts (speed, temps, voltage, tachometer, ...).
// ESP32 ---- WiFi Access Point + web server: live dashboard, CAN monitor,
//            pot/IMU calibration, speed PID tuning, system settings.
//
// See include/config.h for pins/first-boot defaults, settings.h for the
// runtime-tunable values (persisted to flash, editable from the web UI),
// and README.md for wiring, VESC Tool settings, and calibration flows.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
#include "driver/twai.h"
#include "config.h"
#include "settings.h"
#include "potcal.h"
#include "vesc_can.h"
#include "imu.h"
#include "speed_pid.h"
#include "web_server.h"
#include "odometry.h"
#include "control_lock.h"

// Set to 1 to just stream raw ADC counts to Serial instead of driving the
// VESC — a low-level hardware bring-up aid, independent of calibration.
#define CALIBRATION_PRINT_RAW 0

Adafruit_ADS1115 ads;
static bool adsOk = false;
static SpeedPid speedPid;

// Fixed-storage moving average for the pot. On first use (and whenever N is
// changed) every slot is seeded from the current real ADC reading. This is a
// deliberate safety property: the filter never ramps from an artificial zero.
static int16_t potFilterBuffer[POT_MOVING_AVERAGE_SAMPLES_MAX];
static int64_t potFilterSum = 0;
static uint16_t potFilterIndex = 0;
static uint16_t potFilterWindow = 0;

static float filterPotReading(int16_t raw, uint16_t requestedWindow) {
    uint16_t window = constrain(requestedWindow, 1, POT_MOVING_AVERAGE_SAMPLES_MAX);
    if (potFilterWindow != window) {
        potFilterWindow = window;
        potFilterIndex = 0;
        potFilterSum = (int64_t)raw * window;
        for (uint16_t i = 0; i < window; i++) potFilterBuffer[i] = raw;
        return raw;
    }

    potFilterSum -= potFilterBuffer[potFilterIndex];
    potFilterBuffer[potFilterIndex] = raw;
    potFilterSum += raw;
    potFilterIndex = (potFilterIndex + 1) % potFilterWindow;
    return (float)potFilterSum / potFilterWindow;
}

// Non-blocking fall-proximity warning beeper state (see updateFallWarningBuzzer()).
static bool fallWarnBuzzerOn = false;
static uint32_t fallWarnNextToggleMs = 0;

// Safety-interruption resume gating state. A fall, web calibration, or motor
// direction change keeps the motor locked until the pot is centered and the
// short warning beep sequence has finished.
static bool postFallLockout = false;
static bool resumeWarnActive = false;
static bool resumeWarnBuzzerOn = false;
static uint32_t resumeWarnNextToggleMs = 0;
static uint8_t resumeWarnBeepsDone = 0;

// Fatal CAN-related failure (e.g. TWAI driver install/start failed): CAN
// LED (IO40) goes solid ON, error LED off, halt forever.
static void haltWithCanError(const char *msg) {
    Serial.println(msg);
    Serial.println("Halting for safety — motor will NOT be driven. CAN LED (IO40) solid ON.");
    digitalWrite(PIN_LED_CAN, HIGH);
    digitalWrite(PIN_LED_ERROR, LOW);
    while (true) {
        delay(1000);
    }
}

// Fatal non-CAN failure (e.g. ADS1115 not found): error LED (IO41) blinks
// forever, CAN LED off, halt forever.
static void haltWithOtherError(const char *msg) {
    Serial.println(msg);
    Serial.println("Halting for safety — motor will NOT be driven. Error LED (IO41) blinking.");
    digitalWrite(PIN_LED_CAN, LOW);
    while (true) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        delay(ERROR_LED_BLINK_MS);
        digitalWrite(PIN_LED_ERROR, LOW);
        delay(ERROR_LED_BLINK_MS);
    }
}

// Gives the user a short, bounded window at boot to request a fresh pot
// calibration (over Serial) instead of always trusting the saved one.
static bool checkForRecalibrationRequest() {
    Serial.printf("Send 'c' + Enter within %ldms to force potentiometer recalibration...\n",
                  (long)CALIB_RECAL_PROMPT_MS);
    uint32_t start = millis();
    while (millis() - start < CALIB_RECAL_PROMPT_MS) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'c' || c == 'C') {
                while (Serial.available()) Serial.read();
                return true;
            }
        }
        delay(10);
    }
    return false;
}

// Picks a TWAI timing config at runtime for a settings-selected baud rate
// (used to be a compile-time #if; now the baud rate is web-configurable).
static bool twaiTimingConfigForBps(uint32_t bps, twai_timing_config_t &out) {
    switch (bps) {
        case 125000:  out = TWAI_TIMING_CONFIG_125KBITS();  return true;
        case 250000:  out = TWAI_TIMING_CONFIG_250KBITS();  return true;
        case 500000:  out = TWAI_TIMING_CONFIG_500KBITS();  return true;
        case 1000000: out = TWAI_TIMING_CONFIG_1MBITS();    return true;
        default: return false;
    }
}

static bool initCan() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    // Default rx_queue_len (5) can fill up quickly once the VESC's status
    // broadcasts are enabled alongside other CAN traffic; give it headroom
    // so poll() doesn't drop frames.
    g_config.rx_queue_len = 12;

    twai_timing_config_t t_config;
    if (!twaiTimingConfigForBps(g_settings.canBaudrateBps, t_config)) {
        Serial.printf("Unsupported CAN baud rate %lu — falling back to 500000\n",
                       (unsigned long)g_settings.canBaudrateBps);
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    }
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }
    VescCan::begin();
    return true;
}

// Raw ADC counts (half-width, centered on 0) treated as "no command" —
// shared by computeErpm()/computeCurrentA() below and by the safety-resume
// gate's "is the pot centered?" check.
static int32_t deadbandCounts(int32_t maxOffsetRaw) {
    return (int32_t)((maxOffsetRaw * g_settings.deadbandPercent) / 100.0f);
}

// Maps a filtered raw ADC offset from center into a deadbanded, saturated
// ERPM command, using the live-calibrated half-range and the given forward/
// backward max-ERPM limits (already resolved to ERPM by the caller,
// regardless of whether the configured limit is in ERPM or km/h — see
// erpmFromKmh() below). If reverseEnabled is off, any offset past center in
// the reverse direction returns exactly 0 — not just a smaller number — so
// the motor gets zero torque commanded that way, period.
static int32_t computeErpm(int32_t offsetFromCenter, int32_t maxOffsetRaw,
                           int32_t maxErpmForward, int32_t maxErpmBackward) {
    bool reverseDir = offsetFromCenter < 0;
    if (reverseDir && !g_settings.reverseEnabled) return 0;

    int32_t magnitude = abs(offsetFromCenter);
    int32_t deadband = deadbandCounts(maxOffsetRaw);

    if (magnitude <= deadband) {
        return 0;
    }

    int32_t usable = magnitude - deadband;
    int32_t usableRange = maxOffsetRaw - deadband;
    if (usableRange < 1) usableRange = 1;
    if (usable > usableRange) usable = usableRange; // saturate

    int32_t maxErpmForDirection = reverseDir ? maxErpmBackward : maxErpmForward;
    int32_t mapped = (int32_t)(((int64_t)usable * maxErpmForDirection) / usableRange);
    return reverseDir ? -mapped : mapped;
}

// Same deadbanded/saturated mapping as computeErpm(), scaled to the
// forward/backward max-current limits instead — used in CURRENT control
// mode. Same reverse-disable behavior: exactly 0.0 in the reverse direction
// when reverseEnabled is off.
static float computeCurrentA(int32_t offsetFromCenter, int32_t maxOffsetRaw) {
    bool reverseDir = offsetFromCenter < 0;
    if (reverseDir && !g_settings.reverseEnabled) return 0.0f;

    int32_t magnitude = abs(offsetFromCenter);
    int32_t deadband = deadbandCounts(maxOffsetRaw);

    if (magnitude <= deadband) {
        return 0.0f;
    }

    int32_t usable = magnitude - deadband;
    int32_t usableRange = maxOffsetRaw - deadband;
    if (usableRange < 1) usableRange = 1;
    if (usable > usableRange) usable = usableRange;

    float maxCurrentForDirection = reverseDir ? g_settings.maxCurrentBackwardA : g_settings.maxCurrentA;
    float mapped = ((float)usable / (float)usableRange) * maxCurrentForDirection;
    return reverseDir ? -mapped : mapped;
}

// Non-blocking fall-proximity warning beeper: silent below
// fallWarningStartPercent of the fall threshold, then beeps faster and
// faster (shrinking half-period) as tilt closes in on the threshold itself.
// Safe to call every loop() iteration; only touches the buzzer pin when its
// on/off state actually needs to change. Doesn't run at all while the pot
// calibration's own (blocking) buzzer use could be active, because that
// only ever happens at boot, before loop() starts.
static void updateFallWarningBuzzer(uint32_t now) {
    // Don't fight the safety-resume "starting now" buzzer.
    // for the shared buzzer pin. In practice tilt is already back under the
    // fall-approach warning's own start threshold by the time postFallLockout
    // is active (recovery requires low tilt), so this is belt-and-suspenders
    // rather than something expected to trigger often.
    if (!g_settings.fallWarningBuzzerEnabled || !Imu::isPresent() || postFallLockout) {
        if (fallWarnBuzzerOn) { noTone(PIN_BUZZER); fallWarnBuzzerOn = false; }
        return;
    }

    float threshold = g_settings.fallAngleThresholdDeg;
    float warnStart = threshold * (g_settings.fallWarningStartPercent / 100.0f);
    float tilt = Imu::tiltDeg();

    if (tilt < warnStart) {
        if (fallWarnBuzzerOn) { noTone(PIN_BUZZER); fallWarnBuzzerOn = false; }
        return;
    }

    float span = threshold - warnStart;
    float t = (span > 0.1f) ? constrain((tilt - warnStart) / span, 0.0f, 1.0f) : 1.0f;
    uint32_t halfPeriod = (uint32_t)(FALL_WARNING_MAX_HALF_PERIOD_MS -
                                      t * (FALL_WARNING_MAX_HALF_PERIOD_MS - FALL_WARNING_MIN_HALF_PERIOD_MS));

    if (now >= fallWarnNextToggleMs) {
        fallWarnNextToggleMs = now + halfPeriod;
        fallWarnBuzzerOn = !fallWarnBuzzerOn;
        if (fallWarnBuzzerOn) tone(PIN_BUZZER, BUZZER_FREQ_HZ);
        else noTone(PIN_BUZZER);
    }
}

// Gates motor resume after any safety interruption. While the interruption is
// active output is locked. Afterwards, the pot must be centered and the
// "starting now" warning must finish before control resumes.
static bool updateSafetyResumeGating(uint32_t now, bool interruptionActive,
                                     int32_t offset, int32_t maxOffsetRaw) {
    bool potCentered = abs(offset) <= deadbandCounts(maxOffsetRaw);

    if (interruptionActive) {
        postFallLockout = true;
        if (resumeWarnActive || resumeWarnBuzzerOn) {
            resumeWarnActive = false;
            noTone(PIN_BUZZER);
            resumeWarnBuzzerOn = false;
        }
        return true;
    }

    if (!postFallLockout) {
        return false; // normal operation -- no interruption to recover from
    }

    if (!resumeWarnActive) {
        if (!potCentered) {
            return true; // recovered, but still waiting for the pot to center
        }
        // Pot just (or already) centered -- start the warning beep sequence.
        resumeWarnActive = true;
        resumeWarnBeepsDone = 0;
        resumeWarnBuzzerOn = false;
        resumeWarnNextToggleMs = now;
        return true;
    }

    // Warning sequence in progress.
    if (!potCentered) {
        // Pot moved back out of the dead zone mid-warning -- abort and wait
        // for it to re-center before trying again, rather than starting the
        // motor right as the pot is on its way somewhere else.
        resumeWarnActive = false;
        noTone(PIN_BUZZER);
        resumeWarnBuzzerOn = false;
        return true;
    }

    if (now >= resumeWarnNextToggleMs) {
        resumeWarnBuzzerOn = !resumeWarnBuzzerOn;
        if (resumeWarnBuzzerOn) {
            tone(PIN_BUZZER, BUZZER_FREQ_HZ);
            resumeWarnNextToggleMs = now + RESUME_WARNING_BEEP_ON_MS;
        } else {
            noTone(PIN_BUZZER);
            resumeWarnNextToggleMs = now + RESUME_WARNING_BEEP_OFF_MS;
            resumeWarnBeepsDone++;
            if (resumeWarnBeepsDone >= RESUME_WARNING_BEEP_COUNT) {
                resumeWarnActive = false;
                postFallLockout = false; // warning finished -- motor may resume
                return false;
            }
        }
    }
    return true;
}

// Signed position as a percentage of calibrated travel: -100% at the "min"
// endpoint, 0% at center, +100% at the "max" endpoint.
static float computePositionPercent(int32_t offsetFromCenter, int32_t maxOffsetRaw) {
    if (maxOffsetRaw < 1) return 0.0f;
    float pct = (offsetFromCenter * 100.0f) / (float)maxOffsetRaw;
    if (pct > 100.0f) pct = 100.0f;
    if (pct < -100.0f) pct = -100.0f;
    return pct;
}

static float wheelCircumferenceM() {
    return (g_settings.wheelDiameterMm / 1000.0f) * PI;
}

// Converts a km/h speed limit into the equivalent ERPM, using the same
// pole-pairs/gear-ratio/wheel-diameter relationship the speed telemetry
// below derives ERPM -> km/h from (just inverted) — so a km/h-based speed
// limit and the reported speed stay consistent with each other. Requires
// motorPolePairs/wheelDiameterMm to actually be set correctly (see the
// System tab's "Speed / distance conversion" card) or this will be as wrong
// as the speed telemetry would be.
static int32_t erpmFromKmh(float kmh) {
    float wheelCirc = wheelCircumferenceM();
    if (wheelCirc < 0.0001f || kmh <= 0.0f) return 0;
    float mps = kmh / 3.6f;
    float mechRpm = mps * 60.0f / wheelCirc;
    float polePairs = (g_settings.motorPolePairs > 0) ? (float)g_settings.motorPolePairs : 1.0f;
    float gear = (g_settings.gearRatio > 0.0001f) ? g_settings.gearRatio : 1.0f;
    return (int32_t)(mechRpm * polePairs * gear);
}

// Resolves the actually-effective max forward/backward ERPM limits for this
// loop iteration, honoring g_settings.speedLimitUnit — either the raw ERPM
// settings directly, or a live km/h -> ERPM conversion (see erpmFromKmh()).
// Used both for computeErpm()'s saturation and the speed PID trim's clamp,
// so the two stay consistent regardless of which unit is selected.
static void resolveSpeedLimitsErpm(int32_t &maxErpmForward, int32_t &maxErpmBackward) {
    if (g_settings.speedLimitUnit == SPEED_LIMIT_UNIT_KMH) {
        maxErpmForward = erpmFromKmh(g_settings.maxSpeedKmh);
        maxErpmBackward = erpmFromKmh(g_settings.maxSpeedKmhBackward);
    } else {
        maxErpmForward = g_settings.maxErpm;
        maxErpmBackward = g_settings.maxErpmBackward;
    }
}

// Drives PIN_LED_CAN: solid ON if there's a live CAN error (bus-off/
// recovering, or the most recent transmit failed); otherwise blinks while
// there's been recent tx/rx activity, and goes off when idle.
static void updateCanStatusLed(uint32_t now, bool lastTxOk, uint32_t lastCanTxMs) {
    twai_status_info_t status;
    bool busError = false;
    if (twai_get_status_info(&status) == ESP_OK) {
        busError = (status.state == TWAI_STATE_BUS_OFF || status.state == TWAI_STATE_RECOVERING);
    }

    if (busError || !lastTxOk) {
        digitalWrite(PIN_LED_CAN, HIGH);
        return;
    }

    const VescCan::Status1 &st = VescCan::getStatus1();
    bool txActive = (now - lastCanTxMs) < CAN_ACTIVITY_TIMEOUT_MS;
    bool rxActive = st.everReceived && (now - st.lastRxMillis) < CAN_ACTIVITY_TIMEOUT_MS;

    static uint32_t lastToggleMs = 0;
    static bool ledOn = false;

    if (txActive || rxActive) {
        if (now - lastToggleMs >= CAN_LED_BLINK_MS) {
            lastToggleMs = now;
            ledOn = !ledOn;
            digitalWrite(PIN_LED_CAN, ledOn ? HIGH : LOW);
        }
    } else {
        ledOn = false;
        digitalWrite(PIN_LED_CAN, LOW);
    }
}

// Reads the battery voltage divider on PIN_VBAT_ADC and returns the
// estimated pack voltage (unfiltered).
static float readBatteryVoltageInstant() {
    uint32_t mv = analogReadMilliVolts(PIN_VBAT_ADC);
    float vAdc = mv / 1000.0f;
    float vBatt = vAdc * (VBAT_DIVIDER_R_UPPER_OHM + VBAT_DIVIDER_R_LOWER_OHM) /
                  VBAT_DIVIDER_R_LOWER_OHM;
    return vBatt * g_settings.vbatCalibrationScale;
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000); // give the USB CDC serial monitor time to attach

    Serial.println();
    Serial.println("=== VESC CAN controller (pot + IMU + web UI) ===");

    pinMode(PIN_LED_CAN, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    digitalWrite(PIN_LED_CAN, LOW);
    digitalWrite(PIN_LED_ERROR, LOW);

    ControlLock::begin();
    g_settings.load();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    adsOk = ads.begin(ADS1115_I2C_ADDR);
    if (!adsOk) {
        haltWithOtherError("ADS1115 not found on I2C bus — check SDA/SCL wiring and address.");
    }
    ads.setGain(POT_ADC_GAIN);
    PotCal::begin(&ads);

    if (!initCan()) {
        haltWithCanError("CAN (TWAI) init failed — check TJA1051 wiring and CAN bus power.");
    }

    bool haveSavedCalibration = PotCal::load();
    bool forceRecalibrate = checkForRecalibrationRequest();
    if (!haveSavedCalibration || forceRecalibrate) {
        if (!PotCal::runBuzzerGuidedCalibration()) {
            Serial.println("Pot calibration is incomplete. CAN motor commands are disabled until "
                           "valid endpoints are captured from the dashboard.");
        }
    }

    if (!Imu::begin()) {
        Serial.println("WARNING: ICM20948 IMU not detected — compass and FALL PROTECTION are "
                        "NOT active. Check FSPI wiring (CS=8 SCK=12 MOSI=11 MISO=13). Continuing "
                        "without it (pot/CAN control still works).");
        // Non-fatal on purpose: losing the IMU shouldn't strand the whole
        // vehicle, but it's loud in the log and surfaced in the web UI.
    }

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    if (!WebServerApp::begin()) {
        Serial.println("WARNING: web server failed to start — dashboard unavailable, "
                        "control loop continues normally.");
    }

    Serial.println("Setup complete. Driving VESC over CAN.");
#if CALIBRATION_PRINT_RAW
    Serial.println("CALIBRATION_PRINT_RAW=1: streaming raw ADC counts only, motor NOT driven.");
#endif
}

void loop() {
    ControlLock::lock();
    static uint32_t lastSend = 0;
    static uint32_t lastDebugPrint = 0;
    static uint32_t lastPidUpdate = 0;
    static int32_t lastPidTrim = 0;
    static float vBattFiltered = -1.0f; // -1 = not yet initialized
    static uint32_t lastCanTxMs = 0;
    static bool lastTxOk = true;
    static bool directionSettingInitialized = false;
    static bool lastInvertMotorDirection = false;
    static bool directionChangeLockout = false;

    // Drain any pending CAN RX frames (VESC status broadcasts etc.) — cheap,
    // do this every loop iteration regardless of send/print timing.
    VescCan::poll(g_settings.vescControllerId);
    Odometry::update();
    Imu::update(); // internally rate-limited to IMU_SAMPLE_INTERVAL_MS

    int16_t raw = PotCal::readRaw();
    float filteredPotValue = filterPotReading(raw, g_settings.potFilterSamples);
    int32_t offset = (int32_t)filteredPotValue - PotCal::centerRaw();

    if (!directionSettingInitialized) {
        lastInvertMotorDirection = g_settings.invertMotorDirection;
        directionSettingInitialized = true;
    } else if (lastInvertMotorDirection != g_settings.invertMotorDirection) {
        lastInvertMotorDirection = g_settings.invertMotorDirection;
        directionChangeLockout = true;
    }
    bool potCentered = abs(offset) <= deadbandCounts(PotCal::maxOffsetRaw());
    // Remember the pre-update state so even the loop that releases a completed
    // calibration is seen as an interruption by the resume gate.
    bool calibrationWasActive = PotCal::isWebCalibrationActive();
    bool calibrationActive = PotCal::updateWebCalibrationLock(potCentered);
    bool calibrationReady = PotCal::isValid();

#if CALIBRATION_PRINT_RAW
    if (millis() - lastDebugPrint >= DEBUG_PRINT_INTERVAL_MS) {
        lastDebugPrint = millis();
        Serial.printf("raw=%d  filtered=%.1f  offsetFromCenter=%ld\n",
                      raw, filteredPotValue, (long)offset);
    }
    delay(10);
    ControlLock::unlock();
    return; // never drives the motor in this mode
#endif

    bool inRpmMode = (g_settings.controlMode != VESC_CONTROL_MODE_CURRENT);

    int32_t maxErpmForward = 0, maxErpmBackward = 0;
    resolveSpeedLimitsErpm(maxErpmForward, maxErpmBackward);

    int32_t feedforwardErpm = computeErpm(offset, PotCal::maxOffsetRaw(), maxErpmForward, maxErpmBackward);
    float feedforwardCurrentA = computeCurrentA(offset, PotCal::maxOffsetRaw());
    float posPct = computePositionPercent(offset, PotCal::maxOffsetRaw());

    uint32_t now = millis();

    // --- Optional closed-loop trim on top of the feedforward ERPM --------
    // (RPM control mode only — commanding current directly has no ERPM
    // setpoint to trim against, see config.h's VESC_CONTROL_MODE_* notes.)
    const VescCan::Status1 &st = VescCan::getStatus1();
    bool statusFresh = st.everReceived && (now - st.lastRxMillis) < VESC_STATUS_TIMEOUT_MS;
    int motorDirectionSign = g_settings.invertMotorDirection ? -1 : 1;
    int32_t actualErpm = st.erpm * motorDirectionSign;
    float actualMotorCurrentA = st.currentA * motorDirectionSign;
    float actualDutyCycle = st.dutyCycle * motorDirectionSign;
    int32_t erpmOut = feedforwardErpm;
    float currentOut = feedforwardCurrentA;
    bool pidActive = false;
    int32_t pidTrim = lastPidTrim;

    if (inRpmMode && g_settings.speedPidEnabled) {
        if (statusFresh && now - lastPidUpdate >= SPEED_PID_UPDATE_INTERVAL_MS) {
            float dt = (lastPidUpdate == 0) ? (SPEED_PID_UPDATE_INTERVAL_MS / 1000.0f)
                                             : (now - lastPidUpdate) / 1000.0f;
            lastPidUpdate = now;
            pidTrim = (int32_t)speedPid.compute(g_settings.pidKp, g_settings.pidKi, g_settings.pidKd,
                                                 (float)feedforwardErpm, (float)actualErpm, dt,
                                                 (float)g_settings.pidMaxTrimErpm);
            lastPidTrim = pidTrim;
            pidActive = true;
        } else if (!statusFresh) {
            speedPid.reset(); // avoid a stale integral once feedback returns
            lastPidTrim = 0;
            pidTrim = 0;
        } else {
            pidActive = true;
        }
        erpmOut = feedforwardErpm + pidTrim;
        if (erpmOut > maxErpmForward) erpmOut = maxErpmForward;
        if (erpmOut < -maxErpmBackward) erpmOut = -maxErpmBackward;
    } else {
        speedPid.reset();
        lastPidTrim = 0;
        pidTrim = 0;
    }

    // --- Reverse-disable: final, unconditional clamp -----------------------
    // Belt-and-suspenders on top of computeErpm()/computeCurrentA() already
    // returning 0 for a reverse-direction pot position: this also catches a
    // PID trim (RPM mode) that could otherwise nudge an at-rest command
    // negative on its own. Braking (below) is intentionally NOT subject to
    // this — it opposes whatever direction the wheel is actually moving, it
    // never drives it, so it must stay available even with reverse disabled.
    if (!g_settings.reverseEnabled) {
        if (erpmOut < 0) erpmOut = 0;
        if (currentOut < 0.0f) currentOut = 0.0f;
    }

    // --- Downhill braking: actual speed notably exceeds current intent -----
    // "Intent" is feedforwardErpm (the pot's own ERPM-equivalent request),
    // which exists regardless of control mode, so this works in both Speed
    // and Current modes. Overrides the normal RPM/current command for this
    // cycle with a SET_CURRENT_BRAKE instead, when engaged.
    bool brakingActive = false;
    if (g_settings.downhillBrakingEnabled && statusFresh) {
        int32_t intendedMag = abs(feedforwardErpm);
        int32_t actualMag = abs(actualErpm);
        brakingActive = actualMag > intendedMag + g_settings.brakeEngageErpmMargin;
    }

    // --- Safety interruptions: unconditionally override to a stop command --
    // Falls, pot calibration and direction changes all pass through the same
    // center-then-beep sequence before motor control is enabled again.
    bool fallen = Imu::isFallen();
    bool parkEnabled = WebServerApp::isParkEnabled();
    bool shaftPowerEnabled = WebServerApp::isShaftPowerEnabled();
    bool interruptionActive = fallen || calibrationWasActive || calibrationActive ||
                              directionChangeLockout || parkEnabled || !shaftPowerEnabled;
    bool motorLockedOut = updateSafetyResumeGating(now, interruptionActive,
                                                    offset, PotCal::maxOffsetRaw());
    // Keep directionChangeLockout asserted for at least one complete loop so
    // the resume gate cannot miss a change made while the pot is already centered.
    if (directionChangeLockout && potCentered) {
        directionChangeLockout = false;
    }
    if (motorLockedOut) {
        erpmOut = 0;
        currentOut = 0.0f;
        brakingActive = false; // stop command, not a brake command, while locked out
    }
    updateFallWarningBuzzer(now);

    // Park uses the VESC's dedicated handbrake controller. A fall or active /
    // invalid calibration still wins: in those states we either transmit zero
    // or no command at all, preserving the existing safety guarantees.
    bool parkHolding = shaftPowerEnabled && parkEnabled && !fallen &&
                       calibrationReady && !calibrationActive;
    if (shaftPowerEnabled && calibrationReady && !calibrationActive &&
        now - lastSend >= g_settings.canSendIntervalMs) {
        lastSend = now;
        lastCanTxMs = now;
        lastTxOk = parkHolding
            ? VescCan::sendSetCurrentHandbrake(g_settings.vescControllerId, g_settings.parkCurrentA)
            : (brakingActive
                ? VescCan::sendSetCurrentBrake(g_settings.vescControllerId, g_settings.brakeCurrentA)
                : (inRpmMode ? VescCan::sendSetRpm(g_settings.vescControllerId, erpmOut * motorDirectionSign)
                             : VescCan::sendSetCurrent(g_settings.vescControllerId, currentOut * motorDirectionSign)));
    }

    updateCanStatusLed(now, lastTxOk, lastCanTxMs);

    // --- Speed, from the VESC's own measured ERPM (not the commanded value)
    float mechRpm = statusFresh
        ? ((float)actualErpm / (g_settings.motorPolePairs > 0 ? g_settings.motorPolePairs : 1) /
           (g_settings.gearRatio > 0.0001f ? g_settings.gearRatio : 1.0f))
        : 0.0f;
    float speedMps = mechRpm / 60.0f * wheelCircumferenceM();

    const VescCan::StatusExtra &se = VescCan::getStatusExtra();

    // --- Battery voltage --------------------------------------------------
    // The divider is always sampled so both readings can be compared in the
    // dashboard. When VESC is selected, Status 5 must be enabled and fresh;
    // there is no silent fallback to a different source.
    float vBattInstant = readBatteryVoltageInstant();
    if (vBattFiltered < 0.0f) {
        vBattFiltered = vBattInstant;
    } else {
        vBattFiltered = VBAT_EMA_ALPHA * vBattInstant + (1.0f - VBAT_EMA_ALPHA) * vBattFiltered;
    }

    bool status2Fresh = se.status2EverReceived && (now - se.status2LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS;
    bool status3Fresh = se.status3EverReceived && (now - se.status3LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS;
    bool status4Fresh = se.status4EverReceived && (now - se.status4LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS;
    bool status5Fresh = se.status5EverReceived && (now - se.status5LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS;
    bool status6Fresh = se.status6EverReceived && (now - se.status6LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS;
    bool useVescVoltage = g_settings.batteryVoltageSource == BATTERY_VOLTAGE_SOURCE_VESC;
    float selectedBatteryVoltage = useVescVoltage ? se.inputVoltageV : vBattFiltered;
    bool selectedBatteryFresh = useVescVoltage ? status5Fresh : true;

    // --- Hand off telemetry to the web layer ------------------------------
    WebServerApp::TelemetrySnapshot snap;
    snap.potPositionPercent = posPct;
    snap.potRaw = (int32_t)filteredPotValue;
    snap.potRawInstant = raw;
    snap.controlMode = g_settings.controlMode;
    snap.targetErpm = inRpmMode ? erpmOut : 0;
    snap.targetCurrentA = inRpmMode ? 0.0f : currentOut;
    snap.actualErpm = actualErpm;
    snap.vescStatusFresh = statusFresh;
    snap.speedMps = speedMps;
    snap.odometryAvailable = Odometry::hasDistanceSource();
    snap.odometryUsingTachometer = Odometry::usingTachometer();
    snap.vBatt = selectedBatteryVoltage;
    snap.vBattEsp32 = vBattFiltered;
    snap.vBattVesc = se.inputVoltageV;
    snap.batteryVoltageSource = g_settings.batteryVoltageSource;
    snap.batteryVoltageFresh = selectedBatteryFresh;
    snap.motorCurrentA = actualMotorCurrentA;
    snap.dutyCycle = actualDutyCycle;
    snap.fetTempC = se.fetTempC;
    snap.motorTempC = se.motorTempC;
    snap.inputCurrentA = se.currentInA;
    snap.ampHoursConsumed = se.ampHoursConsumed;
    snap.ampHoursCharged = se.ampHoursCharged;
    snap.wattHoursConsumed = se.wattHoursConsumed;
    snap.wattHoursCharged = se.wattHoursCharged;
    snap.pidPositionDeg = se.pidPositionDeg;
    snap.tachometerRaw = se.tachometerRaw;
    snap.adc1V = se.adc1V;
    snap.adc2V = se.adc2V;
    snap.adc3V = se.adc3V;
    snap.ppm = se.ppm;
    snap.status2Fresh = status2Fresh;
    snap.status3Fresh = status3Fresh;
    snap.status4Fresh = status4Fresh;
    snap.status5Fresh = status5Fresh;
    snap.status6Fresh = status6Fresh;
    snap.status1Ever = st.everReceived;
    snap.status2Ever = se.status2EverReceived;
    snap.status3Ever = se.status3EverReceived;
    snap.status4Ever = se.status4EverReceived;
    snap.status5Ever = se.status5EverReceived;
    snap.status6Ever = se.status6EverReceived;
    snap.status1AgeMs = st.everReceived ? now - st.lastRxMillis : 0;
    snap.status2AgeMs = se.status2EverReceived ? now - se.status2LastRxMillis : 0;
    snap.status3AgeMs = se.status3EverReceived ? now - se.status3LastRxMillis : 0;
    snap.status4AgeMs = se.status4EverReceived ? now - se.status4LastRxMillis : 0;
    snap.status5AgeMs = se.status5EverReceived ? now - se.status5LastRxMillis : 0;
    snap.status6AgeMs = se.status6EverReceived ? now - se.status6LastRxMillis : 0;
    snap.pidActive = pidActive;
    snap.pidTrimErpm = pidTrim;
    snap.brakingActive = brakingActive;
    snap.parkEnabled = parkEnabled;
    snap.parkHolding = parkHolding;
    snap.shaftPowerEnabled = shaftPowerEnabled;
    snap.postFallLockout = postFallLockout;
    snap.resumeWarnActive = resumeWarnActive;
    snap.calibrationReady = calibrationReady;
    snap.calibrationActive = calibrationActive;
    snap.calibrationWaitingForCenter = PotCal::isWaitingForCenter();
    snap.directionChangeLockout = directionChangeLockout;
    WebServerApp::setTelemetry(snap);
    ControlLock::unlock();
    WebServerApp::loop();

    if (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL_MS) {
        lastDebugPrint = now;

#if DEBUG_MODE == DEBUG_MODE_VALUES
        Serial.printf(
            "pos=%.1f%% speed=%.2f m/s (%.1f km/h) trip=%.1f m | Vbat=%.2f V | tilt=%.1f "
            "heading=%.0f fallen=%d\n",
            posPct, speedMps, speedMps * 3.6f, Odometry::tripDistanceM(), selectedBatteryVoltage,
            Imu::tiltDeg(), Imu::headingDeg(), fallen ? 1 : 0);
#elif DEBUG_MODE == DEBUG_MODE_CAN
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            const char *stateStr =
                status.state == TWAI_STATE_STOPPED ? "STOPPED" :
                status.state == TWAI_STATE_RUNNING ? "RUNNING" :
                status.state == TWAI_STATE_BUS_OFF ? "BUS_OFF" :
                status.state == TWAI_STATE_RECOVERING ? "RECOVERING" : "?";
            Serial.printf(
                "CAN status: state=%s txErr=%lu rxErr=%lu txFailed=%lu rxMissed=%lu "
                "rxOverrun=%lu arbLost=%lu busErr=%lu\n",
                stateStr,
                (unsigned long)status.tx_error_counter, (unsigned long)status.rx_error_counter,
                (unsigned long)status.tx_failed_count, (unsigned long)status.rx_missed_count,
                (unsigned long)status.rx_overrun_count, (unsigned long)status.arb_lost_count,
                (unsigned long)status.bus_error_count);
        } else {
            Serial.println("CAN status: twai_get_status_info() failed");
        }
#endif
    }
}
