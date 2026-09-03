#include "potcal.h"
#include <Preferences.h>
#include "buzzer.h"
#include "config.h"

namespace PotCal {

namespace {
Adafruit_ADS1115 *ads = nullptr;
Preferences prefs;

int32_t g_centerRaw = 0;
int32_t g_minRaw = 0;
int32_t g_maxRaw = 0;
int32_t g_maxOffsetRaw = POT_ADC_MAX_OFFSET;
bool g_valid = false;
bool g_haveMin = false;
bool g_haveMax = false;
bool g_webCalibrationActive = false;
bool g_webCalibrationReadyToFinish = false;
bool g_stagedHaveMin = false;
bool g_stagedHaveMax = false;
int32_t g_stagedMinRaw = 0;
int32_t g_stagedMaxRaw = 0;

void blinkErrorLedWarning(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        delay(ERROR_LED_BLINK_MS);
        digitalWrite(PIN_LED_ERROR, LOW);
        delay(ERROR_LED_BLINK_MS);
    }
}

// Derives center/maxOffset from a min/max pair. Returns false if the span
// looks too small to trust; main.cpp then keeps motor CAN output disabled.
bool applyCalibration(int32_t minR, int32_t maxR) {
    g_minRaw = minR;
    g_maxRaw = maxR;
    g_centerRaw = (g_minRaw + g_maxRaw) / 2;

    int32_t span = g_maxRaw - g_minRaw;
    g_valid = span >= CALIB_MIN_VALID_SPAN_RAW;
    g_maxOffsetRaw = g_valid ? (span / 2) : POT_ADC_MAX_OFFSET;
    return g_valid;
}

void persist() {
    prefs.begin(CALIB_NVS_NAMESPACE, false);
    prefs.putInt("min", g_minRaw);
    prefs.putInt("max", g_maxRaw);
    prefs.end();
}
} // namespace

void begin(Adafruit_ADS1115 *adsDevice) {
    ads = adsDevice;
}

int32_t readAveraged(uint32_t durationMs) {
    long sum = 0;
    long count = 0;
    uint32_t start = millis();
    do {
        sum += ads->readADC_SingleEnded(POT_ADC_CHANNEL);
        count++;
        delay(CALIB_SAMPLE_INTERVAL_MS);
    } while (millis() - start < durationMs);
    return count > 0 ? (int32_t)(sum / count) : 0;
}

int16_t readRaw() {
    return ads ? ads->readADC_SingleEnded(POT_ADC_CHANNEL) : 0;
}

void startWebCalibration() {
    g_webCalibrationActive = true;
    g_webCalibrationReadyToFinish = false;
    g_stagedHaveMin = false;
    g_stagedHaveMax = false;
}

void cancelWebCalibration() {
    if (g_webCalibrationActive) g_webCalibrationReadyToFinish = true;
}

bool isWebCalibrationActive() { return g_webCalibrationActive; }
bool isWaitingForCenter() { return g_webCalibrationActive && g_webCalibrationReadyToFinish; }

bool updateWebCalibrationLock(bool potCentered) {
    if (g_webCalibrationActive && g_webCalibrationReadyToFinish && potCentered) {
        g_webCalibrationActive = false;
        g_webCalibrationReadyToFinish = false;
        g_stagedHaveMin = false;
        g_stagedHaveMax = false;
    }
    return g_webCalibrationActive;
}

bool load() {
    prefs.begin(CALIB_NVS_NAMESPACE, true);
    bool hasKeys = prefs.isKey("min") && prefs.isKey("max");
    int32_t savedMin = prefs.getInt("min", 0);
    int32_t savedMax = prefs.getInt("max", 0);
    prefs.end();

    if (!hasKeys) return false;

    bool valid = applyCalibration(savedMin, savedMax);
    if (!valid) {
        Serial.println("WARNING: saved pot calibration span looked invalid — ignoring it.");
        return false;
    }
    g_haveMin = true;
    g_haveMax = true;

    Serial.printf(
        "Loaded saved pot calibration: min=%ld max=%ld center=%ld maxOffset=%ld\n",
        (long)g_minRaw, (long)g_maxRaw, (long)g_centerRaw, (long)g_maxOffsetRaw);
    return true;
}

bool runBuzzerGuidedCalibration() {
    Serial.println("=== Potentiometer calibration ===");

    Serial.println("Buzz: move the potentiometer to ONE END now...");
    Buzzer::beepBlocking(BUZZ_SHORT_MS);
    delay(CALIB_MOVE_WINDOW_MS);
    int32_t extreme1 = readAveraged(CALIB_SETTLE_SAMPLE_MS);
    Serial.printf("  end 1 raw = %ld\n", (long)extreme1);

    Serial.println("Buzz: now move the potentiometer to the OTHER END...");
    Buzzer::beepBlocking(BUZZ_SHORT_MS);
    delay(CALIB_MOVE_WINDOW_MS);
    int32_t extreme2 = readAveraged(CALIB_SETTLE_SAMPLE_MS);
    Serial.printf("  end 2 raw = %ld\n", (long)extreme2);

    g_haveMin = true;
    g_haveMax = true;
    bool valid = applyCalibration(min(extreme1, extreme2), max(extreme1, extreme2));
    if (!valid) {
        Serial.printf(
            "WARNING: calibration span too small (%ld counts) — pot may not have been "
            "moved, or is wired incorrectly. Motor CAN output remains disabled.\n",
            (long)(g_maxRaw - g_minRaw));
        blinkErrorLedWarning(6);
        g_haveMin = false;
        g_haveMax = false;
        return false;
    }

    Serial.printf("Calibrated: min=%ld max=%ld center=%ld maxOffset=%ld\n",
                  (long)g_minRaw, (long)g_maxRaw, (long)g_centerRaw, (long)g_maxOffsetRaw);

    Buzzer::beepBlocking(BUZZ_COMPLETE_MS);
    persist();
    Serial.println("Calibration saved to flash.");
    return true;
}

bool captureEndpoint(int32_t raw, bool isMax) {
    if (!g_webCalibrationActive || g_webCalibrationReadyToFinish) return false;

    if (isMax) {
        g_stagedMaxRaw = raw;
        g_stagedHaveMax = true;
    } else {
        g_stagedMinRaw = raw;
        g_stagedHaveMin = true;
    }
    if (!g_stagedHaveMin || !g_stagedHaveMax) return false;

    int32_t newMin = min(g_stagedMinRaw, g_stagedMaxRaw);
    int32_t newMax = max(g_stagedMinRaw, g_stagedMaxRaw);
    if (newMax - newMin < CALIB_MIN_VALID_SPAN_RAW) return false;

    applyCalibration(newMin, newMax);
    g_haveMin = true;
    g_haveMax = true;
    persist();
    g_webCalibrationReadyToFinish = true;
    return true;
}

bool setEndpoints(int32_t minRaw, int32_t maxRaw) {
    if (!g_webCalibrationActive || g_webCalibrationReadyToFinish) return false;
    int32_t prevMin = g_minRaw, prevMax = g_maxRaw;
    bool previouslyValid = g_valid;
    bool valid = applyCalibration(min(minRaw, maxRaw), max(minRaw, maxRaw));
    if (valid) {
        g_haveMin = true;
        g_haveMax = true;
        persist();
        g_webCalibrationReadyToFinish = true;
    } else if (previouslyValid) {
        applyCalibration(prevMin, prevMax);
    }
    return valid;
}

int32_t centerRaw() { return g_centerRaw; }
int32_t minRaw() { return g_webCalibrationActive && g_stagedHaveMin ? g_stagedMinRaw : g_minRaw; }
int32_t maxRaw() { return g_webCalibrationActive && g_stagedHaveMax ? g_stagedMaxRaw : g_maxRaw; }
int32_t maxOffsetRaw() { return g_maxOffsetRaw; }
bool isValid() { return g_valid; }
bool hasCapturedMin() { return g_webCalibrationActive ? g_stagedHaveMin : g_haveMin; }
bool hasCapturedMax() { return g_webCalibrationActive ? g_stagedHaveMax : g_haveMax; }

} // namespace PotCal
