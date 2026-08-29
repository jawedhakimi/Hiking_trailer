#include "imu.h"
#include <SPI.h>
#include <ICM_20948.h>
#include <Preferences.h>
#include <math.h>
#include "config.h"
#include "settings.h"

namespace Imu {

namespace {

ICM_20948_SPI icm;
Preferences prefs;
bool present = false;

// Complementary-filter fused angles (degrees), conventional Z-up mounting.
float g_pitchDeg = 0.0f;
float g_rollDeg = 0.0f;

// Calibrated "upright" gravity reference (unit vector, sensor frame).
float g0x = 0.0f, g0y = 0.0f, g0z = 1.0f;

// Low-pass-filtered gravity vector used for the fall-detection tilt angle
// (separate from the pitch/roll complementary filter above — this one
// only needs to reject single-sample vibration, not track fast motion).
float g_lpAx = 0.0f, g_lpAy = 0.0f, g_lpAz = 1.0f;
bool g_lpSeeded = false;
#define TILT_LPF_ALPHA 0.3f

float g_tiltDeg = 0.0f;
float g_headingDeg = 0.0f;
float g_accelMagG = 1.0f;

// Hard-iron magnetometer offset (uT), persisted.
float magOffX = MAG_CAL_DEFAULT_OFFSET_X;
float magOffY = MAG_CAL_DEFAULT_OFFSET_Y;
float magOffZ = MAG_CAL_DEFAULT_OFFSET_Z;

bool magCalActive = false;
float magCalMinX, magCalMaxX, magCalMinY, magCalMaxY, magCalMinZ, magCalMaxZ;

// --- Fall/impact state machine --------------------------------------------
bool g_fallLatched = false;
uint32_t g_tiltExceedSinceMs = 0;
uint32_t g_recoverOkSinceMs = 0;

FallEvent g_eventLog[FALL_EVENT_LOG_SIZE];
size_t g_eventCount = 0;   // total ever, may exceed log size
size_t g_eventHead = 0;    // next write index (ring buffer)

void logFallEvent(FallTrigger trig, float angle) {
    g_eventLog[g_eventHead] = {millis(), trig, angle};
    g_eventHead = (g_eventHead + 1) % FALL_EVENT_LOG_SIZE;
    g_eventCount++;
}

uint32_t g_lastSampleMs = 0;

void loadCalibration() {
    prefs.begin("imucal", true);
    if (prefs.isKey("g0x")) {
        g0x = prefs.getFloat("g0x", 0.0f);
        g0y = prefs.getFloat("g0y", 0.0f);
        g0z = prefs.getFloat("g0z", 1.0f);
    }
    magOffX = prefs.getFloat("magOffX", magOffX);
    magOffY = prefs.getFloat("magOffY", magOffY);
    magOffZ = prefs.getFloat("magOffZ", magOffZ);
    prefs.end();
}

} // namespace

bool begin() {
    SPI.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, PIN_IMU_CS);
    pinMode(PIN_IMU_INT, INPUT); // not used as an interrupt yet; reserved

    ICM_20948_Status_e st = icm.begin(PIN_IMU_CS, SPI, IMU_SPI_CLOCK_HZ);
    if (st != ICM_20948_Stat_Ok) {
        Serial.printf("ICM20948 init failed: %s\n", icm.statusString(st));
        present = false;
        return false;
    }

    icm.swReset();
    delay(50);
    icm.sleep(false);
    icm.lowPower(false);

    icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mag),
                       ICM_20948_Sample_Mode_Continuous);

    ICM_20948_fss_t fss;
    fss.a = gpm4;   // +/-4g accelerometer full scale — plenty for fall/impact detection
    fss.g = dps500; // +/-500 dps gyro full scale
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

    ICM_20948_dlpcfg_t dlp;
    dlp.a = acc_d111bw4_n136bw;
    dlp.g = gyr_d119bw5_n154bw3;
    icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
    icm.enableDLPF(ICM_20948_Internal_Acc, true);
    icm.enableDLPF(ICM_20948_Internal_Gyr, true);

    // Brings up the AK09916 magnetometer through the ICM20948's internal
    // I2C master — works the same whether the ICM20948 itself is on I2C or
    // SPI (which is why this "just works" over our SPI link).
    icm.startupMagnetometer();

    present = (icm.status == ICM_20948_Stat_Ok);
    if (present) loadCalibration();
    return present;
}

bool isPresent() { return present; }

bool update() {
    if (!present) return false;

    uint32_t now = millis();
    if (now - g_lastSampleMs < IMU_SAMPLE_INTERVAL_MS) return false;
    float dt = (g_lastSampleMs == 0) ? (IMU_SAMPLE_INTERVAL_MS / 1000.0f)
                                      : (now - g_lastSampleMs) / 1000.0f;
    g_lastSampleMs = now;

    if (!icm.dataReady()) return false;
    icm.getAGMT();

    // mg -> g, dps stays dps, uT stays uT.
    float ax = icm.accX() / 1000.0f;
    float ay = icm.accY() / 1000.0f;
    float az = icm.accZ() / 1000.0f;
    float gx = icm.gyrX(); // dps
    float gy = icm.gyrY();
    float mx = icm.magX() - magOffX;
    float my = icm.magY() - magOffY;
    float mz = icm.magZ() - magOffZ;

    g_accelMagG = sqrtf(ax * ax + ay * ay + az * az);

    // --- Pitch/roll complementary filter (conventional Z-up mounting) ----
    float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float accelRoll = atan2f(ay, az) * RAD_TO_DEG;
    // NOTE: if pitch or roll reads backwards on your mounting, flip the
    // sign of the corresponding gyro term below (gy for pitch, gx for
    // roll) — this depends on which way the board is physically mounted.
    g_pitchDeg = IMU_COMPLEMENTARY_ALPHA * (g_pitchDeg + gy * dt) +
                 (1.0f - IMU_COMPLEMENTARY_ALPHA) * accelPitch;
    g_rollDeg = IMU_COMPLEMENTARY_ALPHA * (g_rollDeg + gx * dt) +
                (1.0f - IMU_COMPLEMENTARY_ALPHA) * accelRoll;

    // --- Tilt-compensated heading -----------------------------------------
    float pitchRad = g_pitchDeg * DEG_TO_RAD;
    float rollRad = g_rollDeg * DEG_TO_RAD;
    float mxComp = mx * cosf(pitchRad) + mz * sinf(pitchRad);
    float myComp = mx * sinf(rollRad) * sinf(pitchRad) + my * cosf(rollRad) -
                    mz * sinf(rollRad) * cosf(pitchRad);
    float heading = atan2f(myComp, mxComp) * RAD_TO_DEG;
    heading += g_settings.magDeclinationDeg;
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    g_headingDeg = heading;

    // --- Fall-detection tilt angle: mounting-independent, vs. calibrated
    //     "upright" gravity vector, on a lightly low-passed accel vector.
    if (!g_lpSeeded) {
        g_lpAx = ax; g_lpAy = ay; g_lpAz = az;
        g_lpSeeded = true;
    } else {
        g_lpAx = TILT_LPF_ALPHA * ax + (1 - TILT_LPF_ALPHA) * g_lpAx;
        g_lpAy = TILT_LPF_ALPHA * ay + (1 - TILT_LPF_ALPHA) * g_lpAy;
        g_lpAz = TILT_LPF_ALPHA * az + (1 - TILT_LPF_ALPHA) * g_lpAz;
    }
    float mag = sqrtf(g_lpAx * g_lpAx + g_lpAy * g_lpAy + g_lpAz * g_lpAz);
    float dot = 1.0f;
    if (mag > 0.01f) {
        dot = (g_lpAx * g0x + g_lpAy * g0y + g_lpAz * g0z) / mag;
    }
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    g_tiltDeg = acosf(dot) * RAD_TO_DEG;

    // --- Magnetometer calibration capture (if a wizard is running) -------
    if (magCalActive) {
        float rawMx = icm.magX(), rawMy = icm.magY(), rawMz = icm.magZ();
        if (rawMx < magCalMinX) magCalMinX = rawMx;
        if (rawMx > magCalMaxX) magCalMaxX = rawMx;
        if (rawMy < magCalMinY) magCalMinY = rawMy;
        if (rawMy > magCalMaxY) magCalMaxY = rawMy;
        if (rawMz < magCalMinZ) magCalMinZ = rawMz;
        if (rawMz > magCalMaxZ) magCalMaxZ = rawMz;
    }

    // --- Fall / impact state machine --------------------------------------
    bool tiltExceeds = g_tiltDeg > g_settings.fallAngleThresholdDeg;
    bool impactHit = g_settings.impactDetectEnabled &&
                      g_accelMagG > g_settings.impactAccelThresholdG;

    if (!g_fallLatched) {
        if (impactHit) {
            g_fallLatched = true;
            logFallEvent(FallTrigger::IMPACT, g_tiltDeg);
            g_recoverOkSinceMs = 0;
        } else if (tiltExceeds) {
            if (g_tiltExceedSinceMs == 0) g_tiltExceedSinceMs = now;
            if (now - g_tiltExceedSinceMs >= g_settings.fallConfirmMs) {
                g_fallLatched = true;
                logFallEvent(FallTrigger::TILT, g_tiltDeg);
                g_recoverOkSinceMs = 0;
            }
        } else {
            g_tiltExceedSinceMs = 0;
        }
    } else {
        float recoverThreshold = g_settings.fallAngleThresholdDeg - g_settings.fallRecoverMarginDeg;
        if (recoverThreshold < 0) recoverThreshold = 0;
        bool withinRecoverBand = (g_tiltDeg < recoverThreshold) && !impactHit;
        if (withinRecoverBand) {
            if (g_recoverOkSinceMs == 0) g_recoverOkSinceMs = now;
            if (now - g_recoverOkSinceMs >= g_settings.fallRecoverStableMs) {
                g_fallLatched = false;
                g_recoverOkSinceMs = 0;
                g_tiltExceedSinceMs = 0;
            }
        } else {
            g_recoverOkSinceMs = 0;
        }
    }

    return true;
}

float pitchDeg() { return g_pitchDeg; }
float rollDeg() { return g_rollDeg; }
float tiltDeg() { return g_tiltDeg; }
float headingDeg() { return g_headingDeg; }
float accelMagnitudeG() { return g_accelMagG; }

bool isFallen() { return g_fallLatched; }
uint32_t fallEventCount() { return (uint32_t)g_eventCount; }

size_t getFallEventLog(FallEvent *out, size_t maxCount) {
    size_t available = g_eventCount < FALL_EVENT_LOG_SIZE ? g_eventCount : FALL_EVENT_LOG_SIZE;
    size_t n = available < maxCount ? available : maxCount;
    // Oldest-first: the ring buffer's oldest valid entry is at g_eventHead
    // once it has wrapped, or index 0 if it hasn't wrapped yet.
    size_t startIdx = (g_eventCount <= FALL_EVENT_LOG_SIZE) ? 0 : g_eventHead;
    for (size_t i = 0; i < n; i++) {
        out[i] = g_eventLog[(startIdx + i) % FALL_EVENT_LOG_SIZE];
    }
    return n;
}

void clearFallLatch() {
    g_fallLatched = false;
    g_recoverOkSinceMs = 0;
    g_tiltExceedSinceMs = 0;
}

void calibrateUprightZero() {
    // Average a short burst of (low-pass) gravity samples for a stable
    // reference rather than trusting a single instantaneous reading.
    float sx = 0, sy = 0, sz = 0;
    const int N = 50;
    for (int i = 0; i < N; i++) {
        icm.getAGMT();
        sx += icm.accX() / 1000.0f;
        sy += icm.accY() / 1000.0f;
        sz += icm.accZ() / 1000.0f;
        delay(5);
    }
    sx /= N; sy /= N; sz /= N;
    float mag = sqrtf(sx * sx + sy * sy + sz * sz);
    if (mag < 0.1f) return; // sensor not producing sane data; leave old cal alone

    g0x = sx / mag; g0y = sy / mag; g0z = sz / mag;
    g_lpAx = sx; g_lpAy = sy; g_lpAz = sz;

    prefs.begin("imucal", false);
    prefs.putFloat("g0x", g0x);
    prefs.putFloat("g0y", g0y);
    prefs.putFloat("g0z", g0z);
    prefs.end();
}

void startMagCalibration() {
    magCalActive = true;
    magCalMinX = magCalMinY = magCalMinZ = 1e6f;
    magCalMaxX = magCalMaxY = magCalMaxZ = -1e6f;
}

void stopMagCalibration(bool save) {
    magCalActive = false;
    if (!save) return;
    if (magCalMaxX < magCalMinX) return; // no samples captured

    magOffX = (magCalMinX + magCalMaxX) / 2.0f;
    magOffY = (magCalMinY + magCalMaxY) / 2.0f;
    magOffZ = (magCalMinZ + magCalMaxZ) / 2.0f;

    prefs.begin("imucal", false);
    prefs.putFloat("magOffX", magOffX);
    prefs.putFloat("magOffY", magOffY);
    prefs.putFloat("magOffZ", magOffZ);
    prefs.end();
}

bool magCalibrationActive() { return magCalActive; }

void getMagCalPreview(float &minX, float &maxX, float &minY, float &maxY,
                       float &minZ, float &maxZ) {
    minX = magCalMinX; maxX = magCalMaxX;
    minY = magCalMinY; maxY = magCalMaxY;
    minZ = magCalMinZ; maxZ = magCalMaxZ;
}

} // namespace Imu
