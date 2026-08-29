#pragma once
//
// imu.h — ICM20948 9-axis IMU (FSPI): tilt-compensated compass heading and
// fall detection.
//
// Fall detection uses the angle between the *current* measured gravity
// vector and the gravity vector captured during calibrateUprightZero() —
// that makes it independent of exactly how the IMU is mounted (it doesn't
// need to be dead level), which is what actually matters for "did this
// thing tip over", so it's used for the safety-critical trigger.
//
// The compass heading instead uses conventional pitch/roll tilt
// compensation, which assumes the IMU is mounted with its Z axis roughly
// vertical (PCB roughly horizontal) — true for a console-mounted board.
// If yours is mounted differently the heading number will be off; the
// fall-detection angle is unaffected either way.
//

#include <Arduino.h>
#include <stdint.h>

namespace Imu {

enum class FallTrigger : uint8_t { NONE = 0, TILT = 1, IMPACT = 2 };

struct FallEvent {
    uint32_t millisAt;
    FallTrigger trigger;
    float angleDeg;
};

// Attempts to detect and initialize the ICM20948 over SPI. Returns false
// (and leaves isPresent() == false) if it's not found / init fails — the
// caller decides whether that's fatal.
bool begin();

bool isPresent();

// Samples + fuses at IMU_SAMPLE_INTERVAL_MS; safe to call every loop()
// iteration. Returns true if a new sample was actually processed.
bool update();

// --- Live orientation -----------------------------------------------------
float pitchDeg();
float rollDeg();
float tiltDeg();          // angle from the calibrated "upright" reference
float headingDeg();       // 0..360, magnetic (or true, if declination set)
float accelMagnitudeG();  // for the impact-detection readout in the UI

// --- Fall / impact safety state --------------------------------------------
bool isFallen();
uint32_t fallEventCount();
size_t getFallEventLog(FallEvent *out, size_t maxCount); // oldest-first
void clearFallLatch(); // manual override; normally recovery is automatic

// --- Calibration ------------------------------------------------------
// Captures the current (averaged) gravity vector as the "upright" zero
// reference used by tiltDeg()/fall detection. Persisted to NVS.
void calibrateUprightZero();

// Magnetometer hard-iron calibration: call startMagCalibration(), have the
// user slowly rotate the device through all axes for ~15-20s, then
// stopMagCalibration(true) to compute and persist the offset (or
// stopMagCalibration(false) to cancel).
void startMagCalibration();
void stopMagCalibration(bool save);
bool magCalibrationActive();
// Live preview while a mag calibration is in progress (min/max seen so
// far on each axis), for the web UI's wizard.
void getMagCalPreview(float &minX, float &maxX, float &minY, float &maxY,
                       float &minZ, float &maxZ);

} // namespace Imu
