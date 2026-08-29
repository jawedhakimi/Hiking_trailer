#pragma once
//
// potcal.h — potentiometer endpoint calibration (min/max raw ADC counts),
// persisted to NVS. Extracted out of main.cpp so both the boot-time
// buzzer-guided flow AND the web UI's live calibration wizard can drive
// the same state instead of duplicating it.
//

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

namespace PotCal {

// Must be called once at boot after ads.begin() succeeds, before the main
// loop starts reading the pot.
void begin(Adafruit_ADS1115 *adsDevice);

// Loads a previously-saved calibration from flash. Returns false if
// nothing valid is stored yet (state is left at whatever it was).
bool load();

// Buzzer-guided two-point calibration (blocking): buzz, wait for the user
// to move to one end, sample; buzz again for the other end, sample; derive
// min/max/center from what was actually measured, save to flash. Returns
// false when the measured span is invalid; motor output remains gated off.
bool runBuzzerGuidedCalibration();

// --- Web UI live calibration wizard --------------------------------------
// Starts a staged calibration session. The existing valid calibration is
// kept until both new endpoints form a valid pair, but motor output remains
// locked for the entire session.
void startWebCalibration();
void cancelWebCalibration();
bool isWebCalibrationActive();
bool isWaitingForCenter();

// Called from the control loop. Returns true while motor output must remain
// locked. A completed/cancelled session only releases after the pot centers.
bool updateWebCalibrationLock(bool potCentered);

// Reads the pot for durationMs and returns the averaged raw ADC count,
// without touching any stored calibration — used by the web UI to show a
// live "current raw reading" number.
int32_t readAveraged(uint32_t durationMs);

// Thread-safe single ADC read for the main control loop.
int16_t readRaw();

// Captures a staged endpoint during an active web calibration. Once both
// endpoints form a valid span they are saved, but the motor remains locked
// until updateWebCalibrationLock() observes a centered pot.
bool captureEndpoint(int32_t raw, bool isMax);

// Directly sets both endpoints (e.g. from a JSON POST) and saves. Returns
// false (without saving) if the span is too small to trust.
bool setEndpoints(int32_t minRaw, int32_t maxRaw);

// --- Read access for main.cpp's control loop -----------------------------
int32_t centerRaw();
int32_t minRaw();
int32_t maxRaw();
int32_t maxOffsetRaw();
bool isValid();
bool hasCapturedMin();
bool hasCapturedMax();

} // namespace PotCal
