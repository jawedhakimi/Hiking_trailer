#pragma once

#include <stdint.h>

namespace DisplayApp {

struct Telemetry {
    float speedKmh = 0.0f;
    float batteryVoltage = 0.0f;
    float tripDistanceM = 0.0f;
    float headingDeg = 0.0f;
    float motorCurrentA = 0.0f;
    float dutyCycle = 0.0f;
    float fetTempC = 0.0f;
    float motorTempC = 0.0f;
    float fallAngleThresholdDeg = 0.0f;
    float potPositionPercent = 0.0f; // -100..100, from computePositionPercent()
    bool vescStatusFresh = false;
    bool batteryFresh = false;
    bool temperatureFresh = false;
    bool odometryAvailable = false;
    bool imuPresent = false;
    bool shaftPowerEnabled = false;
    bool fallen = false;
};

// Initializes the vendor ST7789/CST816S hardware and the generated
// SquareLine UI. Failure is non-fatal to the motor-control firmware.
bool begin();

// Services LVGL/touch every call and refreshes visible telemetry at the
// configured rate. Safe to call even if begin() failed.
void update(const Telemetry &telemetry);

bool isReady();

} // namespace DisplayApp
