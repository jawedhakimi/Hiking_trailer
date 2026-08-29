#include "odometry.h"
#include <Arduino.h>
#include <math.h>
#include "vesc_can.h"
#include "settings.h"
#include "config.h"

namespace Odometry {

namespace {
int32_t lastTacho = 0;
bool haveBaseline = false;
float tripM = 0.0f;
uint32_t lastTachoRxProcessedMs = 0;
uint32_t lastErpmIntegrationMs = 0;
float pendingErpmDistanceM = 0.0f;
bool erpmFallbackActive = true;

float distanceForElectricalRevs(float electricalRevs) {
    float poles = g_settings.motorPolePairs > 0 ? (float)g_settings.motorPolePairs : 1.0f;
    float gear = g_settings.gearRatio > 0.0001f ? g_settings.gearRatio : 1.0f;
    float wheelCircM = (g_settings.wheelDiameterMm / 1000.0f) * PI;
    return fabsf((electricalRevs / poles / gear) * wheelCircM);
}
} // namespace

void update() {
    uint32_t now = millis();
    const VescCan::StatusExtra &se = VescCan::getStatusExtra();
    const VescCan::Status1 &st = VescCan::getStatus1();

    // Continuously calculate a Status 1 fallback. While Status 5 is healthy,
    // keep it pending rather than adding it; the next exact tachometer delta
    // replaces it. If Status 5 never arrives or becomes stale, commit/use it.
    if (lastErpmIntegrationMs == 0) {
        lastErpmIntegrationMs = now;
    } else {
        uint32_t elapsedMs = now - lastErpmIntegrationMs;
        lastErpmIntegrationMs = now;
        bool status1Fresh = st.everReceived && (now - st.lastRxMillis) < VESC_STATUS_TIMEOUT_MS;
        // Cap a single interval so a long blocking operation or millis anomaly
        // cannot create a large false distance jump.
        if (status1Fresh && elapsedMs <= 1000) {
            float electricalRevs = (float)st.erpm * (elapsedMs / 60000.0f);
            float distanceM = distanceForElectricalRevs(electricalRevs);
            if (erpmFallbackActive || !se.tachometerEverReceived) tripM += distanceM;
            else pendingErpmDistanceM += distanceM;
        }
    }

    // Process each Status 5 frame once. After ERPM fallback operation, use the
    // first returning frame only as a new baseline so the same travel is not
    // counted a second time.
    bool newTachoFrame = se.tachometerEverReceived &&
                         se.status5LastRxMillis != lastTachoRxProcessedMs;
    if (newTachoFrame) {
        if (haveBaseline && !erpmFallbackActive) {
            // Unsigned subtraction followed by reinterpretation handles the
            // VESC's signed 32-bit tachometer wrapping at either limit.
            int32_t deltaSteps = (int32_t)((uint32_t)se.tachometerRaw - (uint32_t)lastTacho);
            tripM += distanceForElectricalRevs(deltaSteps / 6.0f);
        }
        lastTacho = se.tachometerRaw;
        haveBaseline = true;
        lastTachoRxProcessedMs = se.status5LastRxMillis;
        pendingErpmDistanceM = 0.0f;
        erpmFallbackActive = false;
    }

    // If Status 5 stops, the pending integrated distance covers the entire
    // gap since its last frame. Subsequent distance is added live via ERPM.
    if (se.tachometerEverReceived && !erpmFallbackActive &&
        (now - se.status5LastRxMillis) >= VESC_EXTRA_STATUS_TIMEOUT_MS) {
        tripM += pendingErpmDistanceM;
        pendingErpmDistanceM = 0.0f;
        erpmFallbackActive = true;
    }
}

float tripDistanceM() { return tripM; }
void resetTrip() {
    tripM = 0.0f;
    pendingErpmDistanceM = 0.0f;
    lastErpmIntegrationMs = millis();
    const VescCan::StatusExtra &se = VescCan::getStatusExtra();
    if (se.tachometerEverReceived) {
        lastTacho = se.tachometerRaw;
        lastTachoRxProcessedMs = se.status5LastRxMillis;
        haveBaseline = true;
    }
}

bool usingTachometer() {
    const VescCan::StatusExtra &se = VescCan::getStatusExtra();
    return se.tachometerEverReceived && !erpmFallbackActive &&
           (millis() - se.status5LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS;
}

bool hasDistanceSource() {
    uint32_t now = millis();
    const VescCan::Status1 &st = VescCan::getStatus1();
    const VescCan::StatusExtra &se = VescCan::getStatusExtra();
    return (st.everReceived && (now - st.lastRxMillis) < VESC_STATUS_TIMEOUT_MS) ||
           (se.tachometerEverReceived &&
            (now - se.status5LastRxMillis) < VESC_EXTRA_STATUS_TIMEOUT_MS);
}

} // namespace Odometry
