#include "buzzer.h"

#include <Arduino.h>

#include "config.h"

// Arduino-ESP32 core 3.x replaced the old channel-based LEDC API
// (ledcSetup/ledcAttachPin/ledcWrite(channel, ...)) with a new pin-based one
// (ledcAttach/ledcWrite/ledcWriteTone(pin, ...)) and dropped the old
// functions entirely. Which one is available depends on which core the
// project's `platform = espressif32` line resolves to, so this file targets
// whichever API the core actually provides instead of assuming the newer
// one — building against an old-API core with new-API calls (or vice versa)
// fails outright with "'ledcAttach' was not declared in this scope".
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define VESC_LEDC_NEW_API 1
#else
#define VESC_LEDC_NEW_API 0
#endif

namespace Buzzer {
namespace {

bool initialized = false;

#if !VESC_LEDC_NEW_API
constexpr uint8_t kLedcChannel = 0;
#endif

} // namespace

bool begin() {
    if (initialized) return true;

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
#if VESC_LEDC_NEW_API
    initialized = ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, 8);
#else
    ledcSetup(kLedcChannel, BUZZER_FREQ_HZ, 8);
    ledcAttachPin(PIN_BUZZER, kLedcChannel);
    initialized = true;
#endif
    if (!initialized) {
        Serial.printf("Buzzer: failed to attach LEDC to GPIO %d\n", PIN_BUZZER);
        return false;
    }
#if VESC_LEDC_NEW_API
    ledcWriteTone(PIN_BUZZER, 0);
#else
    ledcWriteTone(kLedcChannel, 0);
#endif
    return true;
}

void on() {
    if (!initialized && !begin()) return;
#if VESC_LEDC_NEW_API
    if (ledcWriteTone(PIN_BUZZER, BUZZER_FREQ_HZ) == 0) {
        Serial.println("Buzzer: failed to start tone");
    }
#else
    if (ledcWriteTone(kLedcChannel, BUZZER_FREQ_HZ) == 0) {
        Serial.println("Buzzer: failed to start tone");
    }
#endif
}

void off() {
    if (!initialized) return;
#if VESC_LEDC_NEW_API
    ledcWriteTone(PIN_BUZZER, 0);
#else
    ledcWriteTone(kLedcChannel, 0);
#endif
}

void beepBlocking(uint32_t durationMs) {
    on();
    delay(durationMs);
    off();
}

} // namespace Buzzer
