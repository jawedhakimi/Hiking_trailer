#pragma once

#include <stdint.h>

namespace Buzzer {

// Reserve one LEDC channel once at startup. Keeping the channel attached is
// more reliable than tone()/noTone() repeatedly allocating and releasing it.
bool begin();
void on();
void off();
void beepBlocking(uint32_t durationMs);

} // namespace Buzzer
