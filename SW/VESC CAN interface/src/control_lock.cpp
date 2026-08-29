#include "control_lock.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace ControlLock {
namespace {
SemaphoreHandle_t mutex = nullptr;
}

void begin() {
    if (!mutex) mutex = xSemaphoreCreateRecursiveMutex();
}

void lock() {
    if (mutex) xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
}

void unlock() {
    if (mutex) xSemaphoreGiveRecursive(mutex);
}

} // namespace ControlLock
