#pragma once

// Serializes the real-time control loop with asynchronous web callbacks.
// ESPAsyncWebServer handlers run on a different FreeRTOS task, so settings,
// calibration state, and sensor buses must not be changed while loop() is
// using them to calculate a motor command.
namespace ControlLock {

void begin();
void lock();
void unlock();

class Guard {
public:
    Guard() { lock(); }
    ~Guard() { unlock(); }
    Guard(const Guard &) = delete;
    Guard &operator=(const Guard &) = delete;
};

} // namespace ControlLock
