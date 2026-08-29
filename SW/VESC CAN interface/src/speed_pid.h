#pragma once
//
// speed_pid.h — small generic PID controller, used as a trim on top of the
// pot's feedforward ERPM mapping (see main.cpp): output = feedforward +
// clamp(PID(targetErpm, actualErpm), -maxTrim, +maxTrim).
//
// Kept generic/standalone (no VESC or settings knowledge) so it's easy to
// unit-reason about and to reuse if you ever want a second control loop.
//

#include <stdint.h>

class SpeedPid {
public:
    void reset();

    // dtSeconds must be > 0. maxOutput is the symmetric clamp applied to
    // the result (and used internally for integral anti-windup, so the
    // integral term can't keep accumulating past what the output clamp
    // would ever let through).
    float compute(float kp, float ki, float kd, float setpoint, float actual,
                  float dtSeconds, float maxOutput);

private:
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    bool havePrevError_ = false;
};
