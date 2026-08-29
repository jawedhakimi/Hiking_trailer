#include "speed_pid.h"

void SpeedPid::reset() {
    integral_ = 0.0f;
    prevError_ = 0.0f;
    havePrevError_ = false;
}

float SpeedPid::compute(float kp, float ki, float kd, float setpoint, float actual,
                         float dtSeconds, float maxOutput) {
    if (dtSeconds <= 0.0f) dtSeconds = 0.001f;

    float error = setpoint - actual;

    integral_ += error * dtSeconds;
    // Anti-windup: clamp the integral term itself so ki*integral can never
    // exceed +/-maxOutput on its own, regardless of kp/kd's contribution.
    if (ki > 0.0001f) {
        float integralLimit = maxOutput / ki;
        if (integral_ > integralLimit) integral_ = integralLimit;
        if (integral_ < -integralLimit) integral_ = -integralLimit;
    } else {
        integral_ = 0.0f;
    }

    float derivative = havePrevError_ ? (error - prevError_) / dtSeconds : 0.0f;
    prevError_ = error;
    havePrevError_ = true;

    float output = kp * error + ki * integral_ + kd * derivative;

    if (output > maxOutput) output = maxOutput;
    if (output < -maxOutput) output = -maxOutput;
    return output;
}
