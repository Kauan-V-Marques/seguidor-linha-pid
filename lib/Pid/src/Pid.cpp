#include "Pid.h"

Pid::Pid(float kp, float ki, float kd, float outMin, float outMax, float derivativeFilter)
    : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax), filter_(derivativeFilter) {}

void Pid::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::reset() {
    integral_ = 0.0f;
    derivative_ = 0.0f;
    previousError_ = 0.0f;
    hasPrevious_ = false;
}

float Pid::clamp(float value) const {
    if (value > outMax_) return outMax_;
    if (value < outMin_) return outMin_;
    return value;
}

float Pid::update(float error, float dtSeconds) {
    const float proportional = kp_ * error;

    if (dtSeconds <= 0.0f) {
        // Sem passagem de tempo não há como integrar nem derivar
        return clamp(proportional + integral_);
    }

    // Anti-windup: o acumulado fica preso nos limites da saída. Assim, depois de um
    // longo período saturado, o controle responde logo quando o erro muda de lado.
    integral_ = clamp(integral_ + ki_ * error * dtSeconds);

    if (hasPrevious_) {
        const float raw = kd_ * (error - previousError_) / dtSeconds;
        derivative_ = filter_ * derivative_ + (1.0f - filter_) * raw;
    }
    previousError_ = error;
    hasPrevious_ = true;

    return clamp(proportional + integral_ + derivative_);
}
