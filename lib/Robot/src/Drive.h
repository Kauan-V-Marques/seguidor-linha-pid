#pragma once

#include <stdint.h>

struct MotorSpeeds {
    int16_t left;
    int16_t right;
};

// Mistura a velocidade base com a correção do PID: correção positiva (linha à direita)
// acelera a roda esquerda e freia a direita, fazendo o robô virar para a direita.
inline MotorSpeeds mixDrive(int16_t base, float correction, int16_t maxSpeed) {
    auto limit = [maxSpeed](float value) -> int16_t {
        if (value > maxSpeed) return maxSpeed;
        if (value < -maxSpeed) return static_cast<int16_t>(-maxSpeed);
        return static_cast<int16_t>(value);
    };
    MotorSpeeds speeds;
    speeds.left = limit(base + correction);
    speeds.right = limit(base - correction);
    return speeds;
}
