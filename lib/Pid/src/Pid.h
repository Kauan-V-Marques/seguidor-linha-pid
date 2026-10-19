#pragma once

// Controlador PID com:
//  - anti-windup: o termo integral nunca passa dos limites da saída
//  - filtro passa-baixa no termo derivativo (reduz o ruído dos sensores)
//  - saída limitada a [outMin, outMax]
class Pid {
public:
    Pid(float kp, float ki, float kd, float outMin, float outMax, float derivativeFilter = 0.0f);

    void setGains(float kp, float ki, float kd);
    void reset();

    // error: erro atual; dtSeconds: tempo desde a última chamada. Retorna a correção.
    float update(float error, float dtSeconds);

    float integral() const { return integral_; }
    float derivative() const { return derivative_; }

private:
    float clamp(float value) const;

    float kp_, ki_, kd_;
    float outMin_, outMax_;
    float filter_;  // 0 = sem filtro; perto de 1 = filtra muito
    float integral_ = 0.0f;
    float derivative_ = 0.0f;
    float previousError_ = 0.0f;
    bool hasPrevious_ = false;
};
