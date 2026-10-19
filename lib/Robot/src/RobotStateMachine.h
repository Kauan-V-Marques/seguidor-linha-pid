#pragma once

#include <stdint.h>

enum class RobotState : uint8_t {
    Calibrating,  // girando no lugar para os sensores verem a linha e o fundo
    Ready,        // calibrado, esperando o botão
    Running,      // seguindo a linha com o PID
    Searching,    // perdeu a linha: gira para o lado em que ela foi vista por último
    Stopped,      // parado (botão ou linha perdida por tempo demais)
};

const char* stateName(RobotState state);

struct RobotInputs {
    uint32_t nowMs;
    bool buttonPressed;  // borda de subida (já com debounce)
    bool calibrated;
    bool lineFound;
};

struct RobotTimings {
    uint32_t minCalibrationMs = 2500;  // tempo mínimo girando na calibração
    uint32_t lostToleranceMs = 120;    // falha curta (cruzamento, linha falhada) não conta
    uint32_t searchTimeoutMs = 1500;   // depois disso o robô para por segurança
};

class RobotStateMachine {
public:
    explicit RobotStateMachine(RobotTimings timings = RobotTimings());

    void begin(uint32_t nowMs);
    RobotState update(const RobotInputs& in);
    RobotState state() const { return state_; }

private:
    void enter(RobotState next, uint32_t nowMs);

    RobotTimings timings_;
    RobotState state_ = RobotState::Calibrating;
    uint32_t enteredAt_ = 0;
    uint32_t lineLostAt_ = 0;
    bool lineLost_ = false;
};
