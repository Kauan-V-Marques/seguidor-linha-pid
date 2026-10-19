#include "RobotStateMachine.h"

const char* stateName(RobotState state) {
    switch (state) {
        case RobotState::Calibrating: return "CALIBRANDO";
        case RobotState::Ready: return "PRONTO";
        case RobotState::Running: return "CORRENDO";
        case RobotState::Searching: return "PROCURANDO";
        case RobotState::Stopped: return "PARADO";
    }
    return "?";
}

RobotStateMachine::RobotStateMachine(RobotTimings timings) : timings_(timings) {}

void RobotStateMachine::begin(uint32_t nowMs) {
    enter(RobotState::Calibrating, nowMs);
}

void RobotStateMachine::enter(RobotState next, uint32_t nowMs) {
    state_ = next;
    enteredAt_ = nowMs;
    lineLost_ = false;
}

RobotState RobotStateMachine::update(const RobotInputs& in) {
    // Subtração em uint32_t continua correta quando o millis() dá a volta (~49 dias)
    const uint32_t inState = in.nowMs - enteredAt_;

    switch (state_) {
        case RobotState::Calibrating:
            if (in.calibrated && inState >= timings_.minCalibrationMs) enter(RobotState::Ready, in.nowMs);
            break;

        case RobotState::Ready:
            if (in.buttonPressed) enter(RobotState::Running, in.nowMs);
            break;

        case RobotState::Running:
            if (in.buttonPressed) {
                enter(RobotState::Stopped, in.nowMs);
            } else if (in.lineFound) {
                lineLost_ = false;
            } else if (!lineLost_) {
                lineLost_ = true;
                lineLostAt_ = in.nowMs;
            } else if (in.nowMs - lineLostAt_ >= timings_.lostToleranceMs) {
                enter(RobotState::Searching, in.nowMs);
            }
            break;

        case RobotState::Searching:
            if (in.buttonPressed) {
                enter(RobotState::Stopped, in.nowMs);
            } else if (in.lineFound) {
                enter(RobotState::Running, in.nowMs);
            } else if (inState >= timings_.searchTimeoutMs) {
                enter(RobotState::Stopped, in.nowMs);
            }
            break;

        case RobotState::Stopped:
            if (in.buttonPressed) enter(RobotState::Ready, in.nowMs);
            break;
    }
    return state_;
}
