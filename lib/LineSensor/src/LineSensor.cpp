#include "LineSensor.h"

LineSensor::LineSensor(uint8_t count, uint16_t detectionThreshold, uint16_t minimumRange)
    : count_(count > MAX_SENSORS ? MAX_SENSORS : count),
      threshold_(detectionThreshold),
      minimumRange_(minimumRange) {
    resetCalibration();
}

void LineSensor::resetCalibration() {
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        min_[i] = 0xFFFF;
        max_[i] = 0;
    }
    lastPosition_ = (count_ - 1) * 500;  // começa supondo a linha no centro
}

void LineSensor::calibrate(const uint16_t* raw) {
    for (uint8_t i = 0; i < count_; i++) {
        if (raw[i] < min_[i]) min_[i] = raw[i];
        if (raw[i] > max_[i]) max_[i] = raw[i];
    }
}

bool LineSensor::isCalibrated() const {
    // Cada sensor precisa ter visto o branco e o preto com diferença suficiente
    for (uint8_t i = 0; i < count_; i++) {
        if (max_[i] < min_[i] || max_[i] - min_[i] < minimumRange_) return false;
    }
    return true;
}

void LineSensor::normalize(const uint16_t* raw, uint16_t* out) const {
    for (uint8_t i = 0; i < count_; i++) {
        const int32_t range = static_cast<int32_t>(max_[i]) - min_[i];
        int32_t value = range > 0 ? (static_cast<int32_t>(raw[i]) - min_[i]) * 1000 / range : 0;
        if (value < 0) value = 0;
        if (value > 1000) value = 1000;
        out[i] = static_cast<uint16_t>(value);
    }
}

LineReading LineSensor::read(const uint16_t* raw) {
    uint16_t values[MAX_SENSORS];
    normalize(raw, values);

    int32_t weighted = 0;
    int32_t total = 0;
    bool found = false;
    for (uint8_t i = 0; i < count_; i++) {
        if (values[i] > threshold_) found = true;
        // Valores abaixo do limiar são ruído do fundo e não entram na média
        if (values[i] > threshold_ / 2) {
            weighted += static_cast<int32_t>(values[i]) * i * 1000;
            total += values[i];
        }
    }

    const int32_t maxPosition = (count_ - 1) * 1000;
    const int32_t center = maxPosition / 2;

    if (found) {
        lastPosition_ = weighted / total;
    } else {
        // Linha perdida: assume que ela saiu pelo lado em que foi vista por último,
        // para o robô continuar virando na direção certa
        lastPosition_ = lastPosition_ < center ? 0 : maxPosition;
    }

    LineReading reading;
    reading.found = found;
    reading.position = lastPosition_;
    reading.error = center > 0 ? static_cast<float>(lastPosition_ - center) / center : 0.0f;
    return reading;
}
