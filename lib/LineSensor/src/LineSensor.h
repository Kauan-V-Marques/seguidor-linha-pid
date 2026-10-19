#pragma once

#include <stdint.h>

struct LineReading {
    bool found;        // false = nenhum sensor vê a linha
    int32_t position;  // 0 (linha no primeiro sensor) até (n - 1) * 1000
    float error;       // -1 (linha toda à esquerda) até +1 (toda à direita)
};

// Barra de sensores de refletância (ex.: QTR-8A). Cada sensor devolve um valor analógico
// alto sobre a linha preta e baixo sobre o fundo branco.
class LineSensor {
public:
    static const uint8_t MAX_SENSORS = 8;

    explicit LineSensor(uint8_t count, uint16_t detectionThreshold = 200, uint16_t minimumRange = 100);

    uint8_t count() const { return count_; }

    // Calibração: passe o robô sobre a linha e o fundo chamando calibrate() várias vezes
    void resetCalibration();
    void calibrate(const uint16_t* raw);
    bool isCalibrated() const;

    // Converte as leituras para 0 (branco) .. 1000 (preto) usando a calibração
    void normalize(const uint16_t* raw, uint16_t* out) const;

    // Média ponderada das posições dos sensores que veem a linha
    LineReading read(const uint16_t* raw);

private:
    uint8_t count_;
    uint16_t threshold_;
    uint16_t minimumRange_;
    uint16_t min_[MAX_SENSORS];
    uint16_t max_[MAX_SENSORS];
    int32_t lastPosition_;
};
