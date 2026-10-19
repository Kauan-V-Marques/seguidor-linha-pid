#pragma once

#include <stdint.h>

// ----------------------------------------------------------------------------
// Pinos (ponte H TB6612FNG + barra de sensores analógicos tipo QTR)
// ----------------------------------------------------------------------------
#if defined(ESP32)
const uint8_t SENSOR_PINS[] = {36, 39, 34, 35, 32, 33, 25, 26};
const uint8_t PIN_PWMA = 16, PIN_AIN1 = 17, PIN_AIN2 = 5;   // motor esquerdo
const uint8_t PIN_PWMB = 18, PIN_BIN1 = 19, PIN_BIN2 = 21;  // motor direito
const uint8_t PIN_STBY = 22;
const uint8_t PIN_BUTTON = 0;  // botão BOOT da placa
const uint8_t PIN_LED = 2;
#else  // Arduino Uno / Nano
const uint8_t SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5};
const uint8_t PIN_PWMA = 5, PIN_AIN1 = 7, PIN_AIN2 = 8;
const uint8_t PIN_PWMB = 6, PIN_BIN1 = 4, PIN_BIN2 = 2;
const uint8_t PIN_STBY = 12;
const uint8_t PIN_BUTTON = 3;
const uint8_t PIN_LED = LED_BUILTIN;
#endif

const uint8_t SENSOR_COUNT = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

// ----------------------------------------------------------------------------
// Controle (o erro do sensor vai de -1 a +1; a saída é em unidades de PWM)
// Ajuste na pista: comece só com KP, depois aumente KD até parar de "serpentear".
// ----------------------------------------------------------------------------
const float KP = 140.0f;
const float KI = 0.0f;
const float KD = 6.0f;
const float DERIVATIVE_FILTER = 0.3f;

const int16_t BASE_SPEED = 150;   // 0..255
const int16_t MAX_SPEED = 255;
const int16_t SEARCH_SPEED = 110;
const int16_t CALIBRATION_SPEED = 90;

const uint16_t LOOP_PERIOD_MS = 4;  // 250 Hz
const uint16_t TELEMETRY_PERIOD_MS = 50;
const uint16_t DEBOUNCE_MS = 40;
