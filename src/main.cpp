// Firmware do robô seguidor de linha.
// A lógica (PID, leitura da linha e estados) fica em lib/ e é testada no PC;
// este arquivo só liga essa lógica ao hardware.

#include <Arduino.h>

#include "Drive.h"
#include "LineSensor.h"
#include "Pid.h"
#include "RobotStateMachine.h"
#include "config.h"

LineSensor lineSensor(SENSOR_COUNT);
Pid pid(KP, KI, KD, -MAX_SPEED, MAX_SPEED, DERIVATIVE_FILTER);
RobotStateMachine robot;

uint16_t raw[LineSensor::MAX_SENSORS];
uint32_t lastLoopMs = 0;
uint32_t lastTelemetryMs = 0;

// ---------------------------------------------------------------- motores

void setMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int16_t speed) {
    digitalWrite(in1, speed >= 0 ? HIGH : LOW);
    digitalWrite(in2, speed >= 0 ? LOW : HIGH);
    analogWrite(pwmPin, constrain(abs(speed), 0, 255));
}

void drive(int16_t left, int16_t right) {
    digitalWrite(PIN_STBY, (left == 0 && right == 0) ? LOW : HIGH);
    setMotor(PIN_PWMA, PIN_AIN1, PIN_AIN2, left);
    setMotor(PIN_PWMB, PIN_BIN1, PIN_BIN2, right);
}

// ---------------------------------------------------------------- entradas

void readSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) raw[i] = analogRead(SENSOR_PINS[i]);
}

// Retorna true uma única vez por toque (borda de descida, botão com pull-up)
bool buttonPressed(uint32_t now) {
    static bool lastStable = HIGH;
    static bool lastRead = HIGH;
    static uint32_t changedAt = 0;

    const bool current = digitalRead(PIN_BUTTON);
    if (current != lastRead) {
        lastRead = current;
        changedAt = now;
    }
    if (now - changedAt >= DEBOUNCE_MS && current != lastStable) {
        lastStable = current;
        return current == LOW;
    }
    return false;
}

// ---------------------------------------------------------------- telemetria
// Formato CSV: dá para ver no Serial Plotter da IDE do Arduino

void telemetry(uint32_t now, const LineReading& line, float correction, const MotorSpeeds& speeds) {
    if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
    lastTelemetryMs = now;
    Serial.print(stateName(robot.state()));
    Serial.print(',');
    Serial.print(line.error, 3);
    Serial.print(',');
    Serial.print(correction, 1);
    Serial.print(',');
    Serial.print(speeds.left);
    Serial.print(',');
    Serial.println(speeds.right);
}

// ---------------------------------------------------------------- setup / loop

void setup() {
    Serial.begin(115200);
    const uint8_t outputs[] = {PIN_PWMA, PIN_AIN1, PIN_AIN2, PIN_PWMB, PIN_BIN1, PIN_BIN2, PIN_STBY, PIN_LED};
    for (uint8_t pin : outputs) pinMode(pin, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    drive(0, 0);
    robot.begin(millis());
    Serial.println(F("estado,erro,correcao,esquerda,direita"));
}

void loop() {
    const uint32_t now = millis();
    if (now - lastLoopMs < LOOP_PERIOD_MS) return;
    const float dt = (now - lastLoopMs) / 1000.0f;
    lastLoopMs = now;

    readSensors();
    if (robot.state() == RobotState::Calibrating) lineSensor.calibrate(raw);
    const LineReading line = lineSensor.read(raw);

    const RobotState previous = robot.state();
    RobotInputs inputs;
    inputs.nowMs = now;
    inputs.buttonPressed = buttonPressed(now);
    inputs.calibrated = lineSensor.isCalibrated();
    inputs.lineFound = line.found;
    const RobotState state = robot.update(inputs);
    if (state == RobotState::Running && previous != RobotState::Running) pid.reset();

    float correction = 0.0f;
    MotorSpeeds speeds = {0, 0};

    switch (state) {
        case RobotState::Calibrating:
            // Gira para um lado e para o outro, passando os sensores sobre a linha
            speeds = ((now / 600) % 2 == 0) ? MotorSpeeds{CALIBRATION_SPEED, -CALIBRATION_SPEED}
                                            : MotorSpeeds{-CALIBRATION_SPEED, CALIBRATION_SPEED};
            digitalWrite(PIN_LED, (now / 100) % 2);
            break;
        case RobotState::Ready:
            digitalWrite(PIN_LED, (now / 500) % 2);
            break;
        case RobotState::Running:
            correction = pid.update(line.error, dt);
            speeds = mixDrive(BASE_SPEED, correction, MAX_SPEED);
            digitalWrite(PIN_LED, HIGH);
            break;
        case RobotState::Searching:
            // O erro saturado (-1 ou +1) indica o lado em que a linha sumiu
            speeds = line.error < 0 ? MotorSpeeds{static_cast<int16_t>(-SEARCH_SPEED), SEARCH_SPEED}
                                    : MotorSpeeds{SEARCH_SPEED, static_cast<int16_t>(-SEARCH_SPEED)};
            break;
        case RobotState::Stopped:
            digitalWrite(PIN_LED, LOW);
            break;
    }

    drive(speeds.left, speeds.right);
    telemetry(now, line, correction, speeds);
}
