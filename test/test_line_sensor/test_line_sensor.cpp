#include <unity.h>

#include "LineSensor.h"

void setUp() {}
void tearDown() {}

// 6 sensores: fundo branco lê ~100, linha preta lê ~900
static const uint16_t WHITE[6] = {100, 110, 95, 105, 100, 98};
static const uint16_t BLACK[6] = {900, 920, 880, 910, 905, 890};

static LineSensor calibratedSensor() {
    LineSensor sensor(6);
    sensor.calibrate(WHITE);
    sensor.calibrate(BLACK);
    return sensor;
}

void test_needs_calibration_before_use() {
    LineSensor sensor(6);
    TEST_ASSERT_FALSE(sensor.isCalibrated());
    sensor.calibrate(WHITE);
    TEST_ASSERT_FALSE(sensor.isCalibrated());  // ainda não viu a linha
    sensor.calibrate(BLACK);
    TEST_ASSERT_TRUE(sensor.isCalibrated());
}

void test_normalize_maps_white_to_0_and_black_to_1000() {
    LineSensor sensor = calibratedSensor();
    uint16_t out[6];

    sensor.normalize(WHITE, out);
    for (int i = 0; i < 6; i++) TEST_ASSERT_EQUAL_UINT16(0, out[i]);

    sensor.normalize(BLACK, out);
    for (int i = 0; i < 6; i++) TEST_ASSERT_EQUAL_UINT16(1000, out[i]);

    const uint16_t outside[6] = {0, 2000, 500, 500, 500, 500};
    sensor.normalize(outside, out);
    TEST_ASSERT_EQUAL_UINT16(0, out[0]);
    TEST_ASSERT_EQUAL_UINT16(1000, out[1]);
}

void test_line_in_the_middle_gives_zero_error() {
    LineSensor sensor = calibratedSensor();
    const uint16_t raw[6] = {100, 110, 880, 910, 100, 98};  // sensores 2 e 3

    LineReading reading = sensor.read(raw);

    TEST_ASSERT_TRUE(reading.found);
    TEST_ASSERT_INT32_WITHIN(30, 2500, reading.position);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, reading.error);
}

void test_line_on_the_edges() {
    LineSensor sensor = calibratedSensor();
    const uint16_t left[6] = {900, 110, 95, 105, 100, 98};
    const uint16_t right[6] = {100, 110, 95, 105, 100, 890};

    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, sensor.read(left).error);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, sensor.read(right).error);
}

void test_position_between_two_sensors_is_interpolated() {
    LineSensor sensor = calibratedSensor();
    const uint16_t raw[6] = {100, 110, 95, 910, 905, 98};  // entre os sensores 3 e 4

    TEST_ASSERT_INT32_WITHIN(30, 3500, sensor.read(raw).position);
}

void test_lost_line_keeps_last_side() {
    LineSensor sensor = calibratedSensor();
    const uint16_t rightSide[6] = {100, 110, 95, 105, 700, 890};
    sensor.read(rightSide);

    LineReading lost = sensor.read(WHITE);

    TEST_ASSERT_FALSE(lost.found);
    TEST_ASSERT_EQUAL_INT32(5000, lost.position);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, lost.error);
}

void test_small_noise_is_not_a_line() {
    LineSensor sensor = calibratedSensor();
    const uint16_t noisy[6] = {150, 160, 140, 170, 150, 145};  // até ~9% do caminho para o preto

    TEST_ASSERT_FALSE(sensor.read(noisy).found);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_needs_calibration_before_use);
    RUN_TEST(test_normalize_maps_white_to_0_and_black_to_1000);
    RUN_TEST(test_line_in_the_middle_gives_zero_error);
    RUN_TEST(test_line_on_the_edges);
    RUN_TEST(test_position_between_two_sensors_is_interpolated);
    RUN_TEST(test_lost_line_keeps_last_side);
    RUN_TEST(test_small_noise_is_not_a_line);
    return UNITY_END();
}
