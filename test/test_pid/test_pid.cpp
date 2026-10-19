#include <unity.h>

#include "Pid.h"

void setUp() {}
void tearDown() {}

void test_proportional_only() {
    Pid pid(100.0f, 0.0f, 0.0f, -255.0f, 255.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, pid.update(0.5f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -25.0f, pid.update(-0.25f, 0.01f));
}

void test_output_is_clamped() {
    Pid pid(1000.0f, 0.0f, 0.0f, -255.0f, 255.0f);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, pid.update(1.0f, 0.01f));
    TEST_ASSERT_EQUAL_FLOAT(-255.0f, pid.update(-1.0f, 0.01f));
}

void test_integral_accumulates_error_over_time() {
    Pid pid(0.0f, 2.0f, 0.0f, -255.0f, 255.0f);
    float out = 0.0f;
    for (int i = 0; i < 10; i++) out = pid.update(0.5f, 0.1f);  // 2 x 0.5 x 1 s
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, out);
}

void test_anti_windup_keeps_integral_inside_limits() {
    Pid pid(0.0f, 100.0f, 0.0f, -255.0f, 255.0f);
    for (int i = 0; i < 1000; i++) pid.update(1.0f, 0.1f);  // saturado por muito tempo
    TEST_ASSERT_EQUAL_FLOAT(255.0f, pid.integral());

    // Quando o erro troca de lado, a saída começa a cair já no primeiro passo
    const float out = pid.update(-1.0f, 0.1f);
    TEST_ASSERT_TRUE(out < 255.0f);
}

void test_derivative_reacts_to_change() {
    Pid pid(0.0f, 0.0f, 1.0f, -255.0f, 255.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(0.0f, 0.5f));  // primeira leitura: sem derivada
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, pid.update(1.0f, 0.5f));  // (1 - 0) / 0.5
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.update(1.0f, 0.5f));  // erro parado
}

void test_derivative_filter_smooths_spikes() {
    Pid raw(0.0f, 0.0f, 1.0f, -255.0f, 255.0f, 0.0f);
    Pid filtered(0.0f, 0.0f, 1.0f, -255.0f, 255.0f, 0.5f);
    raw.update(0.0f, 0.1f);
    filtered.update(0.0f, 0.1f);

    const float spikeRaw = raw.update(1.0f, 0.1f);
    const float spikeFiltered = filtered.update(1.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, spikeRaw);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, spikeFiltered);
}

void test_reset_clears_memory() {
    Pid pid(0.0f, 10.0f, 1.0f, -255.0f, 255.0f);
    pid.update(1.0f, 0.1f);
    pid.update(0.5f, 0.1f);
    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(0.0f, 0.1f));
}

void test_zero_dt_does_not_integrate_or_derive() {
    Pid pid(10.0f, 10.0f, 10.0f, -255.0f, 255.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, pid.update(0.5f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_output_is_clamped);
    RUN_TEST(test_integral_accumulates_error_over_time);
    RUN_TEST(test_anti_windup_keeps_integral_inside_limits);
    RUN_TEST(test_derivative_reacts_to_change);
    RUN_TEST(test_derivative_filter_smooths_spikes);
    RUN_TEST(test_reset_clears_memory);
    RUN_TEST(test_zero_dt_does_not_integrate_or_derive);
    return UNITY_END();
}
