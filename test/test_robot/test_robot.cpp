#include <unity.h>

#include "Drive.h"
#include "RobotStateMachine.h"

void setUp() {}
void tearDown() {}

static RobotInputs at(uint32_t ms, bool button = false, bool calibrated = true, bool line = true) {
    RobotInputs in;
    in.nowMs = ms;
    in.buttonPressed = button;
    in.calibrated = calibrated;
    in.lineFound = line;
    return in;
}

// Robô já calibrado e correndo a partir de t = 3000 ms
static RobotStateMachine runningRobot() {
    RobotStateMachine robot;
    robot.begin(0);
    robot.update(at(2600));
    robot.update(at(3000, true));
    return robot;
}

void test_calibration_needs_valid_readings_and_minimum_time() {
    RobotStateMachine robot;
    robot.begin(0);
    TEST_ASSERT_TRUE(robot.update(at(1000)) == RobotState::Calibrating);              // cedo demais
    TEST_ASSERT_TRUE(robot.update(at(3000, false, false)) == RobotState::Calibrating);  // sensores ruins
    TEST_ASSERT_TRUE(robot.update(at(3100)) == RobotState::Ready);
}

void test_button_starts_the_run() {
    RobotStateMachine robot = runningRobot();
    TEST_ASSERT_TRUE(robot.state() == RobotState::Running);
}

void test_short_line_loss_is_tolerated() {
    RobotStateMachine robot = runningRobot();
    robot.update(at(3100, false, true, false));
    TEST_ASSERT_TRUE(robot.update(at(3200, false, true, false)) == RobotState::Running);  // 100 ms
    TEST_ASSERT_TRUE(robot.update(at(3204)) == RobotState::Running);
    // a contagem recomeça depois que a linha volta
    robot.update(at(3300, false, true, false));
    TEST_ASSERT_TRUE(robot.update(at(3400, false, true, false)) == RobotState::Running);
}

void test_long_line_loss_starts_search_and_finding_it_resumes() {
    RobotStateMachine robot = runningRobot();
    robot.update(at(3100, false, true, false));
    TEST_ASSERT_TRUE(robot.update(at(3250, false, true, false)) == RobotState::Searching);
    TEST_ASSERT_TRUE(robot.update(at(3400, false, true, true)) == RobotState::Running);
}

void test_search_gives_up_and_stops_for_safety() {
    RobotStateMachine robot = runningRobot();
    robot.update(at(3100, false, true, false));
    robot.update(at(3250, false, true, false));
    TEST_ASSERT_TRUE(robot.update(at(4000, false, true, false)) == RobotState::Searching);
    TEST_ASSERT_TRUE(robot.update(at(4800, false, true, false)) == RobotState::Stopped);
}

void test_button_stops_and_rearms() {
    RobotStateMachine robot = runningRobot();
    TEST_ASSERT_TRUE(robot.update(at(5000, true)) == RobotState::Stopped);
    TEST_ASSERT_TRUE(robot.update(at(6000, true)) == RobotState::Ready);
    TEST_ASSERT_TRUE(robot.update(at(7000, true)) == RobotState::Running);
}

void test_timing_survives_millis_overflow() {
    RobotStateMachine robot;
    const uint32_t nearOverflow = 0xFFFFFFFFu - 1000u;
    robot.begin(nearOverflow);
    // 2600 ms depois, o millis() já deu a volta e vale ~1600
    TEST_ASSERT_TRUE(robot.update(at(nearOverflow + 2600u)) == RobotState::Ready);
}

void test_drive_mixing_turns_toward_the_line_and_clamps() {
    MotorSpeeds straight = mixDrive(150, 0.0f, 255);
    TEST_ASSERT_EQUAL_INT16(150, straight.left);
    TEST_ASSERT_EQUAL_INT16(150, straight.right);

    MotorSpeeds right = mixDrive(150, 60.0f, 255);  // linha à direita: roda esquerda mais rápida
    TEST_ASSERT_EQUAL_INT16(210, right.left);
    TEST_ASSERT_EQUAL_INT16(90, right.right);

    MotorSpeeds hard = mixDrive(150, 500.0f, 255);
    TEST_ASSERT_EQUAL_INT16(255, hard.left);
    TEST_ASSERT_EQUAL_INT16(-255, hard.right);
}

void test_state_names_for_telemetry() {
    TEST_ASSERT_EQUAL_STRING("CORRENDO", stateName(RobotState::Running));
    TEST_ASSERT_EQUAL_STRING("PROCURANDO", stateName(RobotState::Searching));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_calibration_needs_valid_readings_and_minimum_time);
    RUN_TEST(test_button_starts_the_run);
    RUN_TEST(test_short_line_loss_is_tolerated);
    RUN_TEST(test_long_line_loss_starts_search_and_finding_it_resumes);
    RUN_TEST(test_search_gives_up_and_stops_for_safety);
    RUN_TEST(test_button_stops_and_rearms);
    RUN_TEST(test_timing_survives_millis_overflow);
    RUN_TEST(test_drive_mixing_turns_toward_the_line_and_clamps);
    RUN_TEST(test_state_names_for_telemetry);
    return UNITY_END();
}
