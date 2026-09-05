/*
 * main.cpp -- test runner for the shared RoboticArm core.
 */
#include "tests.h"

int g_checks = 0;
int g_failures = 0;

void reportFailure(const char *file, int line, const char *expr,
                   const char *detail) {
    ++g_failures;
    std::printf("  FAIL %s:%d  %s", file, line, expr);
    if (detail != 0 && detail[0] != '\0') std::printf("  (%s)", detail);
    std::printf("\n");
}

int main() {
    std::printf("RoboticArm shared-core unit tests\n");

    runMathTests();
    runProtocolTests();
    runSignalTests();
    runServoTests();
    runAttitudeTests();
    runCalibrationTests();

    std::printf("\n-----------------------------------------\n");
    std::printf("%d checks, %d failure%s\n", g_checks, g_failures,
                g_failures == 1 ? "" : "s");
    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("TESTS FAILED\n");
    return 1;
}
