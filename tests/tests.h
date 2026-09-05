/*
 * tests.h -- a 40-line assertion framework.
 *
 * The firmware's decision-making code (protocol framing, filtering,
 * calibration maths, rate limiting, failsafe supervision) lives in `common/`
 * as plain C++ with no Arduino dependency, which means it can be compiled and
 * executed on a desktop.  These tests do exactly that: they are the reason a
 * change to the shared core can be trusted without putting hands near a
 * 7-servo arm.
 *
 * Run them with `make -C tests run`.  CI runs them on every push.
 */
#ifndef GLOVE_TESTS_H
#define GLOVE_TESTS_H

#include <cstdio>
#include <cstring>
#include <cmath>

extern int g_checks;
extern int g_failures;

void reportFailure(const char *file, int line, const char *expr,
                   const char *detail);

#define CHECK(expr)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(expr)) reportFailure(__FILE__, __LINE__, #expr, "");           \
    } while (0)

#define CHECK_EQ(actual, expected)                                           \
    do {                                                                     \
        ++g_checks;                                                          \
        const long a_ = static_cast<long>(actual);                           \
        const long e_ = static_cast<long>(expected);                         \
        if (a_ != e_) {                                                      \
            char detail_[96];                                                \
            std::snprintf(detail_, sizeof(detail_), "got %ld, want %ld", a_, \
                          e_);                                               \
            reportFailure(__FILE__, __LINE__, #actual " == " #expected,      \
                          detail_);                                          \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                              \
    do {                                                                     \
        ++g_checks;                                                          \
        const double a_ = static_cast<double>(actual);                       \
        const double e_ = static_cast<double>(expected);                     \
        if (std::fabs(a_ - e_) > static_cast<double>(tolerance)) {           \
            char detail_[96];                                                \
            std::snprintf(detail_, sizeof(detail_),                          \
                          "got %g, want %g +/- %g", a_, e_,                  \
                          static_cast<double>(tolerance));                   \
            reportFailure(__FILE__, __LINE__, #actual " ~= " #expected,      \
                          detail_);                                          \
        }                                                                    \
    } while (0)

#define SECTION(name) std::printf("\n[%s]\n", name)

/* Suite entry points. */
void runMathTests();
void runProtocolTests();
void runSignalTests();
void runServoTests();
void runAttitudeTests();
void runCalibrationTests();

#endif /* GLOVE_TESTS_H */
