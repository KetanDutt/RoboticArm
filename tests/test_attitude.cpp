/*
 * test_attitude.cpp -- integer square root, fixed-point arctangent and the
 *                      complementary filter that replaced the original
 *                      "map the raw accelerometer onto a servo angle".
 */
#include "tests.h"

#include "Attitude.h"
#include "GloveConfig.h"

#include <cmath>

using namespace glove;

namespace {

/* Wrap an angle in degrees into (-180, +180]. */
double wrap180(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

} /* namespace */

void runAttitudeTests() {
    SECTION("isqrt32");
    CHECK_EQ(isqrt32(0), 0u);
    CHECK_EQ(isqrt32(1), 1u);
    CHECK_EQ(isqrt32(3), 1u);
    CHECK_EQ(isqrt32(4), 2u);
    CHECK_EQ(isqrt32(15), 3u);
    CHECK_EQ(isqrt32(16), 4u);
    CHECK_EQ(isqrt32(1000000), 1000u);
    CHECK_EQ(isqrt32(0xFFFFFFFEu), 65535u);
    for (uint32_t v = 0; v < 5000; ++v) {
        const uint32_t r = isqrt32(v);
        CHECK(r * r <= v);
        CHECK((r + 1) * (r + 1) > v);
    }

    SECTION("atan2Q8 on the axes");
    CHECK_EQ(atan2Q8(0, 1000), 0);
    CHECK_EQ(atan2Q8(1000, 0), 23040);   /* +90 deg */
    CHECK_EQ(atan2Q8(0, -1000), 46080);  /* 180 deg */
    CHECK_EQ(atan2Q8(-1000, 0), -23040); /* -90 deg */
    CHECK_EQ(atan2Q8(1000, 1000), 11520);     /* +45 deg */
    CHECK_EQ(atan2Q8(-1000, 1000), -11520);   /* -45 deg */
    CHECK_EQ(atan2Q8(1000, -1000), 34560);    /* +135 deg */
    CHECK_EQ(atan2Q8(-1000, -1000), -34560);  /* -135 deg */
    CHECK_EQ(atan2Q8(0, 0), 0);

    SECTION("atan2Q8 matches atan2f to well under one servo step");
    double worstError = 0.0;
    for (int deg = -180; deg <= 180; ++deg) {
        const double rad = deg * 3.14159265358979 / 180.0;
        const int32_t x = static_cast<int32_t>(std::cos(rad) * 16384.0);
        const int32_t y = static_cast<int32_t>(std::sin(rad) * 16384.0);
        const double got = q8ToDeg(atan2Q8(y, x));
        const double want = wrap180(static_cast<double>(deg));
        double err = std::fabs(got - want);
        if (err > 180.0) err = 360.0 - err; /* the 180/-180 seam */
        if (err > worstError) worstError = err;
        CHECK(err <= 1.0);
    }
    /* Also across magnitudes.  Below about 64 LSB the *inputs* themselves are
     * quantised to a handful of integer values, so the angle is meaningless
     * before atan2Q8 ever sees it; that is a property of a 16-bit sensor, not
     * of this function. */
    for (int mag = 64; mag <= 20000; mag *= 7) {
        for (int deg = -179; deg < 180; deg += 13) {
            const double rad = deg * 3.14159265358979 / 180.0;
            const int32_t x = static_cast<int32_t>(std::cos(rad) * mag);
            const int32_t y = static_cast<int32_t>(std::sin(rad) * mag);
            const double got = q8ToDeg(atan2Q8(y, x));
            const double want = wrap180(static_cast<double>(deg));
            double err = std::fabs(got - want);
            if (err > 180.0) err = 360.0 - err; /* the 180/-180 seam */
            CHECK(err <= 1.5);
        }
    }
    std::printf("  (worst atan2Q8 error over the sweep: %.2f deg)\n",
                worstError);

    SECTION("AttitudeFilter: level and still");
    AttitudeFilter att;
    const uint16_t gyroLsb = static_cast<uint16_t>(32768 / GLOVE_IMU_GYRO_FS_DPS);
    const uint16_t accelLsb = 16384; /* +/-2 g */
    att.begin(GLOVE_IMU_COMP_ALPHA_PCT, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 200; ++i) att.update(0, 0, 16384, 0, 0, 0, 10);
    CHECK_NEAR(att.pitchDeg(), 0, 1);
    CHECK_NEAR(att.rollDeg(), 0, 1);

    SECTION("AttitudeFilter: converges on a static tilt");
    /* ax = 16384 sin(30 deg), az = 16384 cos(30 deg) */
    att.begin(GLOVE_IMU_COMP_ALPHA_PCT, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 500; ++i) att.update(8192, 0, 14189, 0, 0, 0, 10);
    CHECK_NEAR(att.pitchDeg(), -30, 1);
    CHECK_NEAR(att.rollDeg(), 0, 1);

    att.begin(GLOVE_IMU_COMP_ALPHA_PCT, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 500; ++i) att.update(0, 8192, 14189, 0, 0, 0, 10);
    CHECK_NEAR(att.rollDeg(), 30, 1);
    CHECK_NEAR(att.pitchDeg(), 0, 1);

    SECTION("AttitudeFilter: gyro integration rate and sign");
    /* Pure gyro (weight 100%) at 100 deg/s about Y for 100 ms == 10 deg. */
    att.begin(100, gyroLsb, accelLsb, 0, 0);
    att.update(0, 0, 16384, 0, static_cast<int16_t>(100 * gyroLsb), 0, 100);
    CHECK_EQ(att.pitchDeg(), -10);
    CHECK_EQ(att.rollDeg(), 0);
    /* Axis signs are configurable: flip pitch and the same input goes up. */
    att.begin(100, gyroLsb, accelLsb, 0, 0);
    att.setGyroAxisSigns(-1, +1);
    att.update(0, 0, 16384, 0, static_cast<int16_t>(100 * gyroLsb), 0, 100);
    CHECK_EQ(att.pitchDeg(), 10);
    /* A sustained rate integrates, and dt is clamped so a stalled loop cannot
     * inject a huge step. */
    att.begin(100, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 10; ++i) {
        att.update(0, 0, 16384, static_cast<int16_t>(100 * gyroLsb), 0, 0, 10);
    }
    CHECK_EQ(att.rollDeg(), -10);
    att.begin(100, gyroLsb, accelLsb, 0, 0);
    att.update(0, 0, 16384, 0, static_cast<int16_t>(100 * gyroLsb), 0, 65535);
    CHECK_EQ(att.pitchDeg(), -10); /* clamped to maxDtMs() == 100 ms */
    CHECK_EQ(AttitudeFilter::maxDtMs(), 100u);

    SECTION("AttitudeFilter: rejects non-gravity accelerometer vectors");
    /* Free fall: |a| ~ 0 g.  The accelerometer knows nothing about attitude
     * here, so the estimate must be held by the gyros instead of being
     * dragged to a random angle. */
    att.begin(GLOVE_IMU_COMP_ALPHA_PCT, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 100; ++i) att.update(0, 0, 0, 0, 0, 0, 10);
    CHECK_NEAR(att.pitchDeg(), 0, 1);
    CHECK_NEAR(att.rollDeg(), 0, 1);
    /* Impact: |a| far above 1 g on several axes. */
    att.begin(GLOVE_IMU_COMP_ALPHA_PCT, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 100; ++i) {
        att.update(24000, 24000, 24000, 0, 0, 0, 10);
    }
    CHECK_NEAR(att.pitchDeg(), 0, 1);
    CHECK_NEAR(att.rollDeg(), 0, 1);

    SECTION("AttitudeFilter: gyro bias removes drift");
    /* A 131 LSB offset is 1 deg/s, i.e. 6 deg over the 6 s simulated here.
     * Loaded as a bias, a still glove must stay put; ignored, it drifts. */
    att.begin(GLOVE_IMU_COMP_ALPHA_PCT, gyroLsb, accelLsb, 0, 0);
    att.setGyroBias(0, 131, 0);
    for (int i = 0; i < 600; ++i) att.update(0, 0, 16384, 0, 131, 0, 10);
    CHECK_NEAR(att.pitchDeg(), 0, 1);

    att.begin(100, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 600; ++i) att.update(0, 0, 16384, 0, 131, 0, 10);
    CHECK_NEAR(att.pitchDeg(), -6, 1);

    SECTION("AttitudeFilter: slow rates integrate instead of rounding away");
    /* 20 LSB is ~0.15 deg/s.  Each 10 ms step moves the estimate by less than
     * one Q8 degree, so a filter without a residual carry would report zero
     * motion forever.  0.15 deg/s over 10 s is 1.5 deg. */
    att.begin(100, gyroLsb, accelLsb, 0, 0);
    for (int i = 0; i < 1000; ++i) att.update(0, 0, 16384, 0, 20, 0, 10);
    CHECK_NEAR(att.pitchQ8(), -384, 40); /* -1.5 deg in Q8 */

    SECTION("AttitudeFilter: reset");
    att.reset(degToQ8(45), degToQ8(-20));
    CHECK_EQ(att.pitchDeg(), 45);
    CHECK_EQ(att.rollDeg(), -20);
    CHECK_EQ(q8ToDeg(degToQ8(-90)), -90);
    CHECK_EQ(q8ToDeg(0), 0);
    CHECK_EQ(q8ToDeg(128), 1);  /* rounds half away from zero */
    CHECK_EQ(q8ToDeg(-128), -1);
}
