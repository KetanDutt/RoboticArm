/*
 * test_math.cpp -- GloveMath: clamping, re-mapping and checksums.
 */
#include <cstring>

#include "tests.h"

#include "GloveMath.h"
#include "GloveConfig.h"

using namespace glove;

void runMathTests() {
    SECTION("GloveMath: clampInt");
    CHECK_EQ(clampInt(50, 0, 180), 50);
    CHECK_EQ(clampInt(-5, 0, 180), 0);
    CHECK_EQ(clampInt(250, 0, 180), 180);
    /* Bounds given the wrong way round must still clamp, not produce junk. */
    CHECK_EQ(clampInt(250, 180, 0), 180);
    CHECK_EQ(clampInt(-250, 180, 0), 0);
    /* The receiver's mechanical window is the one that really matters. */
    CHECK_EQ(clampInt(0, GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX),
             GLOVE_RX_ANGLE_MIN);
    CHECK_EQ(clampInt(180, GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX),
             GLOVE_RX_ANGLE_MAX);

    SECTION("GloveMath: mapRange");
    CHECK_EQ(mapRange(0, 0, 180, 750, 2250), 750);
    CHECK_EQ(mapRange(180, 0, 180, 750, 2250), 2250);
    CHECK_EQ(mapRange(90, 0, 180, 750, 2250), 1500);
    /* Reversed input range -- exactly what a flex sensor that reads lower
     * when bent needs. */
    CHECK_EQ(mapRange(220, 220, 90, 0, 180), 0);
    CHECK_EQ(mapRange(90, 220, 90, 0, 180), 180);
    CHECK_EQ(mapRange(155, 220, 90, 0, 180), 90);
    /* Reversed output range. */
    CHECK_EQ(mapRange(0, 0, 180, 180, 0), 180);
    CHECK_EQ(mapRange(180, 0, 180, 180, 0), 0);
    /* Rounding instead of truncation. */
    CHECK_EQ(mapRange(1, 0, 3, 0, 100), 33);
    CHECK_EQ(mapRange(2, 0, 3, 0, 100), 67);
    /* Degenerate window must not divide by zero. */
    CHECK_EQ(mapRange(12345, 100, 100, 0, 180), 0);
    /* Values outside the calibrated window are NOT clamped by mapRange --
     * that is the caller's job, and the firmware does clamp. */
    CHECK(mapRange(1000, 90, 220, 0, 180) > 180);
    CHECK(mapRange(0, 90, 220, 0, 180) < 0);
    /* Large inputs must not overflow. */
    CHECK_EQ(mapRange(17000, 17000, -17000, 0, 180), 0);
    CHECK_EQ(mapRange(-17000, 17000, -17000, 0, 180), 180);
    CHECK_EQ(mapRange(0, 17000, -17000, 0, 180), 90);

    SECTION("GloveMath: crc16Ccitt");
    uint8_t a[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t b[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t ffs[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    CHECK_EQ(crc16Ccitt(a, sizeof(a)), crc16Ccitt(b, sizeof(b)));
    /* Every single-bit flip must be caught. */
    for (size_t i = 0; i < sizeof(a); ++i) {
        for (uint8_t bit = 0; bit < 8; ++bit) {
            uint8_t flipped[8];
            std::memcpy(flipped, a, sizeof(a));
            flipped[i] = static_cast<uint8_t>(flipped[i] ^ (1u << bit));
            CHECK(crc16Ccitt(flipped, sizeof(flipped)) !=
                  crc16Ccitt(a, sizeof(a)));
        }
    }
    /* Blank EEPROM (all 0xFF) and erased EEPROM (all 0x00) must be
     * distinguishable from each other -- a mod-255 Fletcher sum aliases
     * these two, which is exactly why a CRC is used here. */
    CHECK(crc16Ccitt(ffs, sizeof(ffs)) != crc16Ccitt(zeros, sizeof(zeros)));
    CHECK_EQ(crc16Ccitt(0, 8), 0u);
    /* Known answer for the reference implementation ("123456789"). */
    const uint8_t kat[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK_EQ(crc16Ccitt(kat, sizeof(kat)), 0x29B1u);

    SECTION("GloveMath: deltaU");
    CHECK_EQ(deltaU(10, 4), 6u);
    CHECK_EQ(deltaU(4, 10), 6u);
    CHECK_EQ(deltaU(7, 7), 0u);
}
