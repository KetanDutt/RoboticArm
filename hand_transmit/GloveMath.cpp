/*
 * GloveMath.cpp -- implementation of the integer helpers in GloveMath.h.
 *
 * Shared file: edit `common/GloveMath.cpp` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#include "GloveMath.h"

namespace glove {

int32_t clampInt(int32_t v, int32_t lo, int32_t hi) {
    if (lo > hi) {
        const int32_t t = lo;
        lo = hi;
        hi = t;
    }
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int32_t mapRange(int32_t x, int32_t inMin, int32_t inMax, int32_t outMin,
                 int32_t outMax) {
    int32_t den = inMax - inMin;
    if (den == 0) {
        /* Degenerate calibration window: refuse to divide by zero and hand
         * back the bottom of the output range. */
        return outMin;
    }

    int32_t num = (x - inMin) * (outMax - outMin);

    /* Work with a positive denominator so the rounding step below is
     * symmetric, then round half away from zero. */
    if (den < 0) {
        den = -den;
        num = -num;
    }
    const int32_t half = den / 2;
    int32_t q;
    if (num >= 0) {
        q = (num + half) / den;
    } else {
        q = -((-num + half) / den);
    }
    return q + outMin;
}

uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
    if (data == 0) return 0;
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000u) != 0) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

uint32_t deltaU(uint32_t a, uint32_t b) { return (a > b) ? (a - b) : (b - a); }

} /* namespace glove */
