/*
 * GloveMath.h -- tiny integer helpers shared by the glove firmware and by
 *                the native unit tests.
 *
 * The Arduino core provides map() and constrain(), but they are (a) not
 * available to the desktop test build and (b) subtly unsafe: constrain()
 * misbehaves when the bounds are mixed types/expressions and map() divides by
 * zero when inMin == inMax.  These replacements are explicit, branch-safe and
 * tested.  They are also integer only -- no floating point anywhere in the
 * hot path, which matters on an ATmega328P with no FPU.
 *
 * Shared file: edit `common/GloveMath.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef GLOVE_MATH_H
#define GLOVE_MATH_H

#include <stdint.h>
#include <stddef.h>

namespace glove {

/* Clamp v into [lo, hi].  If lo > hi the bounds are swapped, so a typo in a
 * configuration header can never yield an out-of-range result. */
int32_t clampInt(int32_t v, int32_t lo, int32_t hi);

/* Re-map x from [inMin, inMax] onto [outMin, outMax], like Arduino's map()
 * but with:
 *   - reversed input ranges supported (inMin > inMax),
 *   - reversed output ranges supported,
 *   - a divide-by-zero guard (returns outMin when inMin == inMax),
 *   - rounding instead of truncation towards zero, which removes a small
 *     systematic bias when scaling sensor counts to degrees,
 *   - int32 intermediates so 16-bit inputs cannot overflow.
 * The result is NOT clamped to the output range (matching map()); call
 * clampInt() when you need that -- sensor values routinely fall outside the
 * calibrated window. */
int32_t mapRange(int32_t x, int32_t inMin, int32_t inMax, int32_t outMin,
                 int32_t outMax);

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor)
 * over a byte buffer, bitwise so it needs no 512-byte lookup table on an AVR.
 * Used to validate the calibration blob read back from EEPROM, where all-0xFF
 * (never written) and all-0x00 (erased) are both plausible and must not be
 * confused with each other or with real data -- a Fletcher/mod-255 sum would
 * alias those two cases. */
uint16_t crc16Ccitt(const uint8_t *data, size_t len);

/* Absolute difference without the overflow surprises of abs(a - b). */
uint32_t deltaU(uint32_t a, uint32_t b);

} /* namespace glove */

#endif /* GLOVE_MATH_H */
