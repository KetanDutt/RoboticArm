/*
 * Attitude.h -- wrist orientation from an MPU-6050, without floating point.
 *
 * The original firmware turned the raw accelerometer into a servo angle with
 *     val1 = map(ax, 17000, -17000, 0, 179);
 * That has three problems, all visible on the arm:
 *   1. an accelerometer measures gravity *plus* every linear acceleration, so
 *      the wrist angle jumps whenever the hand moves,
 *   2. the mapping is only meaningful near +/-1 g; anything else is clamped
 *      into a wildly wrong angle (and, being stored in an int16 that was later
 *      squeezed into a uint8_t, negative values wrapped to 200+),
 *   3. no filtering at all, so the servo buzzed.
 *
 * Replaced here by a proper complementary filter:
 *
 *     angle = a * (angle + gyro * dt) + (1 - a) * accelAngle
 *
 * The gyro term is smooth and drifts; the accelerometer term is noisy and
 * absolute.  Fusing them (97/3 by default, GLOVE_IMU_COMP_ALPHA_PCT) gives a
 * wrist angle that is both steady and self-correcting.
 *
 * Everything is integer/fixed point (angles in 1/256 degree, "Q8"), including
 * the arctangent, so an ATmega328P spends microseconds rather than
 * milliseconds on it.  tests/test_attitude.cpp checks the fixed-point
 * arctangent against the host's atan2f to prove the approximation is good to
 * well under a servo step.
 *
 * Axis convention (MPU-6050 on the glove, X along the fingers, Y across the
 * hand, Z out of the back of the hand):
 *     pitch = atan2(-ax, sqrt(ay^2 + az^2))   nodding the wrist up/down
 *     roll  = atan2( ay, az )                 tilting the wrist side to side
 * If your board is mounted differently, fix it with the invert masks in the
 * calibration blob and the channel order in GloveConfig.h -- never by editing
 * this file.
 *
 * Shared file: edit `common/Attitude.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>
#include <stddef.h>

#include "GloveMath.h"

/* Only trust the accelerometer's "which way is down" when the sensed
 * magnitude is within this window around 1 g.  Outside it the hand is
 * accelerating and the vector is not gravity, so that step is integrated from
 * the gyroscopes alone. */
#define GLOVE_ATTITUDE_TRUST_MIN_MG 600
#define GLOVE_ATTITUDE_TRUST_MAX_MG 1600
/* Bias-corrected accelerometer samples are clamped to this many LSB
 * (24576 = 1.5 g at the +/-2 g full-scale range) before being squared. */
#define GLOVE_ATTITUDE_ACCEL_CLAMP_LSB 24576

/* Default gyro axis signs implied by the pitch/roll definitions above, for a
 * module mounted as documented.  Override at runtime with
 * AttitudeFilter::setGyroAxisSigns() (the sketches take their values from
 * GloveConfig.h). */
#define GLOVE_ATTITUDE_ROLL_SIGN_DEFAULT  (-1)
#define GLOVE_ATTITUDE_PITCH_SIGN_DEFAULT (-1)

namespace glove {

/* Integer square root (bit-by-bit).  Exact, no float, ~16 iterations. */
uint32_t isqrt32(uint32_t v);

/* atan2 in fixed-point Q8 degrees (1/256 deg), range (-46080, +46080].
 * Uses octant reduction plus a cubic fit that is exact at 0 and 90 degrees
 * and within about 0.3 degrees everywhere else.  atan2Q8(0, 0) == 0. */
int32_t atan2Q8(int32_t y, int32_t x);

/* Convert Q8 degrees to whole degrees, rounded half away from zero. */
int16_t q8ToDeg(int32_t q8);

/* Degrees to Q8. */
int32_t degToQ8(int16_t deg);

/* One degree in Q8 units. */
#define GLOVE_Q8_PER_DEG 256

class AttitudeFilter {
public:
    AttitudeFilter();

    /* gyroWeightPct   weight of the gyro integration, 0..100 (97 is typical)
     * gyroLsbPerDps   gyro sensitivity, e.g. 131 for a +/-250 dps range
     * accelLsbPerG    accel sensitivity, e.g. 16384 for a +/-2 g range
     * startPitchQ8/startRollQ8  initial estimate */
    void begin(uint8_t gyroWeightPct, uint16_t gyroLsbPerDps,
               uint16_t accelLsbPerG, int32_t startPitchQ8,
               int32_t startRollQ8);

    /* Zero offsets, in raw LSB (see GloveCalibration.h for the convention). */
    void setGyroBias(int16_t x, int16_t y, int16_t z);
    void setAccelBias(int16_t x, int16_t y, int16_t z);

    /* Which way each axis turns, +1 or -1.  Defaults are derived in
     * Attitude.cpp from the pitch/roll definitions above and are correct for
     * the documented mounting; flip one if your servo moves opposite to your
     * hand.  rollSign applies to gx, pitchSign to gy. */
    void setGyroAxisSigns(int8_t rollSign, int8_t pitchSign);

    /* Feed one raw MPU-6050 reading plus the time since the previous one.
     * dtMs is clamped internally so a stalled loop cannot inject a huge
     * integration step. */
    void update(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
                int16_t gz, uint16_t dtMs);

    int32_t pitchQ8() const;
    int32_t rollQ8() const;
    int16_t pitchDeg() const;
    int16_t rollDeg() const;

    void reset(int32_t pitchQ8, int32_t rollQ8);

    /* Longest dt that will be integrated, in ms. */
    static uint16_t maxDtMs();

private:
    uint8_t gyroWeightPct_;
    uint16_t gyroLsbPerDps_;
    uint16_t accelLsbPerG_;
    int16_t gyroBias_[3];
    int16_t accelBias_[3];
    int8_t gyroSign_[2]; /* [0] = roll/gx, [1] = pitch/gy */
    int32_t pitchQ8_;
    int32_t rollQ8_;
    /* Remainder of the last gyro integration step, in numerator units.  A
     * slow rotation (a fraction of a Q8 degree per sample) would otherwise be
     * rounded away to nothing every single time. */
    int32_t pitchResid_;
    int32_t rollResid_;
};

} /* namespace glove */

#endif /* ATTITUDE_H */
