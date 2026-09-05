/*
 * Attitude.cpp -- fixed-point complementary filter implementation.
 *
 * Shared file: edit `common/Attitude.cpp` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#include "Attitude.h"

namespace glove {

namespace {

/* Clamp a bias-corrected accelerometer sample so the squares below stay
 * inside uint32/int32 and so an impact cannot masquerade as gravity. */
int32_t clampAccel(int32_t v) {
    return clampInt(v, -static_cast<int32_t>(GLOVE_ATTITUDE_ACCEL_CLAMP_LSB),
                    static_cast<int32_t>(GLOVE_ATTITUDE_ACCEL_CLAMP_LSB));
}

/* Integer division rounded half away from zero.  Plain truncation would bias
 * every negative rotation the same way, which shows up as a slow drift. */
int32_t divRound(int32_t num, int32_t den) {
    if (den == 0) return 0;
    const int32_t half = den / 2;
    if (num >= 0) return (num + half) / den;
    return -(((-num) + half) / den);
}

} /* namespace */

uint32_t isqrt32(uint32_t v) {
    uint32_t res = 0;
    uint32_t bit = 1u << 30;

    /* Start at the highest power of four not exceeding v. */
    while (bit > v) bit >>= 2;

    while (bit != 0) {
        if (v >= res + bit) {
            v -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

int32_t atan2Q8(int32_t y, int32_t x) {
    if (x == 0 && y == 0) return 0;

    int32_t ax = (x < 0) ? -x : x;
    int32_t ay = (y < 0) ? -y : y;

    /* Scale both down together until the Q10 ratio cannot overflow.  Only
     * kicks in for inputs far larger than a 16-bit ADC sample. */
    while (ax > (1 << 21) || ay > (1 << 21)) {
        ax >>= 1;
        ay >>= 1;
    }

    const bool swap = (ax < ay);
    const int32_t mn = swap ? ax : ay;
    const int32_t mx = swap ? ay : ax;

    /* z = min/max in Q10, so 0 <= z <= 1024. */
    const int32_t z = (mn << 10) / mx;
    const int32_t z2 = (z * z) >> 10;
    const int32_t z3 = (z2 * z) >> 10;

    /* Cubic fit of atan on [0,1] scaled to Q8 degrees:
     *     atan(zr) = 57.2958 zr - 4.3674 zr^2 - 7.9284 zr^3
     * exact at zr = 0 and zr = 1, maximum error ~0.3 deg in between. */
    int32_t q8 = (14668 * z - 1118 * z2 - 2030 * z3) >> 10;

    if (swap) q8 = 23040 - q8; /* complement into the second octant (90 deg) */
    if (y < 0) q8 = -q8;
    if (x < 0) q8 = (y >= 0) ? (46080 - q8) : (-46080 - q8);

    return q8;
}

int16_t q8ToDeg(int32_t q8) {
    if (q8 >= 0) return static_cast<int16_t>((q8 + 128) >> 8);
    return static_cast<int16_t>(-(((-q8) + 128) >> 8));
}

int32_t degToQ8(int16_t deg) {
    return static_cast<int32_t>(deg) * GLOVE_Q8_PER_DEG;
}

/* -------------------------------------------------------- attitude filter */

AttitudeFilter::AttitudeFilter()
    : gyroWeightPct_(97),
      gyroLsbPerDps_(131),
      accelLsbPerG_(16384),
      pitchQ8_(0),
      rollQ8_(0),
      pitchResid_(0),
      rollResid_(0) {
    gyroBias_[0] = gyroBias_[1] = gyroBias_[2] = 0;
    accelBias_[0] = accelBias_[1] = accelBias_[2] = 0;
    gyroSign_[0] = -1; /* roll  is driven by gx */
    gyroSign_[1] = -1; /* pitch is driven by gy */
}

void AttitudeFilter::begin(uint8_t gyroWeightPct, uint16_t gyroLsbPerDps,
                           uint16_t accelLsbPerG, int32_t startPitchQ8,
                           int32_t startRollQ8) {
    gyroWeightPct_ = (gyroWeightPct > 100) ? 100 : gyroWeightPct;
    gyroLsbPerDps_ = (gyroLsbPerDps == 0) ? 131 : gyroLsbPerDps;
    accelLsbPerG_ = (accelLsbPerG == 0) ? 16384 : accelLsbPerG;
    pitchQ8_ = startPitchQ8;
    rollQ8_ = startRollQ8;
    pitchResid_ = 0;
    rollResid_ = 0;
    /* begin() is a full reset: a caller must never inherit a bias or an axis
     * sign from a previous configuration. */
    gyroBias_[0] = gyroBias_[1] = gyroBias_[2] = 0;
    accelBias_[0] = accelBias_[1] = accelBias_[2] = 0;
    gyroSign_[0] = GLOVE_ATTITUDE_ROLL_SIGN_DEFAULT;
    gyroSign_[1] = GLOVE_ATTITUDE_PITCH_SIGN_DEFAULT;
}

void AttitudeFilter::setGyroBias(int16_t x, int16_t y, int16_t z) {
    gyroBias_[0] = x;
    gyroBias_[1] = y;
    gyroBias_[2] = z;
}

void AttitudeFilter::setAccelBias(int16_t x, int16_t y, int16_t z) {
    accelBias_[0] = x;
    accelBias_[1] = y;
    accelBias_[2] = z;
}

void AttitudeFilter::setGyroAxisSigns(int8_t rollSign, int8_t pitchSign) {
    gyroSign_[0] = (rollSign >= 0) ? 1 : -1;
    gyroSign_[1] = (pitchSign >= 0) ? 1 : -1;
}

uint16_t AttitudeFilter::maxDtMs() { return 100u; }

void AttitudeFilter::update(int16_t ax, int16_t ay, int16_t az, int16_t gx,
                            int16_t gy, int16_t gz, uint16_t dtMs) {
    (void)gz; /* yaw is deliberately not tracked: it would need a magnetometer
                 and would drift without one. */

    if (dtMs == 0) dtMs = 1;
    if (dtMs > maxDtMs()) dtMs = maxDtMs();

    /* Bias-correct, then clamp the accelerometer to +/-1.5 g.  Anything
     * larger is an impact or a fast hand movement, not gravity, and it also
     * keeps the squares below inside int32. */
    const int32_t cx = clampAccel(ax - accelBias_[0]);
    const int32_t cy = clampAccel(ay - accelBias_[1]);
    const int32_t cz = clampAccel(az - accelBias_[2]);
    const int32_t cgx = static_cast<int32_t>(gx) - gyroBias_[0];
    const int32_t cgy = static_cast<int32_t>(gy) - gyroBias_[1];

    /* --- accelerometer attitude (absolute but noisy) --- */
    const uint32_t sumSq = static_cast<uint32_t>(cy * cy) +
                           static_cast<uint32_t>(cz * cz);
    const int32_t horiz = static_cast<int32_t>(isqrt32(sumSq));
    const int32_t accelPitchQ8 = atan2Q8(-cx, horiz);
    const int32_t accelRollQ8 = atan2Q8(cy, cz);

    /* Trust the accelerometer only when the sensed magnitude is close to 1 g.
     * During a swing or a knock the vector is not gravity any more, so that
     * step falls back to pure gyro integration. */
    const uint32_t magLsb = isqrt32(sumSq + static_cast<uint32_t>(cx * cx));
    const uint32_t milliG =
        (magLsb * 1000u) / static_cast<uint32_t>(accelLsbPerG_);
    const bool accelTrusted = (milliG >= GLOVE_ATTITUDE_TRUST_MIN_MG) &&
                              (milliG <= GLOVE_ATTITUDE_TRUST_MAX_MG);

    /* --- gyro integration (smooth but drifting) ---
     * delta[Q8 deg] = raw/lsbPerDps * dt[s] * 256
     *               = raw * dt[ms] * 32 / (lsbPerDps * 125)
     * Rounded to nearest so a long run of small rates cannot bias downward. */
    const int32_t denom = static_cast<int32_t>(gyroLsbPerDps_) * 125;
    /* Carry the remainder of the previous step into this one so that rates
     * below one Q8 degree per sample still integrate instead of being
     * rounded away forever. */
    const int32_t numRoll =
        cgx * gyroSign_[0] * static_cast<int32_t>(dtMs) * 32 + rollResid_;
    const int32_t numPitch =
        cgy * gyroSign_[1] * static_cast<int32_t>(dtMs) * 32 + pitchResid_;
    const int32_t dRollQ8 = divRound(numRoll, denom);
    const int32_t dPitchQ8 = divRound(numPitch, denom);
    rollResid_ = numRoll - dRollQ8 * denom;
    pitchResid_ = numPitch - dPitchQ8 * denom;

    const int32_t gyroRollQ8 = rollQ8_ + dRollQ8;
    const int32_t gyroPitchQ8 = pitchQ8_ + dPitchQ8;

    const uint8_t w = accelTrusted ? gyroWeightPct_ : 100u;
    const uint8_t wa = static_cast<uint8_t>(100u - w);

    rollQ8_ = (static_cast<int32_t>(w) * gyroRollQ8 +
               static_cast<int32_t>(wa) * accelRollQ8) /
              100;
    pitchQ8_ = (static_cast<int32_t>(w) * gyroPitchQ8 +
                static_cast<int32_t>(wa) * accelPitchQ8) /
               100;
}

int32_t AttitudeFilter::pitchQ8() const { return pitchQ8_; }

int32_t AttitudeFilter::rollQ8() const { return rollQ8_; }

int16_t AttitudeFilter::pitchDeg() const { return q8ToDeg(pitchQ8_); }

int16_t AttitudeFilter::rollDeg() const { return q8ToDeg(rollQ8_); }

void AttitudeFilter::reset(int32_t pitchQ8, int32_t rollQ8) {
    pitchQ8_ = pitchQ8;
    rollQ8_ = rollQ8;
    pitchResid_ = 0;
    rollResid_ = 0;
}

} /* namespace glove */
