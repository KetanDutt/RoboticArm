/*
 * GloveCalibration.h -- the calibration blob that makes a glove actually fit
 *                       the hand wearing it.
 *
 * The original firmware hard-coded magic numbers such as
 *     servoposition1 = map(flexposition1, 220,  90, 0, 180);
 *     servoposition3 = map(flexposition3,  90, 220, 0, 180);
 * which only ever worked for the exact resistor divider and sensor mounting
 * of one particular glove (and note the two directions -- two sensors were
 * fitted the other way round).  Every sensor, every divider and every
 * mounting differs, so calibration is now captured from the real hardware and
 * stored in EEPROM:
 *
 *   - per-finger raw span (low/high) learned by moving the finger through its
 *     full range,
 *   - per-finger direction, so a backwards-mounted sensor is fixed in
 *     software instead of with a soldering iron,
 *   - per-wrist physical angle window (pitch/roll),
 *   - IMU zero biases.
 *
 * The struct is a fixed-layout POD that is written to and read from EEPROM
 * verbatim, guarded by a magic and a checksum so a blank or half-written
 * EEPROM is detected and defaults are used instead.
 *
 * See docs/CALIBRATION.md for the operator procedure.
 *
 * Shared file: edit `common/GloveCalibration.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef GLOVE_CALIBRATION_H
#define GLOVE_CALIBRATION_H

#include <stdint.h>
#include <stddef.h>

#include "GlovePlatform.h"
#include "GloveConfig.h"
#include "GloveMath.h"
#include "SignalProcessing.h"

#define GLOVE_CAL_MAGIC0 'G'
#define GLOVE_CAL_MAGIC1 'C'
#define GLOVE_CAL_VERSION 1u

namespace glove {

/* One calibrated span.  `low` is always the numerically smaller raw value and
 * `high` the larger; the direction the sensor moves in is carried by the
 * invert masks, not by swapping these. */
struct ChannelCal {
    int16_t low;
    int16_t high;
};

struct CalibrationData {
    uint8_t magic0;          /* GLOVE_CAL_MAGIC0 */
    uint8_t magic1;          /* GLOVE_CAL_MAGIC1 */
    uint8_t version;         /* GLOVE_CAL_VERSION */
    uint8_t flexInvertMask;  /* bit n: finger n's raw value FALLS when bent */
    uint8_t wristInvertMask; /* bit 0: pitch, bit 1: roll */
    uint8_t reserved;        /* keep the following members 2-byte aligned */

    ChannelCal flex[GLOVE_FINGER_CHANNELS]; /* raw ADC counts */
    ChannelCal wrist[2];                    /* physical degrees (pitch, roll) */

    int16_t accelBias[3]; /* raw accel LSB with the glove flat and still */
    int16_t gyroBias[3];  /* raw gyro LSB with the glove still */

    uint16_t checksum; /* crc16Ccitt() over everything above */

    /* Fill with the compile-time defaults and compute the checksum. */
    void setDefaults();

    bool hasMagic() const;
    void updateChecksum();
    bool verifyChecksum() const;
    /* magic + version + checksum: safe to trust the numbers inside. */
    bool isValid() const;

    /* True when the stored span for a finger is wide enough to be real. */
    bool flexUsable(uint8_t ch) const;
    /* True when the stored wrist window is wide enough to be real. */
    bool wristUsable(uint8_t ch) const;

    /* Store a captured span; returns false (and changes nothing) when the
     * capture is too narrow to be believable. */
    bool setFlexFromRanger(uint8_t ch, const AutoRanger &ranger);
    bool setWristFromRanger(uint8_t ch, const AutoRanger &ranger);

    /* Convert a raw ADC count for finger `ch` into a servo angle 0..180. */
    int16_t flexToAngle(uint8_t ch, int16_t raw) const;
    /* Convert a physical wrist angle (deg) for axis `ch` into 0..180. */
    int16_t wristToAngle(uint8_t ch, int16_t degrees) const;

    bool flexInverted(uint8_t ch) const;
    bool wristInverted(uint8_t ch) const;
    void setFlexInverted(uint8_t ch, bool inverted);
    void setWristInverted(uint8_t ch, bool inverted);
};

} /* namespace glove */

/* The blob lives in a 1 KB EEPROM alongside anything else a future feature
 * might want to store, and it is written byte-by-byte.  Keep it small. */
static_assert(sizeof(glove::CalibrationData) <= 128,
              "CalibrationData must fit in 128 bytes of EEPROM");

#endif /* GLOVE_CALIBRATION_H */
