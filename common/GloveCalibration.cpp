/*
 * GloveCalibration.cpp -- implementation of the calibration blob.
 *
 * Shared file: edit `common/GloveCalibration.cpp` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/CALIBRATION.md.
 */
#include "GloveCalibration.h"

namespace glove {

namespace {

/* Default span used whenever a stored one is missing or too narrow. */
ChannelCal defaultFlexCal() {
    ChannelCal c;
    c.low = static_cast<int16_t>(GLOVE_FLEX_DEFAULT_RAW_MIN);
    c.high = static_cast<int16_t>(GLOVE_FLEX_DEFAULT_RAW_MAX);
    if (c.low > c.high) {
        const int16_t t = c.low;
        c.low = c.high;
        c.high = t;
    }
    return c;
}

ChannelCal defaultWristCal(uint8_t axis) {
    ChannelCal c;
    if (axis == 0) {
        c.low = static_cast<int16_t>(GLOVE_WRIST_PITCH_MIN_DEG);
        c.high = static_cast<int16_t>(GLOVE_WRIST_PITCH_MAX_DEG);
    } else {
        c.low = static_cast<int16_t>(GLOVE_WRIST_ROLL_MIN_DEG);
        c.high = static_cast<int16_t>(GLOVE_WRIST_ROLL_MAX_DEG);
    }
    if (c.low > c.high) {
        const int16_t t = c.low;
        c.low = c.high;
        c.high = t;
    }
    return c;
}

} /* namespace */

void CalibrationData::setDefaults() {
    magic0 = GLOVE_CAL_MAGIC0;
    magic1 = GLOVE_CAL_MAGIC1;
    version = GLOVE_CAL_VERSION;
    flexInvertMask = static_cast<uint8_t>(GLOVE_FLEX_INVERT_MASK);
    wristInvertMask = static_cast<uint8_t>(GLOVE_WRIST_INVERT_MASK);
    reserved = 0;

    const ChannelCal flexDefault = defaultFlexCal();
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        flex[i] = flexDefault;
    }
    wrist[0] = defaultWristCal(0);
    wrist[1] = defaultWristCal(1);

    /* Accel/gyro zero offsets in raw LSB.  Convention:
     *   gyroBias  = average raw reading while the glove is still (ideal 0),
     *   accelBias = average raw reading while the glove lies FLAT with the
     *               PCB facing up, minus the ideal rest vector, which is
     *               (0, 0, +16384) LSB at a +/-2 g full-scale range.
     * Subtracting these removes sensor offset without removing gravity
     * itself, which the attitude filter needs as its reference. */
    accelBias[0] = 0;
    accelBias[1] = 0;
    accelBias[2] = 0;
    gyroBias[0] = 0;
    gyroBias[1] = 0;
    gyroBias[2] = 0;

    updateChecksum();
}

bool CalibrationData::hasMagic() const {
    return magic0 == GLOVE_CAL_MAGIC0 && magic1 == GLOVE_CAL_MAGIC1;
}

void CalibrationData::updateChecksum() {
    checksum = crc16Ccitt(reinterpret_cast<const uint8_t *>(this),
                          offsetof(CalibrationData, checksum));
}

bool CalibrationData::verifyChecksum() const {
    return checksum ==
           crc16Ccitt(reinterpret_cast<const uint8_t *>(this),
                      offsetof(CalibrationData, checksum));
}

bool CalibrationData::isValid() const {
    return hasMagic() && version == GLOVE_CAL_VERSION && verifyChecksum();
}

bool CalibrationData::flexUsable(uint8_t ch) const {
    if (ch >= GLOVE_FINGER_CHANNELS) return false;
    const int16_t span = static_cast<int16_t>(flex[ch].high - flex[ch].low);
    return span >= static_cast<int16_t>(GLOVE_CAL_MIN_SPAN);
}

bool CalibrationData::wristUsable(uint8_t ch) const {
    if (ch >= 2) return false;
    const int16_t span = static_cast<int16_t>(wrist[ch].high - wrist[ch].low);
    return span >= static_cast<int16_t>(GLOVE_CAL_MIN_SPAN);
}

bool CalibrationData::setFlexFromRanger(uint8_t ch, const AutoRanger &ranger) {
    if (ch >= GLOVE_FINGER_CHANNELS) return false;
    if (!ranger.valid(static_cast<int16_t>(GLOVE_CAL_MIN_SPAN))) return false;
    flex[ch].low = ranger.lowest();
    flex[ch].high = ranger.highest();
    return true;
}

bool CalibrationData::setWristFromRanger(uint8_t ch, const AutoRanger &ranger) {
    if (ch >= 2) return false;
    if (!ranger.valid(static_cast<int16_t>(GLOVE_CAL_MIN_SPAN))) return false;
    wrist[ch].low = ranger.lowest();
    wrist[ch].high = ranger.highest();
    return true;
}

bool CalibrationData::flexInverted(uint8_t ch) const {
    if (ch >= GLOVE_FINGER_CHANNELS) return false;
    return (flexInvertMask & static_cast<uint8_t>(1u << ch)) != 0;
}

bool CalibrationData::wristInverted(uint8_t ch) const {
    if (ch >= 2) return false;
    return (wristInvertMask & static_cast<uint8_t>(1u << ch)) != 0;
}

void CalibrationData::setFlexInverted(uint8_t ch, bool inverted) {
    if (ch >= GLOVE_FINGER_CHANNELS) return;
    const uint8_t bit = static_cast<uint8_t>(1u << ch);
    if (inverted) {
        flexInvertMask |= bit;
    } else {
        flexInvertMask = static_cast<uint8_t>(flexInvertMask & ~bit);
    }
}

void CalibrationData::setWristInverted(uint8_t ch, bool inverted) {
    if (ch >= 2) return;
    const uint8_t bit = static_cast<uint8_t>(1u << ch);
    if (inverted) {
        wristInvertMask |= bit;
    } else {
        wristInvertMask = static_cast<uint8_t>(wristInvertMask & ~bit);
    }
}

int16_t CalibrationData::flexToAngle(uint8_t ch, int16_t raw) const {
    ChannelCal cal = (ch < GLOVE_FINGER_CHANNELS && flexUsable(ch))
                         ? flex[ch]
                         : defaultFlexCal();

    int32_t angle = mapRange(raw, cal.low, cal.high, GLOVE_ANGLE_FLOOR,
                             GLOVE_ANGLE_CEILING);
    if (flexInverted(ch)) {
        angle = GLOVE_ANGLE_CEILING - angle;
    }
    return static_cast<int16_t>(
        clampInt(angle, GLOVE_ANGLE_FLOOR, GLOVE_ANGLE_CEILING));
}

int16_t CalibrationData::wristToAngle(uint8_t ch, int16_t degrees) const {
    ChannelCal cal = wristUsable(ch) ? wrist[ch] : defaultWristCal(ch);

    int32_t angle = mapRange(degrees, cal.low, cal.high, GLOVE_ANGLE_FLOOR,
                             GLOVE_ANGLE_CEILING);
    if (wristInverted(ch)) {
        angle = GLOVE_ANGLE_CEILING - angle;
    }
    return static_cast<int16_t>(
        clampInt(angle, GLOVE_ANGLE_FLOOR, GLOVE_ANGLE_CEILING));
}

} /* namespace glove */
