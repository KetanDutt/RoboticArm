/*
 * test_calibration.cpp -- the EEPROM calibration blob: defaults, validation,
 *                         capture and angle conversion.
 */
#include "tests.h"

#include "GloveCalibration.h"
#include "GloveProtocol.h"
#include "GloveConfig.h"
#include "SignalProcessing.h"

using namespace glove;

void runCalibrationTests() {
    SECTION("CalibrationData: layout is EEPROM safe");
    /* The checksum must be the last member and nothing may be padded after
     * it, because updateChecksum() hashes everything before it. */
    CHECK_EQ(offsetof(CalibrationData, checksum) + 2,
             sizeof(CalibrationData));
    CHECK(sizeof(CalibrationData) <= 128u); /* ATmega328P has 1024 B EEPROM */

    SECTION("CalibrationData: defaults are valid");
    CalibrationData cal;
    std::memset(&cal, 0xFF, sizeof(cal)); /* pretend the EEPROM is blank */
    CHECK(!cal.isValid());
    std::memset(&cal, 0x00, sizeof(cal));
    CHECK(!cal.isValid());

    cal.setDefaults();
    CHECK(cal.hasMagic());
    CHECK(cal.verifyChecksum());
    CHECK(cal.isValid());
    CHECK_EQ(cal.version, GLOVE_CAL_VERSION);
    CHECK_EQ(cal.flexInvertMask, GLOVE_FLEX_INVERT_MASK);
    CHECK_EQ(cal.wristInvertMask, GLOVE_WRIST_INVERT_MASK);
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        CHECK(cal.flexUsable(i));
        CHECK_EQ(cal.flex[i].low, GLOVE_FLEX_DEFAULT_RAW_MIN);
        CHECK_EQ(cal.flex[i].high, GLOVE_FLEX_DEFAULT_RAW_MAX);
    }
    CHECK(!cal.flexUsable(GLOVE_FINGER_CHANNELS)); /* out of range */
    CHECK(!cal.wristUsable(2));

    SECTION("CalibrationData: corruption is detected");
    cal.flex[2].high = 900;
    CHECK(!cal.verifyChecksum());
    CHECK(!cal.isValid());
    cal.updateChecksum(); /* an explicit re-save repairs it */
    CHECK(cal.isValid());
    CalibrationData tampered = cal;
    tampered.magic0 = 'X';
    CHECK(!tampered.isValid());
    tampered = cal;
    tampered.version = GLOVE_CAL_VERSION + 1;
    CHECK(!tampered.isValid());

    SECTION("CalibrationData: default flex mapping (inverted by default)");
    cal.setDefaults();
    CHECK(cal.flexInverted(0));
    /* Raw low == fully bent == 180 deg; raw high == straight == 0 deg. */
    CHECK_EQ(cal.flexToAngle(0, GLOVE_FLEX_DEFAULT_RAW_MIN), 180);
    CHECK_EQ(cal.flexToAngle(0, GLOVE_FLEX_DEFAULT_RAW_MAX), 0);
    const int16_t mid =
        static_cast<int16_t>((GLOVE_FLEX_DEFAULT_RAW_MIN +
                              GLOVE_FLEX_DEFAULT_RAW_MAX) / 2);
    CHECK_NEAR(cal.flexToAngle(0, mid), 90, 1);
    /* Way outside the calibrated span: clamped, never wrapped.  The original
     * firmware stored a negative map() result in a uint8_t and got 200+. */
    CHECK_EQ(cal.flexToAngle(0, 0), 180);
    CHECK_EQ(cal.flexToAngle(0, 4095), 0);
    /* Unknown channel index must be safe. */
    CHECK(cal.flexToAngle(99, 500) >= 0);
    CHECK(cal.flexToAngle(99, 500) <= 180);

    SECTION("CalibrationData: direction is fixable in software");
    cal.setFlexInverted(0, false);
    CHECK(!cal.flexInverted(0));
    CHECK_EQ(cal.flexToAngle(0, GLOVE_FLEX_DEFAULT_RAW_MIN), 0);
    CHECK_EQ(cal.flexToAngle(0, GLOVE_FLEX_DEFAULT_RAW_MAX), 180);
    cal.setFlexInverted(0, true);
    CHECK(cal.flexInverted(0));
    cal.setFlexInverted(99, false); /* out of range: no crash, no change */
    CHECK_EQ(cal.flexInvertMask, GLOVE_FLEX_INVERT_MASK);

    SECTION("CalibrationData: capturing a span");
    AutoRanger narrow;
    narrow.update(300);
    narrow.update(310);
    CHECK(!cal.setFlexFromRanger(0, narrow)); /* too narrow: rejected */
    CHECK_EQ(cal.flex[0].high, GLOVE_FLEX_DEFAULT_RAW_MAX);

    AutoRanger ranger;
    ranger.update(400);
    ranger.update(120);
    ranger.update(640);
    ranger.update(300);
    CHECK(cal.setFlexFromRanger(0, ranger));
    CHECK_EQ(cal.flex[0].low, 120);
    CHECK_EQ(cal.flex[0].high, 640);
    cal.updateChecksum();
    CHECK(cal.isValid());
    /* Inverted: the low end of the captured span is the bent end. */
    CHECK_EQ(cal.flexToAngle(0, 120), 180);
    CHECK_EQ(cal.flexToAngle(0, 640), 0);
    CHECK_NEAR(cal.flexToAngle(0, 380), 90, 1);

    SECTION("CalibrationData: a degenerate stored span falls back");
    cal.flex[1].low = 300;
    cal.flex[1].high = 300;
    CHECK(!cal.flexUsable(1));
    CHECK_EQ(cal.flexToAngle(1, GLOVE_FLEX_DEFAULT_RAW_MIN), 180);
    CHECK_EQ(cal.flexToAngle(1, GLOVE_FLEX_DEFAULT_RAW_MAX), 0);

    SECTION("CalibrationData: wrist windows");
    cal.setDefaults();
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, GLOVE_WRIST_PITCH_MIN_DEG), 0);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, GLOVE_WRIST_PITCH_MAX_DEG), 180);
    CHECK_NEAR(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, 0), 90, 1);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, -180), 0);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, 180), 180);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_ROLL, GLOVE_WRIST_ROLL_MIN_DEG), 0);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_ROLL, GLOVE_WRIST_ROLL_MAX_DEG), 180);
    /* Captured wrist window replaces the compile-time default. */
    AutoRanger wrist;
    wrist.update(-40);
    wrist.update(50);
    wrist.update(10);
    CHECK(cal.setWristFromRanger(GLOVE_CH_WRIST_PITCH, wrist));
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, -40), 0);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, 50), 180);
    /* Direction flip for a servo mounted the other way round. */
    cal.setWristInverted(GLOVE_CH_WRIST_PITCH, true);
    CHECK(cal.wristInverted(GLOVE_CH_WRIST_PITCH));
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, -40), 180);
    CHECK_EQ(cal.wristToAngle(GLOVE_CH_WRIST_PITCH, 50), 0);
    /* Roll is unaffected by a pitch flip. */
    CHECK(!cal.wristInverted(GLOVE_CH_WRIST_ROLL));
}
