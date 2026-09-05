/* =========================================================================
 * hand_transmit.ino -- the data glove (transmitter half)
 *
 * Reads a wrist IMU plus up to five finger flex sensors, filters and
 * calibrates them, packs the result into a 9-byte frame and streams it to the
 * arm over a 433 MHz ASK link at the fastest rate the radio allows.
 *
 *   sensors --> median/EMA filter --> calibration --> 0..180 servo angles
 *                                       |
 *                                       +--> GloveProtocol frame --> radio
 *
 * Nothing in this file blocks: there is no delay() and no vw_wait_tx() in the
 * main loop.  Sampling, radio transmission, telemetry and the serial command
 * line each run on their own millis() schedule, so a slow radio can never
 * stall the sensor loop and a slow sensor read can never stall the radio.
 *
 * Wiring, configuration, calibration procedure and the protocol all live in
 * the `docs/` folder; every tunable is in common/GloveConfig.h.
 *
 * Part of the RoboticArm project.  See README.md.
 * ========================================================================= */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include "GlovePlatform.h"
#include "GloveConfig.h"
#include "GloveMath.h"
#include "GloveProtocol.h"
#include "SignalProcessing.h"
#include "GloveCalibration.h"
#include "Attitude.h"
#include "RfLink.h"

#if GLOVE_IMU_ENABLE
#include <I2Cdev.h>
#include <MPU6050.h>
#endif

#if defined(__AVR__) && GLOVE_WATCHDOG_ENABLE
#include <avr/wdt.h>
#define GLOVE_WDT_ACTIVE 1
#else
#define GLOVE_WDT_ACTIVE 0
#endif

/* ---------------------------------------------------------------- config */

/* Is there any way for an operator to talk to this glove?  The status dumps
 * and the calibration save are only reachable through the serial CLI or the
 * button, so a build with neither must not compile them (that would be dead
 * code *and* a warning). */
#if GLOVE_TX_SERIAL_CLI || (GLOVE_TX_BUTTON_PIN != 0)
#define GLOVE_TX_HAS_OPERATOR_INPUT 1
#else
#define GLOVE_TX_HAS_OPERATOR_INPUT 0
#endif

/* Sensitivities implied by the configured full-scale ranges. */
#define GLOVE_ACCEL_LSB_PER_G (32768 / GLOVE_IMU_ACCEL_FS_G)
#define GLOVE_GYRO_LSB_PER_DPS (32768 / GLOVE_IMU_GYRO_FS_DPS)

/* ------------------------------------------------------------- state --- */

#if GLOVE_IMU_ENABLE
static MPU6050 g_imu;
#endif

static glove::CalibrationData g_cal;
static glove::AnalogChannel g_flex[GLOVE_FINGER_CHANNELS];
static glove::AutoRanger g_flexRanger[GLOVE_FINGER_CHANNELS];
static glove::AutoRanger g_wristRanger[2];
static glove::AttitudeFilter g_attitude;
static glove::Button g_button;
static glove::Frame g_frame;

/* Per-channel state for the deadband and for telemetry. */
static int16_t g_flexRaw[GLOVE_FINGER_CHANNELS];
static int16_t g_lastSent[GLOVE_CHANNEL_COUNT];

static uint8_t g_seq = 0;
static bool g_imuOk = false;
static bool g_telemetry = (GLOVE_TELEMETRY_ENABLE != 0);
static bool g_capturing = false;

/* Which one-shot command the next frames should carry, and until when. */
static uint8_t g_command = GLOVE_CMD_NONE;
static uint32_t g_commandUntilMs = 0;

/* Scheduler state. */
static uint32_t g_lastFlexMs = 0;
static uint32_t g_lastImuMs = 0;
static uint32_t g_lastSendMs = 0;
static uint32_t g_lastTelemetryMs = 0;
static uint32_t g_captureStartMs = 0;

/* Link statistics. */
static uint32_t g_framesSent = 0;
static uint32_t g_framesDeferred = 0;

/* Loop-timing instrumentation.  g_loopMaxUs is the longest interval between two
 * entries into loop(), which is where any accidental blocking shows up: a stray
 * delay(), a full serial buffer, an I2C retry, a radio call that waits.  It is
 * reported by the 's' command and costs two uint32_t plus a micros() read. */
static uint32_t g_lastLoopUs = 0;
static uint32_t g_loopMaxUs = 0;

#if GLOVE_TX_SERIAL_CLI
static char g_line[24];
static uint8_t g_lineLen = 0;
#endif

/* --------------------------------------------------- forward declarations */
/* Declared explicitly rather than relying on the Arduino IDE's automatic
 * prototype generation, so this sketch also builds with plain avr-g++ or
 * PlatformIO and so the mutual recursion between reporting and calibration
 * is obvious. */
static int16_t readFlexRaw(uint8_t ch);
static void loadCalibration();
static void finishCapture();
static void printCalibration();
static void printTelemetry(uint32_t now);
static void sampleFlex(uint32_t now);
static void updateChannels();
static void sendFrame(uint32_t now);
static void handleButton(uint32_t now);
#if GLOVE_TX_HAS_OPERATOR_INPUT
static void triggerCommand(uint8_t command, uint32_t now);
static void saveCalibration();
static void startCapture();
static void printHelp();
static void printStatus();
static void printDump();
#endif
#if GLOVE_TX_BUTTON_PIN != 0
static void nextPreset(uint32_t now);
#endif
#if GLOVE_IMU_ENABLE
static void imuConfigure();
static bool imuInit();
static void imuCaptureBias();
static void imuSample(uint32_t dtMs);
#endif
#if GLOVE_TX_SERIAL_CLI
static bool parseArg(const char *line, long *value);
static void handleCommand(const char *line);
static void pollSerial();
#endif

/* ------------------------------------------------------- hardware reads */

/* Read one flex channel.  With the multiplexer profile this selects the 4051
 * channel first and waits for it to settle; otherwise it is a plain ADC read
 * of the pin assigned to that finger. */
static int16_t readFlexRaw(uint8_t ch) {
#if GLOVE_FLEX_MUX_ENABLE
    static const uint8_t kAddrPins[3] = GLOVE_FLEX_MUX_ADDR_PINS;
    static const uint8_t kMuxChannel[GLOVE_FLEX_COUNT] = GLOVE_FLEX_MUX_CHANNELS;
    if (ch >= GLOVE_FLEX_COUNT) return 0;
    const uint8_t sel = kMuxChannel[ch];
    for (uint8_t b = 0; b < 3; ++b) {
        digitalWrite(kAddrPins[b], (sel >> b) & 0x01);
    }
    delayMicroseconds(GLOVE_FLEX_MUX_SETTLE_US);
    return static_cast<int16_t>(analogRead(GLOVE_FLEX_MUX_SIGNAL_PIN));
#else
    static const uint8_t kPins[GLOVE_FLEX_COUNT] = GLOVE_FLEX_PINS;
    if (ch >= GLOVE_FLEX_COUNT) return 0;
    return static_cast<int16_t>(analogRead(kPins[ch]));
#endif
}

/* --------------------------------------------------------------- IMU --- */

#if GLOVE_IMU_ENABLE

/* Configure the MPU-6050 for gesture work: the narrowest full-scale ranges
 * (best resolution) and a 20 Hz digital low pass filter. */
static void imuConfigure() {
#if GLOVE_IMU_ACCEL_FS_G == 2
    g_imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
#elif GLOVE_IMU_ACCEL_FS_G == 4
    g_imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
#elif GLOVE_IMU_ACCEL_FS_G == 8
    g_imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
#else
    g_imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
#endif

#if GLOVE_IMU_GYRO_FS_DPS == 250
    g_imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
#elif GLOVE_IMU_GYRO_FS_DPS == 500
    g_imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
#elif GLOVE_IMU_GYRO_FS_DPS == 1000
    g_imu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
#else
    g_imu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
#endif

#if GLOVE_IMU_DLPF_HZ >= 188
    g_imu.setDLPFMode(1);
#elif GLOVE_IMU_DLPF_HZ >= 98
    g_imu.setDLPFMode(2);
#elif GLOVE_IMU_DLPF_HZ >= 42
    g_imu.setDLPFMode(3);
#elif GLOVE_IMU_DLPF_HZ >= 20
    g_imu.setDLPFMode(4);
#elif GLOVE_IMU_DLPF_HZ >= 10
    g_imu.setDLPFMode(5);
#else
    g_imu.setDLPFMode(6);
#endif
    g_imu.setRate(0); /* 1 kHz internal sampling, filtered by the DLPF */
    g_imu.setSleepEnabled(false);
}

/* Try to bring the IMU up.  A missing or mis-wired MPU-6050 must not stop the
 * glove from working: the fingers still teleoperate and the wrist channels
 * simply hold at neutral, with the IMU_OK flag cleared so the arm can tell. */
static bool imuInit() {
    for (uint8_t attempt = 1; attempt <= GLOVE_IMU_RETRY_COUNT; ++attempt) {
        g_imu.initialize();
        if (g_imu.testConnection()) {
            imuConfigure();
            return true;
        }
        Serial.print(F("[imu] no MPU-6050 answering on I2C (attempt "));
        Serial.print(attempt);
        Serial.println(F(")"));
        delay(GLOVE_IMU_RETRY_DELAY_MS);
    }
    Serial.println(F("[imu] giving up: check SDA/SCL, the AD0 strap and "
                     "that the profile in GloveConfig.h leaves A4/A5 free"));
    return false;
}

/* Average a burst of samples taken while the glove lies flat and still to
 * find the zero offsets.  The gyro bias is always taken; the accelerometer
 * bias is only taken when the rest vector really does look like 1 g pointing
 * up, otherwise the "bias" would swallow gravity and the wrist reference
 * would be meaningless. */
static void imuCaptureBias() {
    if (!g_imuOk) {
        Serial.println(F("[imu] not present, bias capture skipped"));
        return;
    }
    Serial.println(F("[imu] lay the glove FLAT and STILL for ~2 s ..."));

    int32_t sumA[3] = {0, 0, 0};
    int32_t sumG[3] = {0, 0, 0};
    int16_t ax, ay, az, gx, gy, gz;
    uint16_t n = 0;

    for (uint16_t i = 0; i < GLOVE_IMU_CAL_SAMPLES; ++i) {
        g_imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumA[0] += ax;
        sumA[1] += ay;
        sumA[2] += az;
        sumG[0] += gx;
        sumG[1] += gy;
        sumG[2] += gz;
        ++n;
        delay(2);
#if GLOVE_WDT_ACTIVE
        wdt_reset();
#endif
    }
    if (n == 0) return;

    const int16_t meanAx = static_cast<int16_t>(sumA[0] / n);
    const int16_t meanAy = static_cast<int16_t>(sumA[1] / n);
    const int16_t meanAz = static_cast<int16_t>(sumA[2] / n);

    g_cal.gyroBias[0] = static_cast<int16_t>(sumG[0] / n);
    g_cal.gyroBias[1] = static_cast<int16_t>(sumG[1] / n);
    g_cal.gyroBias[2] = static_cast<int16_t>(sumG[2] / n);

    /* Is the measured rest vector within ~10% of 1 g, and roughly upright? */
    const uint32_t magLsb = glove::isqrt32(
        static_cast<uint32_t>(meanAx) * static_cast<uint32_t>(meanAx) +
        static_cast<uint32_t>(meanAy) * static_cast<uint32_t>(meanAy) +
        static_cast<uint32_t>(meanAz) * static_cast<uint32_t>(meanAz));
    const uint32_t milliG = (magLsb * 1000u) / GLOVE_ACCEL_LSB_PER_G;
    /* More than 0.2 g of lateral acceleration means the glove was not lying
     * flat, so the rest vector cannot be used as an offset reference. */
    const int32_t absAx = (meanAx < 0) ? -static_cast<int32_t>(meanAx) : meanAx;
    const int32_t absAy = (meanAy < 0) ? -static_cast<int32_t>(meanAy) : meanAy;
    const int32_t lateralLimit =
        static_cast<int32_t>(GLOVE_ACCEL_LSB_PER_G) / 5;
    const bool lateral = (absAx > lateralLimit) || (absAy > lateralLimit);

    if (milliG >= 900 && milliG <= 1100 && !lateral) {
        g_cal.accelBias[0] = meanAx;
        g_cal.accelBias[1] = meanAy;
        g_cal.accelBias[2] = static_cast<int16_t>(meanAz - GLOVE_ACCEL_LSB_PER_G);
        Serial.print(F("[imu] accel offset captured at "));
        Serial.print(static_cast<uint32_t>(milliG));
        Serial.println(F(" mG"));
    } else {
        Serial.print(F("[imu] rest vector is "));
        Serial.print(static_cast<uint32_t>(milliG));
        Serial.println(F(" mG and/or not flat: gyro bias captured, accel "
                         "offset left alone"));
    }

    g_attitude.setGyroBias(g_cal.gyroBias[0], g_cal.gyroBias[1],
                           g_cal.gyroBias[2]);
    g_attitude.setAccelBias(g_cal.accelBias[0], g_cal.accelBias[1],
                            g_cal.accelBias[2]);
    Serial.println(F("[imu] bias loaded (use 'w' to store it in EEPROM)"));
}

/* One IMU sample + one complementary-filter step. */
static void imuSample(uint32_t dtMs) {
    int16_t ax, ay, az, gx, gy, gz;
    g_imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    g_attitude.update(ax, ay, az, gx, gy, gz, static_cast<uint16_t>(dtMs));

    if (g_capturing) {
        g_wristRanger[0].update(g_attitude.pitchDeg());
        g_wristRanger[1].update(g_attitude.rollDeg());
    }
}

#endif /* GLOVE_IMU_ENABLE */

/* ------------------------------------------------------- calibration --- */

static void loadCalibration() {
    EEPROM.get(GLOVE_CAL_EEPROM_ADDR, g_cal);
    if (!g_cal.isValid()) {
        g_cal.setDefaults();
        Serial.println(F("[cal] EEPROM empty or corrupt: using defaults. "
                         "Run 'c' then 'w' to calibrate."));
    } else {
        Serial.println(F("[cal] calibration loaded from EEPROM"));
    }
    g_attitude.setGyroBias(g_cal.gyroBias[0], g_cal.gyroBias[1],
                           g_cal.gyroBias[2]);
    g_attitude.setAccelBias(g_cal.accelBias[0], g_cal.accelBias[1],
                            g_cal.accelBias[2]);
}

#if GLOVE_TX_HAS_OPERATOR_INPUT

static void saveCalibration() {
    g_cal.updateChecksum();
    EEPROM.put(GLOVE_CAL_EEPROM_ADDR, g_cal);
    /* Read it back: EEPROM writes are worth verifying before you trust them. */
    glove::CalibrationData readBack;
    EEPROM.get(GLOVE_CAL_EEPROM_ADDR, readBack);
    if (readBack.isValid() && readBack.checksum == g_cal.checksum) {
        Serial.println(F("[cal] saved to EEPROM and verified"));
    } else {
        Serial.println(F("[cal] EEPROM VERIFY FAILED -- values kept in RAM "
                         "only for this session"));
    }
}

static void startCapture() {
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) g_flexRanger[i].reset();
    g_wristRanger[0].reset();
    g_wristRanger[1].reset();
    g_capturing = true;
    g_captureStartMs = millis();
    Serial.println(F("[cal] CAPTURING: slowly bend and straighten every "
                     "finger, then tilt your wrist fully up/down and "
                     "left/right"));
}

#endif /* GLOVE_TX_HAS_OPERATOR_INPUT */

static void finishCapture() {
    g_capturing = false;
    uint8_t accepted = 0;
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        if (i < GLOVE_FLEX_COUNT && g_cal.setFlexFromRanger(i, g_flexRanger[i])) {
            ++accepted;
        }
    }
#if GLOVE_IMU_ENABLE
    if (g_cal.setWristFromRanger(0, g_wristRanger[0])) ++accepted;
    if (g_cal.setWristFromRanger(1, g_wristRanger[1])) ++accepted;
#endif
    Serial.print(F("[cal] capture done: "));
    Serial.print(accepted);
    Serial.println(F(" channel(s) updated"));
    if (accepted == 0) {
        Serial.println(F("[cal] nothing was wide enough to believe -- are the "
                         "sensors connected? (see GLOVE_CAL_MIN_SPAN)"));
    } else {
        Serial.println(F("[cal] use 'w' to store it in EEPROM"));
    }
    printCalibration();
}

static void printCalibration() {
    Serial.println(F("[cal] finger spans (raw low..high, direction):"));
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        Serial.print(F("   ch"));
        Serial.print(i);
        Serial.print(F(": "));
        if (i >= GLOVE_FLEX_COUNT) {
            Serial.println(F("<no sensor on this profile>"));
            continue;
        }
        Serial.print(g_cal.flex[i].low);
        Serial.print(F(".."));
        Serial.print(g_cal.flex[i].high);
        Serial.print(F("  span="));
        Serial.print(g_cal.flex[i].high - g_cal.flex[i].low);
        Serial.print(g_cal.flexInverted(i) ? F("  falls-when-bent")
                                           : F("  rises-when-bent"));
        Serial.println(g_cal.flexUsable(i) ? F("") : F("  [UNUSABLE]"));
    }
    Serial.print(F("[cal] wrist pitch "));
    Serial.print(g_cal.wrist[0].low);
    Serial.print(F(".."));
    Serial.print(g_cal.wrist[0].high);
    Serial.print(F(" deg, roll "));
    Serial.print(g_cal.wrist[1].low);
    Serial.print(F(".."));
    Serial.print(g_cal.wrist[1].high);
    Serial.println(F(" deg"));
}

/* ---------------------------------------------------------- reporting --- */

#if GLOVE_TX_HAS_OPERATOR_INPUT

static void printHelp() {
    Serial.println(F("--- glove commands ---------------------------------"));
    Serial.println(F(" h        this help"));
    Serial.println(F(" s        status: config, calibration, radio, IMU"));
    Serial.println(F(" d        dump one set of raw + filtered values"));
    Serial.println(F(" c        start/stop the calibration capture"));
    Serial.println(F(" w        write calibration to EEPROM (verified)"));
    Serial.println(F(" r        load factory defaults into RAM"));
    Serial.println(F(" b        capture IMU zero bias (glove flat, still)"));
    Serial.println(F(" i <n>    flip the direction of finger channel n"));
    Serial.println(F(" p <n>    send preset n: 1 open, 2 fist, 3 home,"
                     " 4 point"));
    Serial.println(F(" t        toggle telemetry"));
    Serial.println(F("----------------------------------------------------"));
}

static void printStatus() {
    Serial.println(F("--- glove status -----------------------------------"));
    Serial.print(F(" profile      : "));
#if GLOVE_PROFILE == GLOVE_PROFILE_UNO_4FLEX
    Serial.println(F("UNO_4FLEX (4 flex + IMU)"));
#elif GLOVE_PROFILE == GLOVE_PROFILE_UNO_MUX
    Serial.println(F("UNO_MUX (5 flex via 74HC4051 + IMU)"));
#else
    Serial.println(F("MEGA_5FLEX (5 flex + IMU)"));
#endif
    Serial.print(F(" flex sensors : "));
    Serial.print(GLOVE_FLEX_COUNT);
    Serial.print(F(" of "));
    Serial.println(GLOVE_FINGER_CHANNELS);
#if GLOVE_FLEX_COUNT < GLOVE_FINGER_CHANNELS
    Serial.println(F(" NOTE         : channels without a sensor are sent as "
                     "the neutral angle (GLOVE_FLEX_ABSENT_ANGLE).  Use the "
                     "UNO_MUX or MEGA_5FLEX profile for all five fingers."));
#endif
    Serial.print(F(" radio        : "));
    Serial.print(rflink::driverName());
    Serial.print(F(" @ "));
    Serial.print(GLOVE_RF_SPEED_BPS);
    Serial.print(F(" bps, TX pin "));
    Serial.print(GLOVE_RF_TX_PIN);
    Serial.print(F(", frame "));
    Serial.print(GLOVE_FRAME_LEN);
    Serial.print(F(" B, air time ~"));
    Serial.print(rflink::frameAirTimeUs(GLOVE_FRAME_LEN) / 1000);
    Serial.println(F(" ms"));
    Serial.print(F(" protocol     : v"));
    Serial.println(GLOVE_PROTOCOL_VERSION);
    Serial.print(F(" imu          : "));
#if GLOVE_IMU_ENABLE
    Serial.println(g_imuOk ? F("MPU-6050 OK") : F("NOT RESPONDING (wrist held "
                                                  "at neutral)"));
#else
    Serial.println(F("disabled in GloveConfig.h"));
#endif
    Serial.print(F(" calibration  : "));
    Serial.println(g_cal.isValid() ? F("valid (from EEPROM)")
                                   : F("DEFAULTS (not saved yet)"));
    printCalibration();
    Serial.print(F(" frames       : "));
    Serial.print(g_framesSent);
    Serial.print(F(" sent, "));
    Serial.print(g_framesDeferred);
    Serial.println(F(" deferred because the radio was busy"));
    Serial.print(F(" loop max     : "));
    Serial.print(g_loopMaxUs);
    Serial.println(F(" us (longest gap between loop() entries)"));
    Serial.println(F("----------------------------------------------------"));
}

static void printDump() {
    Serial.print(F("raw:"));
    for (uint8_t i = 0; i < GLOVE_FLEX_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(g_flexRaw[i]);
    }
    Serial.print(F("  deg:"));
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(g_frame.channels[i]);
    }
#if GLOVE_IMU_ENABLE
    Serial.print(F("  wrist(p,r): "));
    Serial.print(g_attitude.pitchDeg());
    Serial.print(',');
    Serial.print(g_attitude.rollDeg());
#endif
    Serial.println();
}

#endif /* GLOVE_TX_HAS_OPERATOR_INPUT */

static void printTelemetry(uint32_t now) {
    (void)now;
#if GLOVE_TX_TELEMETRY_CSV
    /* Parsed by tools/glove_monitor.py.  Keep the column order in sync with
     * docs/FIRMWARE.md. */
    Serial.print(F("G,"));
    Serial.print(g_seq);
    Serial.print(',');
    Serial.print(g_imuOk ? 1 : 0);
    Serial.print(',');
    Serial.print(g_capturing ? 1 : 0);
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        Serial.print(',');
        Serial.print(g_frame.channels[i]);
    }
    Serial.print(',');
#if GLOVE_IMU_ENABLE
    Serial.print(g_attitude.pitchDeg());
    Serial.print(',');
    Serial.print(g_attitude.rollDeg());
#else
    Serial.print(F("0,0"));
#endif
    Serial.print(',');
    Serial.print(g_framesSent);
    Serial.print(',');
    Serial.println(g_framesDeferred);
#else
    Serial.print(F("ch:"));
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(g_frame.channels[i]);
    }
#if GLOVE_IMU_ENABLE
    Serial.print(F(" | wrist "));
    Serial.print(g_attitude.pitchDeg());
    Serial.print('/');
    Serial.print(g_attitude.rollDeg());
#endif
    Serial.print(F(" | tx "));
    Serial.print(g_framesSent);
    Serial.print('/');
    Serial.print(g_framesDeferred);
    if (g_capturing) Serial.print(F(" | CALIBRATING"));
    Serial.println();
#endif
}

/* ------------------------------------------------------------ sampling --- */

static void sampleFlex(uint32_t now) {
    (void)now;
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        if (i >= GLOVE_FLEX_COUNT) {
            g_flexRaw[i] = 0;
            continue;
        }
        const int16_t raw = readFlexRaw(i);
        g_flexRaw[i] = raw;
        const int16_t filtered = g_flex[i].update(raw);
        if (g_capturing) g_flexRanger[i].update(filtered);
    }
}

/* Turn the filtered sensor values into the seven channel angles that go on
 * the air.  Each channel is passed through a deadband against the last value
 * actually sent, so a resting servo is not asked to move one degree at a
 * time. */
static void updateChannels() {
    uint8_t angles[GLOVE_CHANNEL_COUNT];

#if GLOVE_IMU_ENABLE
    angles[GLOVE_CH_WRIST_PITCH] =
        static_cast<uint8_t>(g_cal.wristToAngle(0, g_attitude.pitchDeg()));
    angles[GLOVE_CH_WRIST_ROLL] =
        static_cast<uint8_t>(g_cal.wristToAngle(1, g_attitude.rollDeg()));
#else
    angles[GLOVE_CH_WRIST_PITCH] = 90;
    angles[GLOVE_CH_WRIST_ROLL] = 90;
#endif

    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        const uint8_t ch = static_cast<uint8_t>(GLOVE_CH_FINGER_FIRST + i);
        if (i >= GLOVE_FLEX_COUNT) {
            angles[ch] = GLOVE_FLEX_ABSENT_ANGLE;
        } else {
            angles[ch] =
                static_cast<uint8_t>(g_cal.flexToAngle(i, g_flex[i].value()));
        }
    }

    for (uint8_t ch = 0; ch < GLOVE_CHANNEL_COUNT; ++ch) {
        if (glove::outsideDeadband(g_lastSent[ch], angles[ch],
                                   GLOVE_DEADBAND_DEG)) {
            g_lastSent[ch] = angles[ch];
        }
        g_frame.channels[ch] = static_cast<uint8_t>(g_lastSent[ch]);
    }
}

static void sendFrame(uint32_t now) {
    g_frame.flags = 0;
    if (g_imuOk) g_frame.flags |= GLOVE_FLAG_IMU_OK;
    if (g_capturing) g_frame.flags |= GLOVE_FLAG_CAL_ACTIVE;
    g_frame.command = (static_cast<int32_t>(now - g_commandUntilMs) < 0)
                          ? g_command
                          : GLOVE_CMD_NONE;
    g_frame.seq = glove::nextSeq(g_seq);

    uint8_t buf[GLOVE_FRAME_LEN];
    const uint8_t len = g_frame.encode(buf, sizeof(buf));
    if (len == 0) return;

    /* Non-blocking: if the radio is still transmitting the previous frame we
     * simply keep the newer sample and try again next loop. */
    if (rflink::send(buf, len)) {
        g_seq = g_frame.seq; /* only count frames that actually left */
        ++g_framesSent;
    } else {
        ++g_framesDeferred;
    }
}

/* --------------------------------------------------------------- input --- */

#if GLOVE_TX_HAS_OPERATOR_INPUT

static void triggerCommand(uint8_t command, uint32_t now) {
    g_command = command;
    g_commandUntilMs = now + GLOVE_TX_CMD_REPEAT_MS;
}

#endif /* GLOVE_TX_HAS_OPERATOR_INPUT */

#if GLOVE_TX_BUTTON_PIN != 0
/* Cycle through the preset poses on a short press.  Presets are useful for
 * checking that the arm's poses are tuned, and for driving a joint that has
 * no sensor on a 4-flex profile. */
static void nextPreset(uint32_t now) {
    static uint8_t preset = GLOVE_CMD_POSE_OPEN;
    if (preset > GLOVE_CMD_POSE_POINT) preset = GLOVE_CMD_POSE_OPEN;
    triggerCommand(preset, now);
    Serial.print(F("[glove] preset "));
    Serial.println(preset);
    preset = static_cast<uint8_t>(preset + 1);
}
#endif /* GLOVE_TX_BUTTON_PIN != 0 */

static void handleButton(uint32_t now) {
#if GLOVE_TX_BUTTON_PIN != 0
    /* INPUT_PULLUP: the button connects the pin to GND, so "pressed" is a
     * low level.  No external resistor needed. */
    const bool pressed = (digitalRead(GLOVE_TX_BUTTON_PIN) == LOW);
    const uint8_t event = g_button.update(pressed, now);
    if (event == GLOVE_BTN_SHORT) {
        nextPreset(now);
    } else if (event == GLOVE_BTN_LONG) {
        if (g_capturing) {
            finishCapture();
        } else {
            startCapture();
        }
    }
#else
    (void)now;
#endif
}

#if GLOVE_TX_SERIAL_CLI

static bool parseArg(const char *line, long *value) {
    const char *p = line + 1;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p < '0' || *p > '9') {
        if (*p != '-') return false;
    }
    *value = strtol(p, NULL, 10);
    return true;
}

static void handleCommand(const char *line) {
    const uint32_t now = millis();
    long arg = 0;
    const bool hasArg = parseArg(line, &arg);

    switch (line[0]) {
    case 'h':
    case '?':
        printHelp();
        break;
    case 's':
        printStatus();
        break;
    case 'd':
        printDump();
        break;
    case 'c':
        if (g_capturing) {
            finishCapture();
        } else {
            startCapture();
        }
        break;
    case 'w':
        saveCalibration();
        break;
    case 'r':
        g_cal.setDefaults();
        g_attitude.setGyroBias(0, 0, 0);
        g_attitude.setAccelBias(0, 0, 0);
        Serial.println(F("[cal] factory defaults loaded into RAM "
                         "(use 'w' to store them)"));
        break;
    case 'b':
#if GLOVE_IMU_ENABLE
        imuCaptureBias();
#else
        Serial.println(F("[imu] disabled in this build"));
#endif
        break;
    case 'i':
        if (!hasArg || arg < 0 || arg >= GLOVE_FINGER_CHANNELS) {
            Serial.println(F("usage: i <channel 0..4>"));
            break;
        }
        g_cal.setFlexInverted(static_cast<uint8_t>(arg),
                              !g_cal.flexInverted(static_cast<uint8_t>(arg)));
        Serial.print(F("[cal] channel "));
        Serial.print(arg);
        Serial.println(g_cal.flexInverted(static_cast<uint8_t>(arg))
                           ? F(" now falls-when-bent")
                           : F(" now rises-when-bent"));
        break;
    case 'p':
        if (!hasArg || arg < GLOVE_CMD_POSE_OPEN || arg > GLOVE_CMD_POSE_POINT) {
            Serial.println(F("usage: p <1 open | 2 fist | 3 home | 4 point>"));
            break;
        }
        triggerCommand(static_cast<uint8_t>(arg), now);
        Serial.print(F("[glove] preset "));
        Serial.println(arg);
        break;
    case 't':
        g_telemetry = !g_telemetry;
        Serial.println(g_telemetry ? F("[telemetry] on") : F("[telemetry] off"));
        break;
    default:
        Serial.println(F("unknown command, 'h' for help"));
        break;
    }
}

static void pollSerial() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (g_lineLen > 0) {
                g_line[g_lineLen] = '\0';
                handleCommand(g_line);
                g_lineLen = 0;
            }
        } else if (g_lineLen < sizeof(g_line) - 1) {
            g_line[g_lineLen++] = c;
        } else {
            g_lineLen = 0; /* line too long: drop it and resynchronise */
        }
    }
}
#endif /* GLOVE_TX_SERIAL_CLI */

/* --------------------------------------------------------------- setup --- */

static void printBanner() {
    Serial.println();
    Serial.println(F("=== RoboticArm data glove (transmitter) ==="));
    Serial.print(F("protocol v"));
    Serial.print(GLOVE_PROTOCOL_VERSION);
    Serial.print(F(", radio "));
    Serial.print(rflink::driverName());
    Serial.print(F(" @ "));
    Serial.print(GLOVE_RF_SPEED_BPS);
    Serial.println(F(" bps"));
    Serial.println(F("type 'h' for the command list"));
}

void setup() {
#if GLOVE_WDT_ACTIVE
    /* Clear the watchdog reset flag first.  Without this the watchdog stays
     * armed through the bootloader after a watchdog reset and the board can
     * end up in a reset loop -- the classic AVR WDT trap. */
    MCUSR &= ~static_cast<uint8_t>(_BV(WDRF));
    wdt_disable();
#endif

    Serial.begin(GLOVE_SERIAL_BAUD);
    /* Give a USB-CDC serial port a moment to enumerate before the banner is
     * printed into the void.  Harmless on a hardware UART. */
    while (!Serial && millis() < 1500) {
    }

#if GLOVE_TX_BUTTON_PIN != 0
    pinMode(GLOVE_TX_BUTTON_PIN, INPUT_PULLUP);
    g_button.begin(GLOVE_TX_BUTTON_DEBOUNCE_MS, GLOVE_TX_BUTTON_LONG_MS);
#endif
#if GLOVE_TX_LED_PIN != 0
    pinMode(GLOVE_TX_LED_PIN, OUTPUT);
#endif
#if GLOVE_FLEX_MUX_ENABLE
    {
        static const uint8_t kAddrPins[3] = GLOVE_FLEX_MUX_ADDR_PINS;
        for (uint8_t b = 0; b < 3; ++b) pinMode(kAddrPins[b], OUTPUT);
    }
#endif

    loadCalibration();

    /* Seed the filters with a real reading instead of zero, so the first
     * frames are not a ramp from 0. */
    for (uint8_t i = 0; i < GLOVE_FINGER_CHANNELS; ++i) {
        const int16_t seed = (i < GLOVE_FLEX_COUNT) ? readFlexRaw(i) : 0;
        g_flexRaw[i] = seed;
        g_flex[i].begin(GLOVE_EMA_ALPHA_Q8, GLOVE_MEDIAN_ENABLE != 0, seed);
    }

#if GLOVE_IMU_ENABLE
    Wire.begin();
#if defined(WIRE_HAS_SET_CLOCK) || defined(ARDUINO_ARCH_AVR)
    Wire.setClock(GLOVE_IMU_I2C_HZ);
#endif
    g_imuOk = imuInit();
    g_attitude.begin(GLOVE_IMU_COMP_ALPHA_PCT, GLOVE_GYRO_LSB_PER_DPS,
                     GLOVE_ACCEL_LSB_PER_G, 0, 0);
    g_attitude.setGyroAxisSigns(GLOVE_ATTITUDE_ROLL_SIGN_DEFAULT,
                                GLOVE_ATTITUDE_PITCH_SIGN_DEFAULT);
    g_attitude.setGyroBias(g_cal.gyroBias[0], g_cal.gyroBias[1],
                           g_cal.gyroBias[2]);
    g_attitude.setAccelBias(g_cal.accelBias[0], g_cal.accelBias[1],
                            g_cal.accelBias[2]);
    if (g_imuOk) imuCaptureBias();
#endif

    for (uint8_t ch = 0; ch < GLOVE_CHANNEL_COUNT; ++ch) {
        g_lastSent[ch] = 90;
        g_frame.channels[ch] = 90;
    }
    g_frame.flags = 0;
    g_frame.command = GLOVE_CMD_NONE;

    rflink::begin();

    g_lastFlexMs = g_lastImuMs = g_lastSendMs = g_lastTelemetryMs = millis();

#if GLOVE_WDT_ACTIVE
    wdt_enable(GLOVE_WATCHDOG_TIMEOUT);
#endif

    printBanner();
    g_lastLoopUs = micros();
}

/* ---------------------------------------------------------------- loop --- */

void loop() {
    const uint32_t now = millis();
    /* Longest gap between loop() entries: the blocking detector.  Unsigned
     * subtraction, so the ~70 minute micros() wrap is handled. */
    const uint32_t nowUs = micros();
    const uint32_t loopUs = nowUs - g_lastLoopUs;
    g_lastLoopUs = nowUs;
    if (loopUs > g_loopMaxUs) g_loopMaxUs = loopUs;

#if GLOVE_WDT_ACTIVE
    wdt_reset();
#endif

#if GLOVE_TX_SERIAL_CLI
    pollSerial();
#endif
    handleButton(now);

    /* --- sensors, each on its own period ----------------------------- */
#if GLOVE_IMU_ENABLE
    if (static_cast<uint32_t>(now - g_lastImuMs) >= GLOVE_TX_IMU_SAMPLE_MS) {
        const uint32_t dt = now - g_lastImuMs;
        g_lastImuMs = now;
        if (g_imuOk) imuSample(dt);
    }
#endif

    if (static_cast<uint32_t>(now - g_lastFlexMs) >= GLOVE_TX_SAMPLE_MS) {
        g_lastFlexMs = now;
        sampleFlex(now);
    }

    /* --- calibration capture window ---------------------------------- */
    if (g_capturing &&
        static_cast<uint32_t>(now - g_captureStartMs) >= GLOVE_CAL_CAPTURE_MS) {
        finishCapture();
    }

    /* --- transmit the newest state, never blocking -------------------- */
    if (static_cast<uint32_t>(now - g_lastSendMs) >= GLOVE_TX_SEND_MS) {
        g_lastSendMs = now;
        updateChannels();
        sendFrame(now);
    }

    /* --- telemetry ---------------------------------------------------- */
    if (g_telemetry &&
        static_cast<uint32_t>(now - g_lastTelemetryMs) >= GLOVE_TX_TELEMETRY_MS) {
        g_lastTelemetryMs = now;
        printTelemetry(now);
    }

#if GLOVE_TX_LED_PIN != 0
    /* One short blink per frame that actually went out: a live link is
     * visible without a serial console. */
    static uint32_t lastSeenSent = 0;
    static uint32_t ledOnMs = 0;
    if (g_framesSent != lastSeenSent) {
        lastSeenSent = g_framesSent;
        if (ledOnMs == 0) {
            digitalWrite(GLOVE_TX_LED_PIN, HIGH);
            ledOnMs = now;
        }
    }
    if (ledOnMs != 0 && static_cast<uint32_t>(now - ledOnMs) >= 10) {
        digitalWrite(GLOVE_TX_LED_PIN, LOW);
        ledOnMs = 0;
    }
#endif
}
