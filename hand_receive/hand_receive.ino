/* =========================================================================
 * hand_receive.ino -- the robotic arm / hand (receiver half)
 *
 * Receives the glove's frames, validates them, and drives seven servos
 * through ServoTimer2.  Between the radio and the servos sit three layers of
 * protection, all in common/ServoDrive.h and all unit tested:
 *
 *   1. Frame::decode()   rejects wrong length, wrong protocol version,
 *                        unknown commands, and clamps every channel into the
 *                        mechanical window (GLOVE_RX_ANGLE_MIN..MAX).
 *   2. SlewLimiter       a servo may only be commanded GLOVE_RX_SLEW_MAX_DPS
 *                        degrees per second, so no packet -- valid, corrupt or
 *                        malicious -- can slam a joint.
 *   3. angleToPulseUs()  converts degrees to the MICROSECONDS that
 *                        ServoTimer2::write() actually wants.  The original
 *                        firmware wrote degrees, which ServoTimer2 clamps to
 *                        its 750 us floor: every servo sat on one end stop and
 *                        ignored all data (docs/BUGS_FIXED.md, B-01).
 *
 * On top of that the receiver supervises the link: no valid frame for
 * GLOVE_RX_FAILSAFE_TIMEOUT_MS and it executes the configured failsafe
 * instead of holding the last command forever.
 *
 * Wiring, tuning and safety: docs/HARDWARE.md and docs/SAFETY.md.
 * Every tunable: common/GloveConfig.h.
 *
 * Part of the RoboticArm project.  See README.md.
 * ========================================================================= */

#include <Arduino.h>
#include <ServoTimer2.h>

#include "GlovePlatform.h"
#include "GloveConfig.h"
#include "GloveMath.h"
#include "GloveProtocol.h"
#include "ServoDrive.h"
#include "RfLink.h"

#if defined(__AVR__) && GLOVE_WATCHDOG_ENABLE
#include <avr/wdt.h>
#define GLOVE_WDT_ACTIVE 1
#else
#define GLOVE_WDT_ACTIVE 0
#endif

/* --------------------------------------------------------- sanity checks */
/* The channel-count and failsafe-timeout checks live in GloveProtocol.h so
 * the host tests enforce them too.  This one is a limit of the servo library
 * itself, so it belongs here. */
#if GLOVE_RX_SERVO_COUNT > 8
#error "ServoTimer2 drives at most 8 channels"
#endif

/* ------------------------------------------------------------- state --- */

/* Operating mode, reported over serial. */
#define MODE_BOOT     0
#define MODE_LIVE     1
#define MODE_POSE     2
#define MODE_PARKED   3
#define MODE_RELAXED  4
#define MODE_FAILSAFE 5
#define MODE_WAITING  6

static ServoTimer2 g_servo[GLOVE_RX_SERVO_COUNT];
static glove::SlewLimiter g_slew[GLOVE_RX_SERVO_COUNT];
static glove::LinkMonitor g_link;
static glove::CommandLatch g_command;
static glove::Frame g_frame;

static uint8_t g_target[GLOVE_RX_SERVO_COUNT];
static uint8_t g_mode = MODE_BOOT;
static bool g_servosAttached = false;
static bool g_linkLostReported = false;
/* Set by an explicit relax request (serial 'r', or a GLOVE_CMD_RELAX frame if
 * the configuration allows it).  While set, nothing -- not even link recovery
 * -- re-energises the servos; only 'a' or 'l' do. */
static bool g_relaxed = false;

static uint32_t g_bootMs = 0;
static uint32_t g_lastUpdateMs = 0;
static uint32_t g_lastTelemetryMs = 0;

static uint32_t g_framesAccepted = 0;
static uint32_t g_framesRejected = 0;

/* Loop-timing instrumentation.  g_loopMaxUs is the longest interval between two
 * entries into loop(), which is where any accidental blocking shows up: a stray
 * delay(), a full serial buffer, an I2C retry, a radio call that waits.  It is
 * reported by the 's' command and costs two uint32_t plus a micros() read. */
static uint32_t g_lastLoopUs = 0;
static uint32_t g_loopMaxUs = 0;

static bool g_telemetry = (GLOVE_TELEMETRY_ENABLE != 0);

static char g_line[16];
static uint8_t g_lineLen = 0;

/* Preset poses.  Tune these for YOUR mechanism: the angle that opens a finger
 * depends entirely on how each servo horn was splined onto its joint. */
static const uint8_t kPoseNeutral[GLOVE_RX_SERVO_COUNT] = GLOVE_RX_NEUTRAL_POSE;
static const uint8_t kPoseOpen[GLOVE_RX_SERVO_COUNT] = GLOVE_RX_OPEN_POSE;
static const uint8_t kPoseFist[GLOVE_RX_SERVO_COUNT] = GLOVE_RX_FIST_POSE;
static const uint8_t kPosePoint[GLOVE_RX_SERVO_COUNT] = GLOVE_RX_POINT_POSE;
static const uint8_t kServoPin[GLOVE_RX_SERVO_COUNT] = GLOVE_RX_SERVO_PINS;
static const uint8_t kServoInvert[GLOVE_RX_SERVO_COUNT] = GLOVE_RX_SERVO_INVERT;

/* --------------------------------------------------- forward declarations */
static void attachServos();
static void detachServos();
static void reportLinkChange(uint32_t now, bool linkLost);
static void copyPose(const uint8_t *pose);
static const uint8_t *poseForCommand(uint8_t command);
static const char *modeName(uint8_t mode);
static void applyServos(uint32_t now);
static void drainRadio(uint32_t now);
static void decideTargets(uint32_t now, bool linkLost);
static void printStatus();
static void printHelp();
static void printTelemetry();
static void handleCommand(const char *line);
static void pollSerial();

/* ------------------------------------------------------------- servos --- */

static void attachServos() {
    if (g_servosAttached) return;
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        g_servo[i].attach(kServoPin[i]);
        /* ServoTimer2 starts pulsing a freshly attached channel at its
         * 1500 us default.  Write the neutral pulse straight away and snap
         * the rate limiters to the same place, so software and hardware agree
         * about where the arm is. */
        const uint16_t us = glove::angleToPulseUs(
            kPoseNeutral[i], GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX,
            kServoInvert[i] ? GLOVE_RX_PULSE_MAX_US : GLOVE_RX_PULSE_MIN_US,
            kServoInvert[i] ? GLOVE_RX_PULSE_MIN_US : GLOVE_RX_PULSE_MAX_US);
        g_servo[i].write(us);
        g_target[i] = kPoseNeutral[i];
        g_slew[i].snap(kPoseNeutral[i], millis());
    }
    g_servosAttached = true;
}

static void detachServos() {
    if (!g_servosAttached) return;
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) g_servo[i].detach();
    g_servosAttached = false;
}

static void copyPose(const uint8_t *pose) {
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        /* Poses come from the configuration, which a human edited: clamp them
         * into the mechanical window exactly like a received frame. */
        g_target[i] = static_cast<uint8_t>(glove::clampInt(
            pose[i], GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX));
    }
}

static const uint8_t *poseForCommand(uint8_t command) {
    switch (command) {
    case GLOVE_CMD_POSE_OPEN: return kPoseOpen;
    case GLOVE_CMD_POSE_FIST: return kPoseFist;
    case GLOVE_CMD_POSE_HOME: return kPoseNeutral;
    case GLOVE_CMD_POSE_POINT: return kPosePoint;
    default: return 0;
    }
}

/* Drive every servo towards its target, rate limited, converting to the pulse
 * width ServoTimer2 needs. */
static void applyServos(uint32_t now) {
    if (!g_servosAttached) return;
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        const int16_t angle =
            g_slew[i].update(static_cast<int16_t>(g_target[i]), now);
        /* Inverting a channel is done by swapping the pulse window, which
         * keeps the mapping linear and exact. */
        const uint16_t pulseLo =
            kServoInvert[i] ? GLOVE_RX_PULSE_MAX_US : GLOVE_RX_PULSE_MIN_US;
        const uint16_t pulseHi =
            kServoInvert[i] ? GLOVE_RX_PULSE_MIN_US : GLOVE_RX_PULSE_MAX_US;
        const uint16_t us = glove::angleToPulseUs(
            angle, GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX, pulseLo, pulseHi);
        g_servo[i].write(us);
    }
}

/* --------------------------------------------------------------- radio --- */

static void drainRadio(uint32_t now) {
    /* Take every frame that is waiting and keep the newest one: on a control
     * link, stale positions are worthless. */
    for (;;) {
        uint8_t buf[GLOVE_RX_BUFFER_LEN];
        /* *len is an in/out parameter for both VirtualWire and RadioHead: it
         * must hold the buffer size on EVERY call.  The original firmware set
         * it once in setup(), so after the first packet it only ever copied
         * that many bytes again. */
        uint8_t len = sizeof(buf);
        if (!rflink::receive(buf, &len)) break;

        glove::Frame decoded;
        if (glove::Frame::decode(buf, len, GLOVE_RX_ANGLE_MIN,
                                 GLOVE_RX_ANGLE_MAX, decoded)) {
            g_frame = decoded;
            g_framesAccepted++;
            g_link.packetReceived(now, decoded.seq);
            g_command.offer(decoded.command, now);
            if (decoded.command == GLOVE_CMD_RESUME_LIVE) g_relaxed = false;
        } else {
            /* FCS was good but the frame is not ours: wrong length, wrong
             * protocol version or an unknown command.  Almost always means the
             * two boards are running different firmware. */
            g_framesRejected++;
            if (g_framesRejected <= 3 && g_telemetry) {
                Serial.print(F("[rx] rejected frame: len="));
                Serial.print(len);
                if (len == GLOVE_FRAME_LEN) {
                    Serial.print(F(" version="));
                    Serial.print(glove::metaVersion(buf[GLOVE_IDX_META]));
                    Serial.print(F(" (this board speaks v"));
                    Serial.print(GLOVE_PROTOCOL_VERSION);
                    Serial.print(')');
                }
                Serial.println(F(" -- are both boards on the same firmware?"));
            }
        }
    }
}

/* ----------------------------------------------------------- decisions --- */

static void decideTargets(uint32_t now, bool linkLost) {
    reportLinkChange(now, linkLost);

    /* 1. Power-on grace period: reach a known pose before obeying anything. */
    if (static_cast<uint32_t>(now - g_bootMs) < GLOVE_RX_STARTUP_POSE_MS) {
        g_mode = MODE_BOOT;
        copyPose(kPoseNeutral);
        return;
    }

    /* 2. An explicit relax request wins over everything, including link
     *    recovery.  Re-energising an arm that someone deliberately let go of
     *    is exactly the kind of surprise a controller must not produce. */
    if (g_relaxed) {
        g_mode = MODE_RELAXED;
        detachServos();
        return;
    }

    /* 3. Link supervision. */
    if (linkLost) {
        g_mode = MODE_FAILSAFE;
#if GLOVE_RX_FAILSAFE_ACTION == GLOVE_FAILSAFE_RELAX
        detachServos();
#elif GLOVE_RX_FAILSAFE_ACTION == GLOVE_FAILSAFE_NEUTRAL
        copyPose(kPoseNeutral);
#else
        /* HOLD: leave the targets exactly where they are.  The servos keep
         * their last commanded angle and their holding torque. */
#endif
        return;
    }

    /* The link is up.  Coming back from a loss the arm may have moved under
     * gravity (or been relaxed by the failsafe), so re-energise from the
     * neutral pose instead of slewing from wherever software thinks it is. */
    if (!g_servosAttached) attachServos();

    /* 4. Latched commands: presets, park and (optionally) relax. */
    const uint8_t command = g_command.active(now);
    if (command != GLOVE_CMD_NONE) {
        if (command == GLOVE_CMD_RELAX) {
#if GLOVE_RX_ALLOW_REMOTE_RELAX
            g_relaxed = true;
            detachServos();
            g_mode = MODE_RELAXED;
            return;
#else
            /* Ignored on purpose: a dropped or garbled frame must never be
             * able to let go of the arm.  Enable GLOVE_RX_ALLOW_REMOTE_RELAX
             * only on an unloaded or counterweighted rig. */
            g_command.clear();
#endif
        }
        if (command == GLOVE_CMD_PARK) {
            g_mode = MODE_PARKED;
            copyPose(kPoseNeutral);
            return;
        }
        const uint8_t *pose = poseForCommand(command);
        if (pose != 0) {
            g_mode = MODE_POSE;
            copyPose(pose);
            return;
        }
    }

    /* 5. Live teleoperation. */
    if (!g_link.everReceived()) {
        g_mode = MODE_WAITING;
        copyPose(kPoseNeutral);
        return;
    }
    g_mode = MODE_LIVE;
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        g_target[i] = g_frame.channels[i];
    }
}

/* Announce link loss and recovery exactly once per edge.  This is the single
 * most useful line the receiver prints when a build "does nothing". */
static void reportLinkChange(uint32_t now, bool linkLost) {
    if (!g_telemetry) {
        g_linkLostReported = linkLost;
        return;
    }
    if (linkLost && !g_linkLostReported) {
        g_linkLostReported = true;
        Serial.print(F("[rx] LINK LOST ("));
        Serial.print(g_link.msSincePacket(now));
        Serial.print(F(" ms since the last frame) -> failsafe "));
#if GLOVE_RX_FAILSAFE_ACTION == GLOVE_FAILSAFE_HOLD
        Serial.println(F("HOLD"));
#elif GLOVE_RX_FAILSAFE_ACTION == GLOVE_FAILSAFE_RELAX
        Serial.println(F("RELAX (servos detached)"));
#else
        Serial.println(F("NEUTRAL"));
#endif
    } else if (!linkLost && g_linkLostReported) {
        g_linkLostReported = false;
        Serial.println(F("[rx] link recovered"));
    }
}

/* ---------------------------------------------------------- reporting --- */

static const char *modeName(uint8_t mode) {
    switch (mode) {
    case MODE_BOOT: return "BOOT";
    case MODE_LIVE: return "LIVE";
    case MODE_POSE: return "PRESET";
    case MODE_PARKED: return "PARKED";
    case MODE_RELAXED: return "RELAXED";
    case MODE_FAILSAFE: return "FAILSAFE";
    case MODE_WAITING: return "WAITING";
    default: return "?";
    }
}

static void printHelp() {
    Serial.println(F("--- arm commands ----------------------------------"));
    Serial.println(F(" h   this help"));
    Serial.println(F(" s   status: link, mode, targets, pulse widths"));
    Serial.println(F(" n   park: go to the neutral pose and stay there"));
    Serial.println(F(" l   live: resume teleoperation"));
    Serial.println(F(" a   energise (attach) the servos"));
    Serial.println(F(" r   relax (detach) the servos -- arm may fall"));
    Serial.println(F(" t   toggle telemetry"));
    Serial.println(F("---------------------------------------------------"));
}

static void printStatus() {
    const uint32_t now = millis();
    Serial.println(F("--- arm status -------------------------------------"));
    Serial.print(F(" mode         : "));
    Serial.println(modeName(g_mode));
    Serial.print(F(" servos       : "));
    Serial.print(g_servosAttached ? F("energised") : F("relaxed"));
    Serial.print(F(" on pins"));
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(kServoPin[i]);
    }
    Serial.println();
    Serial.print(F(" radio        : "));
    Serial.print(rflink::driverName());
    Serial.print(F(" @ "));
    Serial.print(GLOVE_RF_SPEED_BPS);
    Serial.print(F(" bps, RX pin "));
    Serial.print(GLOVE_RF_RX_PIN);
    Serial.print(F(", air time ~"));
    Serial.print(rflink::frameAirTimeUs(GLOVE_FRAME_LEN) / 1000);
    Serial.println(F(" ms/frame"));
    Serial.print(F(" link         : "));
    if (!g_link.everReceived()) {
        Serial.println(F("nothing received yet"));
    } else {
        Serial.print(g_link.msSincePacket(now));
        Serial.print(F(" ms since last frame, quality "));
        Serial.print(g_link.qualityPercent());
        Serial.print(F("% ("));
        Serial.print(g_link.goodPackets());
        Serial.print(F(" ok, "));
        Serial.print(g_link.lostPackets());
        Serial.print(F(" lost, "));
        Serial.print(g_link.stalePackets());
        Serial.println(F(" stale)"));
    }
    Serial.print(F(" frames       : "));
    Serial.print(g_framesAccepted);
    Serial.print(F(" accepted, "));
    Serial.print(g_framesRejected);
    Serial.print(F(" rejected, failsafe events "));
    Serial.println(g_link.failsafeEvents());
    Serial.print(F(" driver rx    : "));
    Serial.print(rflink::rxGood());
    Serial.print(F(" good, "));
    Serial.print(rflink::rxBad());
    Serial.println(F(" bad FCS (bad wiring or noise if this grows fast)"));
    Serial.print(F(" targets      :"));
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(g_target[i]);
    }
    Serial.println(F(" deg"));
    Serial.print(F(" commanded    :"));
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(g_slew[i].value());
        Serial.print('/');
        Serial.print(glove::angleToPulseUs(
            g_slew[i].value(), GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX,
            kServoInvert[i] ? GLOVE_RX_PULSE_MAX_US : GLOVE_RX_PULSE_MIN_US,
            kServoInvert[i] ? GLOVE_RX_PULSE_MIN_US : GLOVE_RX_PULSE_MAX_US));
    }
    Serial.println(F(" deg/us"));
    Serial.print(F(" limits       : "));
    Serial.print(GLOVE_RX_ANGLE_MIN);
    Serial.print(F(".."));
    Serial.print(GLOVE_RX_ANGLE_MAX);
    Serial.print(F(" deg, "));
    Serial.print(GLOVE_RX_PULSE_MIN_US);
    Serial.print(F(".."));
    Serial.print(GLOVE_RX_PULSE_MAX_US);
    Serial.print(F(" us, "));
    Serial.print(GLOVE_RX_SLEW_MAX_DPS);
    Serial.println(F(" deg/s max"));
    Serial.print(F(" glove IMU    : "));
    Serial.println((g_frame.flags & GLOVE_FLAG_IMU_OK) ? F("present")
                                                       : F("not reporting"));
    Serial.print(F(" loop max     : "));
    Serial.print(g_loopMaxUs);
    Serial.println(F(" us (longest gap between loop() entries)"));
    Serial.println(F("----------------------------------------------------"));
}

static void printTelemetry() {
#if GLOVE_RX_TELEMETRY_CSV
    Serial.print(F("R,"));
    Serial.print(g_link.msSincePacket(millis()));
    Serial.print(',');
    Serial.print(g_link.qualityPercent());
    Serial.print(',');
    Serial.print(g_link.goodPackets());
    Serial.print(',');
    Serial.print(g_link.lostPackets());
    Serial.print(',');
    Serial.print(g_link.failsafeEvents());
    Serial.print(',');
    Serial.print(g_mode);
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        Serial.print(',');
        Serial.print(g_slew[i].value());
    }
    Serial.print(',');
    Serial.println(g_framesRejected);
#else
    Serial.print(modeName(g_mode));
    Serial.print(F(" | "));
    if (g_link.everReceived()) {
        Serial.print(g_link.msSincePacket(millis()));
        Serial.print(F("ms q"));
        Serial.print(g_link.qualityPercent());
        Serial.print('%');
    } else {
        Serial.print(F("no signal"));
    }
    Serial.print(F(" |"));
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(g_slew[i].value());
    }
    Serial.print(F(" | lost "));
    Serial.print(g_link.lostPackets());
    Serial.print(F(" fs "));
    Serial.print(g_link.failsafeEvents());
    if (g_framesRejected > 0) {
        Serial.print(F(" | REJECTED "));
        Serial.print(g_framesRejected);
    }
    Serial.println();
#endif
}

/* --------------------------------------------------------------- CLI ---- */

static void handleCommand(const char *line) {
    switch (line[0]) {
    case 'h':
    case '?':
        printHelp();
        break;
    case 's':
        printStatus();
        break;
    case 'n':
        g_command.offer(GLOVE_CMD_PARK, millis());
        Serial.println(F("[arm] parked: neutral pose until 'l'"));
        break;
    case 'l':
        g_relaxed = false;
        g_command.offer(GLOVE_CMD_RESUME_LIVE, millis());
        attachServos();
        Serial.println(F("[arm] live control resumed"));
        break;
    case 'a':
        g_relaxed = false;
        g_command.offer(GLOVE_CMD_RESUME_LIVE, millis());
        attachServos();
        Serial.println(F("[arm] servos energised at the neutral pose"));
        break;
    case 'r':
        g_relaxed = true;
        detachServos();
        Serial.println(F("[arm] servos relaxed -- no holding torque, the arm "
                         "may fall.  'a' to energise again"));
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
            g_lineLen = 0;
        }
    }
}

/* ------------------------------------------------------------- setup ---- */

void setup() {
#if GLOVE_WDT_ACTIVE
    /* Clear the watchdog reset flag before re-arming it, or a watchdog reset
     * leaves it enabled through the bootloader and the board reset-loops. */
    MCUSR &= ~static_cast<uint8_t>(_BV(WDRF));
    wdt_disable();
#endif

    Serial.begin(GLOVE_SERIAL_BAUD);
    while (!Serial && millis() < 1500) {
    }

    g_bootMs = millis();
    g_frame.clear();

    g_link.begin(GLOVE_RX_FAILSAFE_TIMEOUT_MS);
    g_command.begin(GLOVE_RX_POSE_HOLD_MS);
    for (uint8_t i = 0; i < GLOVE_RX_SERVO_COUNT; ++i) {
        g_slew[i].begin(kPoseNeutral[i], GLOVE_RX_SLEW_MAX_DPS);
        g_target[i] = kPoseNeutral[i];
    }

    rflink::begin();
    rflink::startRx();

    attachServos();

#if GLOVE_WDT_ACTIVE
    wdt_enable(GLOVE_WATCHDOG_TIMEOUT);
#endif

    g_lastUpdateMs = g_lastTelemetryMs = millis();

    Serial.println();
    Serial.println(F("=== RoboticArm receiver ==="));
    Serial.print(F("protocol v"));
    Serial.print(GLOVE_PROTOCOL_VERSION);
    Serial.print(F(", radio "));
    Serial.print(rflink::driverName());
    Serial.print(F(" @ "));
    Serial.print(GLOVE_RF_SPEED_BPS);
    Serial.print(F(" bps, "));
    Serial.print(GLOVE_RX_SERVO_COUNT);
    Serial.println(F(" servos"));
    Serial.print(F("failsafe after "));
    Serial.print(GLOVE_RX_FAILSAFE_TIMEOUT_MS);
    Serial.print(F(" ms -> "));
#if GLOVE_RX_FAILSAFE_ACTION == GLOVE_FAILSAFE_HOLD
    Serial.println(F("HOLD last position"));
#elif GLOVE_RX_FAILSAFE_ACTION == GLOVE_FAILSAFE_RELAX
    Serial.println(F("RELAX (servos detached -- the arm will fall unless it "
                     "is counterweighted)"));
#else
    Serial.println(F("return to NEUTRAL"));
#endif
    Serial.println(F("type 'h' for the command list"));

    /* Prime the loop-timing baseline so the first measured gap is a real loop
     * period and not the whole of setup(). */
    g_lastLoopUs = micros();
}

/* -------------------------------------------------------------- loop ---- */

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

    pollSerial();
    drainRadio(now);

    const bool linkLost = g_link.poll(now);
    decideTargets(now, linkLost);

    if (static_cast<uint32_t>(now - g_lastUpdateMs) >= GLOVE_RX_UPDATE_MS) {
        g_lastUpdateMs = now;
        applyServos(now);
    }

    if (g_telemetry &&
        static_cast<uint32_t>(now - g_lastTelemetryMs) >= GLOVE_RX_TELEMETRY_MS) {
        g_lastTelemetryMs = now;
        printTelemetry();
    }
}
