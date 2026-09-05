/*
 * ServoDrive.h -- everything between "a decoded frame" and "a servo moves".
 *
 * Three jobs live here:
 *
 *  1. angleToPulseUs()  -- the single most important line in the receiver.
 *     ServoTimer2::write() takes a pulse width in MICROSECONDS (750..2250),
 *     not an angle in degrees like the stock Servo library.  The original
 *     firmware wrote 0..180 to it, which ServoTimer2 clamps to its 750 us
 *     minimum: every servo sat pinned at one end stop and ignored all data.
 *     See docs/BUGS_FIXED.md (B-01).
 *
 *  2. SlewLimiter      -- rate limiting.  A servo may only be commanded
 *     GLOVE_RX_SLEW_MAX_DPS degrees per second.  This protects the gearbox,
 *     keeps the supply from brown-outing on simultaneous starts, and turns a
 *     corrupt packet into a slow correction instead of a violent one.
 *
 *  3. LinkMonitor / CommandLatch -- link supervision and preset handling.
 *     LinkMonitor turns the protocol's sequence numbers into packet-loss
 *     statistics and drives the failsafe; CommandLatch holds a preset pose for
 *     a configurable time before blending back to live control.
 *
 * All of it is pure C++ with timestamps passed in, so it unit tests on a
 * desktop (tests/test_servo.cpp).
 *
 * Shared file: edit `common/ServoDrive.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <stdint.h>
#include <stddef.h>

#include "GloveMath.h"
#include "GloveProtocol.h"

namespace glove {

/* Convert a servo angle into the pulse width ServoTimer2 expects.
 *   angle    commanded angle (clamped into [angleMin, angleMax] first)
 *   angleMin/angleMax  the mechanical window of this joint
 *   pulseMinUs/pulseMaxUs  the pulse widths that correspond to those angles
 * Returns a value guaranteed to lie inside [pulseMinUs, pulseMaxUs]. */
uint16_t angleToPulseUs(int16_t angle, int16_t angleMin, int16_t angleMax,
                        uint16_t pulseMinUs, uint16_t pulseMaxUs);

/* Rate limiter with an integer, wrap-around-safe time base. */
class SlewLimiter {
public:
    SlewLimiter();
    /* startAngle: where the servo physically is right now.
     * maxDegPerSec: 0 means "unlimited" (jump straight to the target). */
    void begin(int16_t startAngle, uint16_t maxDegPerSec);
    void setMaxRate(uint16_t maxDegPerSec);
    /* Move towards targetAngle by no more than the elapsed time allows. */
    int16_t update(int16_t targetAngle, uint32_t nowMs);
    int16_t value() const;
    /* Jump to an angle without animating (used for power-on positioning). */
    void snap(int16_t angle, uint32_t nowMs);
    uint16_t maxRate() const;

private:
    int16_t current_;
    uint16_t maxDps_;
    uint32_t lastMs_;
    bool primed_;
};

/* Supervises the radio link using the frame sequence numbers. */
class LinkMonitor {
public:
    LinkMonitor();
    void begin(uint32_t timeoutMs);
    void setTimeoutMs(uint32_t timeoutMs);
    /* Feed one successfully decoded frame. */
    void packetReceived(uint32_t nowMs, uint8_t seq);
    /* Call once per loop: returns the timed-out state and counts the
     * transition into it, so failsafe events can be reported. */
    bool poll(uint32_t nowMs);
    bool timedOut(uint32_t nowMs) const;
    uint32_t msSincePacket(uint32_t nowMs) const;
    uint32_t goodPackets() const;
    /* Estimated frames lost, derived from gaps in the sequence numbers. */
    uint32_t lostPackets() const;
    /* Frames that repeated or arrived out of order. */
    uint32_t stalePackets() const;
    uint32_t failsafeEvents() const;
    uint8_t lastSeq() const;
    bool everReceived() const;
    /* good / (good + lost) as a percentage, 100 when nothing is lost. */
    uint8_t qualityPercent() const;
    void reset();

private:
    uint32_t timeoutMs_;
    uint32_t lastRxMs_;
    uint32_t good_;
    uint32_t lost_;
    uint32_t stale_;
    uint32_t failsafeEvents_;
    uint8_t lastSeq_;
    bool have_;
    bool timedOutLatched_;
};

/* Holds a preset-pose / park command for a while.
 *   - pose commands (OPEN, FIST, HOME, POINT) expire after holdMs,
 *   - PARK and RELAX latch until GLOVE_CMD_RESUME_LIVE arrives,
 *   - every repeat of the same command refreshes the hold window, which is
 *     what makes a one-shot button press survive packet loss. */
class CommandLatch {
public:
    CommandLatch();
    void begin(uint32_t holdMs);
    void offer(uint8_t command, uint32_t nowMs);
    /* The command currently in effect, or GLOVE_CMD_NONE. */
    uint8_t active(uint32_t nowMs);
    bool poseActive(uint32_t nowMs);
    void clear();

private:
    static bool latchesForever(uint8_t command);

    uint32_t holdMs_;
    uint8_t latched_;
    uint32_t latchedAtMs_;
    bool have_;
};

} /* namespace glove */

#endif /* SERVO_DRIVE_H */
