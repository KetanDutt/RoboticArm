/*
 * ServoDrive.cpp -- implementation of the servo-side control logic.
 *
 * Shared file: edit `common/ServoDrive.cpp` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#include "ServoDrive.h"

namespace glove {

/* ------------------------------------------------------------ angle -> us */

uint16_t angleToPulseUs(int16_t angle, int16_t angleMin, int16_t angleMax,
                        uint16_t pulseMinUs, uint16_t pulseMaxUs) {
    const int32_t clampedAngle = clampInt(angle, angleMin, angleMax);
    int32_t us = mapRange(clampedAngle, angleMin, angleMax,
                          static_cast<int32_t>(pulseMinUs),
                          static_cast<int32_t>(pulseMaxUs));
    us = clampInt(us, static_cast<int32_t>(pulseMinUs),
                  static_cast<int32_t>(pulseMaxUs));
    return static_cast<uint16_t>(us);
}

/* ----------------------------------------------------------- slew limiter */

SlewLimiter::SlewLimiter() : current_(0), maxDps_(0), lastMs_(0), primed_(false) {}

void SlewLimiter::begin(int16_t startAngle, uint16_t maxDegPerSec) {
    current_ = startAngle;
    maxDps_ = maxDegPerSec;
    primed_ = false;
    lastMs_ = 0;
}

void SlewLimiter::setMaxRate(uint16_t maxDegPerSec) { maxDps_ = maxDegPerSec; }

uint16_t SlewLimiter::maxRate() const { return maxDps_; }

int16_t SlewLimiter::value() const { return current_; }

void SlewLimiter::snap(int16_t angle, uint32_t nowMs) {
    current_ = angle;
    lastMs_ = nowMs;
    primed_ = true;
}

int16_t SlewLimiter::update(int16_t targetAngle, uint32_t nowMs) {
    if (!primed_) {
        /* First call after begin(): establish the time base and hold the
         * starting angle.  Snapping to the target here would defeat rate
         * limiting at exactly the moment it matters most (power-on). */
        lastMs_ = nowMs;
        primed_ = true;
        return current_;
    }
    if (maxDps_ == 0) {
        /* Unlimited: go straight there. */
        current_ = targetAngle;
        lastMs_ = nowMs;
        return current_;
    }

    /* Unsigned subtraction is correct even across a millis() rollover. */
    const uint32_t elapsed = static_cast<uint32_t>(nowMs - lastMs_);
    if (elapsed == 0) return current_;

    const int32_t budget =
        (static_cast<int32_t>(maxDps_) * static_cast<int32_t>(elapsed)) / 1000;
    if (budget < 1) {
        /* Not even one degree earned yet: keep the clock where it is so the
         * fractional time accumulates instead of being thrown away.  Without
         * this, any rate below 1000 deg/s would stall forever when update()
         * is called every millisecond. */
        return current_;
    }
    lastMs_ = nowMs;

    const int32_t diff = static_cast<int32_t>(targetAngle) - current_;
    if (diff > 0) {
        current_ = static_cast<int16_t>(current_ + (diff > budget ? budget : diff));
    } else if (diff < 0) {
        const int32_t back = -diff;
        current_ = static_cast<int16_t>(current_ - (back > budget ? budget : back));
    }
    return current_;
}

/* ------------------------------------------------------------ link monitor */

LinkMonitor::LinkMonitor()
    : timeoutMs_(0),
      lastRxMs_(0),
      good_(0),
      lost_(0),
      stale_(0),
      failsafeEvents_(0),
      lastSeq_(0),
      have_(false),
      timedOutLatched_(false) {}

void LinkMonitor::begin(uint32_t timeoutMs) {
    reset();
    timeoutMs_ = timeoutMs;
}

void LinkMonitor::setTimeoutMs(uint32_t timeoutMs) { timeoutMs_ = timeoutMs; }

void LinkMonitor::reset() {
    lastRxMs_ = 0;
    good_ = 0;
    lost_ = 0;
    stale_ = 0;
    failsafeEvents_ = 0;
    lastSeq_ = 0;
    have_ = false;
    timedOutLatched_ = false;
}

void LinkMonitor::packetReceived(uint32_t nowMs, uint8_t seq) {
    ++good_;
    if (have_) {
        if (seqIsAfter(lastSeq_, seq)) {
            lost_ += seqGap(lastSeq_, seq);
        } else {
            /* Repeated or out-of-order frame: counted, but it still proves
             * the link is alive, so it refreshes the timeout below. */
            ++stale_;
        }
    }
    lastSeq_ = seq;
    have_ = true;
    lastRxMs_ = nowMs;
}

bool LinkMonitor::timedOut(uint32_t nowMs) const {
    if (!have_) return false; /* nothing received yet: not a *lost* link */
    return static_cast<uint32_t>(nowMs - lastRxMs_) >= timeoutMs_;
}

bool LinkMonitor::poll(uint32_t nowMs) {
    const bool out = timedOut(nowMs);
    if (out && !timedOutLatched_) {
        ++failsafeEvents_;
    }
    timedOutLatched_ = out;
    return out;
}

uint32_t LinkMonitor::msSincePacket(uint32_t nowMs) const {
    if (!have_) return 0xFFFFFFFFu;
    return static_cast<uint32_t>(nowMs - lastRxMs_);
}

uint32_t LinkMonitor::goodPackets() const { return good_; }

uint32_t LinkMonitor::lostPackets() const { return lost_; }

uint32_t LinkMonitor::stalePackets() const { return stale_; }

uint32_t LinkMonitor::failsafeEvents() const { return failsafeEvents_; }

uint8_t LinkMonitor::lastSeq() const { return lastSeq_; }

bool LinkMonitor::everReceived() const { return have_; }

uint8_t LinkMonitor::qualityPercent() const {
    const uint32_t total = good_ + lost_;
    if (total == 0) return 100u;
    return static_cast<uint8_t>((good_ * 100u) / total);
}

/* ---------------------------------------------------------- command latch */

CommandLatch::CommandLatch() : holdMs_(0), latched_(GLOVE_CMD_NONE),
                               latchedAtMs_(0), have_(false) {}

void CommandLatch::begin(uint32_t holdMs) {
    holdMs_ = holdMs;
    clear();
}

void CommandLatch::clear() {
    latched_ = GLOVE_CMD_NONE;
    latchedAtMs_ = 0;
    have_ = false;
}

bool CommandLatch::latchesForever(uint8_t command) {
    return command == GLOVE_CMD_PARK || command == GLOVE_CMD_RELAX;
}

void CommandLatch::offer(uint8_t command, uint32_t nowMs) {
    if (command == GLOVE_CMD_NONE) return; /* heartbeat frame: keep the latch */
    if (command == GLOVE_CMD_RESUME_LIVE) {
        clear();
        return;
    }
    latched_ = command;
    latchedAtMs_ = nowMs;
    have_ = true;
}

uint8_t CommandLatch::active(uint32_t nowMs) {
    if (!have_) return GLOVE_CMD_NONE;
    if (!latchesForever(latched_) &&
        static_cast<uint32_t>(nowMs - latchedAtMs_) >= holdMs_) {
        clear();
        return GLOVE_CMD_NONE;
    }
    return latched_;
}

bool CommandLatch::poseActive(uint32_t nowMs) {
    return active(nowMs) != GLOVE_CMD_NONE;
}

} /* namespace glove */
