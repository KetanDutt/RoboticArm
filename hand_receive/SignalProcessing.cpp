/*
 * SignalProcessing.cpp -- implementation of the filters in
 *                         SignalProcessing.h.
 *
 * Shared file: edit `common/SignalProcessing.cpp` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#include "SignalProcessing.h"

namespace glove {

/* ------------------------------------------------------------------ EMA */

EmaFilter::EmaFilter() : alphaQ8_(255), accQ8_(0) {}

void EmaFilter::begin(uint8_t alphaQ8, int16_t initial) {
    setAlpha(alphaQ8);
    reset(initial);
}

void EmaFilter::setAlpha(uint8_t alphaQ8) { alphaQ8_ = alphaQ8; }

void EmaFilter::reset(int16_t v) { accQ8_ = static_cast<int32_t>(v) << 8; }

int16_t EmaFilter::update(int16_t sample) {
    const int32_t targetQ8 = static_cast<int32_t>(sample) << 8;
    /* acc += (target - acc) * alpha / 256
     *
     * Written as a two-term decomposition so it cannot overflow int32 for any
     * 16-bit input.  With d = deltaQ8, q = d >> 8 (floor) and r = d & 0xFF:
     *     d * alpha / 256 == q * alpha + (r * alpha) / 256
     * and each term stays well inside 32 bits (|q| <= 2^16, alpha <= 255). */
    const int32_t d = targetQ8 - accQ8_;
    const int32_t alpha = static_cast<int32_t>(alphaQ8_);
    accQ8_ += (d >> 8) * alpha + (((d & 0xFF) * alpha) >> 8);
    return value();
}

int16_t EmaFilter::value() const {
    /* Round half away from zero rather than truncating towards it. */
    if (accQ8_ >= 0) {
        return static_cast<int16_t>((accQ8_ + 128) >> 8);
    }
    return static_cast<int16_t>(-(((-accQ8_) + 128) >> 8));
}

/* -------------------------------------------------------- median of three */

MedianFilter3::MedianFilter3() : idx_(0) {
    buf_[0] = buf_[1] = buf_[2] = 0;
}

void MedianFilter3::reset(int16_t seed) {
    buf_[0] = buf_[1] = buf_[2] = seed;
    idx_ = 0;
}

int16_t MedianFilter3::update(int16_t sample) {
    buf_[idx_] = sample;
    idx_ = static_cast<uint8_t>((idx_ + 1u) % 3u);

    const int16_t a = buf_[0];
    const int16_t b = buf_[1];
    const int16_t c = buf_[2];
    /* Branch-free-ish median of three. */
    if (a < b) {
        if (b < c) return b;
        return (a < c) ? c : a;
    }
    if (a < c) return a;
    return (b < c) ? c : b;
}

int16_t MedianFilter3::value() const {
    const int16_t a = buf_[0];
    const int16_t b = buf_[1];
    const int16_t c = buf_[2];
    if (a < b) {
        if (b < c) return b;
        return (a < c) ? c : a;
    }
    if (a < c) return a;
    return (b < c) ? c : b;
}

/* ------------------------------------------------------- analog pipeline */

AnalogChannel::AnalogChannel() : useMedian_(true) {}

void AnalogChannel::begin(uint8_t emaAlphaQ8, bool useMedian, int16_t seed) {
    useMedian_ = useMedian;
    median_.reset(seed);
    ema_.begin(emaAlphaQ8, seed);
}

int16_t AnalogChannel::update(int16_t rawSample) {
    const int16_t pre = useMedian_ ? median_.update(rawSample) : rawSample;
    return ema_.update(pre);
}

int16_t AnalogChannel::value() const { return ema_.value(); }

/* ---------------------------------------------------------- auto ranging */

AutoRanger::AutoRanger() : min_(0), max_(0), have_(false) {}

void AutoRanger::reset() {
    min_ = 0;
    max_ = 0;
    have_ = false;
}

void AutoRanger::update(int16_t v) {
    if (!have_) {
        min_ = max_ = v;
        have_ = true;
        return;
    }
    if (v < min_) min_ = v;
    if (v > max_) max_ = v;
}

bool AutoRanger::haveSample() const { return have_; }

int16_t AutoRanger::lowest() const { return min_; }

int16_t AutoRanger::highest() const { return max_; }

int16_t AutoRanger::span() const {
    return have_ ? static_cast<int16_t>(max_ - min_) : 0;
}

bool AutoRanger::valid(int16_t minSpan) const {
    return have_ && span() >= minSpan;
}

/* ------------------------------------------------------------ deadband */

bool outsideDeadband(int16_t reference, int16_t candidate, int16_t threshold) {
    const int32_t d = static_cast<int32_t>(candidate) -
                      static_cast<int32_t>(reference);
    return (d < 0 ? -d : d) > static_cast<int32_t>(threshold);
}

/* --------------------------------------------------------------- button */

Button::Button()
    : debounceMs_(20),
      longPressMs_(1500),
      candidate_(false),
      stable_(false),
      longFired_(false),
      candidateSinceMs_(0),
      pressStartMs_(0) {}

void Button::begin(uint16_t debounceMs, uint16_t longPressMs) {
    debounceMs_ = debounceMs;
    longPressMs_ = longPressMs;
    candidate_ = false;
    stable_ = false;
    longFired_ = false;
    candidateSinceMs_ = 0;
    pressStartMs_ = 0;
}

uint8_t Button::update(bool rawPressed, uint32_t nowMs) {
    uint8_t event = GLOVE_BTN_NONE;

    if (rawPressed != candidate_) {
        /* Input just changed: start (or restart) the debounce window. */
        candidate_ = rawPressed;
        candidateSinceMs_ = nowMs;
    } else if (candidate_ != stable_ &&
               static_cast<uint32_t>(nowMs - candidateSinceMs_) >=
                   debounceMs_) {
        stable_ = candidate_;
        if (stable_) {
            pressStartMs_ = nowMs;
            longFired_ = false;
        } else if (!longFired_) {
            event = GLOVE_BTN_SHORT;
        }
    }

    if (stable_ && !longFired_ &&
        static_cast<uint32_t>(nowMs - pressStartMs_) >= longPressMs_) {
        longFired_ = true;
        event = GLOVE_BTN_LONG;
    }

    return event;
}

bool Button::isHeld() const { return stable_; }

} /* namespace glove */
