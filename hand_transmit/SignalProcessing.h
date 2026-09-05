/*
 * SignalProcessing.h -- noise, spike and bounce handling for the glove's
 *                       sensors and button.
 *
 * Raw flex sensors and a raw MPU-6050 are noisy enough that an unfiltered
 * value makes a servo buzz audibly and, worse, makes it hunt constantly.
 * Everything here is integer-only fixed point so it is cheap on an AVR and
 * deterministic enough to unit test (see tests/test_signal.cpp).
 *
 * Pipeline used per flex channel:
 *
 *     raw ADC --> MedianFilter3 --> EmaFilter --> calibration --> angle
 *                 (kills spikes)   (kills noise)  (GloveCalibration)
 *
 * Shared file: edit `common/SignalProcessing.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>
#include <stddef.h>

namespace glove {

/* Exponential moving average with the weight expressed in 1/256 units
 * (Q8), so no floating point is needed.  alphaQ8 == 256 passes samples
 * straight through; alphaQ8 == 0 freezes the output. */
class EmaFilter {
public:
    EmaFilter();
    void begin(uint8_t alphaQ8, int16_t initial);
    void setAlpha(uint8_t alphaQ8);
    /* Feed one sample, get the filtered value (rounded). */
    int16_t update(int16_t sample);
    int16_t value() const;
    void reset(int16_t v);

private:
    uint8_t alphaQ8_;
    int32_t accQ8_; /* current value scaled by 256 */
};

/* Median of the last three samples: removes single-sample spikes (the kind
 * an ASK transmitter or a motor brush injects into an ADC reading) without
 * adding the lag a deeper moving average would. */
class MedianFilter3 {
public:
    MedianFilter3();
    void reset(int16_t seed);
    int16_t update(int16_t sample);
    int16_t value() const;

private:
    int16_t buf_[3];
    uint8_t idx_;
};

/* Convenience wrapper: median (optional) then EMA.  One object per analog
 * channel, one call per sample. */
class AnalogChannel {
public:
    AnalogChannel();
    void begin(uint8_t emaAlphaQ8, bool useMedian, int16_t seed);
    int16_t update(int16_t rawSample);
    int16_t value() const;

private:
    MedianFilter3 median_;
    EmaFilter ema_;
    bool useMedian_;
};

/* Tracks the extreme values seen since the last reset.  This is what the
 * calibration capture uses to learn each sensor's real span. */
class AutoRanger {
public:
    AutoRanger();
    void reset();
    void update(int16_t v);
    bool haveSample() const;
    int16_t lowest() const;   /* not min(): Arduino.h defines min/max as macros */
    int16_t highest() const;
    int16_t span() const;
    /* A span this small means the sensor was not moved (or is unplugged). */
    bool valid(int16_t minSpan) const;

private:
    int16_t min_;
    int16_t max_;
    bool have_;
};

/* True when `candidate` has moved far enough from `reference` to be worth
 * acting on.  Used to suppress 1-degree servo twitching. */
bool outsideDeadband(int16_t reference, int16_t candidate, int16_t threshold);

/* Button events reported by Button::update(). */
#define GLOVE_BTN_NONE  0u
#define GLOVE_BTN_SHORT 1u /* released before the long-press threshold */
#define GLOVE_BTN_LONG  2u /* fired once, while still held down */

/* Debounced push-button with short/long press detection.  Purely a function
 * of (raw level, timestamp), so it is testable without hardware. */
class Button {
public:
    Button();
    void begin(uint16_t debounceMs, uint16_t longPressMs);
    /* Call from the main loop with the debounced-nothing raw pin level. */
    uint8_t update(bool rawPressed, uint32_t nowMs);
    bool isHeld() const;

private:
    uint16_t debounceMs_;
    uint16_t longPressMs_;
    bool candidate_;
    bool stable_;
    bool longFired_;
    uint32_t candidateSinceMs_;
    uint32_t pressStartMs_;
};

} /* namespace glove */

#endif /* SIGNAL_PROCESSING_H */
