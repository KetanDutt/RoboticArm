/*
 * test_signal.cpp -- filters, auto-ranging and the glove button.
 */
#include "tests.h"

#include "SignalProcessing.h"
#include "GloveConfig.h"

using namespace glove;

void runSignalTests() {
    SECTION("EmaFilter");
    EmaFilter f;
    f.begin(GLOVE_EMA_ALPHA_Q8, 500);
    CHECK_EQ(f.value(), 500); /* seeded, not zeroed */
    /* A step input must converge towards the new value, monotonically. */
    int16_t prev = f.value();
    for (int i = 0; i < 200; ++i) {
        const int16_t now = f.update(900);
        CHECK(now >= prev);
        prev = now;
    }
    CHECK_EQ(f.value(), 900);
    /* Roughly a 30 ms time constant at 5 ms sampling: after 6 samples with
     * alpha = 40/256 the filter should have covered ~64% of the step. */
    f.begin(GLOVE_EMA_ALPHA_Q8, 0);
    for (int i = 0; i < 6; ++i) f.update(1000);
    CHECK(f.value() > 500);
    CHECK(f.value() < 800);
    /* alpha 255 tracks almost instantly; alpha 0 freezes. */
    f.begin(255, 0);
    f.update(1000);
    CHECK(f.value() > 990);
    f.begin(0, 123);
    f.update(900);
    f.update(900);
    CHECK_EQ(f.value(), 123);
    /* Extreme inputs must not overflow the fixed point. */
    f.begin(GLOVE_EMA_ALPHA_Q8, -32768);
    for (int i = 0; i < 400; ++i) f.update(32767);
    CHECK_EQ(f.value(), 32767);
    f.reset(-32768);
    for (int i = 0; i < 400; ++i) f.update(-32768);
    CHECK_EQ(f.value(), -32768);

    SECTION("MedianFilter3");
    MedianFilter3 m;
    m.reset(500);
    CHECK_EQ(m.update(500), 500);
    CHECK_EQ(m.update(502), 500);
    /* One wild sample out of three must be rejected outright. */
    CHECK_EQ(m.update(4000), 502);
    CHECK_EQ(m.update(500), 502);
    CHECK_EQ(m.update(501), 501);
    /* A sustained change still gets through (this is a filter, not a latch). */
    m.reset(100);
    m.update(800);
    m.update(800);
    CHECK_EQ(m.update(800), 800);

    SECTION("AnalogChannel pipeline");
    AnalogChannel ch;
    ch.begin(GLOVE_EMA_ALPHA_Q8, GLOVE_MEDIAN_ENABLE != 0, 500);
    CHECK_EQ(ch.value(), 500);
    /* A single ADC glitch must not reach the output. */
    ch.update(500);
    ch.update(4095);
    const int16_t afterSpike = ch.update(500);
    CHECK(afterSpike < 560);
    /* With the median disabled the same spike does move the EMA. */
    AnalogChannel noMedian;
    noMedian.begin(GLOVE_EMA_ALPHA_Q8, false, 500);
    noMedian.update(4095);
    CHECK(noMedian.value() > 700);

    SECTION("AutoRanger");
    AutoRanger r;
    CHECK(!r.haveSample());
    CHECK_EQ(r.span(), 0);
    CHECK(!r.valid(GLOVE_CAL_MIN_SPAN));
    r.update(300);
    CHECK_EQ(r.lowest(), 300);
    CHECK_EQ(r.highest(), 300);
    CHECK(!r.valid(GLOVE_CAL_MIN_SPAN)); /* not moved enough yet */
    r.update(120);
    r.update(640);
    r.update(400);
    CHECK_EQ(r.lowest(), 120);
    CHECK_EQ(r.highest(), 640);
    CHECK_EQ(r.span(), 520);
    CHECK(r.valid(GLOVE_CAL_MIN_SPAN));
    r.reset();
    CHECK(!r.haveSample());

    SECTION("outsideDeadband");
    CHECK(!outsideDeadband(90, 90, GLOVE_DEADBAND_DEG));
    CHECK(!outsideDeadband(90, 92, GLOVE_DEADBAND_DEG));
    CHECK(outsideDeadband(90, 93, GLOVE_DEADBAND_DEG));
    CHECK(outsideDeadband(90, 87, GLOVE_DEADBAND_DEG));
    CHECK(!outsideDeadband(0, 180, 200));

    SECTION("Button: debounce");
    Button b;
    b.begin(GLOVE_TX_BUTTON_DEBOUNCE_MS, GLOVE_TX_BUTTON_LONG_MS);
    /* A 5 ms contact bounce must produce no event at all. */
    CHECK_EQ(b.update(false, 0), GLOVE_BTN_NONE);
    CHECK_EQ(b.update(true, 5), GLOVE_BTN_NONE);
    CHECK_EQ(b.update(false, 8), GLOVE_BTN_NONE);
    for (uint32_t t = 10; t < 60; t += 5) {
        CHECK_EQ(b.update(false, t), GLOVE_BTN_NONE);
    }

    SECTION("Button: short press");
    b.begin(GLOVE_TX_BUTTON_DEBOUNCE_MS, GLOVE_TX_BUTTON_LONG_MS);
    uint32_t t = 0;
    CHECK_EQ(b.update(true, t), GLOVE_BTN_NONE);
    t += GLOVE_TX_BUTTON_DEBOUNCE_MS;
    CHECK_EQ(b.update(true, t), GLOVE_BTN_NONE); /* accepted, held */
    CHECK(b.isHeld());
    t += 100;
    CHECK_EQ(b.update(false, t), GLOVE_BTN_NONE); /* bouncing open */
    t += GLOVE_TX_BUTTON_DEBOUNCE_MS;
    CHECK_EQ(b.update(false, t), GLOVE_BTN_SHORT);
    CHECK_EQ(b.update(false, t + 50), GLOVE_BTN_NONE); /* one-shot */

    SECTION("Button: long press");
    b.begin(GLOVE_TX_BUTTON_DEBOUNCE_MS, GLOVE_TX_BUTTON_LONG_MS);
    t = 0;
    uint8_t events = 0;
    for (t = 0; t < GLOVE_TX_BUTTON_LONG_MS + 500; t += 10) {
        if (b.update(true, t) == GLOVE_BTN_LONG) ++events;
    }
    CHECK_EQ(events, 1u); /* fires exactly once while held */
    CHECK(b.isHeld());
    t += 20;
    CHECK_EQ(b.update(false, t), GLOVE_BTN_NONE);
    t += GLOVE_TX_BUTTON_DEBOUNCE_MS;
    /* Releasing after a long press must not also report a short press. */
    CHECK_EQ(b.update(false, t), GLOVE_BTN_NONE);
}
