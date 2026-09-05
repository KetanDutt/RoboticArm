/*
 * test_servo.cpp -- angle-to-pulse conversion, rate limiting, link
 *                   supervision and preset latching.
 */
#include "tests.h"

#include "ServoDrive.h"
#include "GloveConfig.h"

using namespace glove;

void runServoTests() {
    SECTION("angleToPulseUs (the ServoTimer2 microseconds bug)");
    /* ServoTimer2::write() wants microseconds.  Anything below 750 is clamped
     * to 750 internally, so the original firmware -- which wrote raw 0..180
     * degree values -- parked all seven servos on one end stop. */
    CHECK_EQ(angleToPulseUs(0, 0, 180, GLOVE_RX_PULSE_MIN_US,
                            GLOVE_RX_PULSE_MAX_US),
             GLOVE_RX_PULSE_MIN_US);
    CHECK_EQ(angleToPulseUs(180, 0, 180, GLOVE_RX_PULSE_MIN_US,
                            GLOVE_RX_PULSE_MAX_US),
             GLOVE_RX_PULSE_MAX_US);
    CHECK_EQ(angleToPulseUs(90, 0, 180, GLOVE_RX_PULSE_MIN_US,
                            GLOVE_RX_PULSE_MAX_US),
             1500u);
    CHECK(angleToPulseUs(90, 0, 180, GLOVE_RX_PULSE_MIN_US,
                         GLOVE_RX_PULSE_MAX_US) >= GLOVE_RX_PULSE_MIN_US);
    /* Out-of-window commands clamp instead of wrapping. */
    CHECK_EQ(angleToPulseUs(-30, 0, 180, 750, 2250), 750u);
    CHECK_EQ(angleToPulseUs(250, 0, 180, 750, 2250), 2250u);
    /* Narrow mechanical window (GLOVE_RX_ANGLE_MIN..MAX). */
    CHECK_EQ(angleToPulseUs(GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MIN,
                            GLOVE_RX_ANGLE_MAX, 750, 2250),
             750u);
    CHECK_EQ(angleToPulseUs(GLOVE_RX_ANGLE_MAX, GLOVE_RX_ANGLE_MIN,
                            GLOVE_RX_ANGLE_MAX, 750, 2250),
             2250u);
    CHECK_EQ(angleToPulseUs(90, GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX, 750,
                            2250),
             1500u);
    /* A servo mounted the other way round: pulse range reversed. */
    CHECK_EQ(angleToPulseUs(0, 0, 180, 2250, 750), 2250u);
    CHECK_EQ(angleToPulseUs(180, 0, 180, 2250, 750), 750u);
    /* Degenerate angle window must not divide by zero. */
    CHECK_EQ(angleToPulseUs(45, 90, 90, 750, 2250), 750u);

    SECTION("SlewLimiter: rate limiting");
    SlewLimiter s;
    s.begin(0, 300); /* 300 deg/s */
    CHECK_EQ(s.update(180, 0), 0);   /* first call only sets the time base */
    CHECK_EQ(s.update(180, 100), 30);
    CHECK_EQ(s.update(180, 200), 60);
    CHECK_EQ(s.update(180, 1000), 180); /* never overshoots the target */
    CHECK_EQ(s.update(180, 1100), 180);
    /* Reverse direction, same rate: the clock advanced to 1100 above, so
     * each 100 ms step moves 30 degrees. */
    CHECK_EQ(s.update(0, 1200), 150);
    CHECK_EQ(s.update(0, 1300), 120);
    CHECK_EQ(s.update(0, 2000), 0); /* and it stops exactly on the target */

    SECTION("SlewLimiter: fractional time accumulates");
    SlewLimiter slow;
    slow.begin(90, 100); /* 100 deg/s -> one degree per 10 ms */
    slow.update(0, 0);
    for (uint32_t t = 1; t <= 9; ++t) {
        CHECK_EQ(slow.update(0, t), 90); /* not yet earned a whole degree */
    }
    CHECK_EQ(slow.update(0, 10), 89);
    CHECK_EQ(slow.update(0, 20), 88);
    /* Calling every millisecond must not stall or lose time. */
    for (uint32_t t = 21; t <= 1000; ++t) slow.update(0, t);
    CHECK_EQ(slow.value(), 0);

    SECTION("SlewLimiter: unlimited and snap");
    SlewLimiter fast;
    fast.begin(0, 0); /* 0 == unlimited */
    CHECK_EQ(fast.update(170, 5), 0); /* first call primes the time base */
    CHECK_EQ(fast.update(170, 6), 170);
    fast.snap(12, 6);
    CHECK_EQ(fast.value(), 12);
    CHECK_EQ(fast.update(12, 7), 12);

    SECTION("SlewLimiter: millis() rollover");
    SlewLimiter wrap;
    wrap.begin(0, 1000); /* 1 deg/ms */
    const uint32_t before = 0xFFFFFFFFu - 5u;
    CHECK_EQ(wrap.update(180, before), 0);
    /* 0xFFFFFFFF - 5 .. 6 is 12 ms once the unsigned wrap is handled. */
    CHECK_EQ(wrap.update(180, 6u), 12);

    SECTION("LinkMonitor: timeout and failsafe counting");
    LinkMonitor lm;
    lm.begin(GLOVE_RX_FAILSAFE_TIMEOUT_MS);
    CHECK(!lm.everReceived());
    CHECK_EQ(lm.msSincePacket(1000), 0xFFFFFFFFu);
    /* Before the first frame the link is not "lost", it is just not up yet. */
    CHECK(!lm.poll(1000));

    lm.packetReceived(1000, 1);
    CHECK(lm.everReceived());
    CHECK_EQ(lm.msSincePacket(1200), 200u);
    CHECK(!lm.poll(1400));
    CHECK(!lm.poll(GLOVE_RX_FAILSAFE_TIMEOUT_MS + 999));
    CHECK(lm.poll(1000 + GLOVE_RX_FAILSAFE_TIMEOUT_MS));
    CHECK_EQ(lm.failsafeEvents(), 1u);
    /* Still timed out on later polls, but the event is counted once. */
    CHECK(lm.poll(5000));
    CHECK(lm.poll(6000));
    CHECK_EQ(lm.failsafeEvents(), 1u);
    /* Recovery clears the timeout; the next loss counts again. */
    lm.packetReceived(6100, 2);
    CHECK(!lm.poll(6200));
    CHECK(lm.poll(6100 + GLOVE_RX_FAILSAFE_TIMEOUT_MS));
    CHECK_EQ(lm.failsafeEvents(), 2u);

    SECTION("LinkMonitor: sequence accounting");
    LinkMonitor stats;
    stats.begin(500);
    stats.packetReceived(0, 10);
    stats.packetReceived(50, 11); /* consecutive: nothing lost */
    CHECK_EQ(stats.goodPackets(), 2u);
    CHECK_EQ(stats.lostPackets(), 0u);
    CHECK_EQ(stats.stalePackets(), 0u);
    CHECK_EQ(stats.qualityPercent(), 100u);
    stats.packetReceived(100, 15); /* 12, 13, 14 never arrived */
    CHECK_EQ(stats.lostPackets(), 3u);
    CHECK_EQ(stats.qualityPercent(), 50u); /* 3 received, 3 lost */
    stats.packetReceived(150, 15); /* duplicate */
    CHECK_EQ(stats.stalePackets(), 1u);
    CHECK_EQ(stats.lostPackets(), 3u);
    stats.packetReceived(200, 14); /* out of order, not a gap */
    CHECK_EQ(stats.stalePackets(), 2u);
    CHECK_EQ(stats.lostPackets(), 3u);
    /* Sequence numbers wrap at 255 -> 0 and the accounting must follow. */
    LinkMonitor wrapped;
    wrapped.begin(500);
    wrapped.packetReceived(0, 250);
    wrapped.packetReceived(50, 2); /* 251..255 and 0, 1 never arrived */
    CHECK_EQ(wrapped.lostPackets(), 7u);
    CHECK_EQ(wrapped.goodPackets(), 2u);

    stats.reset();
    CHECK_EQ(stats.goodPackets(), 0u);
    CHECK(!stats.everReceived());

    SECTION("CommandLatch: pose hold and expiry");
    CommandLatch cl;
    cl.begin(GLOVE_RX_POSE_HOLD_MS);
    CHECK_EQ(cl.active(0), GLOVE_CMD_NONE);
    cl.offer(GLOVE_CMD_POSE_OPEN, 1000);
    CHECK_EQ(cl.active(1500), GLOVE_CMD_POSE_OPEN);
    CHECK(cl.poseActive(1500));
    CHECK_EQ(cl.active(1000 + GLOVE_RX_POSE_HOLD_MS - 1), GLOVE_CMD_POSE_OPEN);
    /* Heartbeat frames (CMD_NONE) must not cancel a preset. */
    cl.offer(GLOVE_CMD_NONE, 1100);
    CHECK_EQ(cl.active(1200), GLOVE_CMD_POSE_OPEN);
    CHECK_EQ(cl.active(1000 + GLOVE_RX_POSE_HOLD_MS), GLOVE_CMD_NONE);
    CHECK(!cl.poseActive(1000 + GLOVE_RX_POSE_HOLD_MS));
    /* A repeat refreshes the window, which is how a single button press
     * survives packet loss. */
    cl.offer(GLOVE_CMD_POSE_FIST, 2000);
    cl.offer(GLOVE_CMD_POSE_FIST, 2500);
    CHECK_EQ(cl.active(2500 + GLOVE_RX_POSE_HOLD_MS - 1), GLOVE_CMD_POSE_FIST);
    CHECK_EQ(cl.active(2500 + GLOVE_RX_POSE_HOLD_MS), GLOVE_CMD_NONE);
    /* RESUME_LIVE cancels immediately. */
    cl.offer(GLOVE_CMD_POSE_POINT, 3000);
    cl.offer(GLOVE_CMD_RESUME_LIVE, 3100);
    CHECK_EQ(cl.active(3200), GLOVE_CMD_NONE);
    /* PARK is deliberate and stays until resumed. */
    cl.offer(GLOVE_CMD_PARK, 4000);
    CHECK_EQ(cl.active(4000 + GLOVE_RX_POSE_HOLD_MS * 10), GLOVE_CMD_PARK);
    cl.offer(GLOVE_CMD_RESUME_LIVE, 9999);
    CHECK_EQ(cl.active(10000), GLOVE_CMD_NONE);
}
