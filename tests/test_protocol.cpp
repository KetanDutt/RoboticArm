/*
 * test_protocol.cpp -- frame packing, validation and sequence accounting.
 */
#include "tests.h"

#include "GloveProtocol.h"
#include "GloveConfig.h"

using namespace glove;

namespace {

Frame makeSampleFrame(uint8_t seq, uint8_t command) {
    Frame f;
    f.clear();
    f.seq = seq;
    f.flags = GLOVE_FLAG_IMU_OK;
    f.command = command;
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        f.channels[i] = static_cast<uint8_t>(20 + i * 20);
    }
    return f;
}

} /* namespace */

void runProtocolTests() {
    SECTION("GloveProtocol: META byte fields");
    const uint8_t meta =
        packMeta(GLOVE_PROTOCOL_VERSION, GLOVE_FLAG_CAL_ACTIVE, GLOVE_CMD_PARK);
    CHECK_EQ(metaVersion(meta), GLOVE_PROTOCOL_VERSION);
    CHECK_EQ(metaFlags(meta), GLOVE_FLAG_CAL_ACTIVE);
    CHECK_EQ(metaCommand(meta), GLOVE_CMD_PARK);
    /* Fields must not bleed into each other. */
    CHECK_EQ(packMeta(3, 3, 15), 0xFF);
    CHECK_EQ(packMeta(0, 0, 0), 0x00);
    CHECK_EQ(metaVersion(0xFF), 3u);
    CHECK_EQ(metaFlags(0xFF), 3u);
    CHECK_EQ(metaCommand(0xFF), 15u);

    SECTION("GloveProtocol: encode");
    uint8_t raw[GLOVE_FRAME_LEN];
    Frame tx = makeSampleFrame(7, GLOVE_CMD_POSE_FIST);
    CHECK_EQ(tx.encode(raw, sizeof(raw)), GLOVE_FRAME_LEN);
    CHECK_EQ(raw[GLOVE_IDX_SEQ], 7u);
    CHECK_EQ(metaVersion(raw[GLOVE_IDX_META]), GLOVE_PROTOCOL_VERSION);
    CHECK_EQ(metaCommand(raw[GLOVE_IDX_META]), GLOVE_CMD_POSE_FIST);
    CHECK_EQ(raw[GLOVE_IDX_CHANNELS + GLOVE_CH_FINGER_LAST], 20 + 6 * 20);
    /* Too-small buffer is refused rather than overrun. */
    CHECK_EQ(tx.encode(raw, GLOVE_FRAME_LEN - 1), 0u);
    CHECK_EQ(tx.encode(0, sizeof(raw)), 0u);
    /* Out-of-range channel values are clamped on the way out. */
    Frame over = tx;
    over.channels[0] = 250;
    CHECK_EQ(over.encode(raw, sizeof(raw)), GLOVE_FRAME_LEN);
    CHECK_EQ(raw[GLOVE_IDX_CHANNELS + 0], GLOVE_ANGLE_CEILING);

    SECTION("GloveProtocol: decode round trip");
    Frame rx;
    rx.clear();
    CHECK(Frame::decode(raw, GLOVE_FRAME_LEN, GLOVE_RX_ANGLE_MIN,
                        GLOVE_RX_ANGLE_MAX, rx));
    CHECK_EQ(rx.seq, over.seq);
    CHECK_EQ(rx.command, over.command);
    CHECK_EQ(rx.flags, over.flags);
    /* 250 was clamped to 180 on encode and to GLOVE_RX_ANGLE_MAX on decode. */
    CHECK_EQ(rx.channels[0], GLOVE_RX_ANGLE_MAX);

    SECTION("GloveProtocol: decode rejects junk");
    uint8_t good[GLOVE_FRAME_LEN];
    Frame ref = makeSampleFrame(42, GLOVE_CMD_NONE);
    CHECK_EQ(ref.encode(good, sizeof(good)), GLOVE_FRAME_LEN);

    Frame out = makeSampleFrame(0, GLOVE_CMD_NONE);
    const Frame untouched = out;
    /* Wrong length. */
    CHECK(!Frame::decode(good, GLOVE_FRAME_LEN - 1, 0, 180, out));
    CHECK(!Frame::decode(good, GLOVE_FRAME_LEN + 1, 0, 180, out));
    CHECK(!Frame::decode(good, 0, 0, 180, out));
    /* Unknown protocol version: this is the "you flashed only one board"
     * case, and it must be reported, not silently obeyed. */
    uint8_t badVersion[GLOVE_FRAME_LEN];
    std::memcpy(badVersion, good, sizeof(good));
    badVersion[GLOVE_IDX_META] =
        packMeta(GLOVE_PROTOCOL_VERSION + 1, GLOVE_FLAG_NONE, GLOVE_CMD_NONE);
    CHECK(!Frame::decode(badVersion, sizeof(badVersion), 0, 180, out));
    /* Out-of-range command id. */
    uint8_t badCmd[GLOVE_FRAME_LEN];
    std::memcpy(badCmd, good, sizeof(good));
    badCmd[GLOVE_IDX_META] =
        packMeta(GLOVE_PROTOCOL_VERSION, GLOVE_FLAG_NONE, GLOVE_CMD_MAX + 1);
    CHECK(!Frame::decode(badCmd, sizeof(badCmd), 0, 180, out));
    /* A rejected frame must leave the caller's struct exactly as it was. */
    CHECK_EQ(out.seq, untouched.seq);
    CHECK_EQ(out.channels[3], untouched.channels[3]);
    CHECK(!Frame::decode(0, GLOVE_FRAME_LEN, 0, 180, out));

    SECTION("GloveProtocol: decode clamps to the mechanical window");
    uint8_t extremes[GLOVE_FRAME_LEN];
    Frame wild = makeSampleFrame(1, GLOVE_CMD_NONE);
    wild.channels[0] = 0;
    wild.channels[1] = 255;
    CHECK_EQ(wild.encode(extremes, sizeof(extremes)), GLOVE_FRAME_LEN);
    Frame clamped;
    CHECK(Frame::decode(extremes, sizeof(extremes), GLOVE_RX_ANGLE_MIN,
                        GLOVE_RX_ANGLE_MAX, clamped));
    CHECK_EQ(clamped.channels[0], GLOVE_RX_ANGLE_MIN);
    /* 255 was clamped to 180 during encode, then to the mechanical maximum. */
    CHECK_EQ(clamped.channels[1], GLOVE_RX_ANGLE_MAX);

    SECTION("GloveProtocol: sequence numbers");
    CHECK_EQ(nextSeq(0), 1u);
    CHECK_EQ(nextSeq(254), 255u);
    CHECK_EQ(nextSeq(255), 0u); /* must wrap, not overflow */
    CHECK(seqIsAfter(1, 2));
    CHECK(seqIsAfter(250, 3));  /* wrap-around forward */
    CHECK(seqIsAfter(255, 0));
    CHECK(!seqIsAfter(5, 5));   /* duplicate */
    CHECK(!seqIsAfter(3, 250)); /* old frame, not a new one */
    CHECK(!seqIsAfter(0, 128)); /* exactly half the space is "not newer" */
    CHECK(seqIsAfter(0, 127));
    CHECK_EQ(seqGap(5, 6), 0u);
    CHECK_EQ(seqGap(5, 8), 2u);
    CHECK_EQ(seqGap(255, 1), 1u);
    CHECK_EQ(seqGap(5, 5), 0u);
    CHECK_EQ(seqGap(8, 5), 0u); /* out of order is not a gap */
}
