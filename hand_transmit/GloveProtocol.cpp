/*
 * GloveProtocol.cpp -- frame encode/decode and sequence helpers.
 *
 * Shared file: edit `common/GloveProtocol.cpp` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/PROTOCOL.md.
 */
#include "GloveProtocol.h"

namespace glove {

void Frame::clear() {
    seq = 0;
    flags = GLOVE_FLAG_NONE;
    command = GLOVE_CMD_NONE;
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        channels[i] = 0;
    }
}

uint8_t packMeta(uint8_t version, uint8_t flags, uint8_t command) {
    return static_cast<uint8_t>(
        ((version & GLOVE_META_VERSION_MASK) << GLOVE_META_VERSION_SHIFT) |
        ((flags & GLOVE_META_FLAGS_MASK) << GLOVE_META_FLAGS_SHIFT) |
        (command & GLOVE_META_COMMAND_MASK));
}

uint8_t metaVersion(uint8_t meta) {
    return static_cast<uint8_t>((meta >> GLOVE_META_VERSION_SHIFT) &
                                GLOVE_META_VERSION_MASK);
}

uint8_t metaFlags(uint8_t meta) {
    return static_cast<uint8_t>((meta >> GLOVE_META_FLAGS_SHIFT) &
                                GLOVE_META_FLAGS_MASK);
}

uint8_t metaCommand(uint8_t meta) {
    return static_cast<uint8_t>(meta & GLOVE_META_COMMAND_MASK);
}

uint8_t Frame::encode(uint8_t *raw, uint8_t rawLen) const {
    if (raw == 0 || rawLen < GLOVE_FRAME_LEN) return 0;
    raw[GLOVE_IDX_SEQ] = seq;
    raw[GLOVE_IDX_META] = packMeta(GLOVE_PROTOCOL_VERSION, flags, command);
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        /* Never transmit an out-of-range angle, even if the caller built one
         * by hand: the receiver clamps too, but belt and braces is cheap. */
        raw[GLOVE_IDX_CHANNELS + i] =
            static_cast<uint8_t>(clampInt(channels[i], GLOVE_ANGLE_FLOOR,
                                          GLOVE_ANGLE_CEILING));
    }
    return GLOVE_FRAME_LEN;
}

bool Frame::decode(const uint8_t *raw, uint8_t len, int16_t angleMin,
                   int16_t angleMax, Frame &out) {
    if (raw == 0 || len != GLOVE_FRAME_LEN) return false;

    const uint8_t meta = raw[GLOVE_IDX_META];
    if (metaVersion(meta) != GLOVE_PROTOCOL_VERSION) return false;

    const uint8_t cmd = metaCommand(meta);
    if (cmd > GLOVE_CMD_MAX) return false;

    /* Decode into a temporary so a rejected frame leaves `out` untouched. */
    Frame tmp;
    tmp.seq = raw[GLOVE_IDX_SEQ];
    tmp.flags = metaFlags(meta);
    tmp.command = cmd;
    for (uint8_t i = 0; i < GLOVE_CHANNEL_COUNT; ++i) {
        tmp.channels[i] = static_cast<uint8_t>(
            clampInt(raw[GLOVE_IDX_CHANNELS + i], angleMin, angleMax));
    }

    out = tmp;
    return true;
}

uint8_t nextSeq(uint8_t seq) { return static_cast<uint8_t>(seq + 1u); }

bool seqIsAfter(uint8_t last, uint8_t candidate) {
    if (candidate == last) return false;
    /* Unsigned 8-bit subtraction gives the forward distance, wrap-safe. */
    const uint8_t forward = static_cast<uint8_t>(candidate - last);
    return forward <= 127u;
}

uint8_t seqGap(uint8_t from, uint8_t to) {
    if (from == to) return 0;
    const uint8_t forward = static_cast<uint8_t>(to - from);
    if (forward > 127u) return 0; /* duplicate / out of order, not a gap */
    return static_cast<uint8_t>(forward - 1u);
}

} /* namespace glove */
