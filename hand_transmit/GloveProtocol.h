/*
 * GloveProtocol.h -- the over-the-air frame format shared by the glove
 *                    (transmitter) and the arm (receiver).
 *
 * A frame is the payload handed to VirtualWire/RH_ASK, which adds its own
 * preamble, length byte and 16-bit FCS around it, so a corrupted frame is
 * already rejected by the radio driver.  What this layer adds is:
 *
 *   - a protocol version, so a mismatched pair of boards says so loudly
 *     instead of twitching randomly,
 *   - a rolling sequence number, so the receiver can measure packet loss and
 *     reject stale/duplicate frames,
 *   - status flags and one-shot commands (preset poses, park, resume),
 *   - hard clamping of every channel on decode, so a decoded-but-weird value
 *     can never reach a servo.
 *
 * Layout (9 bytes, little ceremony, all big-endian-free plain bytes):
 *
 *   byte 0  SEQ   rolling 0..255 sequence number
 *   byte 1  META  bits 7-6  protocol version (see GLOVE_PROTOCOL_VERSION)
 *                 bits 5-4  status flags     (see GLOVE_FLAG_*)
 *                 bits 3-0  command          (see GLOVE_CMD_*)
 *   byte 2  CH0   wrist pitch, servo angle
 *   byte 3  CH1   wrist roll,  servo angle
 *   byte 4  CH2   thumb   \
 *   byte 5  CH3   index    |  finger servo angles
 *   byte 6  CH4   middle   |
 *   byte 7  CH5   ring     |
 *   byte 8  CH6   little  /
 *
 * Full description, air-time budget and version history: docs/PROTOCOL.md.
 *
 * Pure C++ with no Arduino dependency, so it is unit tested in tests/.
 *
 * Shared file: edit `common/GloveProtocol.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef GLOVE_PROTOCOL_H
#define GLOVE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#include "GloveMath.h"
#include "GloveConfig.h"

/* Bump when the layout above changes in any incompatible way, and update
 * docs/PROTOCOL.md + docs/CHANGELOG.md in the same commit. */
#define GLOVE_PROTOCOL_VERSION 2u

/* --- geometry ---------------------------------------------------------- */
#define GLOVE_FRAME_LEN       9u   /* total payload bytes */
#define GLOVE_CHANNEL_COUNT   7u   /* CH0..CH6 */
#define GLOVE_IDX_SEQ         0u
#define GLOVE_IDX_META        1u
#define GLOVE_IDX_CHANNELS    2u   /* first channel byte */

/* --- META byte field masks --------------------------------------------- */
#define GLOVE_META_VERSION_SHIFT 6u
#define GLOVE_META_VERSION_MASK  0x03u   /* 2 bits */
#define GLOVE_META_FLAGS_SHIFT   4u
#define GLOVE_META_FLAGS_MASK    0x03u   /* 2 bits */
#define GLOVE_META_COMMAND_MASK  0x0Fu   /* 4 bits */

/* --- status flags (META bits 5-4) -------------------------------------- */
#define GLOVE_FLAG_NONE        0x00u
#define GLOVE_FLAG_IMU_OK      0x01u   /* glove has a live MPU-6050 */
#define GLOVE_FLAG_CAL_ACTIVE  0x02u   /* glove is capturing calibration */

/* --- commands (META bits 3-0) ------------------------------------------ */
#define GLOVE_CMD_NONE         0u
#define GLOVE_CMD_POSE_OPEN    1u
#define GLOVE_CMD_POSE_FIST    2u
#define GLOVE_CMD_POSE_HOME    3u
#define GLOVE_CMD_POSE_POINT   4u
#define GLOVE_CMD_PARK         5u   /* go to neutral now and stay there */
#define GLOVE_CMD_RESUME_LIVE  6u   /* release a latched pose / park */
#define GLOVE_CMD_RELAX        7u   /* detach servos (only if permitted) */
#define GLOVE_CMD_MAX          GLOVE_CMD_RELAX

/* --- compile-time geometry checks ----------------------------------------
 * static_assert (C++11, which the AVR core has used since Arduino 1.6) so a
 * layout change that forgets one of these constants cannot be built at all. */
static_assert(GLOVE_CHANNEL_COUNT + 2u == GLOVE_FRAME_LEN,
              "a frame is SEQ + META + GLOVE_CHANNEL_COUNT channels");
static_assert(GLOVE_CHANNEL_COUNT == GLOVE_FINGER_CHANNELS + 2u,
              "channels are 2 wrist axes plus GLOVE_FINGER_CHANNELS fingers");
static_assert(GLOVE_RX_BUFFER_LEN >= GLOVE_FRAME_LEN,
              "GLOVE_RX_BUFFER_LEN must hold a whole frame");
static_assert(GLOVE_CMD_MAX <= GLOVE_META_COMMAND_MASK,
              "commands must fit in the META command field");
static_assert(GLOVE_FLAG_CAL_ACTIVE <= GLOVE_META_FLAGS_MASK,
              "flags must fit in the META flags field");

/* --- air time ------------------------------------------------------------
 * VirtualWire and RH_ASK put 12 bits on the air per payload byte, plus 84 bits
 * of preamble, start symbol, length byte and FCS.  Everything time-related in
 * this project is derived from this one number; see docs/PROTOCOL.md. */
#define GLOVE_AIRTIME_BITS  ((GLOVE_FRAME_LEN * 12u) + 84u)
#define GLOVE_AIRTIME_MS    ((GLOVE_AIRTIME_BITS * 1000u) / GLOVE_RF_SPEED_BPS)

/* --- configuration sanity checks that need the frame geometry ------------
 * Refused at compile time rather than discovered at run time. */
#if GLOVE_RX_SERVO_COUNT > GLOVE_CHANNEL_COUNT
#error "GLOVE_RX_SERVO_COUNT cannot exceed the protocol's channel count"
#endif

/* A command is repeated for this long, so one lost frame must not swallow it. */
#if GLOVE_TX_CMD_REPEAT_MS < (2u * GLOVE_AIRTIME_MS)
#error "GLOVE_TX_CMD_REPEAT_MS must cover at least two frame air times"
#endif

/* Below three air times the failsafe would trip during normal operation. */
#if GLOVE_RX_FAILSAFE_TIMEOUT_MS < (3u * GLOVE_AIRTIME_MS)
#error "GLOVE_RX_FAILSAFE_TIMEOUT_MS must be at least three frame air times"
#endif

/* --- servo angle window: see GLOVE_ANGLE_FLOOR/CEILING in GloveConfig.h -- */
/* Channel roles, so calling code never depends on byte positions. */
#define GLOVE_CH_WRIST_PITCH   0u
#define GLOVE_CH_WRIST_ROLL    1u
#define GLOVE_CH_FINGER_FIRST  2u /* thumb */
#define GLOVE_CH_FINGER_LAST   6u /* little */

namespace glove {

/* A decoded frame ready to be applied to servos. */
struct Frame {
    uint8_t seq;
    uint8_t flags;
    uint8_t command;
    uint8_t channels[GLOVE_CHANNEL_COUNT];

    void clear();
    /* Fill every field from a raw payload.  Returns true when `len` is
     * exactly GLOVE_FRAME_LEN, the version matches and every channel could
     * be clamped into [angleMin, angleMax].  On failure `out` is untouched
     * except for `out.seq`, which is only written on success. */
    static bool decode(const uint8_t *raw, uint8_t len, int16_t angleMin,
                       int16_t angleMax, Frame &out);
    /* Serialise into `raw`; returns the number of bytes written
     * (GLOVE_FRAME_LEN) or 0 if the buffer is too small.  Channels are
     * clamped to 0..180 on the way out. */
    uint8_t encode(uint8_t *raw, uint8_t rawLen) const;
};

/* Pack a META byte from its three fields (each is masked to its width). */
uint8_t packMeta(uint8_t version, uint8_t flags, uint8_t command);
uint8_t metaVersion(uint8_t meta);
uint8_t metaFlags(uint8_t meta);
uint8_t metaCommand(uint8_t meta);

/* Sequence helpers ------------------------------------------------------ */
uint8_t nextSeq(uint8_t seq);
/* True when `candidate` is after `last`, allowing for 8-bit wraparound.
 * A forward distance of more than 127 is treated as "old", not "new". */
bool seqIsAfter(uint8_t last, uint8_t candidate);
/* Number of frames lost between `from` and `to` (0 when they are
 * consecutive).  Returns 0 for duplicates/out-of-order pairs. */
uint8_t seqGap(uint8_t from, uint8_t to);

} /* namespace glove */

#endif /* GLOVE_PROTOCOL_H */
