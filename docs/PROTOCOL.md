# Radio protocol

Everything that crosses the air, and why it looks the way it does.

Defined in `common/GloveProtocol.h` / `.cpp`, currently **version 2**.

- [1. Layering](#1-layering)
- [2. Frame layout](#2-frame-layout)
- [3. Fields](#3-fields)
- [4. A frame on the wire](#4-a-frame-on-the-wire)
- [5. Versioning and compatibility](#5-versioning-and-compatibility)
- [6. Link supervision](#6-link-supervision)
- [7. Air time and latency](#7-air-time-and-latency)
- [8. API reference](#8-api-reference)
- [9. Extending the protocol](#9-extending-the-protocol)
- [10. What the protocol deliberately does not do](#10-what-the-protocol-deliberately-does-not-do)

## 1. Layering

VirtualWire and RH_ASK already provide, per packet:

- a 36-bit preamble and start symbol (receiver synchronisation),
- a length byte,
- 6-bit symbol expansion of every payload byte,
- a **16-bit FCS** checked by the driver — a corrupted packet is discarded
  before `receive()` ever sees it.

So this layer does not need a start-of-frame marker or its own checksum. What
it adds is everything the radio *cannot* know:

| Added by this protocol | Why |
| --- | --- |
| protocol version | so a half-flashed system reports a mismatch instead of twitching |
| sequence number | so packet loss is measurable, and duplicates/stale frames are detectable |
| status flags | so the arm knows whether the wrist data is real |
| commands | so a button press can do something other than mirror |
| hard clamping on decode | so a decoded-but-weird value can never reach a servo |

## 2. Frame layout

A frame is a **9-byte snapshot of the whole hand**. There are no partial
updates and no inter-frame state: any single frame is sufficient to reproduce
the pose. That is what makes packet loss survivable and duplicate delivery
harmless.

```
 byte  0      1        2       3      4       5       6       7       8
      +------+------+--------+-------+-------+-------+-------+-------+-------+
      | SEQ  | META | CH0    | CH1   | CH2   | CH3   | CH4   | CH5   | CH6   |
      +------+------+--------+-------+-------+-------+-------+-------+-------+
                     wrist    wrist   thumb   index   middle  ring    little
                     pitch    roll

      META:  bits 7-6  version   (GLOVE_PROTOCOL_VERSION & 0x03)
             bits 5-4  flags     (GLOVE_FLAG_*)
             bits 3-0  command   (GLOVE_CMD_*)
```

| Symbol | Value |
| --- | --- |
| `GLOVE_FRAME_LEN` | 9 |
| `GLOVE_CHANNEL_COUNT` | 7 |
| `GLOVE_IDX_SEQ` / `GLOVE_IDX_META` / `GLOVE_IDX_CHANNELS` | 0 / 1 / 2 |

## 3. Fields

### `SEQ` (byte 0)

Free-running 8-bit counter, incremented once per transmitted frame
(`nextSeq()`). It exists purely to make loss visible; nothing depends on its
absolute value. Wrap-around is handled by `seqIsAfter()` / `seqGap()`:

```cpp
/* distance of `to` from `from`, allowing for wrap */
uint8_t d = static_cast<uint8_t>(to - from);
if (d == 0 || d > 127) return 0;      /* duplicate, out-of-order, or a restart */
return d - 1;                         /* frames lost in between */
```

A forward distance greater than 127 is treated as *stale*, not as 200 lost
frames — which is exactly what the glove rebooting looks like, and keeps the
quality statistic meaningful.

### `META` (byte 1)

Packed by `packMeta(version, flags, command)`, unpacked by `metaVersion()`,
`metaFlags()`, `metaCommand()`. Each field is masked to its width on the way
in, so a programming error cannot corrupt its neighbour.

### Flags (META bits 5–4)

Flags use **positive logic**: a flag is set when the good thing is true. The
reason is fail-safe by construction — a zeroed META byte (erased flash, a
garbage frame that survived the FCS, a glove that has not finished booting)
reads as "no IMU", which is the interpretation the arm can act on safely.

| Flag | Value | Meaning |
| --- | --- | --- |
| `GLOVE_FLAG_NONE` | 0x00 | no IMU, not calibrating |
| `GLOVE_FLAG_IMU_OK` | 0x01 | the glove has a live MPU-6050; CH0/CH1 are real attitude angles |
| `GLOVE_FLAG_CAL_ACTIVE` | 0x02 | the glove is currently capturing calibration |

When `IMU_OK` is clear the glove still transmits CH0/CH1, but as its configured
neutral wrist angles — so the arm holds the wrist steady rather than following
noise. Both flags are two bits, so `0x03` is "IMU present and calibrating".
Unknown flag combinations are reported in telemetry, not rejected: a newer
glove can add flags without breaking an older arm.

### Commands (META bits 3–0)

| Command | Value | Effect on the arm | Lifetime |
| --- | --- | --- | --- |
| `GLOVE_CMD_NONE` | 0 | live mirroring | — |
| `GLOVE_CMD_POSE_OPEN` | 1 | latch `GLOVE_RX_OPEN_POSE` | `GLOVE_RX_POSE_HOLD_MS` (2 s) |
| `GLOVE_CMD_POSE_FIST` | 2 | latch `GLOVE_RX_FIST_POSE` | 2 s |
| `GLOVE_CMD_POSE_HOME` | 3 | latch `GLOVE_RX_NEUTRAL_POSE` | 2 s |
| `GLOVE_CMD_POSE_POINT` | 4 | latch `GLOVE_RX_POINT_POSE` | 2 s |
| `GLOVE_CMD_PARK` | 5 | hold neutral, ignore live angles | until `RESUME_LIVE` |
| `GLOVE_CMD_RESUME_LIVE` | 6 | back to live mirroring (also clears a relax latch) | — |
| `GLOVE_CMD_RELAX` | 7 | detach the servos — **only honoured if `GLOVE_RX_ALLOW_REMOTE_RELAX` is 1** | until resumed |
| `GLOVE_CMD_MAX` | 7 | anything above this makes `decode()` fail | — |

Commands are **repeated for `GLOVE_TX_CMD_REPEAT_MS` (400 ms ≈ four frames)**
while the glove's button latch is set, because at ~10 frames/s with real packet
loss a single-shot command would be lost regularly. Re-latching is idempotent:
it just restarts the hold timer.

`GLOVE_CMD_RELAX` deserves its own paragraph. Letting go of a loaded mechanism
on a radio command is how fingers get pinched, so it is **compiled out by
default**. With `GLOVE_RX_ALLOW_REMOTE_RELAX 0` the arm counts such a frame as
rejected and keeps holding. Enabling it is a deliberate act, documented in
[SAFETY.md](SAFETY.md).

### Channel bytes (2–8)

Absolute servo **angles in degrees**, 0–180. Not microseconds, not deltas.
The arm converts to pulse width itself, so the mechanical calibration lives
with the mechanism it describes.

| Channel | Index | Joint | Glove source |
| --- | --- | --- | --- |
| `GLOVE_CH_WRIST_PITCH` | 0 | wrist pitch | `AttitudeFilter::pitchDeg()` → `wristToAngle()` |
| `GLOVE_CH_WRIST_ROLL` | 1 | wrist roll | `AttitudeFilter::rollDeg()` → `wristToAngle()` |
| `GLOVE_CH_FINGER_FIRST` | 2 | thumb | flex 0 → `flexToAngle()` |
| — | 3 | index | flex 1 |
| — | 4 | middle | flex 2 |
| — | 5 | ring | flex 3 |
| `GLOVE_CH_FINGER_LAST` | 6 | little | flex 4, or `GLOVE_FLEX_ABSENT_ANGLE` if that sensor does not exist |

A finger with no physical sensor is filled with the absent angle (90°) rather
than omitted, so the frame length never depends on the build. That matters:
the arm cannot tell "glove has four sensors" from "glove sent a short frame",
and only the second one is an error.

## 4. A frame on the wire

Hand flat, fingers half curled, IMU present, no command latched:

```
 3F | 90 | 5A | 5A | 6E | 78 | 64 | 5F | 5A
 SEQ  META pitch roll thumb index middle ring little
```

`META = 0x90 = 0b10_01_0000` → version 2, flags `IMU_OK`, command `NONE`.

The same hand in other states:

| Situation | version | flags | cmd | META |
| --- | --- | --- | --- | --- |
| normal, IMU present | 2 | `IMU_OK` (0x1) | NONE | `0x90` |
| normal, calibrating | 2 | `IMU_OK \| CAL_ACTIVE` (0x3) | NONE | `0xB0` |
| IMU failed to boot | 2 | 0x0 | NONE | `0x80` |
| button → fist preset | 2 | `IMU_OK` | POSE_FIST (2) | `0x92` |
| button → park | 2 | `IMU_OK` | PARK (5) | `0x95` |
| resume live | 2 | `IMU_OK` | RESUME_LIVE (6) | `0x96` |

Wrist channels of `0x5A` = 90° = neutral; fingers 110°, 120°, 100°, 95°, 90°.

Over the air, the driver wraps these 9 bytes in its own framing, which is why
they cost 192 bits rather than 72 — see [air time](#7-air-time-and-latency).

## 5. Versioning and compatibility

- `GLOVE_PROTOCOL_VERSION` is **2**. It occupies 2 bits, so the usable values
  are 0–3; version 4 would transmit as 0.
- `Frame::decode()` rejects any frame whose version field is not exactly
  `GLOVE_PROTOCOL_VERSION`. The arm prints
  `WARN protocol mismatch: frame vX arm vY` (rate-limited) and increments
  `g_framesRejected`.
- Consequence, and it is the point: **flashing only one board produces a
  visibly non-working link**, with a message that says why, instead of a
  subtly wrong one. The original firmware would happily drive servos from a
  differently shaped packet.
- Version 0 means "the original firmware", version 1 the first rewrite. Both
  are rejected, never interpreted.
- When version 3 is used up, the next change must widen the field to a whole
  byte — a known, tracked limitation ([ROADMAP.md](ROADMAP.md)), and a good
  moment to add a link-quality or battery field at the same time.

## 6. Link supervision

Implemented by `LinkMonitor` (`common/ServoDrive.cpp`), fed by the arm's
`drainRadio()`.

| Event | How it is detected | What happens |
| --- | --- | --- |
| frame accepted | `decode()` returned true | `packetReceived(now, seq)`, timeout refreshed |
| frame rejected | wrong length, version or command | `g_framesRejected++`; **the timeout is NOT refreshed** |
| packet lost | `seqGap()` > 0 | `lostPackets` += gap |
| duplicate / out of order | `seqIsAfter()` false | `stalePackets++`, but the timeout *is* refreshed — a duplicate still proves the link is alive |
| glove restarted | forward distance > 127 | counted as stale, not as 127 lost frames |
| link lost | `msSincePacket(now) > GLOVE_RX_FAILSAFE_TIMEOUT_MS` (500 ms) | failsafe pose applied, `failsafeEvents++` on the transition |
| link restored | first good frame after a timeout | failsafe released, `LINK OK q=NN%` printed |
| quality | `good × 100 / (good + lost)`, cumulative since boot | reported in telemetry and in `s` |

A rejected frame deliberately does **not** refresh the timeout. If the only
traffic on the channel is garbage, that is a lost link, not a live one.

### Compile-time guards

Three checks at the top of `hand_receive.ino`, plus the ones in
`GloveProtocol.h`, mean a dangerous configuration cannot be flashed at all:

```cpp
/* GloveProtocol.h -- shared with the host tests */
#if GLOVE_RX_SERVO_COUNT > GLOVE_CHANNEL_COUNT
#error "GLOVE_RX_SERVO_COUNT cannot exceed the protocol's channel count"
#endif
#if GLOVE_TX_CMD_REPEAT_MS < (2u * GLOVE_AIRTIME_MS)
#error "GLOVE_TX_CMD_REPEAT_MS must cover at least two frame air times"
#endif
#if GLOVE_RX_FAILSAFE_TIMEOUT_MS < (3u * GLOVE_AIRTIME_MS)
#error "GLOVE_RX_FAILSAFE_TIMEOUT_MS must be at least three frame air times"
#endif

/* hand_receive.ino -- a limit of the servo library itself */
#if GLOVE_RX_SERVO_COUNT > 8
#error "ServoTimer2 drives at most 8 channels"
#endif
```

Note that the timing guards are written in terms of `GLOVE_AIRTIME_MS`, which
is derived from the frame length and the radio speed. Change either and the
guard re-evaluates: you cannot accidentally leave a 500 ms timeout behind when
you move to a 20-byte frame at 1000 bps.

CI compiles both sketches with `-DGLOVE_RX_FAILSAFE_TIMEOUT_MS=50` and with
`-DGLOVE_PROFILE=99` and **requires the build to fail**. A guard that is never
exercised is a comment, not a check.

There is deliberately no guard tying `GLOVE_RX_POSE_HOLD_MS` to the failsafe
timeout: if the link dies while a preset is latched, the failsafe *should* win.
Safety outranks the pose.

## 7. Air time and latency

VirtualWire/RH_ASK put **12 bits on air per payload byte** (4-bit nibbles
expanded to 6-bit symbols), plus 84 bits of framing:

```
GLOVE_AIRTIME_BITS = 12 × 9 + 84          = 192 bits
GLOVE_AIRTIME_MS   = 192 × 1000 / 2000    =  96 ms   (at 2000 bps)
                                          =  48 ms   (at 4000 bps)
theoretical maximum = 1000 / 96           ≈ 10.4 frames/s
```

`tools/rf_airtime.py --budget` prints the end-to-end picture:

```
  glove samples sensors          ~     5 ms   (GLOVE_TX_SAMPLE_MS)
  radio air time                 ~  96.0 ms   (at 2000 bps)
  arm drains + decodes frame     ~     1 ms
  arm servo refresh              ~    20 ms   (GLOVE_RX_UPDATE_MS)
  servo mechanical response      ~   100 ms   (typical hobby servo, no load)
  --> command latency            ~ 122 ms (8.2 Hz control rate)
```

What follows from that:

- **The radio sets the control rate, not the CPU.** The glove samples every
  5 ms — 20× faster than it can transmit — so the newest sample always wins.
  Making the glove faster changes nothing.
- **`GLOVE_RF_SPEED_BPS 4000` halves the latency** to ~74 ms and doubles the
  frame rate to ~20 Hz, at the cost of range and noise immunity. It is a
  one-line change; test it in your environment.
- **Every extra payload byte costs 6 ms at 2000 bps.** An eighth servo channel
  is not free, and it pushes on the failsafe-timeout guard.
- 96 ms is why the failsafe timeout is 500 ms (≈ 5 frames) and the command
  repeat window is 400 ms (≈ 4 frames). They are not round numbers chosen for
  aesthetics.

## 8. API reference

```cpp
namespace glove {

struct Frame {
    uint8_t seq;
    uint8_t flags;
    uint8_t command;
    uint8_t channels[GLOVE_CHANNEL_COUNT];

    void clear();

    /* Fill every field from a raw payload.  Returns true only when `len` is
     * exactly GLOVE_FRAME_LEN, the version matches, the command is known, and
     * every channel could be clamped into [angleMin, angleMax].  On failure
     * `out` is untouched. */
    static bool decode(const uint8_t *raw, uint8_t len,
                       int16_t angleMin, int16_t angleMax, Frame &out);

    /* Serialise into `raw`; returns GLOVE_FRAME_LEN, or 0 if the buffer is too
     * small.  Channels are clamped to 0..180 on the way out. */
    uint8_t encode(uint8_t *raw, uint8_t rawLen) const;
};

uint8_t packMeta(uint8_t version, uint8_t flags, uint8_t command);
uint8_t metaVersion(uint8_t meta);
uint8_t metaFlags(uint8_t meta);
uint8_t metaCommand(uint8_t meta);

uint8_t nextSeq(uint8_t seq);
bool    seqIsAfter(uint8_t last, uint8_t candidate);
uint8_t seqGap(uint8_t from, uint8_t to);   /* frames lost between them */

}
```

`decode()` clamping into `[angleMin, angleMax]` is what lets the arm pass its
*mechanical* window (`GLOVE_RX_ANGLE_MIN`/`MAX`) straight into the protocol
layer: an out-of-range channel is not an error, it is a value that gets pulled
back inside the safe window.

## 9. Extending the protocol

1. Change `common/GloveProtocol.h` and bump `GLOVE_PROTOCOL_VERSION`.
2. Update `common/GloveConfig.h` — channel count, pose arrays, servo pins.
3. Update the glove's `buildFrame()` and the arm's `updateServos()`.
4. Extend `tests/test_protocol.cpp`: layout, round-trip, version rejection,
   unknown-command rejection, clamping, and the sequence-wrap cases.
5. Run `python3 tools/rf_airtime.py --payload <new length>` and re-check the
   timing guards — they may now reject your old failsafe timeout, which is
   them doing their job.
6. `python3 tools/sync_common.py && make -C tests check && ./tools/check_configs.sh`.
7. Flash **both** boards, and record the protocol change in
   [CHANGELOG.md](CHANGELOG.md) — mismatched boards will not talk.

## 10. What the protocol deliberately does not do

| Not done | Why | What replaces it |
| --- | --- | --- |
| Acknowledgements | One-way ASK hardware cannot acknowledge; a bidirectional protocol needs different modules | Sequence numbers + loss statistics + a failsafe give the same *safety* property without a return path |
| Retransmission | On a control link a stale retransmission is worse than a dropped frame | The next frame arrives 96 ms later with newer data; the command repeat window covers the few commands that must not be lost |
| Delta / variable-length encoding | Fixed layout means a corrupt length can never desynchronise the parser | If air time ever matters more than simplicity, see [ROADMAP.md](ROADMAP.md) |
| Encryption or authentication | ASK is a public band and the payload is a pose, not a secret; a rolling code costs bytes and complexity | The arm validates everything it can, clamps everything else, and refuses remote relax by default |
| A magic/sync byte | The radio driver already verifies a 16-bit FCS and delivers whole messages | The version field is worth far more than a constant byte would be |
| A checksum of our own | Same reason; it would cost 2 of the 9 bytes | `crc16Ccitt()` is used where it matters — the EEPROM calibration blob, which has no radio FCS protecting it |
