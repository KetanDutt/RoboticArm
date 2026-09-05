# Calibration

Two things need calibrating, on two different boards, for two different
reasons:

- the **glove** needs to know *your* hand — every flex sensor has a different
  span, and every IMU has its own bias;
- the **arm** needs to know *your* mechanism — servo horns are splined on by
  hand, so "90°" means something different on every joint.

- [1. What is stored where](#1-what-is-stored-where)
- [2. Calibrating the glove](#2-calibrating-the-glove)
- [3. Calibrating the IMU bias](#3-calibrating-the-imu-bias)
- [4. Verifying a calibration](#4-verifying-a-calibration)
- [5. Calibrating the arm](#5-calibrating-the-arm)
- [6. How the maths works](#6-how-the-maths-works)
- [7. When calibration goes wrong](#7-when-calibration-goes-wrong)

## 1. What is stored where

| Data | Where | Survives power-off? | How to change |
| --- | --- | --- | --- |
| Flex sensor spans + directions | glove EEPROM (`CalibrationData`) | Yes | `c` capture, `w` save |
| Wrist pitch/roll physical range | glove EEPROM | Yes | same capture |
| IMU accel/gyro zero bias | glove EEPROM | Yes | `b` (also captured at boot) |
| Servo mechanical window, neutral and preset poses | arm flash (`GloveConfig.h`) | Yes (it is compiled in) | edit, sync, reflash |
| Servo direction flips | arm flash | Yes | edit `GLOVE_RX_SERVO_INVERT` |
| Live trims (nothing yet) | — | — | — |

The split is deliberate: **per-glove data lives in EEPROM** because it changes
with the person wearing it; **per-mechanism data lives in the firmware** because
it changes when you rebuild the hand, and because it is safety-critical — a
mechanical limit that could be altered over a radio link or a serial port is not
a limit.

The EEPROM blob is `glove::CalibrationData`:

```cpp
struct CalibrationData {
    uint8_t  magic0, magic1;      /* 'G','C' */
    uint8_t  version;             /* GLOVE_CAL_VERSION */
    uint8_t  flexInvertMask;      /* bit n: finger n's raw value falls when bent */
    uint8_t  wristInvertMask;     /* bit 0 pitch, bit 1 roll */
    uint8_t  reserved;
    ChannelCal flex[5];           /* raw ADC low..high per finger */
    ChannelCal wrist[2];          /* physical degrees low..high */
    int16_t  accelBias[3];        /* raw LSB, glove flat and still */
    int16_t  gyroBias[3];         /* raw LSB, glove still */
    uint16_t checksum;            /* crc16Ccitt over everything above */
};
```

`static_assert(sizeof(CalibrationData) <= 128)` keeps it inside a small corner
of the 1 KB EEPROM. On boot the blob is loaded and rejected — falling back to
compile-time defaults — unless the magic bytes, the version and the CRC all
agree. A corrupted blob can therefore never produce a wild calibration; the
worst case is a glove that behaves like a fresh one.

## 2. Calibrating the glove

Do this once per glove, and again after any mechanical change (a sensor moved,
a resistor changed, a different glove).

**Preparation**

1. Power the glove, connect serial at 115200 baud.
2. Check the sensors respond: send `d`. Each finger channel should show a raw
   ADC value that moves by at least ~100 counts when you bend that finger. If
   one does not, fix the wiring first — calibration cannot rescue a dead
   sensor.
3. Put the glove on the hand it will be used with. Sensor position changes the
   span.

**Capture**

4. Send `c` (or hold the button for 1.5 s). You get:

   ```
   [cal] CAPTURING: slowly bend and straighten every finger, then tilt your
   wrist fully up/down and left/right
   ```

5. For the next `GLOVE_CAL_CAPTURE_MS` (8 seconds), move **every** joint through
   its **full, comfortable** range, slowly and repeatedly:
   - each finger: fully straight → fully curled → straight,
   - wrist: pitch down to its limit, up to its limit,
   - wrist: roll left to its limit, right to its limit.

   Do not exceed the range you actually want to use — the extremes you reach
   become the ends of the map. Do not rush: the capture records the minimum and
   maximum of the *filtered* signal, and a fast movement with a strong
   filter can miss its own peak.

6. The capture ends by itself after 8 s (or send `c` again to stop early):

   ```
   [cal] capture done: 6 channel(s) updated
   [cal] use 'w' to store it in EEPROM
   [cal] finger spans (raw low..high, direction):
      ch0: 92..214  span=122  falls-when-bent
      ch1: 88..219  span=131  falls-when-bent
      ch2: 95..201  span=106  falls-when-bent
      ch3: 90..216  span=126  falls-when-bent
      ch4: <no sensor on this profile>
   [cal] wrist pitch -51..48 deg, roll -66..63 deg
   ```

7. **Read that table before saving.** A channel marked `[UNUSABLE]` had a span
   narrower than `GLOVE_CAL_MIN_SPAN` (40 counts) and was *not* updated — it
   keeps its previous value. That means a sensor that was not moved, a sensor
   that is not connected, or a divider resistor that is too large.

8. Save with `w`:

   ```
   [cal] saved to EEPROM and verified
   ```

   The firmware writes the blob, reads it back and checks the CRC. If you ever
   see `EEPROM VERIFY FAILED`, the values are still live in RAM for this
   session but will not survive a power cycle — re-save, and if it keeps
   failing suspect the board's EEPROM.

**Direction**

If a finger moves the wrong way (bending it opens the hand), flip it:

```
i 2
[cal] channel 2 now rises-when-bent
w
```

`i <n>` toggles finger `n` (0 = thumb … 4 = little) and `w` stores it. There is
no need to re-capture: the span is stored as `low..high` and the direction is a
separate flag, so flipping is exact. The wrist equivalent is
`GLOVE_WRIST_INVERT_MASK` in `GloveConfig.h` (bit 0 pitch, bit 1 roll).

## 3. Calibrating the IMU bias

The attitude filter subtracts a zero bias from both the accelerometer and the
gyro. It is captured automatically at boot and can be re-captured with `b`.

**The glove must be lying flat and completely still.** The capture averages
`GLOVE_IMU_CAL_SAMPLES` (400) readings, then refuses the result unless it makes
physical sense:

- the specific-force magnitude must be within 900–1100 milli-g (i.e. the glove
  is not being accelerated or tilted), and
- neither horizontal accelerometer axis may exceed ~0.2 g (i.e. it really is
  flat).

```
[imu] accel offset captured at 1002 mG
[imu] gyro bias captured: -41 12 7
```

If it refuses:

```
[imu] bias capture rejected: hold the glove flat and still
```

…the glove moved, is tilted, or is sitting on something vibrating. Put it on a
level surface and try again. **A rejected capture leaves the previous bias in
place**, so a failed attempt cannot make things worse.

Notes:

- Gyro bias drifts with temperature. If the wrist angle wanders after the glove
  has been on for ten minutes, re-capture with `b` (and `w` to keep it).
- The accelerometer bias is stored as an offset from the *ideal* rest vector
  `(0, 0, +16384 LSB at ±2 g)`, not as the raw reading, so a glove that rests
  at a slight angle still gets a correct gravity reference.
- The attitude filter additionally ignores the accelerometer whenever the
  measured magnitude falls outside 600–1600 milli-g, so a swung or bumped hand
  cannot drag the estimate off true — see [PERFORMANCE.md](PERFORMANCE.md).

## 4. Verifying a calibration

`s` prints the whole state in one screen. Check these, in order:

| Check | Good | Bad, and what it means |
| --- | --- | --- |
| finger spans | 80–200 counts each | < 40: sensor not moving or wrong divider; `[UNUSABLE]`: not captured |
| direction | matches how you wired it | flipped finger → `i <n>` |
| wrist range | roughly ±50 pitch, ±65 roll | much smaller: you did not move the wrist fully; much larger: the IMU bias is wrong |
| IMU | `present` | `NOT FOUND` → wiring, AD0 address, or the A4/A5 conflict |
| angles at rest | all channels near their neutral | a channel pinned at 0 or 180 means the span is inverted or the sensor is disconnected |
| `tx sent/deferred` | sent rising, deferred roughly flat | deferred growing fast → you are outrunning the radio |

Then do the practical test: with the arm powered and in `LIVE`, move one joint
at a time and confirm that (a) only that servo moves, (b) it moves in the
expected direction, and (c) it reaches both ends of its comfortable range
without straining.

## 5. Calibrating the arm

The arm has no EEPROM calibration — its geometry is compiled in, in
`common/GloveConfig.h`. Tune it with the servos **powered from the external
supply** and the arm **able to move freely**.

1. **Set the mechanical window.** `GLOVE_RX_ANGLE_MIN` / `MAX` (default 10/170)
   is the hard clamp applied to every received angle, before anything else.
   Narrow it until nothing binds at either end. This is the single most
   valuable safety setting in the project: it is the difference between a joint
   that stops at its limit and one that stalls a servo against a link.
2. **Set the neutral pose.** `GLOVE_RX_NEUTRAL_POSE` is where the arm goes at
   power-on and on a `NEUTRAL` failsafe. For a hand, that is usually
   half-open, not the geometric middle.
3. **Set the presets.** `GLOVE_RX_OPEN_POSE`, `FIST`, `POINT` (preset 3 is
   `HOME` = neutral). Test each with the glove CLI: `p 1`, `p 2`, `p 4`.
4. **Fix directions.** If a servo moves opposite to the glove, set its entry in
   `GLOVE_RX_SERVO_INVERT` to 1. That swaps the pulse-width ends, which is
   cleaner than flipping the glove channel, because it keeps the glove's
   calibration independent of which hand it is on.
5. **Check the pulse range.** `GLOVE_RX_PULSE_MIN_US`/`MAX_US` (750/2250) suits
   most 180° servos. Continuous-rotation servos and some digital servos want
   1000/2000 — if a servo twitches or will not reach an end, this is why.
6. **Set the slew rate.** `GLOVE_RX_SLEW_MAX_DPS` (300) is a full sweep in
   600 ms. Lower it for a heavy arm or a fragile mechanism; raise it only if
   the arm feels sluggish *and* the power supply can take the inrush.

After every edit: `python3 tools/sync_common.py`, reflash the arm, retest.

A useful trick while tuning: send `n` on the arm's serial to **park** it. It
goes to the neutral pose and ignores the glove until you send `l`, so you can
edit config, reflash and compare poses without the glove fighting you.

## 6. How the maths works

Finger channel `n`, raw ADC reading `r`:

```
if the sensor falls when bent (invert bit set):  t = (high - r) / (high - low)
else:                                            t = (r - low) / (high - low)
angle = clamp(round(t × (CEILING - FLOOR)) + FLOOR, FLOOR, CEILING)
      = clamp(round(t × 180), 0, 180)
```

`mapRange()` in `common/GloveMath.cpp` does this with rounding and with a
guard against a zero span (which would otherwise divide by zero and return
garbage). `flexToAngle()` additionally falls back to the neutral angle for a
channel whose stored span is narrower than `GLOVE_CAL_MIN_SPAN` — an unusable
calibration must not produce a wild angle.

Wrist axis `a`, physical angle `d` from the attitude filter: exactly the same
map, with `low`/`high` in degrees (default −55…+55 pitch, −70…+70 roll) instead
of ADC counts.

Both paths end at the same hard limits (`GLOVE_ANGLE_FLOOR`/`CEILING`), and the
arm then applies its own, narrower window. Two independent clamps, at the two
ends of the link.

## 7. When calibration goes wrong

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `[cal] capture done: 0 channel(s) updated` | nothing moved far enough | move each joint fully; check `GLOVE_CAL_MIN_SPAN`; verify the sensors with `d` |
| A finger's angle is pinned at 0 or 180 | span inverted, or sensor disconnected | `i <n>`, or check the divider with a multimeter |
| Finger works but is twitchy | span too narrow (small divider swing) | use a divider resistor closer to the sensor's straight resistance |
| Wrist drifts slowly over minutes | gyro bias, or a warm-up drift | `b` then `w`; consider raising `GLOVE_IMU_COMP_ALPHA_PCT` |
| Wrist jumps when you move your arm fast | accelerometer being trusted during motion | it should not be — check that the trust gate constants in `common/Attitude.h` were not modified |
| Wrist is fine at rest, wrong when tilted | IMU mounted at an angle and bias captured while tilted | lay the glove flat, `b`, `w` |
| Calibration is lost every power cycle | EEPROM write failed | look for `EEPROM VERIFY FAILED`; try `r` then `w`; suspect the board |
| Arm reaches a joint limit and buzzes | mechanical window too wide | narrow `GLOVE_RX_ANGLE_MIN`/`MAX` |
| Arm moves the wrong joint for a finger | servo pin order does not match your build | reorder `GLOVE_RX_SERVO_PINS` |

More symptoms, and the ones that are not calibration-related:
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).
