# RoboticArm — glove teleoperation for a 7-servo robotic hand

A data glove reads your wrist orientation (MPU-6050) and five finger flex
sensors, and streams the result over a 433 MHz ASK radio link to a robotic
hand/arm that mirrors the pose on seven servos.

```
   DATA GLOVE (Arduino Uno)                     ROBOTIC HAND (Arduino Uno)
 ┌───────────────────────────┐               ┌────────────────────────────┐
 │ MPU-6050  ── I2C ──┐      │   433 MHz ASK │      ┌── ServoTimer2 ──┐   │
 │ 5 × flex ── ADC ──┐│      │  ┌──────────┐ │      │ 7 servos        │   │
 │ button ─── GPIO ──┤│      │  │ 9-byte   │ │      │ pitch/roll      │   │
 │                   ▼▼      │  │ framed   ├─┼─────►│ thumb..little   │   │
 │  filter → calibrate →     │  │ packet   │ │      │                 │   │
 │  complementary attitude   │  └──────────┘ │      │ rate limited +  │   │
 │  → protocol frame         │               │      │ failsafe        │   │
 └───────────────────────────┘               └────────────────────────────┘
        hand_transmit/                              hand_receive/
```

This repository started as two sketches with no documentation. It has since
been rebuilt around a tested shared core: **every bug found in the original
firmware is fixed, documented and pinned by a test** (see
[docs/BUGS_FIXED.md](docs/BUGS_FIXED.md)), and the arm now has the safety
behaviour a mechanism with seven motors under radio control always should have
had.

---

## Contents

- [What was wrong, what changed](#what-was-wrong-what-changed)
- [Quick start](#quick-start)
- [Features](#features)
- [Repository layout](#repository-layout)
- [Build and test](#build-and-test)
- [Configuration](#configuration)
- [Documentation index](#documentation-index)
- [Status and limitations](#status-and-limitations)
- [Contributing](#contributing)
- [Licensing](#licensing)

---

## What was wrong, what changed

The original firmware could not have worked as written. Three show-stoppers,
all verified against the library sources they depend on:

| # | Original code | Effect | Fix |
|---|---------------|--------|-----|
| B-01 | `myservo1.write(msg[0])` with `ServoTimer2` | `ServoTimer2::write()` takes **microseconds** (750–2250), not degrees. Every value 0–180 was clamped to its 750 µs floor: all seven servos sat on one end stop and ignored all data. | `angleToPulseUs()` converts degrees → µs, tested and used for every write |
| B-02 | `uint8_t msg[2]` but `buflen = 7` and `msg[0..6]` read | Buffer over-read on every packet — undefined behaviour on a 2 KB-RAM MCU | `uint8_t buf[GLOVE_RX_BUFFER_LEN]`, length validated on every call |
| B-03 | `myservo2.write(msg[1]) … myservo2.write(msg[6])` | Copy-paste: six channels were written to one servo; servos 3–7 never moved | Each channel drives its own servo, through its own rate limiter |

…and 20 more: a flex sensor wired onto the I²C SDA pin, unconstrained `map()`
results truncated into `uint8_t`, a receive length that was never reset, no
frame validation, no failsafe, blocking serial and `vw_wait_tx()` in the sensor
loop, hard-coded calibration magic numbers, a raw accelerometer used as an
orientation estimate. The full audit — original snippet, impact, fix, and the
test that now guards it — is in **[docs/BUGS_FIXED.md](docs/BUGS_FIXED.md)**.

New in this revision:

- **Tested shared core** in `common/` (protocol, filtering, calibration maths,
  rate limiting, link supervision, attitude filter) compiled and unit tested on
  a desktop — 11 000+ assertions, `make -C tests check`.
- **Safety by construction**: mechanical angle clamps, slew-rate limiting,
  link-loss failsafe, remote-relax disabled by default, AVR watchdog, and a
  compile-time refusal of unsafe configuration.
- **A real protocol**: versioned 9-byte frame, sequence numbers, status flags,
  preset commands, and receiver-side link-quality statistics.
- **Calibration that persists**: capture each sensor's true span and direction,
  store it in EEPROM with a CRC, adjust it from a serial CLI without
  reflashing.
- **Proper wrist tracking**: a fixed-point complementary filter over the
  MPU-6050 (with bias capture and accelerometer rejection) instead of
  `map(ax, 17000, -17000, 0, 179)`.
- **Non-blocking firmware**: no `delay()`, no `vw_wait_tx()`; sensors, radio,
  telemetry and CLI all run on their own `millis()` schedules.
- **CI**: unit tests, an 8-configuration build matrix, negative tests for the
  safety guards, and real `avr-gcc` builds of both sketches for Uno and Mega.

## Quick start

1. **Build the hardware** — see [docs/HARDWARE.md](docs/HARDWARE.md) for the
   bill of materials, wiring tables and the power rules. Pay attention to the
   flex-sensor/I²C pin conflict documented there; the default firmware profile
   avoids it by using four flex sensors on A0–A3.
2. **Install the libraries** — VirtualWire, ServoTimer2, I2Cdev and MPU6050
   (`tools/build.sh` does this for you if you have
   [arduino-cli](https://arduino.github.io/arduino-cli/)).
3. **Flash the glove** — open `hand_transmit/hand_transmit.ino` in the Arduino
   IDE and upload to the glove's board.
4. **Flash the arm** — open `hand_receive/hand_receive.ino` and upload to the
   arm's board. **Both boards must run the same revision**: the protocol is
   versioned and the receiver says so on serial if they disagree.
5. **Power the servos from a proper supply** (5–6 V, ≥ 2 A per servo group,
   common ground with the Arduino). Never from the Arduino's USB 5 V pin.
6. **Calibrate the glove** — hold the glove button for 1.5 s (or send `c` over
   serial), move every finger through its full range, then `w` to store. Full
   procedure: [docs/CALIBRATION.md](docs/CALIBRATION.md).
7. **Tune the arm's poses and limits** in `common/GloveConfig.h`, re-sync and
   reflash: `python3 tools/sync_common.py`.

## Features

**Glove (`hand_transmit`)**

- MPU-6050 wrist pitch/roll via a complementary filter (integer-only, with a
  fixed-point `atan2`), plus gyro/accel bias capture at boot or on demand.
- Up to 5 flex channels through either direct ADC pins or a 74HC4051 analog
  multiplexer (board profiles in `GloveConfig.h`).
- Median-of-3 + exponential moving average filtering, per-channel calibration
  span and direction stored in EEPROM behind a CRC.
- Button: short press cycles preset poses, long press starts/stops calibration
  capture.
- Serial CLI: status, dump, calibration capture/save/reset, IMU bias, channel
  direction flip, preset trigger, telemetry toggle.
- Fully non-blocking; frames are transmitted whenever the radio is free, so the
  newest sample always wins.
- Runs (degraded, with the wrist held neutral and a flag saying so) if the IMU
  is missing or mis-wired.

**Arm (`hand_receive`)**

- Validates every frame: length, protocol version, command range, and clamps
  each channel into the configured mechanical window.
- Slew-rate limits every servo (default 300 °/s) — a corrupt packet becomes a
  slow correction, never a slam.
- Link supervision with packet-loss statistics and a configurable failsafe
  (hold / return to neutral / relax) after 500 ms without a valid frame.
- Preset poses (open, fist, home, point) latched for a configurable time, plus
  a `PARK` command that holds neutral until explicitly released.
- Power-on grace period: the arm moves gently to a known pose before it obeys
  anything.
- Serial CLI and link telemetry (`s` prints quality %, lost frames, failsafe
  events, commanded pulse widths).

## Repository layout

```
common/            shared core -- the only place logic is written
  GloveConfig.h        every tunable for both boards
  GlovePlatform.h      host/Arduino portability shims
  GloveMath.*          clamp, map, CRC-16
  GloveProtocol.*      frame layout, encode/decode, sequence maths
  SignalProcessing.*   EMA, median-of-3, auto-ranging, button, deadband
  Attitude.*           integer complementary filter + fixed-point atan2
  GloveCalibration.*   EEPROM calibration blob
  ServoDrive.*         angle→µs, slew limiter, link monitor, command latch
  RfLink.h             VirtualWire / RadioHead abstraction
hand_transmit/     glove sketch + generated copies of common/
hand_receive/      arm sketch   + generated copies of common/
tests/             native unit tests + Arduino API stubs (host builds)
tools/             sync, build, monitor, air-time, config matrix
docs/              all documentation
ci/                 GitHub Actions workflow (see the install note inside)
```

`common/` is the single source of truth. The Arduino toolchain only compiles
files inside a sketch folder, so the copies next to each `.ino` are generated
by `tools/sync_common.py` — edit `common/`, never the copies. CI fails if they
drift.

## Build and test

```bash
make -C tests check           # unit tests + compile/link/smoke-run both sketches
make -C tests test            # unit tests only (11k+ assertions)
make -C tests sketch-check    # both sketches against the host stubs
./tools/check_configs.sh      # build the whole GloveConfig variant matrix
./tools/build.sh              # real avr-gcc build (needs arduino-cli)
python3 tools/sync_common.py  # regenerate the copies inside the sketch folders
python3 tools/rf_airtime.py --budget
```

No AVR hardware and no Arduino toolchain are needed for anything except the
last one. See [docs/TESTING.md](docs/TESTING.md).

## Configuration

Everything lives in `common/GloveConfig.h`: board profile, pin maps, radio
driver and speed, filtering strengths, servo pulse ranges, mechanical angle
limits, slew rate, failsafe behaviour and timeout, preset poses, telemetry
formats and the watchdog. After editing it, run `python3 tools/sync_common.py`
and reflash **both** boards. Reference table:
[docs/FIRMWARE.md](docs/FIRMWARE.md).

## Documentation index

| Document | What it covers |
| --- | --- |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | System design, data flow, module map, design constraints |
| [HARDWARE.md](docs/HARDWARE.md) | BOM, wiring tables, pin conflicts, power, assembly |
| [PROTOCOL.md](docs/PROTOCOL.md) | Frame layout, commands, versioning, air-time maths |
| [FIRMWARE.md](docs/FIRMWARE.md) | Build/flash, configuration reference, serial CLI, telemetry |
| [CALIBRATION.md](docs/CALIBRATION.md) | Calibrating flex sensors, wrist range and IMU bias |
| [SAFETY.md](docs/SAFETY.md) | Failsafe, limits, first power-on checklist |
| [PERFORMANCE.md](docs/PERFORMANCE.md) | Latency and resource budgets, what was optimised |
| [TESTING.md](docs/TESTING.md) | Test strategy, host stubs, CI, adding tests |
| [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Symptom → cause → fix |
| [BUGS_FIXED.md](docs/BUGS_FIXED.md) | Audit of the original firmware |
| [ROADMAP.md](docs/ROADMAP.md) | Suggested next steps and rejected alternatives |
| [CHANGELOG.md](docs/CHANGELOG.md) | What changed, and the breaking ones flagged |

## Status and limitations

Be honest about what this is:

- **Not tested on hardware in this revision.** The logic is unit tested and
  both sketches compile, link and smoke-run against host stubs across eight
  build configurations, but no physical glove/arm was available. Follow
  [docs/SAFETY.md](docs/SAFETY.md) for the first power-on procedure
  (servos disconnected, then connected one at a time).
- **The ASK radio is the bottleneck.** A 9-byte frame occupies the air for
  ~96 ms at 2000 bps, which caps the control rate at ~10 Hz. That is inherent
  to VirtualWire/RH_ASK framing, not to this code. `tools/rf_airtime.py`
  quantifies it; [docs/ROADMAP.md](docs/ROADMAP.md) describes the nRF24L01+
  upgrade that would take it to ~1 kHz.
- **Yaw is not tracked.** Pitch and roll come from gravity + the gyros; yaw
  would drift without a magnetometer, so it is deliberately not attempted.
- **Four fingers by default on an Uno.** The MPU-6050 owns A4/A5, which leaves
  four ADC pins. The 74HC4051 profile (`GLOVE_PROFILE_UNO_MUX`) or a Mega
  restores all five — see [docs/HARDWARE.md](docs/HARDWARE.md).
- **Pose angles are mechanism-specific.** `GLOVE_RX_*_POSE` ships with neutral
  values; you must tune them to how your servo horns were splined on.

## Contributing

1. Edit logic in `common/`, then `python3 tools/sync_common.py`.
2. Add a test in `tests/` for anything that makes a decision.
3. `make -C tests check` and `./tools/check_configs.sh` must pass.
4. Update `docs/` in the same commit — a configuration knob that is not in
   `docs/FIRMWARE.md` does not exist.
5. Bump `GLOVE_PROTOCOL_VERSION` and note it in `docs/CHANGELOG.md` if the
   over-the-air frame changes.

## Licensing

The firmware in this repository has no licence file yet, and that is worth
fixing deliberately rather than by accident: **VirtualWire is GPL-2.0** and
RadioHead is GPL-2/3 (or commercial), so a project that links them should be
distributed under a GPL-compatible licence. ServoTimer2 and the jrowberg
I2Cdev/MPU6050 libraries carry their own terms (MIT-style). See
[docs/ROADMAP.md](docs/ROADMAP.md#licensing) for the recommendation.
