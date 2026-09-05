# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/spec/v2.0.0.html) with one addition:
**the over-the-air protocol has its own version** (`GLOVE_PROTOCOL_VERSION`,
currently 2), and a change to it is always a breaking change for a mixed pair of
boards.

## [Unreleased]

Nothing pending.

---

## [2.0.0] — 2026-09-05

A ground-up rebuild of both sketches around a tested shared core. Every bug
found in the original firmware is fixed, documented in
[BUGS_FIXED.md](BUGS_FIXED.md) with its original snippet, and pinned by a test.

### Breaking changes

These will bite an existing build. Read them before flashing.

- **The receiver sketch folder was renamed** `hand_recieve/` → `hand_receive/`
  (B-24). Re-open the sketch from its new path.
- **The radio protocol changed and is now versioned.** The frame is 9 bytes
  (`SEQ`, `META`, `CH0..CH6`) instead of 7 positional bytes, `META` carries a
  2-bit protocol version, 2 flag bits and a 4-bit command. **A glove and an arm
  running different versions will not talk** — the arm prints
  `WARN protocol mismatch` and counts rejected frames. Flash both boards.
- **Servo output is now in microseconds, as ServoTimer2 requires** (B-01). Any
  code or configuration that assumed degrees was producing a servo clamped to
  its 750 µs floor. `GLOVE_RX_PULSE_MIN_US`/`MAX_US` replace any implicit
  assumption.
- **Serial baud rate is 115200**, not 38400.
- **Servo behaviour on power-on changed**: the arm now slews to
  `GLOVE_RX_NEUTRAL_POSE` over `GLOVE_RX_STARTUP_POSE_MS` (1.5 s) and ignores
  the radio during that window. Expect motion at reset that did not happen
  before.
- **On link loss the arm moves.** It previously held its last pose forever
  (B-08); it now applies `GLOVE_RX_FAILSAFE_ACTION` (default: slew to neutral)
  after 500 ms. If your mechanism must not move when the link drops, set the
  action to `GLOVE_FAILSAFE_HOLD`.
- **Calibration is no longer hard-coded.** The 90/220 flex constants survive
  only as factory defaults; a new glove should be calibrated (`c`, then `w`) or
  it will run on defaults that describe someone else's hardware.
- **All configuration moved to `common/GloveConfig.h`**, and the shared core is
  copied into each sketch folder by `tools/sync_common.py`. Editing
  `hand_receive/GloveConfig.h` directly will be overwritten.

### Added

**Safety**

- Mechanical angle window (`GLOVE_RX_ANGLE_MIN`/`MAX`) applied to every value
  that reaches a servo, including presets and failsafe poses.
- Per-servo slew-rate limiting (`SlewLimiter`, default 300 °/s) so no command
  can slam a joint (B-09).
- Link supervision with a configurable failsafe — `HOLD`, `NEUTRAL` or `RELAX`
  — plus loss/stale/failsafe counters and a link-quality percentage (B-08).
- Power-on grace period: gentle slew to a known pose before obeying the radio.
- `attachServos()` snaps software to hardware, removing the jump caused by
  ServoTimer2's 1500 µs attach default.
- Rejected frames do **not** refresh the link timeout, so a channel full of
  valid-FCS garbage reads as a lost link.
- Remote relax is compiled out by default (`GLOVE_RX_ALLOW_REMOTE_RELAX 0`).
- AVR watchdog on both boards, with the `WDRF`-clearing sequence that avoids
  the classic bootloader reset loop (B-21).
- Compile-time guards that refuse dangerous configurations: failsafe timeout
  shorter than three frame air times, command repeat shorter than two, more
  servos than channels or than ServoTimer2 can drive, empty mechanical window,
  sampling slower than sending, unknown board profile, and `static_assert`s on
  the frame geometry, the receive buffer size and the EEPROM blob size.

**Protocol and link**

- Versioned 9-byte frame with sequence numbers, status flags and commands
  (`OPEN`, `FIST`, `HOME`, `POINT`, `PARK`, `RESUME_LIVE`, `RELAX`).
- `Frame::decode()` validates length, version and command, and clamps every
  channel into the arm's mechanical window (B-07, B-18).
- Wrap-safe sequence arithmetic (`nextSeq`, `seqIsAfter`, `seqGap`) so loss is
  measurable and a glove restart is not counted as 127 lost frames.
- `RfLink.h`: one API over VirtualWire and RadioHead `RH_ASK`, with a software
  transmit deadline for the RadioHead path (which has no `tx_active()`
  equivalent) and an air-time calculator.
- Command repeat window (`GLOVE_TX_CMD_REPEAT_MS`, 400 ms) so a button press
  survives packet loss.

**Sensing and calibration**

- `AttitudeFilter`: an all-integer complementary filter over the MPU-6050 with
  `isqrt32`, a fixed-point `atan2Q8` (~0.3° worst case), Q8 degrees with
  residual carry, a `dt` clamp, and an accelerometer trust gate at 600–1600
  milli-g that ignores specific force during motion (B-12).
- IMU bias capture with a flatness and magnitude check that refuses an
  implausible capture, plus boot-time capture and retries.
- Per-glove calibration in EEPROM: spans, per-channel direction, wrist windows
  and IMU biases, behind a CRC-16/CCITT-FALSE, verified by read-back on save
  and re-validated on load (B-14).
- `MedianFilter3` + `EmaFilter` (`AnalogChannel`) and a 2° deadband, removing
  servo buzz from ADC and RF noise (B-13).
- Graceful degradation: with no IMU the wrist is held neutral, `IMU_OK` stays
  clear, and `GLOVE_IMU_ENABLE 0` compiles the I²C stack out entirely (B-15).

**Operator interface**

- Serial CLI on both boards (glove: status, dump, capture, save, reset, IMU
  bias, direction flip, preset, telemetry; arm: status, park, live, attach,
  relax, telemetry).
- Rate-limited telemetry in human-readable or CSV form, switchable at runtime,
  and compilable out.
- Glove button: short press cycles presets, long press captures calibration.
- Status LED: link/mode indication on the arm, per-frame blink on the glove.
- Boot banners on both boards reporting protocol version, radio driver, speed,
  servo count and failsafe policy.
- Loop-timing instrumentation (`loop max : NNN us` in `s` on both boards) — the
  blocking detector that proves the schedulers really are non-blocking.

**Board support**

- `GLOVE_PROFILE` system: `UNO_4FLEX` (default), `UNO_MUX` (5–8 sensors through
  a 74HC4051/CD4051) and `MEGA_5FLEX`. This resolves the flex-sensor/I²C
  collision on A4/A5 that made the original glove's fifth channel and its IMU
  mutually exclusive (B-05).
- Preset poses, servo direction inversion and pulse ranges are all
  configuration, not code.

**Verification and tooling**

- `common/`: a shared core of 15 files, compiled by both sketches and by the
  tests, with no Arduino dependency outside `RfLink.h`.
- `tests/`: 305 check sites executing **11 037 assertions** across six modules,
  built with `-Wall -Wextra -Werror -Wshadow -Wconversion`.
- `tests/arduino_stub/`: signature-faithful stubs of `Arduino.h`, `Wire`,
  `EEPROM`, `VirtualWire`, `RH_ASK`, `ServoTimer2`, `I2Cdev`, `MPU6050` and
  `avr/wdt.h`, plus a smoke-test `main()` that runs `setup()` and 20 000
  `loop()` iterations (~20 s of simulated runtime) for both sketches.
- `tools/sync_common.py` (with `--check` for CI), `tools/build.sh` (arduino-cli
  build with automatic core and library installation), `tools/check_configs.sh`
  (8-configuration build matrix), `tools/rf_airtime.py` (air-time and latency
  budget), `tools/glove_monitor.py` (serial dashboard, CLI bridge, CSV logger).
- `ci/ci.yml` (install to `.github/workflows/ci.yml` — see
  [TESTING.md](TESTING.md#8-continuous-integration)): unit tests, the
  configuration matrix, negative
  tests that require unsafe configurations to be refused, a shared-copy sync
  check, and real `avr-gcc` builds for Uno and Mega.
- `docs/`: architecture, hardware, protocol, firmware/configuration reference,
  calibration, safety, performance, testing, troubleshooting, this changelog,
  the bug audit and the roadmap.
- `.gitignore` (build artefacts, `arduino-libraries/`, Python caches) and
  `.editorconfig`.

### Changed

- **Both loops are non-blocking.** `vw_wait_tx()` and `delay(10)` are gone
  (B-10): the glove samples on 5 ms and 10 ms ticks and offers a frame to the
  radio every 20 ms *only if the radio is idle*, counting deferred frames.
  The glove's effective sensor rate went from ~10 Hz to 200 Hz (flex) and
  100 Hz (IMU).
- Serial output is out of the sampling path and rate-limited (B-11).
- I²C runs at 400 kHz instead of the Wire default, making each IMU read ~4×
  faster.
- Every conversion ends in a saturating clamp; `mapRange()` rounds and guards
  against a zero span, unlike Arduino's `map()` (B-06).
- The checksum used for persisted data is CRC-16/CCITT-FALSE. An earlier
  Fletcher/mod-255 sum was replaced because it aliases all-`0xFF` with
  all-`0x00` — precisely the states an erased EEPROM and a shorted bus produce.
- The arm no longer includes Wire, I2Cdev or MPU6050, and no longer calls
  `Wire.begin()` (B-17).
- Receive buffers are sized from configuration and `static_assert`ed against
  the frame length, and the receive length is reset before every call (B-02,
  B-04).
- Servo state is array-based, so "six channels written to one servo" (B-03) is
  no longer expressible.
- `AutoRanger`'s accessors are `lowest()`/`highest()` rather than
  `min()`/`max()`, which collide with the Arduino macros of the same name.
- Diagnostics are unambiguous and separated, with a documented CSV variant
  (B-22).
- No unused globals and no commented-out code (B-23); configuration variants
  are `#if` branches built by CI.

### Fixed

The complete list, with original code, impact, fix and the test that now guards
it, is in **[BUGS_FIXED.md](BUGS_FIXED.md)**. By identifier:

B-01 ServoTimer2 written in degrees · B-02 receive buffer overrun · B-03 six
channels onto one servo · B-04 receive length never reset · B-05 flex sensor on
the I²C bus · B-06 unconstrained `map()` truncated into a byte · B-07 no
validation of received values · B-08 no failsafe · B-09 no rate limiting ·
B-10 blocking transmit and delay · B-11 blocking serial in the hot loop ·
B-12 accelerometer used as an orientation estimate · B-13 no filtering · B-14
hard-coded calibration and inconsistent directions · B-15 IMU never checked ·
B-16 dead code that misleads · B-17 unused I²C stack on the arm · B-18 no
protocol version · B-19 change detection comparing the wrong quantity · B-20
uninitialised globals as protocol state · B-21 no watchdog · B-22 garbled
serial output · B-23 dead variables and commented-out blocks · B-24 misspelled
sketch folder · B-25 nothing was tested.

### Removed

- `hand_recieve/` (renamed, see above).
- ~40 unused globals and every commented-out code block from both sketches.
- The receiver's unused I²C stack.
- `vw_wait_tx()` and every `delay()` in the control paths.

### Known limitations (unchanged or introduced deliberately)

- The control rate is bounded by ASK air time: ~10 Hz at 2000 bps, ~20 Hz at
  4000. See [PERFORMANCE.md](PERFORMANCE.md) and roadmap item 1.1.
- No yaw: pitch and roll only, because yaw needs a magnetometer.
- Four fingers by default on an Uno; five need the mux profile or a Mega.
- The protocol version field is 2 bits, so version 3 is the last one available
  before it must widen.
- The sketches' scheduling logic is smoke-tested but not unit-asserted;
  roadmap item 5.1 addresses that.
- No position feedback, no collision detection, no force limiting — see
  [SAFETY.md](SAFETY.md#7-what-this-project-does-not-protect-against).
- No `LICENSE` file yet. VirtualWire is GPL-2.0, so the choice is constrained;
  see [ROADMAP.md](ROADMAP.md#licensing).

---

## [0.1.0] — original import (commit `aed1b32`, "Added First Files")

The project as it was found: two Arduino sketches and nothing else.

- `hand_transmit/hand_transmit.ino` — MPU-6050 and five flex sensors on A0–A4,
  `map()` to 0–179/0–180, seven positional bytes over VirtualWire at 2000 bps,
  `vw_wait_tx()` + `delay(10)` per frame, serial output every iteration at
  38400 baud.
- `hand_recieve/hand_recieve.ino` — VirtualWire receive into a 2-byte buffer
  with a claimed length of 7, seven `ServoTimer2` objects attached to pins
  3/9/4/5/6/7/8, six of the seven channels written to servo 2, values passed to
  `write()` as degrees.
- No shared code, no configuration, no tests, no documentation, no licence, no
  `.gitignore`, no CI.

It is preserved in git history and quoted throughout
[BUGS_FIXED.md](BUGS_FIXED.md). It could not have driven a servo correctly:
B-01 alone held all seven servos at one end stop.

[Unreleased]: https://github.com/KetanDutt/RoboticArm/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/KetanDutt/RoboticArm/compare/aed1b32...v2.0.0
[0.1.0]: https://github.com/KetanDutt/RoboticArm/commit/aed1b32
