# Testing

What is verified, how, and — just as important — what is *not* verified.

- [1. Strategy](#1-strategy)
- [2. Running the checks](#2-running-the-checks)
- [3. Unit tests](#3-unit-tests)
- [4. The Arduino stub layer](#4-the-arduino-stub-layer)
- [5. Sketch smoke tests](#5-sketch-smoke-tests)
- [6. Configuration matrix](#6-configuration-matrix)
- [7. Negative tests](#7-negative-tests)
- [8. Continuous integration](#8-continuous-integration)
- [9. Adding a test](#9-adding-a-test)
- [10. What is not tested](#10-what-is-not-tested)

## 1. Strategy

The firmware runs on a 2 KB-RAM microcontroller attached to a radio and seven
motors. It cannot be debugged by print statements alone, and a mistake is
observable as a broken mechanism rather than a stack trace. So the verification
is arranged in four layers, cheapest and most numerous first:

| Layer | Runs on | Catches | Cost |
| --- | --- | --- | --- |
| **Unit tests** of `common/` | host (g++) | wrong maths, wrong logic, wrong edge cases | seconds |
| **Sketch smoke tests** — compile, link, run `setup()` + 20 000 × `loop()` | host, against stubs | missing symbols, bad types, dead config branches, crashes, runaway loops | seconds |
| **Configuration matrix** — 8 build variants | host | a profile or driver that nobody uses today but that is silently broken | ~10 s |
| **Negative tests** — configurations that must be *refused* | host | safety guards that have quietly stopped working | seconds |
| **Real avr-gcc build** — Uno and Mega | CI | flash/RAM overflow, AVR-specific compile errors, library incompatibility | minutes |
| **Bench procedure** | hardware | everything else | [SAFETY.md](SAFETY.md#3-first-power-on-procedure) |

The design rule that makes the first four layers possible: **all logic lives in
`common/`, and `common/` has no Arduino dependency** (except `RfLink.h`, which
wraps the radio drivers). The sketches contain hardware access and scheduling;
the decisions are in testable modules. When a bug is found, the fix goes into
`common/` and gets a test, rather than into a sketch where nothing can verify
it.

The tests compile the *same `.cpp` files* the firmware compiles — not copies,
not reimplementations. `tests/Makefile` has `vpath %.cpp ../common`, so a test
can never drift away from the code it protects.

## 2. Running the checks

```bash
make -C tests test          # unit tests only
make -C tests sketch-check  # compile + link + smoke-run both sketches
make -C tests check         # both — this is what CI runs
make -C tests clean

./tools/check_configs.sh              # the whole configuration matrix
./tools/check_configs.sh radiohead    # one configuration by name

python3 tools/sync_common.py --check  # shared copies have not drifted
./tools/build.sh                      # real avr-gcc build (needs arduino-cli)
```

Requirements: a C++11 compiler (`g++`) and GNU make. No Arduino toolchain, no
hardware, no network.

A passing run ends with:

```
11037 checks, 0 failures
ALL TESTS PASSED
== sketch-check: hand_transmit ==
   compiled, linked and smoke-ran setup() + 20000 loop() passes
== sketch-check: hand_receive ==
   compiled, linked and smoke-ran setup() + 20000 loop() passes
ALL HOST CHECKS PASSED
```

The unit tests are built with `-Wall -Wextra -Werror -Wshadow -Wconversion`, so
a narrowing conversion or a shadowed name in `common/` fails the build. The
sketch builds use `-Wall -Wextra` without `-Werror`, because the Arduino core
API itself is not `-Wconversion` clean — but they are expected to be
warning-free, and CI shows any warning.

## 3. Unit tests

305 check sites, executed **11 037 assertions** (most sites are inside sweeps
and loops). One file per module.

| File | Sections | Notable properties pinned |
| --- | --- | --- |
| `test_math.cpp` (31 sites) | `clampInt`, `mapRange`, `crc16Ccitt`, `deltaU` | saturating clamp at INT32 bounds; `mapRange` rounding, inverted ranges, zero span (must not divide by zero); **CRC-16/CCITT-FALSE known-answer test `"123456789"` → `0x29B1`**; wrap-safe time subtraction |
| `test_protocol.cpp` (50) | META fields, encode, decode round trip, junk rejection, clamping, sequence numbers | frame length; version mismatch rejected; unknown command rejected; short/long buffer rejected; channels clamped into the mechanical window; `nextSeq` wrap at 255; `seqIsAfter`/`seqGap` correct across the wrap and for duplicates |
| `test_signal.cpp` (48) | `EmaFilter`, `MedianFilter3`, `AnalogChannel`, `AutoRanger`, `outsideDeadband`, `Button` | EMA step response and **no int32 overflow at extreme alphas**; median rejects a single outlier; deadband hysteresis in both directions; ranger `lowest()`/`highest()`/`span()`/`valid()`; button debounce, short press, long press firing exactly once while held |
| `test_servo.cpp` (71) | `angleToPulseUs`, `SlewLimiter` ×4, `LinkMonitor` ×2, `CommandLatch` | **the microseconds contract that B-01 violated**: endpoints, midpoint, inversion, clamping outside the window, monotonicity; slew rate per step, exact traverse time, fractional accumulation, `snap()`, **`millis()` rollover**; timeout transition counting, sequence accounting incl. duplicates and restarts; pose hold and expiry |
| `test_attitude.cpp` (46) | `isqrt32`, `atan2Q8` ×2, `AttitudeFilter` ×7 | `isqrt32` against `sqrt()` over a sweep; **`atan2Q8` against `atan2f` over the full circle, worst error well under one servo step (~0.3°)**; level-and-still equilibrium; convergence on a static tilt; gyro integration rate and sign; **accelerometer vectors outside the trust gate are ignored**; gyro bias removes drift; slow rates integrate instead of rounding away (the residual-carry fix); `reset()` and full state reset in `begin()` |
| `test_calibration.cpp` (59) | layout, defaults, corruption, flex mapping, direction, span capture, degenerate span, wrist windows | `sizeof(CalibrationData)` and member order are EEPROM-safe; defaults validate; **every single-byte corruption is detected by the CRC**; both direction mappings; `flexUsable` threshold; a degenerate stored span falls back to neutral rather than producing a wild angle |

Two of those sections exist because of specific bugs and are worth naming:

- *angleToPulseUs (the ServoTimer2 microseconds bug)* — B-01. If anyone ever
  "simplifies" that function back to passing degrees, the test fails.
- *AttitudeFilter: slow rates integrate instead of rounding away* — an early
  implementation lost fractional degrees per update, so a slowly rotating wrist
  stalled. The residual carry is asserted directly.

## 4. The Arduino stub layer

`tests/arduino_stub/` contains signature-faithful replacements for the Arduino
core and every third-party library the firmware uses:

```
Arduino.h        Wire.h        EEPROM.h      VirtualWire.h   RH_ASK.h
ServoTimer2.h    I2Cdev.h      MPU6050.h     avr/wdt.h       stubs.cpp
stub_main.cpp    README.md
```

Rules the stubs follow:

- **Signatures match the real libraries exactly**, including `const`ness,
  reference-vs-pointer parameters and return types. A call that would not
  compile against the real library does not compile against the stub. That is
  the property that makes `sketch-check` meaningful rather than decorative.
- **Behaviour is plausible, not faithful.** `analogRead()` returns a mid-scale
  value, `millis()` advances 1 ms per call, `vw_get_message()` returns no
  message, `ServoTimer2::write()` records the value. The stubs exist to let the
  firmware *run*, not to simulate hardware.
- `avr/wdt.h`, `MCUSR` and `_BV(WDRF)` are provided so the watchdog code path
  compiles and runs under `-D__AVR__`.
- Nothing in the stubs is allowed to influence the firmware's logic. If a test
  needs a specific sensor value, it drives the real module directly
  (`AttitudeFilter::update(ax, ay, az, …)`), not through a stub.

The stubs cannot catch: real interrupt timing, actual I²C behaviour, radio
reception, servo pulse accuracy, ADC noise, EEPROM wear, or anything about
flash/RAM size. Those need hardware or avr-gcc.

## 5. Sketch smoke tests

`make -C tests sketch-check` does, for each sketch:

1. copy `hand_x/hand_x.ino` to a `.cpp` (what the Arduino build does),
2. compile it with `-D__AVR__ -Wall -Wextra` against the stubs and the sketch's
   own synced copies of `common/`,
3. compile every `.cpp` in the sketch folder (the shared core),
4. compile `stubs.cpp` and `stub_main.cpp`,
5. **link** — so every referenced symbol must exist,
6. **run** `setup()` once and `loop()` 20 000 times.

Step 6 is what makes this more than a syntax check. With `millis()` advancing
1 ms per `loop()` call, 20 000 iterations is ~20 seconds of simulated runtime —
enough for every scheduler branch to fire many times: the 5 ms sample tick, the
10 ms IMU tick, the 20 ms send and servo ticks, the 400 ms command repeat
window, the 500 ms failsafe timeout, the 1500 ms startup grace period and the
2000 ms pose hold. It catches null-pointer dereferences, array overruns,
infinite loops, uninitialised state used as a divisor, and `millis()`
wrap-around arithmetic that is wrong in a way a single iteration would hide.

It does not assert on the firmware's *decisions* — those are the unit tests'
job, in modules the sketches call.

## 6. Configuration matrix

`tools/check_configs.sh` builds and smoke-runs both sketches in eight
configurations, so that every `#if` branch in `GloveConfig.h` is compiled by
something:

| Name | Overrides | What it exercises |
| --- | --- | --- |
| `default` | — | Uno, 4 flex, VirtualWire, 2000 bps, NEUTRAL failsafe |
| `uno-mux` | `GLOVE_PROFILE=2` | the 74HC4051 multiplexer path |
| `mega-5flex` | `GLOVE_PROFILE=3` | five direct ADC channels, Mega pin aliases |
| `radiohead` | `GLOVE_RF_DRIVER=2` | the `RH_ASK` branch of `RfLink.h`, including the software TX deadline (RH_ASK has no `tx_active()` equivalent) |
| `no-imu-csv` | `GLOVE_IMU_ENABLE=0`, both CSV flags | no I²C code at all; CSV telemetry on both ends |
| `no-cli-no-peripherals` | CLI, button, LED off; watchdog off | the `GLOVE_TX_HAS_OPERATOR_INPUT == 0` dead-code-elimination path |
| `failsafe-hold` | `GLOVE_RX_FAILSAFE_ACTION=0` | the HOLD branch |
| `failsafe-relax-4k` | RELAX + remote relax allowed + 4000 bps + telemetry off | the most permissive and fastest configuration, and the guards that must still hold at 4000 bps |

All eight must build **warning-free**. The script runs `sync_common.py --check`
first, because a stale synced copy would silently test the wrong configuration
— that failure mode was hit once during development and is now impossible.

## 7. Negative tests

A guard that has never been seen to fire is a comment. CI therefore builds with
configurations that must be **refused**:

| Override | Expected | Guard |
| --- | --- | --- |
| `-DGLOVE_RX_FAILSAFE_TIMEOUT_MS=50` | compile error | failsafe timeout < 3 air times (`GloveProtocol.h`) |
| `-DGLOVE_PROFILE=99` | compile error | unknown board profile (`GloveConfig.h`) |
| `-DGLOVE_RX_BUFFER_LEN=4` | compile error | `static_assert` buffer ≥ frame |
| `-DGLOVE_EMA_ALPHA_Q8=0` | compile error | Q8 weight out of range |
| `-DGLOVE_TX_SAMPLE_MS=50` | compile error | sampling slower than sending |
| `-DGLOVE_RX_ANGLE_MIN=175` | compile error | empty mechanical window |

CI runs the first two; the rest are available for a local
`make -C tests sketch-check EXTRA_FLAGS=...`. If you add a guard, add a
negative test for it in the same commit.

## 8. Continuous integration

The workflow lives at `ci/ci.yml` and must be copied to
`.github/workflows/ci.yml` to become active:

```bash
mkdir -p .github/workflows && cp ci/ci.yml .github/workflows/ci.yml
```

It is kept outside `.github/` because the automation that produced this branch
runs as a GitHub App without the `workflows` permission, and GitHub rejects such
a push outright. A push from a normal account has no such restriction, so this
is a one-time, one-command step. The file header repeats the instruction.

Two jobs:

**`host`** (ubuntu, seconds)

1. `sync_common.py --check` — the shared copies have not drifted
2. `make -C tests test` — 11 037 assertions
3. `./tools/check_configs.sh` — 8 configurations, warning-free
4. negative tests — the guards above must fire
5. `py_compile` on every tool
6. `rf_airtime.py --budget` — the latency budget, printed for the record

**`firmware`** (ubuntu, minutes)

1. install arduino-cli
2. `tools/build.sh` for `arduino:avr:uno` — installs the AVR core, clones
   VirtualWire, ServoTimer2, I2Cdev and MPU6050, builds both sketches and
   prints the flash/RAM report
3. the same for `arduino:avr:mega`

The firmware job is the only one that needs network access, and it is the only
check that can catch a flash-size regression or an incompatibility with the
real libraries.

## 9. Adding a test

1. Put the logic in `common/`. If it is in a sketch, it cannot be tested —
   that is a design signal, not a testing inconvenience.
2. Add cases to the matching `tests/test_<module>.cpp`, or create a new file
   and register its `run<Module>Tests()` in `tests/tests.h` and
   `tests/main.cpp`.
3. Use the existing macros: `CHECK(expr)`, `CHECK_EQ(actual, expected)`,
   `CHECK_NEAR(actual, expected, tolerance)`, and `SECTION("name")` to group.
4. Conventions the existing tests follow, and that keep them trustworthy:
   - **no heap, no randomness, no wall-clock time.** Pass time in explicitly
     (`update(target, nowMs)`), so a test is a deterministic function of its
     inputs.
   - compare fixed-point values with an explicit tolerance derived from the
     format (`CHECK_NEAR(..., 2)` for Q8 degrees ≈ 0.008°), never with `==`.
   - when testing against a reference implementation (`atan2f`, `sqrt`), sweep
     the whole input domain, not a few samples — that is how the `mag=1`
     quantisation edge in `atan2Q8` was found.
   - test the *contract*, including the failure modes: what happens at zero
     span, at the wrap boundary, with an out-of-range argument, after `begin()`
     is called twice.
5. `make -C tests check && ./tools/check_configs.sh`.

## 10. What is not tested

Stated plainly, so nobody over-trusts a green build:

| Not covered | Why | What covers it instead |
| --- | --- | --- |
| Real radio behaviour — reception, FCS, packet loss | needs two boards and an air interface | bench procedure; `q=`/`lost` telemetry in use |
| ServoTimer2's actual pulse timing | it is an ISR driving Timer2 | the microseconds contract is unit tested; the pulses are verified with a scope or by observation |
| I²C transactions with a real MPU-6050 | needs the device | `IMU: OK/NOT RESPONDING` at boot, plus retries |
| ADC noise and real sensor spans | hardware-specific | per-glove calibration, `d` dumps |
| Flash and RAM usage | host builds are not AVR builds | the CI `firmware` job's avr-gcc size report |
| Watchdog reset behaviour | requires an actual AVR reset | the `WDRF`-clearing sequence follows the documented AVR errata; verified by observation |
| EEPROM wear and write timing | needs hardware | writes are explicit and verified by read-back |
| Mechanical safety | not a software property | [SAFETY.md](SAFETY.md), and a human watching the first run |
| The sketches' *decisions* (as opposed to their compilation) | scheduling logic lives in the `.ino` files | partially: the smoke test runs it 20 000 times; fully: moving more of it into `common/` — see [ROADMAP.md](ROADMAP.md) |
