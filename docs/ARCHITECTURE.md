# Architecture

How the system is put together, and why it is put together that way.

- [1. The two halves](#1-the-two-halves)
- [2. Data path](#2-data-path)
- [3. Repository layout](#3-repository-layout)
- [4. The shared core](#4-the-shared-core)
- [5. Design constraints](#5-design-constraints)
- [6. Design decisions and their trade-offs](#6-design-decisions-and-their-trade-offs)
- [7. Extending the system](#7-extending-the-system)

## 1. The two halves

Two independent programs, one shared core, one radio link.

| Board | Sketch | Job |
| --- | --- | --- |
| glove | `hand_transmit/hand_transmit.ino` | sense the hand, filter it, calibrate it, encode it, transmit it |
| arm | `hand_receive/hand_receive.ino` | receive it, validate it, limit it, drive seven servos with it |

The split is deliberate and asymmetric:

- The glove is **allowed to be wrong**. A noisy sample, a failed IMU, a
  half-finished calibration — the glove reports all of it in the frame's status
  flags and keeps transmitting.
- The arm is **not allowed to be wrong**. It cannot see the sensors, cannot
  verify the hand, and has a mechanism attached to it. So every value it acts
  on passes through validation, clamping and rate limiting, and the absence of
  data has a defined, safe meaning.

Nothing in the arm trusts the glove. That is the single most important property
of this design, and most of the code in `ServoDrive.cpp` exists to enforce it.

## 2. Data path

```
   GLOVE (hand_transmit)
   ───────────────────────────────────────────────────────────────────────
   5 × flex divider ──► analogRead ──► MedianFilter3 ──► EmaFilter ──┐
   (4 or 5, or via 74HC4051)                                        │
                                                                    ▼
                                                          AutoRanger (cal capture)
                                                                    │
   MPU-6050 ──I²C 400 kHz──► accel, gyro ──► AttitudeFilter ────────┤
   (200 Hz-ish sample)      (bias-corrected)  (complementary,        │
                                               integer, Q8 degrees)  │
                                                                     ▼
                                                    CalibrationData::flexToAngle()
                                                    CalibrationData::wristToAngle()
                                                                     │
                                                        deadband vs. last sent value
                                                                     ▼
                                                    glove::Frame {seq, flags, cmd, ch[7]}
                                                                     │
                                                          Frame::encode() → 9 bytes
                                                                     ▼
                                                    rflink::send()   (if the radio is idle)
                                                                     ▼
                                                          ~96 ms of air time @ 2000 bps

   ARM (hand_receive)
   ───────────────────────────────────────────────────────────────────────
   ASK RX ──► rflink::receive() ──► Frame::decode(len, version, cmd, clamp)
                                             │            │
                                       rejected?      accepted
                                             │            │
                                             ▼            ▼
                                    g_framesRejected   LinkMonitor::packetReceived(seq)
                                                          │
                                        ┌─────────────────┼──────────────────┐
                                        ▼                 ▼                  ▼
                                  no link yet       link healthy         link timed out
                                  → STARTUP         → LIVE / POSE        → FAILSAFE
                                        │                 │                  │
                                        └─────────────────┴──────────────────┘
                                                          ▼
                                              target angle per servo (clamped)
                                                          ▼
                                              SlewLimiter (≤ GLOVE_RX_SLEW_MAX_DPS)
                                                          ▼
                                              angleToPulseUs() → 750..2250 µs
                                                          ▼
                                              ServoTimer2::write() → Timer2 ISR
```

Three properties of that path are worth stating explicitly:

1. **No step blocks.** Both sketches are `millis()` schedulers. The glove does
   not call `delay()` or `vw_wait_tx()`; the arm does not wait for a packet.
   The original firmware blocked for ~90 ms per frame inside `vw_wait_tx()`,
   which is why its sensors were sampled so rarely.
2. **The radio is offered frames, not fed them.** If the transmitter is still
   busy when the next send slot arrives, the frame is dropped (`g_framesDeferred`
   counts these) and the newest sample wins. Stale control data is worse than no
   control data.
3. **Servo pulses are refreshed on their own clock** (`GLOVE_RX_UPDATE_MS`,
   20 ms), independently of when packets arrive, and ServoTimer2's Timer2
   interrupt keeps generating pulses between refreshes. A lost packet therefore
   cannot cause a servo to twitch.

## 3. Repository layout

```
common/                     the shared core — the only place logic lives
    GlovePlatform.h             host/Arduino portability shims
    GloveConfig.h               every tunable for both boards
    GloveMath.{h,cpp}           clampInt, mapRange, crc16Ccitt, deltaU
    GloveProtocol.{h,cpp}       frame layout, encode/decode, sequence maths
    SignalProcessing.{h,cpp}    EMA, median-of-3, AutoRanger, Button, deadband
    ServoDrive.{h,cpp}          angleToPulseUs, SlewLimiter, LinkMonitor,
                                CommandLatch
    GloveCalibration.{h,cpp}    the EEPROM calibration blob
    Attitude.{h,cpp}            integer complementary filter, atan2Q8, isqrt32
    RfLink.h                    VirtualWire / RadioHead abstraction

hand_transmit/              glove sketch + generated copies of common/
hand_receive/               arm sketch   + generated copies of common/

tests/                      native verification
    Makefile                    test / sketch-check / check
    arduino_stub/               Arduino + library API stubs, smoke-test main
    test_*.cpp                  one file per core module

tools/
    sync_common.py              common/ → sketch folders (--check for CI)
    check_configs.sh            build the whole configuration matrix
    build.sh                    real avr-gcc build via arduino-cli
    rf_airtime.py               air-time and latency budget
    glove_monitor.py            serial dashboard, CLI bridge, CSV logger

docs/                       this folder
ci/ci.yml                   GitHub Actions workflow (copy to
                            .github/workflows/ to activate -- see below)
```

**`common/` is the single source of truth.** The Arduino build only compiles
files inside a sketch folder, so `tools/sync_common.py` copies `common/` into
both sketch folders. Edit the originals, run the sync, commit both. CI runs
`sync_common.py --check` and fails if the copies have drifted.

## 4. The shared core

| Module | Responsibility | Why it is separate |
| --- | --- | --- |
| `GlovePlatform.h` | Makes `common/` compile on a host as well as on AVR: pin aliases (`GLOVE_PIN_*`), progmem shims | Lets every other module be unit tested on a PC |
| `GloveConfig.h` | All configuration, nothing else. Every knob wrapped in `#if !defined(...)` so it can be overridden from the command line | One file to read to know what the system does; the CI matrix depends on the override trick |
| `GloveMath` | `clampInt` (saturating, 64-bit-safe), `mapRange` (rounding, zero-span safe), `crc16Ccitt`, `deltaU` (wrap-safe time difference) | Arduino's `constrain()` and `map()` are subtly unsafe: no rounding, no overflow guard, `constrain` misbehaves with mixed signedness |
| `GloveProtocol` | The frame: geometry, META packing, `Frame::encode/decode`, sequence maths | Both boards must agree bit-for-bit; a shared definition makes disagreement impossible |
| `SignalProcessing` | `MedianFilter3`, `EmaFilter`, `AnalogChannel` (both, chained), `AutoRanger` (min/max capture for calibration), `Button` (debounce + long press), `outsideDeadband` | Sensor conditioning is where most of the "it feels jittery" complaints come from; it must be testable in isolation |
| `Attitude` | `isqrt32`, `atan2Q8`, `q8ToDeg`/`degToQ8`, `AttitudeFilter` (complementary, all-integer, Q8 degrees, residual carry, accelerometer trust gate) | Replacing `map(ax, 17000, -17000, 0, 179)` with something that is actually an orientation estimate, and doing it without floats on an ATmega328P |
| `GloveCalibration` | `CalibrationData`: magic, version, per-channel raw min/max/invert, wrist physical range, CRC-16; `flexToAngle`/`wristToAngle`; EEPROM load/save/verify | A glove that must be reflashed to fit a hand is a glove nobody calibrates |
| `ServoDrive` | `angleToPulseUs`, `SlewLimiter`, `LinkMonitor`, `CommandLatch` | This is the safety layer. It is shared so the *tests* exercise the same code the arm runs |
| `RfLink.h` | `begin/startRx/send/receive/txBusy/available`, driver name, air-time estimate. Wraps VirtualWire or RadioHead behind one API | VirtualWire is end-of-life; RadioHead is its maintained, wire-compatible successor. The project should not be welded to either |

`RfLink.h` is header-only and Arduino-only (it includes a radio driver), so it
is verified by compiling and smoke-running the sketches rather than by unit
tests.

### The three classes that matter most

**`SlewLimiter`** — one per servo. `update(target, now)` moves the commanded
angle toward the target by at most `maxDegPerSec × dt`, with unsigned wrap-safe
`dt`. The first update primes the time base instead of jumping. At the default
300 °/s a full 0→180° sweep takes 600 ms: fast enough to feel responsive, slow
enough that a corrupted frame cannot slam a joint.

**`LinkMonitor`** — the arm's view of the radio. `packetReceived(now, seq)`
counts good packets and derives loss from sequence gaps; a duplicate or
out-of-order frame (including the backward jump a glove reboot produces) is
counted as *stale* rather than as loss, but it still refreshes the timeout,
because a repeated frame proves the link is alive. `poll(now)` returns the
timed-out state and counts the transition into it, so failsafe events can be
reported. `qualityPercent()` is `good × 100 / (good + lost)` since boot. This
is what makes the failsafe a decision rather than a guess.

**`AttitudeFilter`** — integer complementary filter. Gyro integration provides
the fast path (Q8 degrees, with the fractional remainder carried between
updates so slow rotation does not stall); the accelerometer provides the slow
correction, but **only when the measured specific force is within
`GLOVE_ATTITUDE_TRUST_MIN_MG..MAX_MG` (600–1600 milli-g)** — i.e. when the hand
is roughly static. During movement the accelerometer says nothing useful about
gravity, so it is ignored rather than averaged in. `atan2Q8` is a fixed-point
cubic approximation with ~0.3° worst-case error; `begin()` fully resets biases
and axis signs so repeated initialisation cannot leak state.

## 5. Design constraints

Everything below is a real limit of the target hardware, and each one shaped the
code.

| Constraint | Value | Consequence |
| --- | --- | --- |
| MCU | ATmega328P @ 16 MHz | 32 KB flash, 2 KB SRAM, 1 KB EEPROM, no FPU |
| Timer1 | claimed by VirtualWire/RH_ASK | The stock `Servo` library cannot be used on the arm — it also wants Timer1 |
| Timer2 | claimed by ServoTimer2 | `analogWrite()` on pins 3 and 11 stops working |
| ServoTimer2 | `write()` takes **microseconds**, 750–2250, 8 channels max | Degrees must be converted; `GLOVE_RX_SERVO_COUNT ≤ 8` is enforced by a `#error` |
| VirtualWire/RH_ASK | 12 bits per payload byte on air, 84 bits of framing | 9-byte frame = 192 bits = **96 ms at 2000 bps** → ~10 Hz control rate ceiling |
| VW payload | ≤ 77 bytes | Not binding here, but it caps any future frame growth |
| ASK radio | one-way, unacknowledged, shared band | Packet loss is normal. Hence sequence numbers, a failsafe, and repeated commands |
| Uno ADC | A4/A5 are SDA/SCL | Five flex sensors + an I²C IMU do not fit — hence the board profiles |
| SRAM | 2 KB | No `String`, no dynamic allocation, no large buffers. All arrays are fixed-size and sized from config |
| EEPROM | 1 KB, ~100k write cycles | One calibration blob (`sizeof(CalibrationData) ≤ 128`), written only on explicit save, CRC-protected |
| Flash | 32 KB | No float maths in hot paths, no printf, `F()` for every string literal |

## 6. Design decisions and their trade-offs

| Decision | Why | What it costs |
| --- | --- | --- |
| Fixed 9-byte full-state frame, no delta encoding | Robust to loss: any single frame is sufficient to reproduce the pose. No resynchronisation state | ~96 ms of air time; an 8th channel would cost another 6 ms |
| Protocol version in the frame (2 bits) | A half-flashed system says `WARN protocol mismatch` instead of twitching | Only 4 versions before the field must widen |
| No magic/sync byte in the payload | The radio driver already checks a 16-bit FCS and delivers whole messages | A foreign ASK device on the same channel could deliver garbage that passes the FCS — mitigated by version + command + range validation |
| Flags use positive logic (`IMU_OK`, `CAL_ACTIVE`) | A zeroed META byte reads as "no IMU", which is the safe interpretation | Slightly less intuitive than `IMU_MISSING` |
| Complementary filter instead of the MPU-6050 DMP | DMP needs ~6 KB of program memory plus a 1.9 kB firmware blob, and is a black box that cannot be unit tested | No yaw; pitch/roll only, and they drift slowly if the accelerometer stays untrusted |
| Integer/fixed-point everywhere (Q8 degrees, Q8 filter weights) | No FPU: a `float atan2f` is ~1 ms on AVR; `atan2Q8` is ~20 µs | One more thing to test — and it is tested against `atan2f` across a full sweep |
| Calibration in EEPROM behind a CRC-16 | Survives power cycles; a corrupt blob is detected and defaults are used | Wear-limited writes; hence explicit `w` to save |
| CRC-16/CCITT-FALSE, not a Fletcher/mod-255 sum | Fletcher **aliases** all-0xFF with all-0x00 — exactly the two states an unwritten/erased EEPROM and a shorted bus produce | 2 bytes instead of 1, ~2 µs per computation |
| Receiver-side slew limiting, not glove-side | The arm cannot trust the glove. A limit enforced at the last possible moment always holds | Large intentional motions take up to 600 ms |
| `GLOVE_RX_ALLOW_REMOTE_RELAX` defaults to **0** | A garbled or malicious frame must never be able to let go of the arm | Relaxing requires the serial CLI (or a deliberate config change) |
| Failsafe defaults to `GLOVE_FAILSAFE_NEUTRAL`, not HOLD | Returning to a known pose is safer than freezing while holding an object under load | If the arm is gripping something, neutral may drop it — hence `HOLD` is available and documented |
| Compile-time guards for dangerous configurations | A misconfiguration that would defeat the failsafe is refused by the compiler, not discovered at runtime | Two `#error` paths that CI must prove actually fire |
| One shared core copied into both sketches | The Arduino build has no other way to share code between sketches | A sync step, enforced by CI |
| VirtualWire by default, RadioHead optional | VirtualWire is what this project has always used and is easy to obtain; RadioHead is maintained and wire-compatible | Two driver paths kept compiling in CI |

## 7. Extending the system

**Add an eighth servo.** Widen `GLOVE_CHANNEL_COUNT` and `GLOVE_FRAME_LEN` in
`GloveProtocol.h`, extend the pose arrays and `GLOVE_RX_SERVO_PINS` in
`GloveConfig.h`, bump `GLOVE_PROTOCOL_VERSION`, extend `tests/test_protocol.cpp`,
then re-run `python3 tools/rf_airtime.py` — every extra byte is 6 ms of air time
at 2000 bps, which feeds directly into the failsafe-timeout guard. ServoTimer2
supports 8 channels, so 8 is the hard ceiling on this driver.

**Add a sensor to the glove.** Read it into an `AnalogChannel`, convert it with
a new `CalibrationData` accessor (keep the conversion in `common/` so it is
testable), and assign it a channel index. If it needs an ADC pin that does not
exist, use the mux profile rather than stealing A4/A5.

**Replace the radio.** Implement the functions in `RfLink.h` for the new module.
An nRF24L01+ or HC-12 removes the 96 ms ceiling entirely; see
[ROADMAP.md](ROADMAP.md). Note that Timer1 becomes free with a non-ASK driver,
which would allow the stock `Servo` library and therefore more than 8 channels.

**Change a safety limit.** Every limit is a `#define` in `GloveConfig.h`, and
every one of them is documented in [FIRMWARE.md](FIRMWARE.md). Change it there,
run `tools/sync_common.py`, and reflash **both** boards.
