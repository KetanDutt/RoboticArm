# Performance

Where the time and the memory actually go, how to measure both, and what is
worth optimising.

- [1. Measure first](#1-measure-first)
- [2. The latency budget](#2-the-latency-budget)
- [3. Where the glove's time goes](#3-where-the-gloves-time-goes)
- [4. Where the arm's time goes](#4-where-the-arms-time-goes)
- [5. Memory budget](#5-memory-budget)
- [6. What the rewrite changed, quantitatively](#6-what-the-rewrite-changed-quantitatively)
- [7. The bottleneck ranking](#7-the-bottleneck-ranking)
- [8. Scaling rules](#8-scaling-rules)
- [9. What is deliberately not optimised](#9-what-is-deliberately-not-optimised)

## 1. Measure first

Everything below is analysis. Four things let you replace it with data:

| Instrument | What it tells you |
| --- | --- |
| `s` on either board → `loop max : NNN us` | The longest gap between two entries into `loop()`. **This is the blocking detector.** In a correctly behaving build it should be a few hundred microseconds on the arm and a couple of milliseconds on the glove (the I²C read). A value in the tens of milliseconds means something is waiting |
| `s` on the glove → `frames : N sent, M deferred` | `deferred` counts frames that were ready but not sent because the radio was busy. Growing steadily means you are asking for more than the air time allows |
| `s` on the arm → `q=NN%`, `lost`, `fs` | Real link quality: packet loss and failsafe events. If quality is below ~90%, the radio — not the code — is your problem |
| `tools/rf_airtime.py --budget` | The theoretical air time and end-to-end latency for any payload length and speed |
| `tools/build.sh` | Flash and RAM report from avr-gcc |

The loop-timing instrumentation costs two `uint32_t` and one `micros()` read
per iteration — about 4 µs — and is the cheapest way to prove that a
"non-blocking" firmware really is.

## 2. The latency budget

From `python3 tools/rf_airtime.py --budget` at the default configuration:

| Stage | Time | Governed by |
| --- | --- | --- |
| glove samples sensors | ~5 ms | `GLOVE_TX_SAMPLE_MS` |
| glove offers the frame to the radio | 0–20 ms | `GLOVE_TX_SEND_MS` |
| **radio air time** | **~96 ms** | frame length and `GLOVE_RF_SPEED_BPS` |
| arm drains and decodes | ~1 ms | CPU |
| arm servo refresh tick | 0–20 ms | `GLOVE_RX_UPDATE_MS` |
| slew limiter, if the move is large | 0–600 ms | `GLOVE_RX_SLEW_MAX_DPS` |
| servo mechanical response | ~100 ms | the servo, not the firmware |
| **total, small movement** | **~122 ms (8.2 Hz)** | |

Two thirds of that is the radio. This is the single most important performance
fact in the project, and it is a property of VirtualWire/RH_ASK framing, not of
this code:

```
bits on air = 84 (preamble + start symbol + length + FCS) + 12 × payload bytes
9-byte frame = 192 bits → 96 ms at 2000 bps → 48 ms at 4000 bps
theoretical ceiling = 10.4 frames/s at 2000 bps, 20.8 at 4000
```

So: no amount of CPU optimisation on the glove will make the arm respond
faster. If you want a faster control rate, change the radio or the payload
([§8](#8-scaling-rules)).

## 3. Where the glove's time goes

Per `loop()` iteration, in the common case where no tick is due: read
`millis()`, read `micros()`, reset the watchdog, poll serial (a register read),
poll the button (a `digitalRead`), compare four timestamps. **Tens of
microseconds.** The loop spins fast and cheaply, which is what makes the
scheduler responsive.

When a tick *is* due (estimates from instruction counts at 16 MHz — measure
with `s` rather than trusting these):

| Work | Period | Estimated cost | Notes |
| --- | --- | --- | --- |
| 4–5 × `analogRead()` | 5 ms | ~110 µs each, ~0.5 ms total | The ADC conversion itself is ~104 µs at the default 125 kHz ADC clock. This is the single largest fixed cost on the glove |
| median-of-3 + EMA per channel | 5 ms | ~20 µs per channel | Integer only; the EMA is a two-term decomposition that avoids a 32-bit multiply-shift overflow |
| `getMotion6()` over I²C | 10 ms | ~320 µs at 400 kHz | 14 bytes × 9 bits / 400 kHz, plus the register write and repeated start. At the Wire default of 100 kHz this is ~1.3 ms — `GLOVE_IMU_I2C_HZ 400000` is a 4× saving |
| `AttitudeFilter::update()` | 10 ms | ~150 µs | Two `atan2Q8` (~20 µs each), one `isqrt32`, several 32-bit multiplies. A `float atan2f` pair would be ~2 ms — **13× slower** — which is why the filter is fixed-point |
| build + `encode()` a frame | 20 ms | ~30 µs | 9 bytes, one `packMeta`, seven clamps |
| `rflink::send()` | 20 ms | ~40 µs of CPU | Copies into the driver's buffer and returns; the transmission itself happens in the Timer1 interrupt over the next 96 ms |

The ADC is worth a note: five `analogRead()` calls are half a millisecond of
pure waiting, during which the CPU could be doing something else. It is not
worth optimising, because the sampling period is 5 ms — 10× the cost — but it
would be if you dropped the period to 1 ms. If you ever need faster flex
sampling, start the conversion and read it later (`ADCSRA`/free-running mode)
rather than calling `analogRead()` in a tight loop.

## 4. Where the arm's time goes

| Work | Period | Estimated cost | Notes |
| --- | --- | --- | --- |
| `drainRadio()` | every loop | ~1 ms per pending frame | `vw_have_message()` is a flag check; a full `decode()` is seven clamps and a few compares |
| `LinkMonitor::poll()` + `decideTargets()` | every loop | ~10 µs | Unsigned comparisons only |
| 7 × `SlewLimiter::update()` + `angleToPulseUs()` + `write()` | 20 ms | ~15 µs per servo, ~100 µs total | `ServoTimer2::write()` just stores a value; the pulse generation is in the Timer2 ISR |
| telemetry line | 1000 ms | ~2–4 ms | At 115200 baud, ~60 characters is ~5 ms if the TX buffer fills. This is the largest single cost in the arm's loop, and it is why telemetry is once per second and switchable |

The arm's loop is otherwise idle, which matters: ServoTimer2's ISR needs the
CPU every ~625 µs to start a pulse, and a loop that blocked for milliseconds
would add jitter to every servo edge.

**Serial is the one genuinely blocking thing left in either firmware.** At
115200 baud a 60-character line takes ~5 ms, and `Serial.print` blocks once the
64-byte hardware buffer is full. That is acceptable at 1 Hz. It would not be
acceptable in the sensor path — which is exactly what the original firmware did
(B-11 in [BUGS_FIXED.md](BUGS_FIXED.md)), at 38400 baud, every iteration.

## 5. Memory budget

Flash and RAM are reported by `tools/build.sh`; measure rather than guess. The
rules that keep the firmware inside an ATmega328P:

| Rule | Why |
| --- | --- |
| No `String`, no `new`/`malloc` | Heap fragmentation on a 2 KB heap is a time bomb; a fixed static array's cost is known at compile time |
| Every literal in `F()` | A 60-character telemetry format costs 60 bytes of RAM without it, 0 with it |
| No `float` in the sensor, filter, attitude or servo paths | `float` pulls in ~1.5–2 KB of soft-float library code and is 10–50× slower per operation |
| The arm includes no Wire/I2Cdev/MPU6050 | The original did, by copy-paste. Removing it saves the Wire library, its ISR vectors and its two 32-byte buffers |
| `GLOVE_IMU_ENABLE 0` compiles the I²C stack out of the glove | The biggest single lever if you are tight on flash |
| `sizeof(CalibrationData) <= 128`, asserted | Keeps the EEPROM blob small and its layout stable |
| Buffers sized from config, not from literals | `GLOVE_RX_BUFFER_LEN` is 24 and `static_assert`ed ≥ `GLOVE_FRAME_LEN` — the original used 2 and over-ran it (B-02) |

Static RAM, roughly: 7 × `ServoTimer2` channel state + 7 × `SlewLimiter` (8 B
each) + `LinkMonitor` (~30 B) + `CommandLatch` (~12 B) + `Frame` (12 B) + the
receive buffer (24 B) + telemetry/CLI line buffer (~32 B) on the arm; the
calibration blob (~100 B) + 5 × `AnalogChannel` + 5 × `AutoRanger` +
`AttitudeFilter` (~40 B) on the glove. Both fit comfortably in 2 KB, which is
the point of counting.

## 6. What the rewrite changed, quantitatively

| Metric | Original | Now | Change |
| --- | --- | --- | --- |
| glove loop period | ~95 ms (`vw_wait_tx()` 84 ms + `delay(10)`) | ~1 ms idle, ~5 ms on a sample tick | **~20–90× more responsive** |
| glove sensor sample rate | ~10 Hz (once per transmitted frame) | 200 Hz flex, 100 Hz IMU | **20× / 10×** |
| blocking calls in the glove loop | 3 (`vw_wait_tx`, `delay`, `Serial.print`) | 0 in the sensor path | — |
| I²C clock | 100 kHz (Wire default) | 400 kHz | **4× faster IMU reads** |
| orientation maths | none — a single raw accelerometer axis | complementary filter, `atan2Q8` ~20 µs | a `float` equivalent would be ~2 ms |
| control rate (radio limited) | ~10 Hz | ~10 Hz at 2000 bps, **~20 Hz at 4000** | unchanged at the default, but now a one-line change that is understood and budgeted |
| received bytes validated | 0 of 7 | 9 of 9, plus version and command | — |
| servo writes per frame | 7 (6 of them to the same servo) | 7, one per servo, each rate limited | — |
| reaction to a lost link | never | 500 ms | — |
| arm-side I²C stack | linked and initialised | absent | flash + ~64 B RAM saved |

The control rate did not improve, because it was never limited by the CPU. That
is worth stating plainly: the performance work here bought **responsiveness,
sample rate, determinism and safety**, not raw throughput. Throughput needs a
different radio.

## 7. The bottleneck ranking

In order of how much each one costs you, worst first:

1. **Radio air time** — 96 ms per frame at 2000 bps. Fixes: `GLOVE_RF_SPEED_BPS
   4000` (halves it, costs range), a shorter frame (6 ms per byte saved), or a
   different module entirely (nRF24L01+ → ~250 µs per frame).
2. **Servo mechanics** — ~100 ms to move. No firmware can fix this; it is why
   the slew limiter's 600 ms full-sweep cost is not the dominant term for
   normal movements.
3. **ADC conversions** — ~110 µs each, ~0.5 ms per sample tick. Fixable with
   free-running conversion if it ever matters.
4. **I²C reads** — ~320 µs at 400 kHz. Already 4× better than the default.
5. **Serial output** — ~5 ms per telemetry line, once per second, switchable.
6. **CPU maths** — filtering, attitude and servo updates together are well
   under a millisecond. Optimising here buys nothing measurable.

## 8. Scaling rules

Numbers to keep in mind when changing anything:

- **Each payload byte = 6 ms at 2000 bps, 3 ms at 4000 bps.** An eighth servo
  channel is not free; it also pushes on the `GLOVE_TX_CMD_REPEAT_MS` and
  `GLOVE_RX_FAILSAFE_TIMEOUT_MS` compile-time guards, which is them doing their
  job.
- **Halving `GLOVE_TX_SEND_MS` does not double the frame rate** — the radio is
  the limit. It just increases `deferred`.
- **Raising `GLOVE_TX_SAMPLE_MS` above `GLOVE_TX_SEND_MS` is refused at compile
  time**, because it would transmit stale samples.
- **Lowering `GLOVE_EMA_ALPHA_Q8` smooths more and lags more.** At alpha 40/256
  the filter's time constant is roughly 6 samples (30 ms at a 5 ms tick) —
  invisible next to the 96 ms of air time. Halving it to 20 doubles the lag and
  buys very little, because the radio is already the low-pass filter.
- **`GLOVE_RX_UPDATE_MS` below 20 ms gains nothing**: servos expect a 50 Hz
  refresh, and ServoTimer2 generates the pulses in its ISR regardless.
- **`GLOVE_RX_SLEW_MAX_DPS` sets the worst-case reaction time**: a 160° move at
  300 °/s takes 533 ms. If that feels sluggish, raise it *and* check the power
  supply — the inrush is what usually breaks first.

## 9. What is deliberately not optimised

- **The MPU-6050 DMP.** It would give better orientation (and yaw) for less
  CPU, at the cost of ~6 KB of flash, a 1.9 kB firmware blob, an opaque
  initialisation sequence and no ability to unit test it. On a 32 KB board that
  is a bad trade; see [ROADMAP.md](ROADMAP.md).
- **Interrupt-driven sensor sampling.** The 5 ms tick is already 20× faster
  than the link. Adding ISR complexity to sample faster would improve a number
  nobody can observe.
- **Delta/variable-length frames.** They would cut air time when the hand is
  still — which is when air time does not matter — and make the parser
  stateful, so a corrupt length could desynchronise it. Fixed 9 bytes is the
  robust choice.
- **Assembly or hand-tuned intrinsics.** The hot paths are already tens of
  microseconds against a 96 ms radio. Readability is worth more.
- **A bigger receive buffer.** 24 bytes for a 9-byte frame is already generous;
  VirtualWire delivers whole messages, so there is no streaming to buffer.
