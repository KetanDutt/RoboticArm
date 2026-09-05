# Bugs found and fixed in the original firmware

An audit of the two sketches as they were at commit `aed1b32`
(`hand_transmit/hand_transmit.ino`, 139 lines, and
`hand_recieve/hand_recieve.ino`, 154 lines). Every item below is quoted from
the original source, not paraphrased.

Severity: **Critical** = the system cannot work, is memory-unsafe, or can
damage hardware. **Major** = wrong or unsafe behaviour under normal use.
**Minor** = waste, confusion or fragility.

| ID | Sev | Board | One-line summary |
| --- | --- | --- | --- |
| [B-01](#b-01-servotimer2-written-in-degrees) | Critical | arm | `ServoTimer2::write()` given degrees instead of microseconds |
| [B-02](#b-02-receive-buffer-overrun) | Critical | arm | 7-byte message written into a 2-byte buffer |
| [B-03](#b-03-six-channels-driven-onto-one-servo) | Critical | arm | `msg[1]…msg[6]` all written to `myservo2` |
| [B-04](#b-04-receive-length-never-reset) | Critical | arm | `buflen` not restored before each `vw_get_message()` |
| [B-05](#b-05-flex-sensor-on-the-i2c-bus) | Critical | glove | fifth flex sensor on A4, which is I²C SDA |
| [B-06](#b-06-unconstrained-map-truncated-into-a-byte) | Critical | glove | negative `map()` results assigned to `uint8_t msg[]` |
| [B-07](#b-07-no-validation-of-received-values) | Major | arm | any received byte reaches a servo |
| [B-08](#b-08-no-failsafe) | Major | arm | a lost link leaves the arm holding its last pose forever |
| [B-09](#b-09-no-rate-limiting) | Major | arm | a single bad frame can slam a joint at full speed |
| [B-10](#b-10-blocking-transmit-and-delay-in-the-sensor-loop) | Major | glove | `vw_wait_tx()` + `delay(10)` ≈ 94 ms of blocking per frame |
| [B-11](#b-11-blocking-serial-in-the-hot-loop) | Major | glove | unconditional `Serial.print` every iteration at 38400 |
| [B-12](#b-12-accelerometer-used-as-an-orientation-estimate) | Major | glove | `map(ax, 17000, -17000, 0, 179)` presented as wrist angle |
| [B-13](#b-13-no-filtering) | Major | glove | raw ADC and raw IMU drive the servos |
| [B-14](#b-14-hard-coded-calibration-and-inconsistent-directions) | Major | glove | magic numbers 90/220/17000, two of five fingers mapped backwards |
| [B-15](#b-15-imu-never-checked) | Major | glove | `mpu.initialize()` with no `testConnection()` |
| [B-16](#b-16-dead-code-that-misleads) | Minor | glove | `val3…val6` computed, three of them from the same axis, none transmitted |
| [B-17](#b-17-unused-i2c-stack-on-the-arm) | Minor | arm | Wire/I2Cdev/MPU6050 included and `Wire.begin()` called on a board with no I²C device |
| [B-18](#b-18-no-protocol-version-or-frame-validation) | Major | both | a mismatched or foreign packet is silently obeyed |
| [B-19](#b-19-change-detection-compares-the-wrong-quantity) | Minor | glove | `int` compared against a truncated `uint8_t` |
| [B-20](#b-20-uninitialised-globals-as-protocol-state) | Minor | glove | `prevVal1/2` are 0 at boot, so the first frame can carry stale bytes |
| [B-21](#b-21-no-watchdog) | Minor | both | a hung board stays hung |
| [B-22](#b-22-typoed-and-garbled-serial-output) | Minor | both | `"Gyro:- "` label on an accelerometer value, `"/t"`, no separators |
| [B-23](#b-23-dead-variables-and-commented-out-blocks) | Minor | both | ~40 unused globals and large commented-out regions |
| [B-24](#b-24-misspelled-sketch-folder) | Minor | repo | `hand_recieve/` |
| [B-25](#b-25-nothing-was-tested) | Major | repo | no tests, no CI, no documentation |

---

## B-01: ServoTimer2 written in degrees

**Severity: Critical. Board: arm. This alone means the arm never moved.**

```cpp
#include <ServoTimer2.h>
...
myservo1.write(msg[0]);      /* msg[0] is 0..180 */
```

`ServoTimer2::write()` does not take degrees. Its implementation is:

```cpp
void ServoTimer2::write(int value)
{
    if(value < MIN_PULSE_WIDTH) value = MIN_PULSE_WIDTH;   /* 750  */
    else if(value > MAX_PULSE_WIDTH) value = MAX_PULSE_WIDTH; /* 2250 */
    this->channel.pinData = value;
}
```

Every value in 0…180 is below `MIN_PULSE_WIDTH`, so every servo was clamped to
750 µs — one end stop — regardless of what the glove sent. Seven servos, all
parked at the same angle, drawing holding current.

**Fix:** `glove::angleToPulseUs()` converts degrees into microseconds across the
configured mechanical window, with optional per-servo inversion:

```cpp
uint16_t us = glove::angleToPulseUs(angle, GLOVE_RX_ANGLE_MIN, GLOVE_RX_ANGLE_MAX,
                                    GLOVE_RX_PULSE_MIN_US, GLOVE_RX_PULSE_MAX_US);
```

**Prevented by:** `tests/test_servo.cpp` pins the endpoints (min angle →
`PULSE_MIN_US`, max angle → `PULSE_MAX_US`, mid → 1500 µs), the inversion, the
clamping outside the window, and the monotonicity of the whole range. The
header comment on `angleToPulseUs` states the microsecond contract explicitly,
and `docs/HARDWARE.md` repeats it where the servos are wired.

---

## B-02: Receive buffer overrun

**Severity: Critical. Board: arm. Stack corruption on every packet.**

```cpp
uint8_t msg[2];
uint8_t buflen;
...
buflen = 7;
vw_rx_start();
...
if (vw_get_message(msg, &buflen))
```

VirtualWire's contract is: *set `*len` to the size of your buffer before each
call*; on success it copies up to `*len` bytes into `buf` and sets `*len` to the
received length. Here the caller claims a 7-byte buffer and passes a 2-byte
array. The transmitter sends 7 bytes, so `vw_get_message` writes 7 bytes into
`msg[2]` — five bytes past the end of a stack array, on a machine with 2 KB of
RAM and no MMU. What those five bytes are is whatever else lives on the stack:
return addresses, locals, the radio driver's own state.

**Fix:**

```cpp
uint8_t buf[GLOVE_RX_BUFFER_LEN];          /* 24, compile-time asserted >= frame */
uint8_t len = sizeof(buf);                 /* reset before EVERY call */
if (rflink::receive(buf, &len)) { ... }
```

**Prevented by:** `static_assert(GLOVE_RX_BUFFER_LEN >= GLOVE_FRAME_LEN, ...)`
in `GloveProtocol.h`, and `Frame::decode()` refusing any length that is not
exactly `GLOVE_FRAME_LEN` — tested in `tests/test_protocol.cpp` with short,
long and exact buffers. `rflink::receive()` also rejects a null or zero `len`.

---

## B-03: Six channels driven onto one servo

**Severity: Critical. Board: arm.**

```cpp
myservo1.write(msg[0]);
myservo2.write(msg[1]);
myservo2.write(msg[2]);
myservo2.write(msg[3]);
myservo2.write(msg[4]);
myservo2.write(msg[5]);
myservo2.write(msg[6]);
```

`myservo3`…`myservo7` are attached in `setup()` and then never written. Servo 2
receives six consecutive writes per packet and ends up at `msg[6]`. So of seven
servos: one got the wrist pitch, one got the little finger, five did nothing.

**Fix:** a loop over `GLOVE_RX_SERVO_COUNT`, one `ServoTimer2` and one
`SlewLimiter` per channel, each written from its own decoded channel value.

**Prevented by:** the array-based design makes the mistake unrepresentable —
there is no per-servo variable name left to typo. The host smoke test runs
20 000 `loop()` iterations against stubs that record every write, so a channel
that is never written would show up as a permanently silent stub.

---

## B-04: Receive length never reset

**Severity: Critical. Board: arm.**

```cpp
uint8_t buflen;
...
buflen = 7;                     /* once, in setup() */
...
void loop() {
  if (vw_get_message(msg, &buflen)) {   /* buflen is overwritten by the call */
```

`vw_get_message` writes the received length back into `buflen`. The next
iteration therefore passes the *previous message's length* as the buffer size.
A single short packet (say 3 bytes) permanently truncates every subsequent
receive to 3 bytes — and because the frame layout is positional, the arm would
then read `msg[3]…msg[6]` from uninitialised stack. This is the kind of bug
that works for an hour and then stops, with no visible cause.

**Fix:** `len` is a local, set to `sizeof(buf)` immediately before every call.

**Prevented by:** the same `decode()` length tests as B-02, plus the code shape
— the length lives in the function that uses it, so there is no persistent
variable to forget.

---

## B-05: Flex sensor on the I2C bus

**Severity: Critical. Board: glove. Hardware/firmware conflict.**

```cpp
void setup() { Wire.begin(); ... mpu.initialize(); }
...
const int flexpin5 = 4;                 /* A4 */
flexposition5 = analogRead(flexpin5);
```

On an Uno, A4 **is** SDA and A5 **is** SCL. Wiring a flex divider to A4 puts a
10 kΩ resistor from the I²C data line to ground and a flex sensor from it to
5 V. Consequences: the MPU-6050 fails to initialise (or works intermittently,
depending on finger bend), the fifth channel reads whatever the I²C traffic
leaves on the line, and the bus pull-ups are loaded by the divider.

**Fix:** board profiles in `GloveConfig.h`. The default `UNO_4FLEX` profile uses
A0–A3 for four flex sensors and leaves A4/A5 to the IMU; `UNO_MUX` puts five
sensors behind a 74HC4051 on A0; `MEGA_5FLEX` uses A0–A4 on a board where I²C
lives on 20/21. A finger with no sensor transmits `GLOVE_FLEX_ABSENT_ANGLE`
rather than a made-up reading.

**Prevented by:** `GLOVE_PROFILE` is validated at compile time
(`#error "GLOVE_PROFILE is not a known profile"`), the pin sets are defined
once per profile rather than scattered through the code, and CI builds all
three profiles. `docs/HARDWARE.md` explains the conflict where the wiring is
described.

---

## B-06: Unconstrained `map()` truncated into a byte

**Severity: Critical. Board: glove.**

```cpp
uint8_t msg[7];
int val1, val2, ...;
...
val1 = map(ax, 17000, -17000, 0, 179);
val2 = map(ay, -17000, 17000, 0, 179);
...
if (val1 != prevVal1) { msg[0] = val1; prevVal1 = val1; }
```

Arduino's `map()` does **not** constrain its result — it extrapolates. With
`ax = 20000` you get a negative number; with `ax = -20000` you get something
above 179. Assigning that to a `uint8_t` wraps: −20 becomes 236, 300 becomes
44. The receiver (which, per B-01, treated it as microseconds anyway) would
have driven the servo to a random hard stop. A hand movement that overshoots
the assumed ±17000 range — which is any brisk movement — produces a violent
output.

Note that the flex channels *were* constrained (`constrain(servoposition1, 0,
180)`), which makes the omission on the wrist channels easy to miss.

**Fix:** every conversion ends in `glove::clampInt()`, `mapRange()` rounds and
guards against a zero span, and `GLOVE_ANGLE_FLOOR`/`CEILING` bound the result
before it is ever stored. `Frame::encode()` clamps again on the way out, and
`Frame::decode()` clamps a third time on the way in.

**Prevented by:** `tests/test_math.cpp` covers `mapRange` with out-of-range,
inverted, zero-span and negative inputs, and `tests/test_protocol.cpp` asserts
that encoding a channel value of 999 or −5 produces a byte inside 0…180.

---

## B-07: No validation of received values

**Severity: Major. Board: arm.**

The receiver applied whatever arrived. No length check (beyond the overrun in
B-02), no version check, no command concept, no range clamp, no sanity check
that the values were even plausible angles.

**Fix:** `Frame::decode()` is the only path from radio to servo, and it returns
false unless the length is exactly right, the protocol version matches, the
command is known, and every channel could be clamped into the arm's mechanical
window. Rejected frames are counted, reported on serial, and — importantly —
**do not refresh the link timeout**, so a channel full of garbage is treated as
a lost link rather than a live one.

**Prevented by:** `tests/test_protocol.cpp` has explicit cases for bad length,
bad version, unknown command and out-of-range channels.

---

## B-08: No failsafe

**Severity: Major. Board: arm.**

If the glove went out of range, ran out of battery or was switched off, the arm
held its last commanded pose indefinitely — servos stalled against a mechanical
limit, drawing several hundred milliamps each, with no indication anywhere that
the link was gone.

**Fix:** `LinkMonitor` + a configurable failsafe. No valid frame for
`GLOVE_RX_FAILSAFE_TIMEOUT_MS` (500 ms) puts the arm into `FAILSAFE` mode and
applies `GLOVE_RX_FAILSAFE_ACTION`: `HOLD`, `NEUTRAL` (default) or `RELAX`. The
transition is counted, printed once (`LINK LOST` / `LINK OK q=NN%`), and the LED
pattern changes.

**Prevented by:** `tests/test_servo.cpp` drives `LinkMonitor` with a simulated
clock across the timeout boundary, including the "still receiving garbage"
case. The compile-time guard
`GLOVE_RX_FAILSAFE_TIMEOUT_MS >= 3 × GLOVE_AIRTIME_MS` stops anyone configuring
a timeout shorter than the link's own frame rate, and CI proves the guard fires.

---

## B-09: No rate limiting

**Severity: Major. Board: arm.**

A hobby servo will move from one end stop to the other in ~150 ms if told to.
With no rate limit, one corrupted or extreme frame — see B-06 — produces a full
speed slam, which is how links get bent and servo gears get stripped.

**Fix:** one `SlewLimiter` per servo, `GLOVE_RX_SLEW_MAX_DPS` (300 °/s) by
default. The limiter uses wrap-safe unsigned `millis()` arithmetic, primes its
time base on the first update rather than jumping, and can be `snap()`ed to a
known position (used at power-on and when re-attaching servos, so software and
hardware agree about where the arm is).

**Prevented by:** `tests/test_servo.cpp` checks the per-step increment, the
exact time to traverse a known distance, wrap-around at the 32-bit `millis()`
boundary, `snap()`, and that a huge target difference never exceeds the rate.

---

## B-10: Blocking transmit and delay in the sensor loop

**Severity: Major. Board: glove.**

```cpp
vw_send(msg, 7);
vw_wait_tx();       /* blocks for the whole frame: ~84 ms at 2000 bps */
delay(10);
```

`vw_wait_tx()` spins until the transmitter interrupt has pushed the last bit
out. At 2000 bps a 7-byte frame is 168 bits ≈ 84 ms, plus the 10 ms delay: the
glove was blind and deaf for ~94 ms out of every ~95 ms. Sensors were sampled
once per frame at best, and the button (had there been one) could not be
debounced.

**Fix:** no `delay()` and no `vw_wait_tx()` anywhere. The glove samples on a
5 ms tick, and offers a frame to the radio on a 20 ms tick *only if the radio is
idle* (`rflink::txBusy()` uses `vw_tx_active()` for VirtualWire and a software
air-time deadline for RH_ASK, which has no equivalent query). Frames that
cannot be sent are dropped and counted as `deferred` — the newest sample always
wins.

**Prevented by:** the host smoke test runs 20 000 `loop()` iterations with a
stub `millis()` that advances 1 ms per call; a blocking loop would show up as a
huge simulated-time cost per iteration, and `tools/check_configs.sh` builds the
RadioHead variant too so both `txBusy()` paths are compiled.

---

## B-11: Blocking serial in the hot loop

**Severity: Major. Board: glove.**

```cpp
Serial.println("Gyro:- ");
Serial.print(ax);
```

Unconditional, every iteration, at 38400 baud. `Serial.print` blocks when the
64-byte TX buffer is full: ~15 characters is ~4 ms of blocking per loop, which
is comparable to the entire sensor budget. It also means the glove's control
rate was partly determined by whether a serial monitor happened to be open.

**Fix:** telemetry is rate-limited (`GLOVE_TX_TELEMETRY_MS`, 500 ms), can be
turned off at runtime (`t`) or compiled out (`GLOVE_TELEMETRY_ENABLE 0`), uses
`F()` for every literal so the strings stay in flash, and is available in CSV
for `tools/glove_monitor.py`. Nothing is printed in the sampling path.

**Prevented by:** the config matrix includes a variant with telemetry and CLI
compiled out, so the no-serial path is built and smoke-run by CI.

---

## B-12: Accelerometer used as an orientation estimate

**Severity: Major. Board: glove.**

```cpp
val1 = map(ax, 17000, -17000, 0, 179);   /* "wrist pitch" */
val2 = map(ay, -17000, 17000, 0, 179);   /* "wrist roll"  */
```

An accelerometer measures *specific force*, not angle. It equals gravity only
when the sensor is not accelerating, so any hand movement — which is the whole
point of a data glove — corrupts the reading. The ±17000 range is also a guess
(±2 g is ±16384 LSB), and one axis of a 3-axis device was used per joint with
no tilt compensation: pitch and roll are not separable from single axes except
near the level position.

**Fix:** `glove::AttitudeFilter`, an all-integer complementary filter. The gyro
provides the fast path (integrated in Q8 degrees with the fractional remainder
carried between updates, so slow rotation does not stall); the accelerometer
provides the slow correction through `atan2Q8`, and is **only trusted when the
measured specific force is inside 600–1600 milli-g** — i.e. when the hand is
roughly static. Biases for both sensors are captured at boot or on demand, with
a flatness check that refuses an implausible capture.

**Prevented by:** `tests/test_attitude.cpp` validates `isqrt32` and `atan2Q8`
against the library functions over full sweeps (worst-case error ~0.3°),
verifies that a synthetic static gravity vector converges to the true
pitch/roll, that the gyro-only path integrates correctly with residual carry,
that acceleration outside the trust gate is ignored, that `dt` is clamped, and
that `begin()` fully resets state.

---

## B-13: No filtering

**Severity: Major. Board: glove.**

Raw `analogRead()` and raw IMU values were mapped straight to servo angles. A
10-bit ADC on a moving, unshielded glove with a radio transmitter centimetres
away produces several counts of noise; the servos reproduce it as an audible
buzz and visible jitter, and the mechanism wears.

**Fix:** `AnalogChannel` = `MedianFilter3` (rejects single-sample spikes) →
`EmaFilter` (Q8 alpha, `GLOVE_EMA_ALPHA_Q8` = 40/256), then a
`GLOVE_DEADBAND_DEG` (2°) hysteresis so an unchanged channel is not re-sent.
Filters are seeded from a real reading at boot rather than from zero, which
otherwise produces a visible sweep from 0 to the true value on power-up.

**Prevented by:** `tests/test_signal.cpp` pins the EMA's step response (no
integer overflow at extreme alphas — an early `(delta*alpha)>>8` formulation
overflowed int32 and was replaced by a two-term decomposition), the median's
rejection of a single outlier, the deadband's hysteresis in both directions,
and `AutoRanger`'s min/max/span/validity logic.

---

## B-14: Hard-coded calibration and inconsistent directions

**Severity: Major. Board: glove.**

```cpp
servoposition1 = map(flexposition1, 220, 90, 0, 180);
servoposition2 = map(flexposition2, 220, 90, 0, 180);
servoposition3 = map(flexposition3,  90, 220, 0, 180);   /* backwards */
servoposition4 = map(flexposition4, 220, 90, 0, 180);
servoposition5 = map(flexposition5,  90, 220, 0, 180);   /* backwards */
```

Two of five fingers were mapped in the opposite direction to the other three,
with no comment explaining why. Either the builder had wired two sensors
backwards and compensated in software without saying so, or two fingers simply
moved the wrong way. And 90/220 are the raw ADC values of *that* builder's
*glove*: every other flex sensor has a different span, so the firmware could
only ever work on one physical glove.

**Fix:** calibration is captured per glove (`c` / `w`) and stored in EEPROM
behind a CRC. Direction is a separate per-channel flag (`i <n>`), so flipping a
finger is exact and does not require re-capturing the span. The 90/220 values
survive only as `GLOVE_FLEX_DEFAULT_RAW_MIN/MAX`, the documented factory
fallback for a glove that has never been calibrated.

**Prevented by:** `tests/test_calibration.cpp` covers `setDefaults`,
`updateChecksum`/`verifyChecksum` round-trips, corruption detection (every
single-byte flip is caught), `flexUsable`/`wristUsable` thresholds, both
directions of `flexToAngle` and `wristToAngle`, out-of-range inputs, and the
fallback for an unusable channel.

---

## B-15: IMU never checked

**Severity: Major. Board: glove.**

```cpp
mpu.initialize();
```

No `testConnection()`. With the IMU absent, mis-wired or suffering from B-05,
`getMotion6()` returns whatever was on the bus — often all zeros or all −1 —
and the glove cheerfully transmits it as wrist angles.

**Fix:** `testConnection()` with `GLOVE_IMU_RETRY_COUNT` retries at
`GLOVE_IMU_RETRY_DELAY_MS`, a clear serial report (`IMU: OK` / `IMU: NOT
FOUND`), and defined degraded behaviour: the wrist channels are held at their
configured neutral, `GLOVE_FLAG_IMU_OK` stays clear so the arm knows, and the
glove keeps working as a finger-only device. `GLOVE_IMU_ENABLE 0` compiles the
I²C stack out entirely for builds with no IMU at all.

**Prevented by:** the config matrix builds the IMU-less variant
(`-DGLOVE_PROFILE_IMU_ENABLE=0`), and the guard around all IMU code is a single
derived macro rather than scattered `#if`s.

---

## B-16: Dead code that misleads

**Severity: Minor. Board: glove.**

```cpp
val3 = map(az, -17000, 17000, 0, 179);
val4 = map(gz, -17000, 17000, 0, 179);
val5 = map(gz, -17000, 17000, 0, 179);
val6 = map(gz, -17000, 17000, 0, 179);
```

Three of the six values are the *same gyro axis*, and none of `val3…val6` is
ever transmitted. Only `msg[0]` and `msg[1]` (from `val1`/`val2`) and
`msg[2…6]` (from the flex sensors) go out. Anyone reading this code — including
the arm's author — would reasonably conclude that six IMU channels were being
sent.

**Fix:** every channel has exactly one producer, named for what it is
(`g_attitude.pitchDeg()`, `g_cal.flexToAngle(n, raw)`), and the mapping from
producer to channel index is a named constant (`GLOVE_CH_WRIST_PITCH`,
`GLOVE_CH_FINGER_FIRST`). There is no computed value that is not transmitted.

---

## B-17: Unused I2C stack on the arm

**Severity: Minor. Board: arm.**

```cpp
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
...
void setup() { Wire.begin(); ... }
```

The arm has no I²C device. These includes and the `Wire.begin()` call were
copied from the transmitter, and they cost flash (the Wire library and its
interrupt vectors) and claim pins A4/A5 on a board that might otherwise use
them.

**Fix:** the arm includes exactly what it uses. `tests/arduino_stub/` provides
signature-faithful stubs, so if anyone re-adds an unused include the host build
still links — but the flash report from `tools/build.sh` will show it.

---

## B-18: No protocol version or frame validation

**Severity: Major. Board: both.**

The two sketches agreed on a 7-byte positional layout by convention only.
Nothing in either program could detect that the other end was running different
firmware, and nothing distinguished a glove packet from any other
VirtualWire device on the same channel.

**Fix:** a 9-byte frame with a 2-bit version field, a sequence number, status
flags and a command nibble — see [PROTOCOL.md](PROTOCOL.md). `Frame::decode()`
validates all of it. A mismatch prints
`WARN protocol mismatch: frame vX arm vY` and is counted as a rejected frame.

**Prevented by:** `tests/test_protocol.cpp` round-trips every field, rejects
every wrong version, rejects unknown commands, and pins the sequence-wrap
arithmetic. CI builds both sketches, so a layout change that is only applied to
one side fails the build.

---

## B-19: Change detection compares the wrong quantity

**Severity: Minor. Board: glove.**

```cpp
if (val1 != prevVal1) { msg[0] = val1; prevVal1 = val1; }
```

`val1` is an unconstrained `int` (see B-06) while `msg[0]` is a `uint8_t`. Two
different `int` values can produce the same transmitted byte (−20 and 236), and
the same `int` can produce different bytes depending on what else was in
`msg[0]` before. The change detector therefore both misses real changes and
reports false ones.

**Fix:** the frame is rebuilt from the current state on every send tick, with
the deadband applied to the *angle* — the quantity that is actually
transmitted — rather than to an intermediate value.

---

## B-20: Uninitialised globals as protocol state

**Severity: Minor. Board: glove.**

`prevVal1…prevVal6` are file-scope `int`s, zero-initialised by the C++ runtime.
If the first computed value happens to be 0, the corresponding `msg[]` byte is
never written and the glove transmits whatever `msg[]` was initialised to. In
practice that is 0, so the first frames command 0° on those channels — an end
stop — until the value changes.

**Fix:** no protocol state is derived from "has this value changed since boot".
`g_frame` is explicitly cleared and then fully populated before the first
transmission.

---

## B-21: No watchdog

**Severity: Minor. Board: both.**

A hung board — an I²C bus locked up by a glitch, a radio driver stuck in an
interrupt — stayed hung until someone physically reset it. On the arm, that
means servos holding their last position with no supervision at all.

**Fix:** `GLOVE_WATCHDOG_ENABLE` (default on) arms the AVR watchdog at
`WDTO_2S` in both sketches. Both `setup()` functions first clear `WDRF` in
`MCUSR` and call `wdt_disable()` — without that, the watchdog stays armed
through the bootloader after a watchdog reset and a board whose bootloader takes
longer than the timeout enters a reset loop. This is the classic AVR WDT trap,
and the comment in the code says so.

The longest blocking operation in either loop is one IMU read (~2 ms), so a 2 s
timeout has three orders of magnitude of margin.

---

## B-22: Typoed and garbled serial output

**Severity: Minor. Board: both.**

```cpp
Serial.println("Gyro:- ");
Serial.print(ax);            /* an accelerometer axis, labelled "Gyro" */
...
Serial.print("/t");          /* in the commented-out block: meant to be "\t" */
```

And on the arm, seven values printed back to back with no separator, so
`90120150` could be read as almost anything.

**Fix:** every diagnostic line has an unambiguous prefix and separated fields
(`ch: 90 90 112 …`, `LIVE | 96ms q97% | … | lost 31 fs 0`), or a documented CSV
column order. `docs/FIRMWARE.md` specifies the formats so a parser can rely on
them.

---

## B-23: Dead variables and commented-out blocks

**Severity: Minor. Board: both.**

In the transmitter, nine live file-scope variables were written or declared and
never used (`pos`, `val3…val6`, `prevVal3…prevVal6`). In the receiver, five of
the seven `ServoTimer2` objects were attached in `setup()` and then never
written (see B-03). Both files also carried large commented-out regions
declaring another twenty-odd variables — including an entire alternative
implementation of the receiver's loop and the transmitter's servo attachments.
On a 2 KB-RAM device unused globals are not free, and commented-out code is not
documentation: it is a record of what the author was unsure about, which is
exactly what a reader needs to be told in words.

**Fix:** no unused variables — the host build compiles both sketches with
`-Wall -Wextra` and is warning-free across all eight configurations — no
commented-out code, and a comment on every non-obvious decision explaining
*why*. Configuration variants are `#if` branches in `GloveConfig.h`, built by
CI, rather than comments.

---

## B-24: Misspelled sketch folder

**Severity: Minor. Board: repo.**

`hand_recieve/` → `hand_receive/`. Renamed with `git rm` so the history is
explicit about the move.

---

## B-25: Nothing was tested

**Severity: Major. Board: repo.**

The original repository was two `.ino` files with no build verification, no
tests and no documentation. Bugs B-01 to B-06 are all detectable by a compiler
or by five minutes of reading a library's source — which is exactly what a test
suite and a CI build force you to do.

**Fix:** 11 000+ assertions across six test files, a signature-faithful Arduino
stub layer that lets both sketches be compiled, linked and smoke-run on a
desktop, an eight-configuration build matrix, negative tests that prove the
safety guards actually fire, and a GitHub Actions workflow that runs all of it
plus a real `avr-gcc` build for Uno and Mega. See [TESTING.md](TESTING.md).

---

## Things that were *not* bugs

Worth recording, because they look wrong and are not:

- **`vw_set_ptt_inverted(true)` without `vw_set_ptt_pin()`** — VirtualWire
  defaults the PTT pin to 10, which is what the wiring uses. It is now explicit
  (`GLOVE_RF_PTT_PIN`, `GLOVE_RF_PTT_INVERTED`) rather than implicit.
- **Servo pins 3, 9, 4, 5, 6, 7, 8** — an odd order, but it is the original
  builder's wiring, and ServoTimer2 has no pin restriction. Kept so existing
  hardware keeps working; configurable via `GLOVE_RX_SERVO_PINS`.
- **`Serial.begin(38400)`** — a legitimate choice. Raised to 115200 because the
  telemetry and CLI are now actually useful, and at 38400 a status screen takes
  long enough to print that it perturbs the loop it is describing.
- **ServoTimer2 rather than `Servo`** — necessary, not a mistake: VirtualWire
  owns Timer1, and the stock `Servo` library needs it. See
  [ARCHITECTURE.md](ARCHITECTURE.md#5-design-constraints).
