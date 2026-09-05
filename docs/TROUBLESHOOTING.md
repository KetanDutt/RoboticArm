# Troubleshooting

Indexed by symptom, because that is what you have in front of you. Every entry
names the check that identifies it and the fix that follows.

- [0. The two commands that answer most questions](#0-the-two-commands-that-answer-most-questions)
- [1. Build and flash](#1-build-and-flash)
- [2. Nothing happens at all](#2-nothing-happens-at-all)
- [3. The arm does not follow the glove](#3-the-arm-does-not-follow-the-glove)
- [4. Jitter, buzz and noise](#4-jitter-buzz-and-noise)
- [5. Link problems](#5-link-problems)
- [6. IMU problems](#6-imu-problems)
- [7. Calibration problems](#7-calibration-problems)
- [8. Power problems](#8-power-problems)
- [9. Intermittent and weird](#9-intermittent-and-weird)

## 0. The two commands that answer most questions

Open serial on **both** boards at 115200 and send `s`.

```
glove:  === glove status ===            arm:  === arm status ===
        profile / flex count / radio          mode / link age / quality
        IMU: OK or NOT RESPONDING             good / lost / stale / failsafe
        calibration: valid or DEFAULTS        rejected frames
        spans per channel                     target and commanded deg/us
        frames sent / deferred                limits, glove IMU flag
        loop max (us)                         loop max (us)
```

Then read this table:

| Observation | Diagnosis | Go to |
| --- | --- | --- |
| protocol versions differ between the two banners | only one board was reflashed | [§1](#1-build-and-flash) |
| arm shows `rejected` > 0 | frames arrive but fail validation | [§5](#5-link-problems) |
| arm shows `no signal` / quality falling | radio problem | [§5](#5-link-problems) |
| glove shows `IMU: NOT RESPONDING` | I²C or the A4/A5 pin conflict | [§6](#6-imu-problems) |
| glove shows `calibration: DEFAULTS` | never calibrated, or the EEPROM blob is bad | [§7](#7-calibration-problems) |
| a channel span < 40 counts | that sensor is not moving or is mis-wired | [§7](#7-calibration-problems) |
| `loop max` in the tens of ms | something is blocking | [§9](#9-intermittent-and-weird) |
| `deferred` growing much faster than `sent` | asking for more than the air time allows | [§5](#5-link-problems) |
| arm banner reappears by itself | brown-out reset | [§8](#8-power-problems) |

## 1. Build and flash

**`VirtualWire.h: No such file or directory`**
Not in the Arduino Library Manager. `git clone
https://github.com/latchdevel/VirtualWire` into your `libraries` folder, or run
`tools/build.sh`, which clones it. Same for
[ServoTimer2](https://github.com/nabontra/ServoTimer2) and
[i2cdevlib](https://github.com/jrowberg/i2cdevlib).

**`'vw_get_rx_good' was not declared`**
You are on RadioHead-only configuration or an old VirtualWire. Check
`GLOVE_RF_DRIVER` matches the library you installed, and that both boards use
the same one.

**`ServoTimer2.h` conflicts with `Servo.h`**
Do not include both. ServoTimer2 exists precisely because VirtualWire owns
Timer1, which the stock `Servo` library needs. See
[ARCHITECTURE.md](ARCHITECTURE.md#5-design-constraints).

**`#error "GLOVE_PROFILE is not a known profile"`**
`GLOVE_PROFILE` must be 1 (`UNO_4FLEX`), 2 (`UNO_MUX`) or 3 (`MEGA_5FLEX`).

**`#error "GLOVE_RX_FAILSAFE_TIMEOUT_MS must be at least three frame air times"`**
The timeout is too short for the configured frame length and radio speed. At
2000 bps a 9-byte frame needs 96 ms, so the minimum is 288 ms. Either raise the
timeout or raise `GLOVE_RF_SPEED_BPS`. This guard exists because a shorter
timeout makes the failsafe fire during normal operation.

**`static_assert failed "a frame is SEQ + META + GLOVE_CHANNEL_COUNT channels"`**
You changed `GLOVE_CHANNEL_COUNT` or `GLOVE_FRAME_LEN` without the other. See
[PROTOCOL.md](PROTOCOL.md#9-extending-the-protocol).

**`error: 'GLOVE_X' was not declared` after editing a synced copy**
You edited `hand_receive/GloveConfig.h` instead of `common/GloveConfig.h`. Edit
the original and run `python3 tools/sync_common.py`.

**The IDE compiles but the arm behaves like the old firmware**
The wrong board was flashed, or the sketch folder was opened from a stale copy.
Check the protocol version in the banner.

## 2. Nothing happens at all

**No serial output from either board.**

1. Baud rate: `GLOVE_SERIAL_BAUD` is 115200, not 9600 (the original used
   38400). A wrong baud rate gives garbage, silence gives a different problem.
2. Correct port and board selected.
3. `GLOVE_TELEMETRY_ENABLE` is 1.
4. The board is actually running: the arm's LED shows link state, the glove's
   blinks per transmitted frame.

**Arm boots but no servo moves.**

1. `s` → is the mode `WAITING`? Then no valid frame has ever arrived → [§5](#5-link-problems).
2. Is the mode `RELAXED`? Someone sent `r`. Send `a` or `l`.
3. Is the servo supply on, and is its ground tied to the Arduino's? Servos
   powered from the Arduino's 5 V pin alone will not move seven of them.
4. During the 1.5 s startup grace period the arm moves to neutral on its own.
   If even that does not happen, it is wiring or power, not the link.

**Glove boots but the arm never sees a frame.**

1. Both banners must show the same protocol version and the same radio driver
   and speed.
2. TX module DATA on D12 of the glove, RX module DATA on D11 of the arm.
   Swapping them is the most common wiring mistake and produces exactly this
   symptom.
3. Both modules must be 433 MHz. A 315 MHz pair works perfectly and never
   talks to a 433 MHz pair.
4. Antennas: ~17 cm of wire on **both** modules. Without them the range is
   centimetres.

## 3. The arm does not follow the glove

**A servo never moves, others do.**
`s` on the arm shows `commanded` per servo. If the value changes but the servo
does not move: signal wiring, a dead servo, or that channel's pin is wrong in
`GLOVE_RX_SERVO_PINS`. If the value does not change: the glove is not sending
that channel — check `ch:` in the glove telemetry.

**All servos sit at one end stop.**
This is what the original firmware did (B-01): `ServoTimer2::write()` takes
microseconds, and a degree value 0–180 clamps to its 750 µs floor. If you see
it in this firmware, `GLOVE_RX_PULSE_MIN_US`/`MAX_US` have been edited to
something wrong, or a servo is being driven by other code.

**Motion is backwards on one joint.**
Set that entry of `GLOVE_RX_SERVO_INVERT` to 1 (arm side), or flip the finger
with `i <n>` on the glove. Prefer the arm side: it keeps the glove's
calibration independent of which hand wears it.

**Motion is backwards on the wrist only.**
`GLOVE_WRIST_INVERT_MASK` — bit 0 pitch, bit 1 roll.

**The wrist barely moves, or saturates.**
The captured wrist range is too narrow (you did not tilt far enough during
capture) or too wide for your mechanism. Check `wrist pitch -51..48 deg` in the
glove's calibration printout against `GLOVE_WRIST_PITCH_MIN_DEG/MAX`.

**The arm lags badly behind the hand.**
Expected: ~122 ms end to end, dominated by 96 ms of air time. If it is much
worse, check `q=` — heavy packet loss means the arm is running on stale frames.
`GLOVE_RF_SPEED_BPS 4000` halves the lag at some cost in range.

**The arm moves when the glove is still.**
Noise. Check the deadband (`GLOVE_DEADBAND_DEG`, 2), the EMA
(`GLOVE_EMA_ALPHA_Q8`, 40) and whether the flex span is very narrow — a 40-count
span means every ADC count is 4.5° of output.

## 4. Jitter, buzz and noise

**Servos buzz continuously.**
Almost always one of:

1. **Stalled against a limit** — narrow `GLOVE_RX_ANGLE_MIN/MAX`. A buzzing
   servo is converting current into heat; do not leave it.
2. **Noisy or sagging supply** — see [§8](#8-power-problems).
3. **Pulse range wrong for the servo** — some digital servos want 1000/2000 µs
   rather than 750/2250.
4. **Signal noise** — unshielded signal wires routed next to servo power leads.

**Fingers twitch by a degree or two.**
ADC noise amplified by a narrow calibration span. Raise
`GLOVE_DEADBAND_DEG` to 3, lower `GLOVE_EMA_ALPHA_Q8` to 25, and make sure the
divider resistor is close to the sensor's straight resistance so the span is
wide.

**The wrist angle wanders slowly.**
Gyro bias drift, usually thermal. Re-capture with `b` (glove flat and still)
and store with `w`. If it wanders fast, raise `GLOVE_IMU_COMP_ALPHA_PCT` toward
99 — but that makes the estimate trust the gyro more, so it will drift further
between accelerometer corrections.

**The wrist jumps when you move your arm.**
The accelerometer is being trusted during motion. It should not be: the filter
only accepts accelerometer corrections when the specific force is within
600–1600 milli-g. If you see this, check that the trust constants in
`common/Attitude.h` were not modified, and that `GLOVE_IMU_ACCEL_FS_G` still
matches the range configured in `AttitudeFilter::begin()`.

**Everything is noisy only when the servos move.**
Brush noise. Bulk capacitance on the servo rail, keep the RX module and antenna
away from the servos, and add a 0.1 µF ceramic per servo. See
[HARDWARE.md](HARDWARE.md#6-power).

## 5. Link problems

**`rejected` is rising.**
A frame arrived with a good radio FCS but failed validation. Causes, in order of
likelihood: protocol version mismatch (reflash both), unknown command (a
different firmware version), wrong length. The arm prints the reason for the
first three rejections — read it.

**`q=` is well below 100% and `lost` climbs.**
Real packet loss. Fixes, cheapest first: antennas on both modules; a
superheterodyne receiver (SYN480R/RXB6) instead of a regenerative one
(XD-RF-5V); move the RX away from servo wiring; reduce `GLOVE_RF_SPEED_BPS` to
2000 if it is at 4000; check for another 433 MHz device nearby (weather
stations, doorbells, other projects).

**The arm goes into failsafe every few seconds.**
Either loss is that bad, or `GLOVE_RX_FAILSAFE_TIMEOUT_MS` is too tight for the
configuration. It must be at least three air times (288 ms at 2000 bps) — the
build refuses smaller values, so if you are seeing this at 500 ms, the loss is
real.

**`deferred` grows much faster than `sent`.**
The glove wants to transmit more often than the air time allows. This is normal
to a degree — `GLOVE_TX_SEND_MS` is 20 ms and one frame takes 96 ms — but if
`sent` is not rising at roughly 10/s at 2000 bps, the transmitter is not
finishing its frames: check the PTT pin configuration
(`GLOVE_RF_PTT_PIN`/`GLOVE_RF_PTT_INVERTED`) and the module's supply.

**Range is a few centimetres.**
Missing antennas, or a FS1000A-class transmitter running below its supply
voltage. Also check that the TX module is not being powered from a 3.3 V pin
when it wants 5 V.

**Works on the bench, fails near the arm.**
Servo and motor noise. This is the classic ASK failure mode: the receiver is
deafened by brush arcing. Capacitance, separation, and a better receiver module.

## 6. IMU problems

**`IMU: NOT RESPONDING`.**

1. **The A4/A5 conflict** — if you wired five flex sensors on A0–A4 with an
   MPU-6050, the fifth sensor is across the I²C bus. Use
   `GLOVE_PROFILE_UNO_4FLEX` (default) or `UNO_MUX`. This is B-05 from the
   original firmware and the most likely cause by far.
2. `AD0` high gives address 0x69; `GLOVE_IMU_I2C_ADDR` must match.
3. SDA/SCL swapped, or not connected at all.
4. A 3.3 V bare module powered from 5 V (or vice versa). Most GY-521 breakouts
   have a regulator and level shifters; bare chips do not.
5. Long, unshielded I²C wiring: drop `GLOVE_IMU_I2C_HZ` to 100000.
6. A dead module. Test it with any I²C scanner sketch.

The glove keeps working without the IMU: wrist channels are held at neutral and
`GLOVE_FLAG_IMU_OK` stays clear, so the arm knows. If you do not want the I²C
code at all, build with `GLOVE_IMU_ENABLE 0`.

**`[imu] bias capture rejected`.**
The glove was not flat and still: the capture requires a specific force of
900–1100 milli-g and less than ~0.2 g on either horizontal axis. Put it on a
level surface, keep your hand off it, and send `b` again. A rejected capture
leaves the previous bias in place, so retrying is always safe.

**Pitch and roll are swapped or mirrored.**
The IMU is mounted rotated relative to the hand. Set the axis signs with
`setGyroAxisSigns()` defaults in `common/Attitude.h`
(`GLOVE_ATTITUDE_ROLL_SIGN_DEFAULT`/`PITCH_SIGN_DEFAULT`), or remount the
module. Do not try to fix it with `GLOVE_WRIST_INVERT_MASK` alone — that
inverts the output angle, not the sensor axis, and will not correct a swap.

## 7. Calibration problems

**`[cal] capture done: 0 channel(s) updated`.**
Nothing moved far enough to be believed: every span was under
`GLOVE_CAL_MIN_SPAN` (40 counts). Move each finger fully and slowly, and check
the raw values with `d` first — if a channel does not change by 100+ counts
when you bend it, calibrating cannot help; fix the wiring.

**A channel says `[UNUSABLE]`.**
Its stored span is too narrow, so `flexToAngle()` returns neutral for it
instead of an angle derived from noise. Re-capture, moving that finger
deliberately.

**Calibration is lost every power cycle.**
Look for `[cal] EEPROM VERIFY FAILED`. The firmware writes, reads back and
checks the CRC; a failure means the write did not take. Retry; if it persists,
the board's EEPROM or its supply is at fault. Values remain live in RAM for the
session.

**Calibration loads as `DEFAULTS` on a board you calibrated before.**
The blob failed its magic/version/CRC check — a different firmware version
wrote it, or it is corrupt. Defaults are used, which is safe but uncalibrated.
Re-capture and `w`.

**A finger is exactly backwards.**
`i <n>` then `w`. Direction is stored separately from the span, so no
re-capture is needed.

**The glove is calibrated but the arm's poses are wrong.**
Two different calibrations. The glove maps sensors to angles; the arm maps
angles to your mechanism. The arm side is `GLOVE_RX_*_POSE` and
`GLOVE_RX_ANGLE_MIN/MAX` in `GloveConfig.h` — see
[CALIBRATION.md](CALIBRATION.md#5-calibrating-the-arm).

## 8. Power problems

**The arm's banner reappears spontaneously.**
Brown-out reset. The servos are dragging the rail down. Fixes: a supply rated
for all servos moving at once (≥ 5 A for seven), 1000 µF bulk capacitance at
the servos, and power the Arduino from its own jack/VIN rather than from the
servo rail.

**The arm resets when a servo starts moving.**
Same cause, and the classic symptom of servos powered from the Arduino's 5 V
pin. Never do that.

**Servos move sluggishly or stutter under load.**
Voltage sag. Measure the servo rail *while* the arm is moving; below ~4.6 V for
a 5 V servo is the problem.

**The glove works on USB but not on battery.**
A 9 V battery through the barrel jack is fine; a partially discharged LiPo
through VIN may not reach the regulator's dropout. Measure the 5 V pin. Also
check that the TX module is not browning out the glove — a radio burst draws
~15 mA, but cheap modules can spike.

**Everything misbehaves when the servos are connected.**
Missing common ground between the servo supply and the Arduino. This is the
first thing to check and the easiest to forget.

## 9. Intermittent and weird

**`loop max` is tens of milliseconds.**
Something blocks. Expected worst cases: a telemetry line (~5 ms at 115200), an
I²C read (~0.3 ms), an ADC sweep (~0.5 ms). Anything above ~10 ms on the arm
means serial output is too verbose or a library call is waiting; on the glove,
check for I²C retries (`GLOVE_IMU_RETRY_COUNT` × `GLOVE_IMU_RETRY_DELAY_MS` can
add a second at boot only).

**Works for ten minutes, then stops.**
Three candidates: thermal drift in the gyro bias (re-capture with `b`), a
brown-out as the battery sags (measure it), or a servo stalling and dragging
the rail down (feel for a hot servo).

**A watchdog reset loop.**
`setup()` taking longer than `GLOVE_WATCHDOG_TIMEOUT`. Both sketches clear
`WDRF` and call `wdt_disable()` before doing anything else, which prevents the
usual cause; if you add blocking work to `setup()` (a long IMU retry chain, say)
keep it under 2 s or raise the timeout.

**The arm does something once, at power-on, that it never does again.**
That is the startup grace period: 1.5 s of slewing to the neutral pose while
ignoring the radio. It is intentional — see [SAFETY.md](SAFETY.md#2-layers-of-protection).

**Values look shifted by one channel.**
A frame-length or layout mismatch, i.e. the two boards are running different
protocol versions. The version check should have caught it; if it did not,
compare the banners and reflash both.

**It only fails when my phone/other project is nearby.**
433 MHz is a shared band. Move the other device, or move to a different link —
see the nRF24L01+ plan in [ROADMAP.md](ROADMAP.md).

### Still stuck?

Collect these and the answer is usually obvious:

```bash
python3 tools/glove_monitor.py --port /dev/ttyUSB0 --baud 115200 --log captures/glove.csv
python3 tools/glove_monitor.py --port /dev/ttyACM0 --baud 115200 --log captures/arm.csv
python3 tools/rf_airtime.py --budget
make -C tests check
./tools/check_configs.sh
tools/build.sh            # flash/RAM report
```

plus: both boot banners, `s` from both boards, the firmware version, the
`GLOVE_PROFILE` in use, and what changed immediately before it broke.
