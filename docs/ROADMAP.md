# Roadmap

What is worth doing next, in the order it is worth doing it, and a few things
that look attractive and are not.

Each item lists the payoff, the cost, and the constraint that decides whether
it is feasible on this hardware.

- [Priority 1 — the radio](#priority-1--the-radio)
- [Priority 2 — sensing](#priority-2--sensing)
- [Priority 3 — the mechanism](#priority-3--the-mechanism)
- [Priority 4 — tooling](#priority-4--tooling)
- [Priority 5 — firmware hygiene](#priority-5--firmware-hygiene)
- [Licensing](#licensing)
- [Rejected alternatives](#rejected-alternatives)
- [A note on scope](#a-note-on-scope)

---

## Priority 1 — the radio

The 96 ms air time of a 9-byte ASK frame at 2000 bps is ~78% of the end-to-end
latency. Nothing else in the system is close. Every other improvement on this
list is a rounding error until this one is addressed.

### 1.1 Move to nRF24L01+ (highest value in the project)

| | |
| --- | --- |
| **Payoff** | ~250 µs per frame instead of 96 ms → **~400× faster link**, control rate limited by the servo refresh (50 Hz) rather than the radio. Two-way, so acknowledgements and retransmission become possible. 2.4 GHz is less crowded than 433 MHz in most buildings |
| **Cost** | ~$2/module, one on each board. New `RfLink.h` backend (~80 lines: `begin`, `send`, `receive`, `txBusy`). SPI on pins 11–13, CE on 9, CSN on 10 |
| **Constraint** | **SPI pins collide with the current wiring.** On the arm, D11 is the ASK RX data pin and D13 is the LED; on the glove, D12/D10 are the TX/PTT. The servo pins 3, 9, 4, 5, 6, 7, 8 also collide with CE (9). A pin re-map is unavoidable — `GLOVE_RX_SERVO_PINS` and `GLOVE_RF_*_PIN` exist to make that a config change rather than a code change |
| **Risk** | Low. `RfLink.h` was written for exactly this: the sketches call six functions and never touch a driver |

Also frees **Timer1** (VirtualWire/RH_ASK claim it), which means the stock
`Servo` library becomes usable — and with it, more than 8 channels on a Mega.

### 1.2 Acknowledgements and retransmission (after 1.1)

With a two-way link the arm can acknowledge, and the glove can retransmit a
*command* (not a pose — a stale pose is worse than none). This turns
`GLOVE_TX_CMD_REPEAT_MS` from a 400 ms broadcast window into a single
confirmed transaction. Cost: ~50 lines and one extra config knob. Only worth it
once 1.1 has removed the latency penalty of the round trip.

### 1.3 Multi-glove addressing

`GLOVE_RF_ADDRESS` on the glove, a filter on the arm, and one byte in the frame
(or an nRF24 pipe address, which is free). Enables two arms from one glove, or
a teacher/student setup. Cheap after 1.1; pointless before it, because the ASK
band has no addressing at all.

### 1.4 Link quality from the physical layer

Most ASK receivers expose a VT/RSSI pin; nRF24 modules report a received-power
measurement directly. `LinkMonitor` currently derives quality from sequence
gaps, which measures *loss* but not *margin*. Adding RSSI would let the arm warn
before the link fails rather than after. Small: one `analogRead()` per second
and one telemetry field.

### 1.5 Battery telemetry

A resistor divider on the glove's supply into a spare ADC pin, one byte in the
frame (or a flag when low). The most common cause of a "mystery" failsafe is a
dying battery, and today the arm can only report the symptom. **Costs one
payload byte = 6 ms of air time at 2000 bps**, which is a real argument for
doing this after 1.1 rather than before.

---

## Priority 2 — sensing

### 2.1 Yaw, via a magnetometer

Pitch and roll come from gravity and are stable. Yaw cannot be observed without
a magnetic reference, so it is deliberately not attempted — a gyro-only yaw
drifts several degrees per minute and would make the wrist unusable over a
session. An MPU-9250 or an HMC5883L on the same I²C bus fixes this properly.
Cost: one sensor, a tilt-compensation routine (~40 lines, testable in
`common/`), and a hard-iron calibration step in
[CALIBRATION.md](CALIBRATION.md). Do not attempt yaw without the magnetometer.

### 2.2 A better fusion filter (Madgwick or Mahony)

The current complementary filter is ~40 lines of integer maths, fully unit
tested, and accurate enough for a wrist. A Madgwick AHRS would converge faster
after large movements and give a quaternion (so all three axes are consistent).
Cost: fixed-point quaternion maths on an ATmega328P is not trivial — expect
~500 µs per update and careful overflow analysis. **Only worth it if 2.1
happens**, since a quaternion is mainly useful when you have three axes to
keep consistent.

### 2.3 Use the MPU-6050 DMP

Rejected once, worth re-stating why: the DMP needs ~6 KB of flash for the
firmware image plus the driver's own code, its initialisation sequence is a
black box that cannot be unit tested, and it consumes the interrupt line. On a
32 KB board that already carries the radio and I²C stacks, it is a poor trade.
Revisit only on a Mega or an ESP32.

### 2.4 Finger-joint sensing

One flex sensor per finger measures one degree of freedom. Real hands have
three per finger. Two sensors per finger (or a Hall sensor at the knuckle plus
a flex at the middle joint) would let the arm reproduce a curl rather than an
approximation of one. This is a mechanical design change first and a firmware
change second — and it needs the extra channels from 1.1.

---

## Priority 3 — the mechanism

### 3.1 More servos via a PCA9685

The hard limit today is ServoTimer2's 8 channels. A PCA9685 16-channel PWM
driver over I²C removes the limit *and* frees Timer2 (so pins 3 and 11 keep
their PWM). Cost: ~$4, one I²C address, and a new backend behind the same
`angleToPulseUs()` → `write()` interface. The natural enabler for a two-handed
system or a 6-DOF arm plus a hand.

### 3.2 Inverse kinematics

The system today is a *joint-space mirror*: glove angle → servo angle. A real
arm needs Cartesian targets — "put the fingertip here" — which means IK. This
is a substantial, self-contained project: a kinematic model of the mechanism, a
solver that fits in the remaining flash (an analytic solution for a 3-link planar
arm is ~100 lines; a numeric Jacobian is not feasible here), and a way to
specify the geometry in `GloveConfig.h`.

Recommended path: implement the solver in `common/` as pure integer/fixed-point
maths with unit tests (exactly like `Attitude`), then have the arm consume
Cartesian channels instead of joint angles. That is a **protocol change** —
bump the version and update [PROTOCOL.md](PROTOCOL.md).

Realistically, IK belongs on a host machine or a bigger MCU, with the ATmega328P
as a servo driver. See [rejected alternatives](#rejected-alternatives).

### 3.3 Force and current sensing

The arm has no idea whether it is gripping a marshmallow or stalling against a
hard stop. A current sensor on the servo rail (INA219, I²C) or per-joint
feedback would enable: grip-force limiting, stall detection, and a failsafe
that reacts to *physical* trouble rather than only to a lost link. This is the
single biggest safety upgrade available, and it is not in the current design at
all — see [SAFETY.md](SAFETY.md#7-what-this-project-does-not-protect-against).

### 3.4 Tendon-driven fingers and the calibration that implies

A tendon-driven hand is nonlinear: servo angle → tendon travel → finger curl is
not a straight line, and it differs per finger. A per-channel *curve* (3–4
points, piecewise-linear interpolation) in `CalibrationData` would fix the
feel. Cost: ~30 bytes of EEPROM, ~40 lines in `common/`, and a calibration
capture that asks for three poses instead of two. Worth doing if the mechanism
is tendon-driven; unnecessary for direct-drive links.

---

## Priority 4 — tooling

### 4.1 A host-side calibration wizard

`tools/glove_monitor.py` already bridges the serial CLI. Extending it to run the
whole calibration — prompt, capture, show the spans graphically, ask for
direction confirmation, save, verify — would remove the most fiddly part of
setting up a new glove. Pure Python, no firmware change, and the CSV telemetry
format already exists to support it.

### 4.2 Record and replay

Log frames to a file (the monitor can already log), then replay them into the
arm from the host. Uses: demonstrating without the glove, regression-testing a
mechanical change against a known motion, and reproducing a reported problem
exactly. Needs a host-side frame encoder — `GloveProtocol` is 120 lines of C++
that would port to Python in an afternoon, and the two must be kept in sync by
a shared test vector file.

### 4.3 A visual dashboard

Live joint angles, link quality and loop timing in a browser, fed by the
monitor over a local socket. Nice for demos and for spotting a degradation
before it becomes a failure. Low priority: the serial output already carries
everything, and `glove_monitor.py` renders it.

### 4.4 Hardware-in-the-loop CI

A physical Uno on a runner, flashed and exercised per commit. Catches the class
of bug the host stubs cannot: real timing, real flash size, real library
behaviour. Expensive to set up and maintain; the avr-gcc CI job already covers
the most common hardware-only failure (it does not fit).

---

## Priority 5 — firmware hygiene

### 5.1 Move more sketch logic into `common/`

The sketches still contain scheduling logic that is only smoke-tested, not
asserted: the glove's frame assembly and command-repeat window, the arm's mode
machine (`BOOT`/`WAITING`/`LIVE`/`POSE`/`PARKED`/`RELAXED`/`FAILSAFE`). Both
are pure decisions over inputs, and both would be better as tested modules —
`GloveScheduler` and `ArmModeMachine` in `common/`, with the sketches reduced
to hardware access and wiring. This is the highest-value refactor left, and it
is invisible to a user, which is exactly why it needs to be scheduled rather
than hoped for.

### 5.2 Widen the protocol version field

Two bits give four versions, and version 2 is already in use. The next frame
change should spend a whole byte on version (or a version byte plus a
reserved byte) — it costs 6 ms of air time, which is free after 1.1 and
expensive before it. Plan it together with 1.5.

### 5.3 A single configuration validator

The `#error` and `static_assert` guards are spread across `GloveConfig.h`,
`GloveProtocol.h` and `hand_receive.ino`. Collecting them into one
`GloveConfigCheck.h` with a comment per guard would make the safety envelope
readable in one screen — useful when reviewing a change, which is when it
matters.

### 5.4 Static analysis in CI

`cppcheck --enable=warning,style` and `clang-tidy` with a small rule set would
be cheap and would catch classes of bug the compiler does not (unused results,
suspicious implicit conversions across the Arduino API boundary). Worth an
hour; keep the rule set small enough that CI stays green.

### 5.5 Per-glove identity

A serial number or name in `CalibrationData`, printed in the banner and
reportable over the CLI. Trivial to add, and it stops the "which glove is
calibrated for which hand?" problem the moment there is more than one glove.

---

## Licensing

The repository has **no licence file**, which currently means "all rights
reserved" — nobody may legally copy or modify it, which is probably not the
intent of a project built on GPL libraries.

The constraint: **VirtualWire is GPL-2.0** and RadioHead is GPL-2/3 (or a
commercial licence). Firmware that links them must be distributed under a
GPL-compatible licence. ServoTimer2 and jrowberg's I2Cdev/MPU6050 are
MIT/permissive, which is compatible.

Recommendation: **GPL-3.0-or-later**. It is compatible with both radio
libraries, it is the standard choice for Arduino firmware, and it keeps the
option of moving to RadioHead's GPL-3 side. Add a `LICENSE` file, an
`SPDX-License-Identifier: GPL-3.0-or-later` header comment to each source file,
and a `THIRD-PARTY-LICENSES.md` crediting VirtualWire, RadioHead,
ServoTimer2 and I2Cdev/MPU6050.

If a permissive licence (MIT/Apache-2.0) is preferred instead, then RadioHead
must be used under its commercial licence, or the radio layer must be replaced
outright (which 1.1 does anyway — nRF24 libraries such as RF24 are GPL-2, so
the same question recurs; `RF24` alternatives under LGPL/MIT exist).

This is a decision for the project owner, not something to do silently. It is
listed here because it is currently the only *legal* blocker to anyone using
this code.

---

## Rejected alternatives

| Idea | Why not |
| --- | --- |
| **Bluetooth (HC-05/HC-06) instead of 433 MHz** | Slower to set up (pairing), higher latency than ASK in practice, and it needs a phone or PC in the loop. It is a good choice for a *wired-replacement* project, not for a glove→arm link with no host. nRF24L01+ beats it on every axis that matters here |
| **ESP32/ESP8266 rewrite** | Wi-Fi adds latency variability, a stack, and a power budget; the ESP32 is a fine chip but this project's bottleneck is the *link*, not the MCU. If you want Wi-Fi telemetry, add an ESP as a serial bridge rather than moving the firmware — the ATmega328P's determinism is an asset in a control loop |
| **ROS integration** | Real value for a research arm, and a big one: it assumes a Linux host, which changes the project from "two Arduinos and a radio" into "a robot with a PC". If that is where you are going, do 3.2 (IK) on the host and keep the Arduino as a servo driver over serial — the frame format in [PROTOCOL.md](PROTOCOL.md) is close to what such a bridge needs |
| **MPU-6050 DMP** | See [2.3](#23-use-the-mpu-6050-dmp): flash cost, untestable, and the complementary filter is good enough for two axes |
| **Delta/variable-length frames** | Saves air time only when the hand is still — which is when air time does not matter — and makes the parser stateful, so a corrupt length could desynchronise it. Fixed frames are the robust choice; if air time becomes critical, change the radio instead |
| **A real-time OS / FreeRTOS** | The scheduler is five `millis()` comparisons. An RTOS would add a kernel, a heap and a class of bug, to solve a problem that does not exist |
| **Rewriting in MicroPython/CircuitPython** | Loses determinism, flash headroom and the entire existing test suite, for a language convenience that does not help a 96 ms radio link |
| **Keeping VirtualWire as the only driver** | It is end-of-life and unmaintained. `RfLink.h` keeps it as the default because it is what this project has always used and it is easy to obtain, but the RadioHead path is built by CI so the escape hatch is never rotting |

---

## A note on scope

The suggestions above are ordered by value per unit of effort **for this
project as it stands**: two ATmega328P boards, a hobby servo hand, and a
one-way radio link. Two of them change the character of the project rather than
improving it — 3.2 (inverse kinematics) turns a joint-space mirror into a
Cartesian robot, and ROS turns a pair of microcontrollers into a node in a
larger system. Both are legitimate directions; neither should be started
before 1.1, because until the link stops costing 96 ms per update, nothing
downstream of it can be evaluated honestly.

If only one item on this list is ever done, do **1.1**.
