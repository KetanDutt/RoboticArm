# Safety

Seven servos, a radio link that can drop packets, and a mechanism that can
pinch. This document is the operating manual for that combination.

- [1. What can go wrong](#1-what-can-go-wrong)
- [2. Layers of protection](#2-layers-of-protection)
- [3. First power-on procedure](#3-first-power-on-procedure)
- [4. Choosing a failsafe action](#4-choosing-a-failsafe-action)
- [5. Why remote relax is disabled](#5-why-remote-relax-is-disabled)
- [6. Operating rules](#6-operating-rules)
- [7. What this project does NOT protect against](#7-what-this-project-does-not-protect-against)
- [8. Changing safety-critical code](#8-changing-safety-critical-code)
- [9. Demonstration and public-use checklist](#9-demonstration-and-public-use-checklist)

## 1. What can go wrong

| Hazard | Mechanism | Who/what gets hurt |
| --- | --- | --- |
| Unexpected motion | A packet arrives after a pause, or a preset fires | Fingers near a joint; the mechanism itself |
| Slam | A corrupted or out-of-range angle is obeyed at full servo speed | Gears, links, whatever is being held |
| Stall | A joint driven past its mechanical limit | Servo overheating, burned driver, sagging power rail |
| Runaway | A hung MCU keeps the last command while nobody is watching | Sustained stall current, heat, fire risk in the worst case |
| Loss of grip | The link drops while the hand is holding something | The object, and anything under it |
| Letting go | A garbled frame is interpreted as "relax the servos" | The arm falls, or releases a held object |
| Brown-out | Servo inrush resets the MCU mid-motion | Uncommanded motion on restart |
| Wrong firmware | Only one board is reflashed; the frame layout differs | Arbitrary values reach the servos |
| Interference | Another 433 MHz device on the same channel | Spurious motion |

Every one of these has a corresponding mitigation in the next section, except
where [§7](#7-what-this-project-does-not-protect-against) says otherwise.

## 2. Layers of protection

Ordered from the last thing that touches a servo backwards. Each layer is
independent, so a failure in one does not disable the others.

| # | Layer | Where | What it stops |
| --- | --- | --- | --- |
| 1 | **Mechanical window** | `GLOVE_RX_ANGLE_MIN/MAX`, applied inside `Frame::decode()` | Any angle outside the window, from any source, including presets and the failsafe pose |
| 2 | **Pose clamping** | `copyPose()` in the arm | A hand-edited preset array that exceeds the window |
| 3 | **Slew rate limit** | `SlewLimiter`, one per servo | A slam: the arm cannot move faster than `GLOVE_RX_SLEW_MAX_DPS` (300 °/s) whatever it is told |
| 4 | **Frame validation** | `Frame::decode()` | Wrong length, wrong protocol version, unknown command |
| 5 | **Link supervision + failsafe** | `LinkMonitor` | A lost, dead or garbage-filled link: after 500 ms the arm goes to a defined pose |
| 6 | **Rejected frames do not refresh the timeout** | `drainRadio()` | A channel full of valid-FCS garbage being treated as a live link |
| 7 | **Remote relax disabled by default** | `GLOVE_RX_ALLOW_REMOTE_RELAX 0` | Any frame — corrupt, replayed or malicious — letting go of the arm |
| 8 | **Power-on grace period** | `GLOVE_RX_STARTUP_POSE_MS` | The arm snapping to an unknown angle at boot; it slews gently to neutral and ignores the radio for 1.5 s |
| 9 | **Attach snaps software to hardware** | `attachServos()` | ServoTimer2's 1500 µs default disagreeing with the rate limiter, which would cause a jump on the first command |
| 10 | **Watchdog** | both sketches, `WDTO_2S` | A hung board: it resets, boots into the grace period, and goes to neutral |
| 11 | **Compile-time guards** | `GloveProtocol.h`, `GloveConfig.h`, `hand_receive.ino` | Flashing a configuration in which the failsafe cannot work |
| 12 | **Glove-side limits** | `GLOVE_ANGLE_FLOOR/CEILING`, deadband, filters | Wild sensor readings ever becoming wild angles |
| 13 | **IMU degraded mode** | `GLOVE_FLAG_IMU_OK` clear | A dead IMU producing noise-driven wrist angles; the wrist is held neutral instead |

Layers 11 and 4–7 are the ones worth understanding properly, because they are
the ones that turn "the radio is unreliable" from a hazard into a nuisance.

### The compile-time guards

```cpp
/* GloveProtocol.h */
#if GLOVE_RX_SERVO_COUNT > GLOVE_CHANNEL_COUNT
#error "GLOVE_RX_SERVO_COUNT cannot exceed the protocol's channel count"
#endif
#if GLOVE_TX_CMD_REPEAT_MS < (2u * GLOVE_AIRTIME_MS)
#error "GLOVE_TX_CMD_REPEAT_MS must cover at least two frame air times"
#endif
#if GLOVE_RX_FAILSAFE_TIMEOUT_MS < (3u * GLOVE_AIRTIME_MS)
#error "GLOVE_RX_FAILSAFE_TIMEOUT_MS must be at least three frame air times"
#endif

/* hand_receive.ino */
#if GLOVE_RX_SERVO_COUNT > 8
#error "ServoTimer2 drives at most 8 channels"
#endif
```

The timing guards are written in terms of `GLOVE_AIRTIME_MS`, which is derived
from the frame length and the radio speed. If you grow the frame or slow the
radio, an old failsafe timeout stops compiling. That is the guard working: a
timeout shorter than the link's own frame period would fire during normal
operation, and the arm would spend its life in failsafe.

CI builds both sketches with `-DGLOVE_RX_FAILSAFE_TIMEOUT_MS=50` and
`-DGLOVE_PROFILE=99` and **requires the build to fail**. A guard nobody has
seen fire is a comment.

## 3. First power-on procedure

Do this every time the hardware changes, and the first time you ever run a new
build. It takes ten minutes and it is the difference between a working arm and
a stripped servo.

**Stage 1 — arm, no servos**

1. Flash the arm. Leave all servo plugs disconnected.
2. Open serial at 115200. Confirm the banner:

   ```
   === RoboticArm receiver ===
   protocol v2, radio VirtualWire @ 2000 bps, 7 servos
   failsafe after 500 ms -> return to NEUTRAL
   ```

   The protocol version must match the glove's banner exactly.
3. Power the glove. Within a second or two the arm should print `LINK OK q=NN%`
   and telemetry should show `LIVE`.
4. Send `s`. Check `good` is rising, `lost` is small, `rejected` is 0. A
   non-zero `rejected` count means the two boards disagree — stop and fix that
   first.
5. Switch the glove off. Within 500 ms the arm must print `LINK LOST` and enter
   `FAILSAFE`. Switch it on again: `LINK OK`, back to `LIVE`.

**Stage 2 — one servo**

6. Power the servo supply (5–6 V, adequately rated, **common ground with the
   Arduino**). Connect **one** servo — the wrist pitch on pin 3.
7. Reset the arm. Watch it: during `GLOVE_RX_STARTUP_POSE_MS` it should move
   smoothly to the neutral pose, not snap.
8. Move your wrist. The servo should follow, smoothly, in the right direction,
   and stop inside its mechanical range.
9. Send `n` (park) and `l` (live) and confirm both behave.
10. Send `r` (relax) — the servo should go limp. Send `a` — it should re-energise
    at neutral. **Support the arm before doing this.**

**Stage 3 — the rest**

11. Connect the remaining servos one at a time, repeating step 8 for each.
    Watch the arm's serial for a repeated boot banner: that is a brown-out, and
    it means the supply is too small (see
    [HARDWARE.md](HARDWARE.md#6-power)).
12. Only now, run the full calibration in
    [CALIBRATION.md](CALIBRATION.md) and tune the poses.

**Stage 4 — the tests that matter**

13. Walk out of radio range with the glove. The arm must failsafe within
    ~500 ms and must not twitch afterwards.
14. Press the glove button. The arm should latch a preset for 2 s and then
    resume live control.
15. Wiggle a flex sensor's wiring to simulate a broken connection. The glove
    should report the channel as `[UNUSABLE]` or hold its last value — it must
    not drive the servo to an end stop.

## 4. Choosing a failsafe action

`GLOVE_RX_FAILSAFE_ACTION` — what the arm does when the link has been gone for
`GLOVE_RX_FAILSAFE_TIMEOUT_MS`.

| Action | Value | Behaviour | Choose it when |
| --- | --- | --- | --- |
| `GLOVE_FAILSAFE_HOLD` | 0 | Freeze at the last commanded pose, servos still energised | The arm is holding something that must not be dropped, and the pose is safe to hold indefinitely. Note: a stalled servo draws current and heats up |
| `GLOVE_FAILSAFE_NEUTRAL` | 1 (**default**) | Slew to `GLOVE_RX_NEUTRAL_POSE` at the rate limit, then hold | Most builds. Predictable, gentle, and the neutral pose is one you have verified |
| `GLOVE_FAILSAFE_RELAX` | 2 | Detach the servos — no holding torque | **Only** if the arm is gravity-supported or counterbalanced. Otherwise it falls |

Two related knobs:

- `GLOVE_RX_FAILSAFE_TIMEOUT_MS` (500 ms ≈ 5 frames at 2000 bps). Longer means
  fewer spurious trips in a noisy RF environment; shorter means a faster
  reaction to a real loss. Do not go below three air times — the build refuses.
- `GLOVE_RX_POSE_HOLD_MS` (2000 ms). How long a preset stays latched. Note
  that the failsafe overrides a latched pose: if the link dies during a preset,
  the failsafe action wins. That is deliberate.

Whatever you choose, **test it** by switching the glove off (step 13 above). A
failsafe that has never been observed is a hypothesis.

## 5. Why remote relax is disabled

`GLOVE_CMD_RELAX` exists in the protocol, and the arm will ignore it unless you
set `GLOVE_RX_ALLOW_REMOTE_RELAX 1`.

The reasoning: relaxing the servos removes all holding torque. It is the one
command whose effect is *not* bounded by the mechanical window, the slew
limiter or the pose tables — every other layer of protection works by limiting
where a servo may go, and "let go" is not a place. On an unacknowledged,
unencrypted ASK link where a corrupt frame that passes the radio's FCS is a
routine occurrence, that is too much authority for too little benefit.

If you want it anyway (a gripper that must release on command, say), then:

1. Set `GLOVE_RX_ALLOW_REMOTE_RELAX 1` and reflash the arm.
2. Make the mechanism safe to drop — counterbalanced, or over a soft surface.
3. Consider requiring two consecutive frames before acting, and a distinct
   command value from the one your button cycles through.
4. Test it deliberately: send a `RELAX` frame with the arm loaded and see what
   actually happens.

The serial CLI `r` command is always available and is the intended way to relax
the arm: a human at a terminal, who can see the mechanism, is a better judge
than a radio packet.

## 6. Operating rules

- **Power the servos from a proper supply.** Never from the Arduino's 5 V pin.
  See [HARDWARE.md](HARDWARE.md#6-power).
- **Common ground, always.** Arduino, servo supply and radio module.
- **Keep hands clear during the power-on grace period.** The arm moves on its
  own for 1.5 s after every reset, including resets caused by a brown-out.
- **Do not disable the watchdog to "debug more easily" and forget to re-enable
  it.** If you must, set `GLOVE_WATCHDOG_ENABLE 0` in `GloveConfig.h` so it is
  visible in the diff, and never in a committed configuration.
- **Do not widen `GLOVE_RX_ANGLE_MIN/MAX` past what you have mechanically
  verified.** The defaults (10/170) are conservative; your mechanism may need
  to be more so.
- **Treat a rising `rejected` count as a stop-work condition.** It means the
  two boards disagree about the protocol.
- **If the arm buzzes, it is stalled.** Something is at a limit or the pulse
  range is wrong. Do not leave it there — a stalled servo converts electrical
  energy into heat with nowhere for it to go.
- **Supervise the first run of any firmware change**, with the servos
  disconnected if the change touched anything in the receive path.

## 7. What this project does NOT protect against

Being explicit about the gaps is part of the safety story.

| Gap | Consequence | Mitigation you can add |
| --- | --- | --- |
| **No position feedback** | The firmware commands a pulse width; it has no idea where the servo actually is. A stalled or broken servo is invisible except through current draw | Add a current sensor on the servo rail, or potentiometer feedback on the joint |
| **No collision detection** | The arm will close on a finger, a cable or itself if the angles say so | Keep the mechanical window narrow; add limit switches; never put a hand inside a moving mechanism |
| **No hardware emergency stop** | The only way to stop it immediately is to cut power | Wire a physical switch or relay in the servo supply line, within reach |
| **No authentication on the link** | Anyone with a 433 MHz transmitter and this protocol can drive the arm | Use it in a controlled environment; consider a rolling code (see [ROADMAP.md](ROADMAP.md)) |
| **The 433 MHz band is shared** | Interference raises the packet loss; at 100% loss you get the failsafe, which is safe but not useful | Monitor `q=` in telemetry; move to a licensed/clean band or a 2.4 GHz module |
| **No thermal or current monitoring** | A servo stalled against a limit will cook itself eventually | `HOLD` failsafe plus a supervised session; or add sensing |
| **The glove is not intrinsically safe** | It is worn on a hand that is near a moving mechanism | Keep the mechanism's force low enough that a human can resist it |
| **Nothing stops you from editing a mechanical limit to something unsafe** | The compiler only checks *consistency*, not *safety* | Review diffs to `GloveConfig.h`; the values are all in one file for exactly this reason |

This is a hobby teleoperation system, not a collaborative robot. It has no
force limiting, no certified safety functions and no redundancy. Do not use it
near people who have not been told what it is, and never as part of anything
that could injure someone if it misbehaved.

## 8. Changing safety-critical code

These files and macros are safety-critical. A change to any of them needs a
test and a note in [CHANGELOG.md](CHANGELOG.md):

| File / macro | Why |
| --- | --- |
| `common/ServoDrive.cpp` — `SlewLimiter`, `LinkMonitor`, `angleToPulseUs` | The last three layers between a packet and a motor |
| `common/GloveProtocol.cpp` — `Frame::decode` | The only validation the arm performs |
| `GLOVE_RX_ANGLE_MIN/MAX`, `GLOVE_RX_SLEW_MAX_DPS`, `GLOVE_RX_FAILSAFE_*`, `GLOVE_RX_ALLOW_REMOTE_RELAX` | The arm's physical authority |
| `GLOVE_RX_*_POSE` | Where the arm goes without being asked |
| the `#error` guards | The checks that stop the above being misconfigured |

The workflow:

```bash
python3 tools/sync_common.py     # after editing common/
make -C tests check              # unit tests + both sketches smoke-run
./tools/check_configs.sh         # all eight configurations, zero warnings
./tools/build.sh                 # real avr-gcc build, check the size report
```

then the bench procedure in §3, then a failsafe test, then the change is
allowed to be called finished.

## 9. Demonstration and public-use checklist

- [ ] Servo supply rated for all servos moving at once, with bulk capacitance
- [ ] Common ground verified between supply, Arduino and radio
- [ ] `GLOVE_RX_ANGLE_MIN/MAX` verified against the real mechanism
- [ ] Failsafe tested by switching the glove off, with the arm loaded as it
      will be during the demo
- [ ] Packet loss measured (`s`) in the actual room, at the actual distance:
      above a few percent, fix the RF before the audience arrives
- [ ] Spare batteries for the glove, charged
- [ ] A physical way to cut servo power within reach of the operator
- [ ] Preset poses rehearsed — a button press moves the arm without the
      operator's hand being in the way
- [ ] Nobody's fingers inside the mechanism's reach while it is powered
- [ ] A sentence prepared for "what happens if it loses signal?" — the answer
      should be "it goes to neutral within half a second, and here is how I
      tested that"
