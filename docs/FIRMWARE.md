# Firmware

Building, flashing, configuring and talking to both boards.

- [1. Prerequisites](#1-prerequisites)
- [2. Building and flashing](#2-building-and-flashing)
- [3. The sync rule](#3-the-sync-rule)
- [4. Boot sequence](#4-boot-sequence)
- [5. Configuration reference](#5-configuration-reference)
- [6. Serial CLI — glove](#6-serial-cli--glove)
- [7. Serial CLI — arm](#7-serial-cli--arm)
- [8. Telemetry formats](#8-telemetry-formats)
- [9. Building variants](#9-building-variants)
- [10. Resource budget](#10-resource-budget)
- [11. Tools](#11-tools)

## 1. Prerequisites

**Libraries** (Sketch → Include Library → Manage Libraries, or git clone into
`~/Arduino/libraries`):

| Library | Used by | Where to get it | In Library Manager? |
| --- | --- | --- | --- |
| VirtualWire | both (default radio driver) | `git clone https://github.com/latchdevel/VirtualWire` | **No** |
| RadioHead | both (optional driver) | Library Manager → "RadioHead" | Yes |
| ServoTimer2 | arm | `git clone https://github.com/nabontra/ServoTimer2` | **No** |
| I2Cdev | glove | `git clone https://github.com/jrowberg/i2cdevlib`, then copy `Arduino/I2Cdev` | No |
| MPU6050 | glove | from the same repo: `Arduino/MPU6050` | No |

`tools/build.sh` clones all of these for you into a directory of your choosing.

**Toolchain**: Arduino IDE 1.8.x/2.x, or
[arduino-cli](https://arduino.github.io/arduino-cli/).

## 2. Building and flashing

### With the Arduino IDE

1. Open `hand_transmit/hand_transmit.ino` → Board: *Arduino Uno* → upload to
   the glove.
2. Open `hand_receive/hand_receive.ino` → upload to the arm.

The sketches are self-contained: everything they include is in their own
folder, so no `.ino`-splitting or extra folders are needed.

### With the script (recommended)

```bash
./tools/build.sh                          # build both sketches for Uno
FQBN=arduino:avr:mega ./tools/build.sh    # build both for Mega 2560
./tools/build.sh --upload                 # build and flash (first board found)
ARDUINO_LIBS=~/Arduino/libraries ./tools/build.sh
```

The script installs the AVR core if needed, clones any missing library into
`$ARDUINO_LIBS` (default `./arduino-libraries`, which is git-ignored), builds
both sketches and prints the flash/RAM report:

```
Sketch uses 18244 bytes (56%) of program storage space.
Global variables use 1123 bytes (54%) of dynamic memory, leaving 925 bytes.
```

Read those numbers. On an Uno you have 32 KB of flash and **2 KB of RAM**, and
the RAM figure is the one that bites.

### Verifying without any hardware

```bash
make -C tests check
```

Compiles both sketches against host stubs, links them, and runs
`setup()` + 20 000 × `loop()`. It will not catch AVR-specific problems
(flash size, interrupt timing), but it catches everything else, in seconds.

## 3. The sync rule

**`common/` is the only place to edit shared code.** The copies inside
`hand_transmit/` and `hand_receive/` are generated, because the Arduino build
cannot see outside a sketch folder.

```bash
# after editing anything in common/
python3 tools/sync_common.py          # copy into both sketch folders
python3 tools/sync_common.py --check  # what CI runs: fail if they differ
```

Every generated file carries a banner saying so. If you find yourself editing
`hand_receive/GloveConfig.h`, stop — the edit will be overwritten, and the two
boards will silently disagree.

## 4. Boot sequence

**Glove**

1. Watchdog: clear `WDRF` in `MCUSR`, `wdt_disable()`. This ordering matters —
   after a watchdog reset the WDT stays armed through the bootloader, and a
   board that takes > 2 s to boot can end up in a reset loop.
2. `Serial.begin(115200)`, wait up to 1.5 s for a USB-CDC port to enumerate.
3. Button/LED/mux-address pins configured.
4. Calibration loaded from EEPROM (CRC verified; defaults on failure).
5. Filters seeded from a real ADC read rather than zero.
6. IMU: `Wire.setClock(400 kHz)`, `initialize()`, `testConnection()` with up to
   `GLOVE_IMU_RETRY_COUNT` retries; on success configure DLPF/ranges and capture
   the zero bias (glove must be still and flat). On failure the glove continues
   with `IMU_OK` clear and the wrist channels held neutral.
7. Radio: `rflink::begin()` (+ `startRx()` on the arm).
8. Banner printed, watchdog re-armed at `GLOVE_WATCHDOG_TIMEOUT`.

**Arm** — the same, plus:

1. Servos attached and driven to `GLOVE_RX_NEUTRAL_POSE` at the slew-limited
   rate for `GLOVE_RX_STARTUP_POSE_MS` (1.5 s). ServoTimer2 starts a freshly
   attached channel at 1500 µs, so the firmware writes the neutral pulse and
   `snap()`s the rate limiters immediately — otherwise software and hardware
   disagree about where the arm is, and the first real command produces a jump.
2. Mode `WAITING` until the first valid frame arrives.

Banner text:

```
=== RoboticArm data glove (transmitter) ===
protocol v2, radio VirtualWire @ 2000 bps
type 'h' for the command list
```

```
=== RoboticArm receiver ===
protocol v2, radio VirtualWire @ 2000 bps, 7 servos
failsafe after 500 ms -> return to NEUTRAL
type 'h' for the command list
```

If the two banners show different protocol versions, the boards will not talk —
that is the version field doing its job.

## 5. Configuration reference

All of it lives in `common/GloveConfig.h`. **Every knob is wrapped in
`#if !defined(...)`, so any of them can be overridden from the command line**
(`-DGLOVE_X=…`) for experiments and for the CI configuration matrix, without
editing the file. Values marked *derived* are computed from other settings and
cannot be overridden.

### 5.1 Board profile and sensors

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_PROFILE` | `GLOVE_PROFILE_UNO_4FLEX` | Board profile: `1` Uno/Nano 4 flex + IMU, `2` Uno/Nano 5 flex via 74HC4051 + IMU, `3` Mega 5 flex + IMU. An unknown value is a compile error |
| `GLOVE_FINGER_CHANNELS` | 5 | Fingers the protocol carries (CH2–CH6). Fixed by the protocol |
| `GLOVE_FLEX_COUNT` | *derived* (4 or 5) | Flex sensors physically present |
| `GLOVE_FLEX_PINS` | *derived* | ADC pins for the direct profiles |
| `GLOVE_FLEX_MUX_ENABLE` | *derived* | 1 for the mux profile |
| `GLOVE_FLEX_MUX_SIGNAL_PIN` | `A0` | Mux common (Z, pin 3) → ADC |
| `GLOVE_FLEX_MUX_ADDR_PINS` | `{4, 5, 6}` | Mux S0/S1/S2 (pins 11/10/9) |
| `GLOVE_FLEX_MUX_CHANNELS` | `{0,1,2,3,4}` | Which mux input each finger uses |
| `GLOVE_FLEX_MUX_SETTLE_US` | 100 | Wait after switching the mux. 200–300 for a CD4051 |
| `GLOVE_FLEX_ABSENT_ANGLE` | 90 | Angle sent for a finger with no sensor |
| `GLOVE_IMU_ENABLE` | *derived* from the profile | Set 0 to build a glove with no MPU-6050 at all: no I²C code, wrist held neutral, `IMU_OK` never set |

### 5.2 Serial and telemetry

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_SERIAL_BAUD` | `115200L` | Both boards |
| `GLOVE_TELEMETRY_ENABLE` | 1 | Banner, diagnostics, CLI help |
| `GLOVE_TX_TELEMETRY_MS` | 500 | Glove periodic line interval |
| `GLOVE_RX_TELEMETRY_MS` | 1000 | Arm periodic line interval |
| `GLOVE_TX_SERIAL_CLI` | 1 | Glove accepts serial commands |
| `GLOVE_TX_TELEMETRY_CSV` | 0 | 1 = CSV for `tools/glove_monitor.py` |
| `GLOVE_RX_TELEMETRY_CSV` | 0 | 1 = CSV for `tools/glove_monitor.py` |
| `GLOVE_TX_HAS_OPERATOR_INPUT` | *derived* | 1 if the CLI or the button is enabled; when 0 all operator-input code is compiled out |

### 5.3 Radio (must match on both boards)

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_RF_DRIVER` | `GLOVE_RF_DRIVER_VW` | `1` VirtualWire, `2` RadioHead `RH_ASK`. Wire compatible with each other |
| `GLOVE_RF_SPEED_BPS` | 2000 | 4000 halves the air time and doubles the control rate; costs range |
| `GLOVE_RF_TX_PIN` | 12 | Glove: module DATA in |
| `GLOVE_RF_RX_PIN` | 11 | Arm: module DATA out |
| `GLOVE_RF_PTT_PIN` | 10 | Transmit-enable pin, if your module has one |
| `GLOVE_RF_PTT_INVERTED` | 1 | PTT polarity |
| `GLOVE_RX_BUFFER_LEN` | 24 | Arm's receive buffer. Must be ≥ `GLOVE_FRAME_LEN`; the old firmware used 2 and over-read it |
| `GLOVE_TX_CMD_REPEAT_MS` | 400 | How long a button command is repeated. Must cover ≥ 2 air times (compile-time checked) |
| `GLOVE_AIRTIME_MS` | *derived* | `(9×12 + 84) × 1000 / speed` = 96 ms at 2000 bps |

### 5.4 Glove timing and operator input

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_TX_SEND_MS` | 20 | Minimum interval between transmit attempts; the radio, not this, sets the real rate |
| `GLOVE_TX_SAMPLE_MS` | 5 | Flex sampling interval. Must be ≤ `GLOVE_TX_SEND_MS` |
| `GLOVE_TX_IMU_SAMPLE_MS` | 10 | IMU sampling interval |
| `GLOVE_TX_BUTTON_PIN` | 2 | 0 disables the button |
| `GLOVE_TX_BUTTON_DEBOUNCE_MS` | 20 | |
| `GLOVE_TX_BUTTON_LONG_MS` | 1500 | Long press starts/stops calibration capture |
| `GLOVE_TX_LED_PIN` | 13 | 0 disables the status LED |

### 5.5 Signal conditioning

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_EMA_ALPHA_Q8` | 40 | Q8 EMA weight: 40/256 ≈ 0.156. Lower = smoother, laggier. Must be 1–255 |
| `GLOVE_MEDIAN_ENABLE` | 1 | Median-of-3 before the EMA; kills single-sample spikes |
| `GLOVE_DEADBAND_DEG` | 2 | A channel is only re-sent when it moves more than this. Removes servo buzz without breaking the heartbeat |

### 5.6 Wrist geometry and IMU

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_WRIST_PITCH_MIN_DEG` / `MAX` | −55 / +55 | Physical wrist range used for calibration and mapping |
| `GLOVE_WRIST_ROLL_MIN_DEG` / `MAX` | −70 / +70 | |
| `GLOVE_WRIST_INVERT_MASK` | `0x00` | Bit per wrist axis to flip |
| `GLOVE_IMU_COMP_ALPHA_PCT` | 97 | Complementary filter: % gyro, % accelerometer. 97/3 is a good default; raise it if the wrist feels laggy, lower it if it drifts |
| `GLOVE_IMU_ACCEL_FS_G` | 2 | Accelerometer full scale. Changing it changes `GLOVE_ACCEL_LSB_PER_G` |
| `GLOVE_IMU_GYRO_FS_DPS` | 250 | Gyro full scale (131 LSB per °/s). If you change it you must also change `GYRO_LSB_PER_DPS_Q8` in `common/Attitude.cpp` |
| `GLOVE_IMU_DLPF_HZ` | 20 | MPU-6050 digital low-pass filter |
| `GLOVE_IMU_CAL_SAMPLES` | 400 | Samples averaged for the zero-bias capture (~2 s) |
| `GLOVE_IMU_RETRY_COUNT` | 5 | `testConnection()` retries |
| `GLOVE_IMU_RETRY_DELAY_MS` | 200 | |
| `GLOVE_IMU_I2C_HZ` | 400000 | Drop to 100000 for long or noisy I²C wiring |

### 5.7 Calibration storage

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_CAL_EEPROM_ADDR` | 0 | Start address of the blob (`sizeof(CalibrationData)` ≤ 128 B) |
| `GLOVE_CAL_CAPTURE_MS` | 8000 | Capture window; move every finger through its full range |
| `GLOVE_CAL_MIN_SPAN` | 40 | Minimum ADC span for a channel to be accepted |
| `GLOVE_FLEX_DEFAULT_RAW_MIN` / `MAX` | 90 / 220 | Factory defaults, from the original hardware |
| `GLOVE_FLEX_INVERT_MASK` | `0x1F` | Bit per finger: 1 = reading falls when bent (the standard 10 kΩ pull-down divider) |
| `GLOVE_ANGLE_FLOOR` / `CEILING` | 0 / 180 | Hard output limits for every conversion |

### 5.8 Arm: servos and mechanical limits

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_RX_SERVO_COUNT` | 7 | ≤ 8 (ServoTimer2) and ≤ `GLOVE_CHANNEL_COUNT`, both compile-time checked |
| `GLOVE_RX_SERVO_PINS` | `{3, 9, 4, 5, 6, 7, 8}` | Pitch, roll, thumb, index, middle, ring, little |
| `GLOVE_RX_PULSE_MIN_US` / `MAX_US` | 750 / 2250 | ServoTimer2's own range. **Microseconds, not degrees** |
| `GLOVE_RX_SERVO_INVERT` | `{0,0,0,0,0,0,0}` | Per-servo direction flip (swaps the pulse ends) |
| `GLOVE_RX_ANGLE_MIN` / `MAX` | 10 / 170 | The mechanical window. Every received angle is clamped into it before anything else happens. Narrow it if your links bind near the ends |
| `GLOVE_RX_SLEW_MAX_DPS` | 300 | Max commanded rate per servo. 300 °/s = a full sweep in 600 ms |
| `GLOVE_RX_UPDATE_MS` | 20 | Servo refresh interval |

### 5.9 Arm: failsafe and poses

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_RX_FAILSAFE_TIMEOUT_MS` | 500 | No valid frame for this long → failsafe. Must be ≥ 3 air times (compile-time checked) |
| `GLOVE_RX_FAILSAFE_ACTION` | `GLOVE_FAILSAFE_NEUTRAL` | `HOLD` (0) freeze at the last pose, `NEUTRAL` (1) slew to neutral, `RELAX` (2) detach the servos — the arm will fall unless it is supported |
| `GLOVE_RX_ALLOW_REMOTE_RELAX` | **0** | Only when 1 will a received `GLOVE_CMD_RELAX` detach the servos. Off by default: a corrupt frame must never be able to let go of the arm |
| `GLOVE_RX_POSE_HOLD_MS` | 2000 | How long a latched preset holds before live control resumes |
| `GLOVE_RX_NEUTRAL_POSE` | all 90 | Neutral/home, also the power-on pose |
| `GLOVE_RX_OPEN_POSE` | `{90,90,30,30,30,30,30}` | Preset 1 |
| `GLOVE_RX_FIST_POSE` | `{90,90,150,150,150,150,150}` | Preset 2 |
| `GLOVE_RX_POINT_POSE` | `{90,90,30,150,150,150,150}` | Preset 4 |
| `GLOVE_RX_STARTUP_POSE_MS` | 1500 | Power-on grace period at the neutral pose |

Presets 1–4 are `OPEN`, `FIST`, `HOME` (= neutral) and `POINT`. **These angles
are placeholders**: they describe a generic hand, not yours. Set them after the
servo horns are splined on — see [CALIBRATION.md](CALIBRATION.md).

### 5.10 Watchdog

| Macro | Default | Meaning |
| --- | --- | --- |
| `GLOVE_WATCHDOG_ENABLE` | 1 | Both boards. Set 0 while debugging with breakpoints |
| `GLOVE_WATCHDOG_TIMEOUT` | `WDTO_2S` | The longest blocking operation in either loop is one IMU read (~2 ms), so the margin is enormous |

## 6. Serial CLI — glove

115200 baud, one command per line. `GLOVE_TX_SERIAL_CLI 0` compiles it out.

| Command | Effect |
| --- | --- |
| `h`, `?` | Help |
| `s` | Status: profile, sensor count, radio driver/speed/frame/air time, IMU state, calibration summary, counters |
| `d` | One dump of raw ADC, filtered value and resulting angle per channel, plus pitch/roll |
| `c` | Start or stop the calibration capture (same as a long button press) |
| `w` | Write the in-RAM calibration to EEPROM, then read it back and verify the CRC |
| `r` | Load factory defaults into RAM (calibration + IMU biases). Does **not** touch EEPROM until `w` |
| `b` | Capture the IMU zero bias. Glove flat and still, or it refuses |
| `i <n>` | Flip the direction of finger channel `n` (0–4) |
| `p <n>` | Send preset `n`: 1 open, 2 fist, 3 home, 4 point |
| `t` | Toggle periodic telemetry |

Button: **short press** cycles the presets; **long press** (1.5 s) starts or
stops the calibration capture.

## 7. Serial CLI — arm

| Command | Effect |
| --- | --- |
| `h`, `?` | Help |
| `s` | Status: mode, link age/quality/good/lost/stale/failsafe counts, rejected frames, target and commanded pulse width per servo, glove flag state |
| `n` | **Park**: go to the neutral pose and stay there until `l` |
| `l` | **Live**: resume teleoperation (also clears a relax latch and re-attaches) |
| `a` | Energise (attach) the servos at the neutral pose |
| `r` | **Relax** (detach) the servos. No holding torque — the arm may fall. `a` to energise again |
| `t` | Toggle periodic telemetry |

There is no remote equivalent of `r` unless you set
`GLOVE_RX_ALLOW_REMOTE_RELAX 1`. That is intentional.

Modes, as reported by `s` and telemetry: `BOOT`, `WAITING` (no frame yet),
`LIVE`, `POSE` (a preset is latched), `PARKED`, `RELAXED`, `FAILSAFE`.

## 8. Telemetry formats

### Glove, human readable (default)

```
ch: 90 90 112 118 96 92 90 | wrist 3/-11 | tx 1042/7 | CALIBRATING
    └─ CH0..CH6 angles ─┘   └ pitch/roll ┘  └ sent/deferred ┘  └ only while capturing
```

`deferred` counts frames that were ready but not sent because the radio was
still busy. A large and growing number means you are asking for more than the
air time allows — see [PERFORMANCE.md](PERFORMANCE.md).

### Arm, human readable (default)

```
LIVE | 96ms q97% | 90 90 112 118 96 92 90 | lost 31 fs 0
     └ age/quality ┘ └ commanded angles ┘   └ loss/failsafe counts
```

`REJECTED n` is appended when frames have been discarded for a bad length,
version or command — that is the line to look for when the two boards disagree.

### CSV (for `tools/glove_monitor.py`)

Set `GLOVE_TX_TELEMETRY_CSV 1` and/or `GLOVE_RX_TELEMETRY_CSV 1`:

```
G,<seq>,<imuOk>,<calibrating>,<ch0>,<ch1>,<ch2>,<ch3>,<ch4>,<ch5>,<ch6>,<pitch>,<roll>,<sent>,<deferred>
R,<ageMs>,<quality%>,<good>,<lost>,<failsafes>,<mode>,<servo0>..<servo6>,<rejected>
```

The monitor tolerates both boards on one log and both formats interleaved. If
you add a column, update `tools/glove_monitor.py` and this section together.

## 9. Building variants

Every `#if` branch in the configuration is compiled by CI, so a branch that
only exists on paper cannot rot:

```bash
./tools/check_configs.sh
```

builds eight variants: default, Uno+mux, Mega 5-flex, RadioHead driver, IMU
present without CSV, no CLI and no peripherals, failsafe HOLD, and a relaxed
4000 bps failsafe configuration.

To build one variant yourself:

```bash
make -C tests sketch-check EXTRA_FLAGS="-DGLOVE_PROFILE=2"
arduino-cli compile --fqbn arduino:avr:uno hand_receive \
    --build-property "compiler.cpp.extra_flags=-DGLOVE_RF_DRIVER=2"
```

Useful experiments: `-DGLOVE_IMU_ENABLE=0` (glove with no IMU),
`-DGLOVE_RX_FAILSAFE_ACTION=0` (hold instead of neutral),
`-DGLOVE_RF_SPEED_BPS=4000` (double the control rate),
`-DGLOVE_TX_TELEMETRY_CSV=1` (machine-readable output).

Configurations that would be *dangerous* rather than merely different are
refused by the compiler — see [PROTOCOL.md](PROTOCOL.md#compile-time-guards).

## 10. Resource budget

**Measure, do not guess.** The exact numbers depend on your core and library
versions, so they are not quoted here — `tools/build.sh` prints them, and CI
keeps the report for every build:

```
Sketch uses 18244 bytes (56%) of program storage space. Maximum is 32256 bytes.
Global variables use 1123 bytes (54%) of dynamic memory, leaving 925 bytes for local variables.
```

What dominates each board, so you know where to look when it grows:

| Board | Biggest flash consumers | Biggest RAM consumers |
| --- | --- | --- |
| glove | MPU6050 + I2Cdev (the DMP tables are *not* included), VirtualWire, `Wire` | Wire's 32-byte I²C buffers, the calibration blob (~100 B), 5 × `AnalogChannel` + `AutoRanger` state |
| arm | ServoTimer2, VirtualWire | 7 × `ServoTimer2` channel state, 7 × `SlewLimiter`, the frame + receive buffer |

The single biggest lever is `GLOVE_IMU_ENABLE=0`, which removes `Wire`, I2Cdev
and MPU6050 from the glove entirely.

Rules that keep the firmware inside an ATmega328P, and that any change should
respect:

- No `String`, no `new`/`malloc`. Every buffer is a fixed-size static.
- Every literal goes through `F()` so it stays in flash.
- No `float` in the sampling, filtering, attitude or servo paths.
- The arm includes no I²C, Wire or MPU6050 headers.
- `sizeof(glove::CalibrationData)` is asserted ≤ 128 bytes so the EEPROM blob
  stays small and its layout stays stable.

If a change pushes RAM past ~1.6 KB on an Uno, it will work today and fail the
day someone adds a feature. Treat that as a bug in the change.

## 11. Tools

| Tool | What it does |
| --- | --- |
| `tools/sync_common.py` | `common/` → both sketch folders. `--check` for CI, `--dry-run` to preview |
| `tools/build.sh` | arduino-cli build/flash with automatic core and library installation |
| `tools/check_configs.sh` | Build the whole configuration matrix against the host stubs |
| `tools/rf_airtime.py` | Air time and end-to-end latency budget for a payload length and speed |
| `tools/glove_monitor.py` | Live serial dashboard, CLI bridge and CSV logger (`pip install pyserial`) |

```bash
python3 tools/glove_monitor.py --port /dev/ttyUSB0 --baud 115200 --log captures/run1.csv
python3 tools/rf_airtime.py --payload 9 --speed 2000 4000 --budget
```
