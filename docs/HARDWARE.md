# Hardware

Bill of materials, wiring tables and the electrical details that decide
whether this project works. Read the [power](#6-power) and
[pin conflict](#3-the-pin-conflict-that-decides-your-glove) sections even if
you skip everything else.

- [1. Bill of materials](#1-bill-of-materials)
- [2. Board profiles](#2-board-profiles)
- [3. The pin conflict that decides your glove](#3-the-pin-conflict-that-decides-your-glove)
- [4. Wiring — glove](#4-wiring--glove)
- [5. Wiring — arm](#5-wiring--arm)
- [6. Power](#6-power)
- [7. Radio modules](#7-radio-modules)
- [8. MPU-6050 notes](#8-mpu-6050-notes)
- [9. Bring-up checklist](#9-bring-up-checklist)

## 1. Bill of materials

**Glove**

| Qty | Part | Notes |
| --- | --- | --- |
| 1 | Arduino Uno / Nano (ATmega328P, 5 V) | A Nano makes a lighter glove |
| 1 | MPU-6050 breakout | 3-axis gyro + accelerometer, I²C |
| 4–5 | Flex sensor (2.2") | Any bend sensor works; span/direction is calibrated per glove |
| 4–5 | 10 kΩ resistor, 1/4 W | One per flex sensor (divider) |
| 1 | 433 MHz ASK transmitter module | e.g. SYN115 / XD-FST / MX-FS-03V |
| 1 | Tactile button | Wrist/forearm mounted, easy to reach with the other hand |
| 1 | 17 cm wire | Antenna for the TX module |
| — | Perfboard, wire, hot glue, velcro strap, glove | |
| *opt* | 74HC4051 or CD4051 | Only for the 5-flex-on-Uno profile |
| *opt* | LiPo 7.4 V + regulator, or 4× AAA holder | Power |

**Arm**

| Qty | Part | Notes |
| --- | --- | --- |
| 1 | Arduino Uno / Mega | Mega only if you want more than 7 servos |
| 7 | Standard hobby servos (5 V, 180°) | MG996R-class for a hand that lifts things; SG90 for a display model |
| 1 | 433 MHz ASK receiver module | e.g. SYN480R / XD-RF-5V. **Not** a 315 MHz module |
| 1 | 17 cm wire | Antenna for the RX module |
| 1 | 5–6 V supply, ≥ 5 A | See [power](#6-power). A 5 V 10 A bench supply or a 2S LiPo + 5 V BEC |
| 1 | 1000 µF electrolytic + 0.1 µF ceramic | Across the servo rail |
| *opt* | Schottky diode per servo | Back-EMF clamp if you see resets |

## 2. Board profiles

`GLOVE_PROFILE` in `common/GloveConfig.h` selects a complete pin map and
sensor count so one firmware suits several builds:

| Profile | Value | Board | Flex sensors | Where they connect | IMU |
| --- | --- | --- | --- | --- | --- |
| `GLOVE_PROFILE_UNO_4FLEX` | 1 (default) | Uno/Nano | 4 | A0, A1, A2, A3 | I²C on A4/A5 |
| `GLOVE_PROFILE_UNO_MUX` | 2 | Uno/Nano | 5 | 74HC4051 on A0, address lines D4/D5/D6 | I²C on A4/A5 |
| `GLOVE_PROFILE_MEGA_5FLEX` | 3 | Mega 2560 | 5 | A0–A4 | I²C on SDA/SCL (pins 20/21) |

Changing the profile changes the channel count, so the two boards must agree on
it only in the sense that the receiver must have at least as many servos as
channels — the protocol carries a fixed 7 channels regardless.

## 3. The pin conflict that decides your glove

On an Arduino Uno, **A4 is SDA and A5 is SCL**. The MPU-6050 needs them. The
original firmware wired five flex sensors to A0–A4 *and* an MPU-6050 — so the
fifth flex divider was connected across the I²C bus, and the bus was connected
across a resistor divider. Neither worked properly: the IMU would fail to
initialise, and the fifth channel read nonsense.

Three ways out, all supported:

1. **Use four flex sensors on A0–A3** (default profile). The finger with no
   sensor — the **little** finger, since channels are filled from the thumb
   upwards — is transmitted as `GLOVE_FLEX_ABSENT_ANGLE` (90°). Reorder
   `GLOVE_FLEX_PINS` to choose which physical sensor maps to which finger.
2. **Use a 74HC4051 analog multiplexer** (`GLOVE_PROFILE_UNO_MUX`). Five (up to
   eight) dividers share one ADC pin, selected by three digital pins. Cost: one
   extra IC and a slightly longer settle time.
3. **Use a Mega** (`GLOVE_PROFILE_MEGA_5FLEX`). A4/A5 there are ordinary pins;
   I²C is on 20/21.

If you build the 5-flex Uno wiring and see `IMU: NOT FOUND` on the glove's
serial output, this is why.

## 4. Wiring — glove

### 4.1 Default profile (`GLOVE_PROFILE_UNO_4FLEX`)

| Arduino pin | To | Notes |
| --- | --- | --- |
| A0 | flex divider 1 (thumb) | |
| A1 | flex divider 2 (index) | |
| A2 | flex divider 3 (middle) | |
| A3 | flex divider 4 (ring) | |
| A4 (SDA) | MPU-6050 SDA | |
| A5 (SCL) | MPU-6050 SCL | |
| D12 | TX module DATA | |
| D10 | TX module enable / PTT (optional) | only if your module has it |
| D2 | button → GND | internal pull-up, so no external resistor |
| D13 | status LED | built-in LED is used by default |
| 5V | divider top rail, IMU VCC, TX module VCC | |
| GND | everything | |

D11 is the VirtualWire default *receive* pin; the glove never receives, so it
can stay unconnected.

### 4.2 The flex divider

```
 5V ──[ flex sensor ]──┬── analog pin
                       │
                    [ 10 kΩ ]
                       │
                      GND
```

With this topology the reading **falls when the finger bends** (more flex
resistance = less voltage at the divider mid-point). That is what the default
`GLOVE_FLEX_INVERT_MASK 0x1F` assumes, and it matches the 90…220 raw values in
the original firmware.

If you wire it the other way round (10 kΩ on top, flex to ground) the reading
rises when bent; either flip the sensor over in `GLOVE_FLEX_INVERT_MASK` or
just run `i 2` / `i 3` … on the glove's serial CLI and save. Calibration
captures whichever direction you wired.

10 kΩ is a good starting point for a 10 kΩ (straight) / 30 kΩ (bent) flex
sensor, giving roughly 2.5 V…1.25 V of usable swing. If your calibration shows
a span under ~200 counts (`s` on the CLI), try a resistor closer to the
sensor's straight resistance to widen the swing.

### 4.3 Multiplexer profile (`GLOVE_PROFILE_UNO_MUX`)

```
              74HC4051 (DIP-16)
        ┌──────────────────────────────┐
   5V ──┤16 VDD          8 VSS├── GND  │
  GND ──┤ 7 VEE          6 INH├── GND  │   (INH low = channels enabled)
   D4 ──┤11 A (S0)      13 X0├── flex divider 1
   D5 ──┤10 B (S1)      14 X1├── flex divider 2
   D6 ──┤ 9 C (S2)      15 X2├── flex divider 3
   A0 ──┤ 3 Z (COM)     12 X3├── flex divider 4
        │               1 X4├── flex divider 5
        │               5 X5├── spare
        │               2 X6├── spare
        │               4 X7├── spare
        └──────────────────────────────┘
```

Pin numbers follow the Nexperia/TI datasheet for the DIP-16 74HC4051 /
CD4051: `Z`=3, `INH`=6, `VEE`=7, `VSS`=8, `A`=11, `B`=10, `C`=9, `VDD`=16,
`X0..X7`=13,14,15,12,1,5,2,4. Several hobby sites publish a different (wrong)
pinout — **check your datasheet before soldering**. If channels come out
scrambled but consistent, the address-bit order is reversed; fix it by
reordering `GLOVE_FLEX_MUX_ADDR_PINS`.

Notes:

- Tie `VEE` and `INH` to GND. Leaving them floating gives random dropouts.
- A 74HC4051 (HC logic) has ~70 Ω on-resistance and works fine at 5 V. A
  CD4051 (4000 series) has 100–300 Ω and a slower switching time — increase
  `GLOVE_FLEX_MUX_SETTLE_US` to 200–300 if you use one.
- Do **not** put a large capacitor on `Z`: it forms an RC with the mux's
  on-resistance and the next channel's reading will be the previous channel's
  value. ≤ 10 nF, or nothing.
- The 10 kΩ divider resistor stays with each flex sensor; the mux sees the
  mid-point.

### 4.4 MPU-6050

| MPU pin | To |
| --- | --- |
| VCC | 5 V (most breakouts have a regulator and a 3.3 V level shift — check yours) |
| GND | GND |
| SDA | A4 (Uno) / pin 20 (Mega) |
| SCL | A5 (Uno) / pin 21 (Mega) |
| AD0 | GND → address 0x68 (**must match `GLOVE_IMU_I2C_ADDR`**) |
| INT | unused by this firmware |

## 5. Wiring — arm

| Arduino pin | Servo | Joint | Config macro |
| --- | --- | --- | --- |
| D3 | 1 | wrist pitch | `GLOVE_RX_PITCH_SERVO_PIN` |
| D9 | 2 | wrist roll | `GLOVE_RX_ROLL_SERVO_PIN` |
| D4 | 3 | thumb | `GLOVE_RX_FINGER_SERVO_PINS[0]` |
| D5 | 4 | index | `[1]` |
| D6 | 5 | middle | `[2]` |
| D7 | 6 | ring | `[3]` |
| D8 | 7 | little | `[4]` |
| D11 | — | RF RX module DATA | `GLOVE_RX_RADIO_PIN` |
| D10 | — | RF RX enable/PTT (optional) | `GLOVE_RX_PTT_PIN` |
| D13 | — | status LED (link) | built-in |

This pin assignment is inherited from the original firmware, so an existing
build keeps working.

**ServoTimer2 disables `analogWrite()` on pins 3 and 11** (it owns Timer2).
Both are already spoken for here (servo 1 and the radio), so nothing is lost —
but do not add an LED-with-fade or a motor-PWM output on those pins.

ServoTimer2 supports 8 channels. For a 9th servo you need a Mega and the stock
`Servo` library (Timer1 is free there only if you use RadioHead with a
different timer, or an nRF24 module) — see `docs/ROADMAP.md`.

Each servo: signal (orange) → the Arduino pin above, V+ (red) → the external
servo supply, GND (brown) → the supply's ground **and** the Arduino's ground.

## 6. Power

Servos are the thing that breaks this project most often, and it is never the
firmware.

- **Never power seven servos from the Arduino's 5 V pin.** The on-board
  regulator and the USB cable cannot supply the inrush; the MCU browns out and
  resets, which looks exactly like "the arm twitches and then goes limp".
- Budget **1 A per servo** for stall, **2–3 A** for the whole hand moving at
  once. Use a 5 V ≥ 5 A supply (or 6 V if your servos are 6 V rated), or a 2S
  LiPo through a 5 V BEC.
- **Common ground is mandatory.** The Arduino, the servo supply and the RX
  module must share a ground, or the servo signal levels float.
- Put a **1000 µF electrolytic** across the servo supply rails right at the
  servos, plus a **0.1 µF ceramic** per servo if you can. This absorbs the
  brush noise that otherwise resets the MCU and deafens the ASK receiver.
- If the board still resets under load, add a Schottky diode (e.g. 1N5819) in
  series with the Arduino's supply so servo back-EMF cannot pull its rail down,
  and power the Arduino from its barrel jack / VIN rather than from the servo
  rail.
- A brown-out reset is visible on the arm's serial output: the boot banner
  appears again. If you see that while the arm is moving, it is a power
  problem, not a software one.

Glove side: a 9 V battery through the barrel jack, a 2S LiPo through the VIN
pin, or 4× AAA. The glove's current draw is small (the radio TX bursts at
~15 mA), so a small pack lasts a long time.

## 7. Radio modules

| Item | Setting |
| --- | --- |
| Type | 433 MHz ASK (OOK), one-way |
| TX module | SYN115, XD-FST, MX-FS-03V, FS1000A |
| RX module | SYN480R, XD-RF-5V, RXB6 |
| Data rate | 2000 bps (default) or 4000 bps (`GLOVE_RF_SPEED`) |
| Antenna | ~17 cm straight wire on **both** modules (λ/4 at 433 MHz) |
| Payload | 9 bytes = 192 bits on air ≈ 96 ms at 2000 bps |
| Range | 10–30 m indoors with an antenna; the FS1000A without one is ~1 m |

Practical notes:

- **Both modules must be the same frequency.** A 315 MHz pair will happily sit
  there doing nothing.
- A **superheterodyne receiver (SYN480R/RXB6) is dramatically better** than the
  cheap regenerative ones (XD-RF-5V). If you are debugging "the arm misses
  packets", change the receiver before you change the code.
- 4000 bps halves the air time and doubles the control rate to ~20 Hz, but it
  reduces range and noise immunity. Test at 2000 first.
- The ASK receiver picks up **servo brush noise** readily. Keep the RX module
  and its antenna away from the servos and the supply wiring, and use the bulk
  capacitor from [power](#6-power).
- Some receivers expose a **VT/RSSI** pin. This firmware does not read it (the
  link quality in `s` comes from sequence-number gaps instead, which needs no
  extra wiring); adding it is a small, worthwhile change — see
  [ROADMAP.md](ROADMAP.md).
- The link is unacknowledged. `s` on the arm's serial shows the measured
  packet loss; if it is above a few percent, fix the hardware — the failsafe
  assumes the link is mostly good.

## 8. MPU-6050 notes

- The firmware runs the gyro at **±250 °/s** (131 LSB per °/s) and the
  accelerometer at **±2 g** (16384 LSB per g), with the digital low-pass filter
  at `GLOVE_IMU_DLPF_HZ` (20 Hz). Those two ranges are the ones the
  complementary filter's fixed-point constants are built for: if you change
  `GLOVE_IMU_GYRO_FS_DPS` you must also change `GYRO_LSB_PER_DPS_Q8` in
  `common/Attitude.cpp` (65 for ±500 °/s). ±250 °/s is the right choice for a
  wrist anyway — a hand rarely exceeds it, and it doubles the resolution.
- `AD0` high gives address 0x69; change `GLOVE_IMU_I2C_ADDR` to match.
- Many breakouts are 3.3 V devices with a 5 V regulator and level shifters on
  board (that is the common GY-521). Bare modules are not — check before
  connecting to 5 V.
- I²C runs at 400 kHz (`Wire.setClock(400000)`). If your wiring is long or you
  see `IMU: NOT FOUND` intermittently, drop to 100 kHz via `GLOVE_IMU_I2C_HZ`.
- The module must be **physically aligned with the hand** and its orientation
  captured at boot with the glove flat: the filter subtracts whatever it sees
  at rest, so the absolute mounting angle does not matter, but *moving* during
  capture does.

## 9. Bring-up checklist

Do this before connecting the servos. Each step has a pass criterion.

| # | Step | Pass criterion |
| --- | --- | --- |
| 1 | Flash the glove, open serial at `GLOVE_SERIAL_BAUD` (default **115200**) | Boot banner, `PROFILE: UNO_4FLEX  FLEX: 4`, and either `IMU: OK` or `IMU: NOT FOUND` |
| 2 | `s` on the glove | Raw ADC values move when you flex each finger; pitch/roll change when you tilt your wrist |
| 3 | Multimeter on each divider mid-point | ~1.2–3.5 V, changing with flex. Rail-to-rail means a broken sensor or wrong resistor |
| 4 | Flash the arm **with the servos unplugged** | Boot banner, then `ARM READY ... FAILSAFE: HOLD` |
| 5 | Watch the arm's serial | `RX <seq> ... q=<quality>%` lines appear every second, and the decoded angles track the glove |
| 6 | `s` on the arm | `link ok  q=9x%  lost=0  rate=10.4 fps` — a healthy rate is ~10 frames/s at 2000 bps |
| 7 | Turn the glove off | Within 500 ms the arm prints `LINK LOST` and enters failsafe |
| 8 | Power the servo supply, connect **one** servo | It moves to the neutral pose at power-on and follows the glove without buzzing |
| 9 | Connect the remaining servos one at a time, watching the supply current | No brown-out banner on the arm's serial |
| 10 | Calibrate, then test at full speed | `docs/CALIBRATION.md` |

If a step fails, go to [docs/TROUBLESHOOTING.md](TROUBLESHOOTING.md) — the
symptoms are indexed there.
