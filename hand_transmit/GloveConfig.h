/*
 * GloveConfig.h -- single place to configure BOTH halves of the RoboticArm
 *                  glove teleoperation system.
 *
 * Every tunable lives here: pin maps, radio settings, protocol limits, sensor
 * filtering, calibration defaults and the receiver's safety limits.  The two
 * sketches contain no magic numbers of their own; they only read this file.
 *
 * Both ends of the link MUST be built with the same radio speed, the same RF
 * driver and the same protocol version, otherwise they cannot talk to each
 * other.  See docs/PROTOCOL.md and docs/HARDWARE.md.
 *
 * This header is pure configuration (no Arduino API), so it is also compiled
 * by the native unit tests in `tests/`.
 *
 * Shared file: edit `common/GloveConfig.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef GLOVE_CONFIG_H
#define GLOVE_CONFIG_H

#include "GlovePlatform.h"

/* =========================================================================
 * 0. Board profile (transmitter / data glove side)
 * =========================================================================
 * The stock Arduino Uno has only six ADC channels and TWO of them (A4/A5)
 * are hard-wired to the I2C bus that the MPU-6050 needs.  The original
 * firmware put five flex sensors on A0..A4, which silently shorted flex
 * sensor #5 into the I2C SDA line -- see docs/BUGS_FIXED.md (B-07).
 *
 * Pick the profile that matches the hardware you actually built:
 *
 *   GLOVE_PROFILE_UNO_4FLEX   Uno/Nano + MPU-6050 + 4 flex sensors (A0..A3).
 *                             The 5th (thumb) channel is sent as a constant
 *                             neutral angle.  No extra parts required.
 *   GLOVE_PROFILE_UNO_MUX     Uno/Nano + MPU-6050 + 5..8 flex sensors read
 *                             through one 74HC4051/CD4051 analog multiplexer.
 *                             Full 5-finger glove on an Uno.
 *   GLOVE_PROFILE_MEGA_5FLEX  Mega2560 (or any board with >= 8 ADC channels
 *                             and I2C on dedicated pins): 5 flex sensors
 *                             straight on A0..A4.
 */
#define GLOVE_PROFILE_UNO_4FLEX  1
#define GLOVE_PROFILE_UNO_MUX    2
#define GLOVE_PROFILE_MEGA_5FLEX 3

#if !defined(GLOVE_PROFILE)
#define GLOVE_PROFILE GLOVE_PROFILE_UNO_4FLEX
#endif

/* Number of flex channels the protocol carries.  Fixed by the protocol:
 * CH0/CH1 are the two wrist channels, CH2..CH6 are the five fingers. */
#if !defined(GLOVE_FINGER_CHANNELS)
#define GLOVE_FINGER_CHANNELS 5
#endif

#if GLOVE_PROFILE == GLOVE_PROFILE_UNO_4FLEX
  #define GLOVE_FLEX_COUNT     4
  #define GLOVE_FLEX_PINS      { GLOVE_PIN_A0, GLOVE_PIN_A1, GLOVE_PIN_A2, GLOVE_PIN_A3 }
  #define GLOVE_FLEX_MUX_ENABLE 0
  #define GLOVE_PROFILE_IMU_ENABLE 1
#elif GLOVE_PROFILE == GLOVE_PROFILE_UNO_MUX
  #define GLOVE_FLEX_COUNT     5
  #define GLOVE_FLEX_MUX_ENABLE 1
  /* 74HC4051 / CD4051 (DIP-16) wiring -- see docs/HARDWARE.md:
   *   pin 3  (Z, common out)  -> GLOVE_FLEX_MUX_SIGNAL_PIN (A0)
   *   pin 11 (S0/A, LSB)      -> GLOVE_FLEX_MUX_ADDR_PINS[0]
   *   pin 10 (S1/B)           -> GLOVE_FLEX_MUX_ADDR_PINS[1]
   *   pin 9  (S2/C, MSB)      -> GLOVE_FLEX_MUX_ADDR_PINS[2]
   *   pin 6  (INH, active hi) -> GND (always enabled)
   *   pin 7  (VEE) -> GND, pin 8 (VSS) -> GND, pin 16 (VDD) -> 5V
   *   channels X0..X7 = pins 13,14,15,12,1,5,2,4
   * Several hobby sites print a different pinout; check your datasheet. */
  #define GLOVE_FLEX_MUX_SIGNAL_PIN GLOVE_PIN_A0
  #define GLOVE_FLEX_MUX_ADDR_PINS  { 4, 5, 6 }
  /* Mux channel (0..7) used by each finger, in protocol order. */
  #define GLOVE_FLEX_MUX_CHANNELS   { 0, 1, 2, 3, 4 }
  #define GLOVE_PROFILE_IMU_ENABLE 1
#elif GLOVE_PROFILE == GLOVE_PROFILE_MEGA_5FLEX
  #define GLOVE_FLEX_COUNT     5
  #define GLOVE_FLEX_PINS      { GLOVE_PIN_A0, GLOVE_PIN_A1, GLOVE_PIN_A2, GLOVE_PIN_A3, GLOVE_PIN_A4 }
  #define GLOVE_FLEX_MUX_ENABLE 0
  #define GLOVE_PROFILE_IMU_ENABLE 1
#else
  #error "GLOVE_PROFILE is not a known profile (see GloveConfig.h)"
#endif

/* Set GLOVE_IMU_ENABLE to 0 to build a glove with no MPU-6050 at all: the
 * wrist channels are then sent as neutral, the IMU_OK flag stays clear and no
 * I2C code is compiled in.  Useful if the IMU dies mid-project, or if you only
 * care about the fingers. */
#if !defined(GLOVE_IMU_ENABLE)
#define GLOVE_IMU_ENABLE GLOVE_PROFILE_IMU_ENABLE
#endif

/* Angle reported for finger channels that have no physical sensor attached.
 * Channels are filled from the thumb upwards, so on the UNO_4FLEX profile the
 * missing one is the little finger (CH6).  90 == neutral/half open. */
#if !defined(GLOVE_FLEX_ABSENT_ANGLE)
#define GLOVE_FLEX_ABSENT_ANGLE 90
#endif

/* =========================================================================
 * 1. Serial / telemetry (both ends)
 * ========================================================================= */
#if !defined(GLOVE_SERIAL_BAUD)
#define GLOVE_SERIAL_BAUD       115200L
#endif
/* Print a startup banner, sensor diagnostics and the interactive help. */
#if !defined(GLOVE_TELEMETRY_ENABLE)
#define GLOVE_TELEMETRY_ENABLE   1
#endif
/* How often periodic telemetry lines are emitted (ms). */
#if !defined(GLOVE_TX_TELEMETRY_MS)
#define GLOVE_TX_TELEMETRY_MS   500
#endif
#if !defined(GLOVE_RX_TELEMETRY_MS)
#define GLOVE_RX_TELEMETRY_MS   1000
#endif
/* Transmitter: accept single-character/line commands on serial. */
#if !defined(GLOVE_TX_SERIAL_CLI)
#define GLOVE_TX_SERIAL_CLI      1
#endif
/* Telemetry format: 0 = human readable, 1 = CSV (for tools/glove_monitor.py). */
#if !defined(GLOVE_TX_TELEMETRY_CSV)
#define GLOVE_TX_TELEMETRY_CSV   0
#endif
#if !defined(GLOVE_RX_TELEMETRY_CSV)
#define GLOVE_RX_TELEMETRY_CSV   0
#endif
/* =========================================================================
 * 2. Radio link (both ends -- MUST match)
 * =========================================================================
 * Driver selection.  VirtualWire is what this project has always used; it is
 * end-of-life upstream, so RadioHead's RH_ASK driver (wire compatible with
 * VirtualWire) is offered as an opt-in migration path.  Use the SAME driver
 * on both boards. */
#define GLOVE_RF_DRIVER_VW 1
#define GLOVE_RF_DRIVER_RH 2

#if !defined(GLOVE_RF_DRIVER)
#define GLOVE_RF_DRIVER GLOVE_RF_DRIVER_VW
#endif

/* Air data rate in bits per second.  2000 is the value the cheap 433 MHz
 * ASK modules are specified for and is the safe default.  A 16 MHz AVR can
 * usually also do 3000-4000 bps, which halves the air time of a frame; try
 * it only on both ends together and watch the receiver's link-quality
 * telemetry.  See docs/PERFORMANCE.md for the air-time budget. */
#if !defined(GLOVE_RF_SPEED_BPS)
#define GLOVE_RF_SPEED_BPS       2000
#endif
/* Default ASK module wiring (VirtualWire/RH_ASK conventions). */
#if !defined(GLOVE_RF_TX_PIN)
#define GLOVE_RF_TX_PIN         12
#endif
#if !defined(GLOVE_RF_RX_PIN)
#define GLOVE_RF_RX_PIN         11
#endif
#if !defined(GLOVE_RF_PTT_PIN)
#define GLOVE_RF_PTT_PIN        10
#endif
/* Most TX modules need PTT active-high; set to 0 if yours is inverted. */
#if !defined(GLOVE_RF_PTT_INVERTED)
#define GLOVE_RF_PTT_INVERTED   1
#endif

/* A preset triggered by the glove button is repeated in this many
 * milliseconds of frames, so a single button press survives the packet loss
 * that cheap ASK links produce. */
#if !defined(GLOVE_TX_CMD_REPEAT_MS)
#define GLOVE_TX_CMD_REPEAT_MS  400
#endif

/* Receive buffer on the arm.  A frame is GLOVE_FRAME_LEN bytes; the extra
 * room lets an oversized (and therefore rejected) packet be drained safely
 * instead of being truncated into something that might look valid. */
#if !defined(GLOVE_RX_BUFFER_LEN)
#define GLOVE_RX_BUFFER_LEN     24
#endif

/* Refuse to send more often than this even if the radio is idle (ms).
 * Because the air time of one frame is ~90 ms at 2000 bps, this is normally
 * not the limiting factor -- the radio is.  Keep it well below the receiver's
 * failsafe timeout so a still hand never looks like a lost link. */
#if !defined(GLOVE_TX_SEND_MS)
#define GLOVE_TX_SEND_MS             20
#endif
/* =========================================================================
 * 3. Transmitter timing and filtering
 * ========================================================================= */
/* Flex sensor sampling period (ms).  200 Hz. */
#if !defined(GLOVE_TX_SAMPLE_MS)
#define GLOVE_TX_SAMPLE_MS      5
#endif
/* IMU sampling / attitude update period (ms).  100 Hz. */
#if !defined(GLOVE_TX_IMU_SAMPLE_MS)
#define GLOVE_TX_IMU_SAMPLE_MS  10
#endif
/* Button on the glove: 0 = none fitted. */
#if !defined(GLOVE_TX_BUTTON_PIN)
#define GLOVE_TX_BUTTON_PIN      2
#endif
#if !defined(GLOVE_TX_BUTTON_DEBOUNCE_MS)
#define GLOVE_TX_BUTTON_DEBOUNCE_MS 20
#endif
#if !defined(GLOVE_TX_BUTTON_LONG_MS)
#define GLOVE_TX_BUTTON_LONG_MS     1500
#endif
/* Status LED: 0 = none fitted. */
#if !defined(GLOVE_TX_LED_PIN)
#define GLOVE_TX_LED_PIN         13
#endif
/* Exponential moving average weight for the flex channels, in 1/256 units.
 * 40/256 ~= 0.156 -> ~30 ms time constant at 200 Hz.  Lower = smoother but
 * laggier. */
#if !defined(GLOVE_EMA_ALPHA_Q8)
#define GLOVE_EMA_ALPHA_Q8      40
#endif
/* Spike rejection: the median of the last 3 raw samples is fed to the EMA. */
#if !defined(GLOVE_MEDIAN_ENABLE)
#define GLOVE_MEDIAN_ENABLE     1
#endif
/* A channel is only re-sent when it moved by more than this many degrees.
 * NOTE: frames are still transmitted at a fixed rate (the receiver's
 * failsafe needs a heartbeat); the deadband only suppresses servo jitter. */
#if !defined(GLOVE_DEADBAND_DEG)
#define GLOVE_DEADBAND_DEG      2
#endif
/* Settling time after switching a 74HC4051 channel (microseconds). */
#if !defined(GLOVE_FLEX_MUX_SETTLE_US)
#define GLOVE_FLEX_MUX_SETTLE_US 100
#endif

/* Wrist attitude: physical hand angle (deg) that maps onto the 0..180 servo
 * range.  Keep these symmetric around 0 and inside your own wrist comfort
 * zone; anything beyond is clamped, never wrapped. */
#if !defined(GLOVE_WRIST_PITCH_MIN_DEG)
#define GLOVE_WRIST_PITCH_MIN_DEG (-55)
#endif
#if !defined(GLOVE_WRIST_PITCH_MAX_DEG)
#define GLOVE_WRIST_PITCH_MAX_DEG (55)
#endif
#if !defined(GLOVE_WRIST_ROLL_MIN_DEG)
#define GLOVE_WRIST_ROLL_MIN_DEG  (-70)
#endif
#if !defined(GLOVE_WRIST_ROLL_MAX_DEG)
#define GLOVE_WRIST_ROLL_MAX_DEG  (70)
#endif
/* Direction of each wrist axis is controlled by GLOVE_WRIST_INVERT_MASK in
 * section 4 (it is stored in the calibration blob, so it can be changed at
 * runtime from the serial CLI without reflashing). */

/* Complementary filter: percentage of the estimate that comes from
 * integrating the gyroscopes (the rest comes from the accelerometer).
 * 97/3 is a good starting point for a hand. */
#if !defined(GLOVE_IMU_COMP_ALPHA_PCT)
#define GLOVE_IMU_COMP_ALPHA_PCT 97
#endif
/* MPU-6050 configuration.  Full-scale ranges: accel 2/4/8/16 g, gyro
 * 250/500/1000/2000 deg/s.  Narrow ranges == better resolution, which is
 * what a hand gesture needs. */
#if !defined(GLOVE_IMU_ACCEL_FS_G)
#define GLOVE_IMU_ACCEL_FS_G    2
#endif
#if !defined(GLOVE_IMU_GYRO_FS_DPS)
#define GLOVE_IMU_GYRO_FS_DPS   250
#endif
/* Digital low pass filter bandwidth in Hz (MPU-6050 register values:
 * 256,188,98,42,20,10,5).  20 Hz kills most hand vibration. */
#if !defined(GLOVE_IMU_DLPF_HZ)
#define GLOVE_IMU_DLPF_HZ       20
#endif
/* Samples averaged to build the gyro/accel zero bias at power-on.  Keep the
 * glove still on the table while this runs (~2 s). */
#if !defined(GLOVE_IMU_CAL_SAMPLES)
#define GLOVE_IMU_CAL_SAMPLES   400
#endif
/* How hard to try before declaring the IMU missing (the glove keeps working
 * without it -- the wrist channels just stay at neutral). */
#if !defined(GLOVE_IMU_RETRY_COUNT)
#define GLOVE_IMU_RETRY_COUNT   5
#endif
#if !defined(GLOVE_IMU_RETRY_DELAY_MS)
#define GLOVE_IMU_RETRY_DELAY_MS 200
#endif
/* I2C clock.  400 kHz is safe for short glove wiring; drop to 100000 if the
 * bus is flaky or the wires are long. */
#if !defined(GLOVE_IMU_I2C_HZ)
#define GLOVE_IMU_I2C_HZ        400000
#endif

/* =========================================================================
 * 4. Calibration storage (transmitter)
 * ========================================================================= */
/* EEPROM address where the CalibrationData struct is kept.  ATmega328P has
 * 1024 bytes; the struct is ~50.  Only explicit saves write to it, so the
 * 100 k write endurance is never a concern. */
#if !defined(GLOVE_CAL_EEPROM_ADDR)
#define GLOVE_CAL_EEPROM_ADDR   0
#endif
/* Seconds of "move every finger through its full range" sampling when the
 * user starts a calibration capture. */
#if !defined(GLOVE_CAL_CAPTURE_MS)
#define GLOVE_CAL_CAPTURE_MS    8000
#endif
/* A channel whose captured span is smaller than this is treated as broken
 * (sensor not plugged in / not moving) and falls back to the default range. */
#if !defined(GLOVE_CAL_MIN_SPAN)
#define GLOVE_CAL_MIN_SPAN      40
#endif
/* Fallback raw span used before the first real calibration, and for any
 * channel whose capture came out too narrow.  These values are the ones the
 * original hardware of this project produced (45 mm flex sensor as the lower
 * leg of a divider, 10-bit ADC).  Run a real capture -- every sensor and
 * divider is different.  See docs/CALIBRATION.md. */
#if !defined(GLOVE_FLEX_DEFAULT_RAW_MIN)
#define GLOVE_FLEX_DEFAULT_RAW_MIN 90
#endif
#if !defined(GLOVE_FLEX_DEFAULT_RAW_MAX)
#define GLOVE_FLEX_DEFAULT_RAW_MAX 220
#endif
/* Per-channel direction.  Bit n = 1 means finger n's raw ADC value FALLS as
 * the finger bends (the usual arrangement), so raw low == fully bent ==
 * 180 deg.  Bit n = 0 means the raw value RISES when it bends.  Flipping a
 * bit is how you correct a sensor that was mounted the other way round --
 * no re-soldering, and it can also be done from the serial CLI (`i <ch>`). */
#if !defined(GLOVE_FLEX_INVERT_MASK)
#define GLOVE_FLEX_INVERT_MASK  0x1F
#endif
/* Bit 0 = wrist pitch, bit 1 = wrist roll.  Set a bit if that servo turns
 * the opposite way to your mechanism. */
#if !defined(GLOVE_WRIST_INVERT_MASK)
#define GLOVE_WRIST_INVERT_MASK 0x00
#endif

/* =========================================================================
 * 5. Receiver: servos
 * =========================================================================
 * The servo angle window that the whole project speaks in.  Angles are
 * transmitted, filtered and clamped as 0..180; the receiver then narrows that
 * to GLOVE_RX_ANGLE_MIN..MAX for the mechanism and converts to microseconds.
 */
#if !defined(GLOVE_ANGLE_FLOOR)
#define GLOVE_ANGLE_FLOOR       0
#endif
#if !defined(GLOVE_ANGLE_CEILING)
#define GLOVE_ANGLE_CEILING     180
#endif

/* ServoTimer2 is used instead of the stock Servo library on purpose:
 * VirtualWire/RH_ASK own Timer1 and so does Servo -- they cannot coexist.
 * ServoTimer2 runs on Timer2 and drives up to 8 channels.  Consequences to
 * remember (docs/HARDWARE.md):
 *   - analogWrite()/PWM on pins 3 and 11 stops working,
 *   - pins 11 and 12 are the radio, so never put a servo there.
 *
 * NOTE: ServoTimer2::write() takes MICROSECONDS (750..2250), not degrees.
 * The conversion lives in common/ServoDrive.h. */
#if !defined(GLOVE_RX_SERVO_COUNT)
#define GLOVE_RX_SERVO_COUNT    7
#endif
/* Original project wiring, kept so existing builds do not need re-soldering.
 * Order: wrist pitch, wrist roll, thumb, index, middle, ring, little. */
#if !defined(GLOVE_RX_SERVO_PINS)
#define GLOVE_RX_SERVO_PINS     { 3, 9, 4, 5, 6, 7, 8 }
#endif
/* Pulse width range of your servos.  750/2250 is the ServoTimer2 default and
 * suits most 180 deg hobby servos.  Narrow it (e.g. 1000/2000) if your
 * servos hit their internal end stops. */
#if !defined(GLOVE_RX_PULSE_MIN_US)
#define GLOVE_RX_PULSE_MIN_US   750
#endif
#if !defined(GLOVE_RX_PULSE_MAX_US)
#define GLOVE_RX_PULSE_MAX_US   2250
#endif
/* Set to 1 per channel to mirror the direction of that servo. */
#if !defined(GLOVE_RX_SERVO_INVERT)
#define GLOVE_RX_SERVO_INVERT   { 0, 0, 0, 0, 0, 0, 0 }
#endif

/* =========================================================================
 * 6. Receiver: safety limits
 * ========================================================================= */
/* Hard mechanical clamps.  No packet, preset or bug can ever command an
 * angle outside this window; values are clamped before they reach a servo. */
#if !defined(GLOVE_RX_ANGLE_MIN)
#define GLOVE_RX_ANGLE_MIN      10
#endif
#if !defined(GLOVE_RX_ANGLE_MAX)
#define GLOVE_RX_ANGLE_MAX      170
#endif
/* Maximum commanded speed, in degrees per second.  Rate limiting protects
 * the gearbox, the power supply and whatever the arm is holding, and it
 * turns a corrupted packet into a small, slow correction instead of a slam. */
#if !defined(GLOVE_RX_SLEW_MAX_DPS)
#define GLOVE_RX_SLEW_MAX_DPS        300
#endif
/* Servo outputs are refreshed on this period (ms).  ServoTimer2 keeps
 * pulsing in the background between refreshes, so nothing jitters. */
#if !defined(GLOVE_RX_UPDATE_MS)
#define GLOVE_RX_UPDATE_MS           20
#endif
/* No valid frame for this long == link lost.  Must be several times the
 * frame air time (~90 ms at 2000 bps) or a still hand triggers it. */
#if !defined(GLOVE_RX_FAILSAFE_TIMEOUT_MS)
#define GLOVE_RX_FAILSAFE_TIMEOUT_MS 500
#endif
/* What to do on link loss:
 *   GLOVE_FAILSAFE_HOLD    freeze every servo at its last commanded angle
 *   GLOVE_FAILSAFE_NEUTRAL rate-limited move to GLOVE_RX_NEUTRAL_POSE
 *   GLOVE_FAILSAFE_RELAX   detach the servos (no holding torque -- the arm
 *                          will collapse under gravity; only use this on a
 *                          counterweighted or unloaded rig) */
#define GLOVE_FAILSAFE_HOLD      0
#define GLOVE_FAILSAFE_NEUTRAL   1
#define GLOVE_FAILSAFE_RELAX     2
#if !defined(GLOVE_RX_FAILSAFE_ACTION)
#define GLOVE_RX_FAILSAFE_ACTION GLOVE_FAILSAFE_NEUTRAL
#endif
/* Allow a received command to detach the servos remotely.  Off by default:
 * a dropped/garbled frame should never be able to drop the arm. */
#if !defined(GLOVE_RX_ALLOW_REMOTE_RELAX)
#define GLOVE_RX_ALLOW_REMOTE_RELAX 0
#endif
/* =========================================================================
 * 7. Receiver: preset poses
 * =========================================================================
 * Presets are requested by the glove button (or the serial CLI) and are
 * latched for GLOVE_RX_POSE_HOLD_MS, after which the slew limiter blends
 * back to live control.  Tune the angles for YOUR mechanism: they depend on
 * how each servo horn was splined onto its joint during assembly. */
#if !defined(GLOVE_RX_POSE_HOLD_MS)
#define GLOVE_RX_POSE_HOLD_MS        2000
#endif
/*                              pitch roll thumb index middle ring little */
#if !defined(GLOVE_RX_NEUTRAL_POSE)
#define GLOVE_RX_NEUTRAL_POSE  {  90,  90,   90,   90,    90,   90,   90 }
#endif
#if !defined(GLOVE_RX_OPEN_POSE)
#define GLOVE_RX_OPEN_POSE     {  90,  90,   30,   30,    30,   30,   30 }
#endif
#if !defined(GLOVE_RX_FIST_POSE)
#define GLOVE_RX_FIST_POSE     {  90,  90,  150,  150,   150,  150,  150 }
#endif
#if !defined(GLOVE_RX_POINT_POSE)
#define GLOVE_RX_POINT_POSE    {  90,  90,   30,  150,   150,  150,  150 }
#endif

/* Angle every servo is driven to, with rate limiting, during power-on so the
 * arm wakes up gently instead of snapping to a random angle. */
#if !defined(GLOVE_RX_STARTUP_POSE_MS)
#define GLOVE_RX_STARTUP_POSE_MS 1500
#endif

/* =========================================================================
 * 8. Watchdog (both ends)
 * =========================================================================
 * A hung board should recover by itself.  The sketch clears the watchdog
 * reset flag and disables the watchdog before re-arming it, which is what
 * makes this safe with a stock bootloader (the classic WDRF boot-loop trap).
 * Set to 0 to disable while debugging (breakpoints + watchdog do not mix). */
#if !defined(GLOVE_WATCHDOG_ENABLE)
#define GLOVE_WATCHDOG_ENABLE    1
#endif
/* WDTO_2S == 2 second timeout.  Must exceed the longest blocking operation
 * in loop(), which is one IMU read (~2 ms) -- so there is huge margin. */
#if !defined(GLOVE_WATCHDOG_TIMEOUT)
#define GLOVE_WATCHDOG_TIMEOUT  WDTO_2S
#endif

/* =========================================================================
 * 9. Configuration sanity checks (compile time, both boards and host tests)
 * =========================================================================
 * A nonsensical configuration should be refused by the compiler, not
 * discovered by a servo.  Checks that need the frame geometry live in
 * GloveProtocol.h; these are the ones that only involve this file.
 * CI builds with values that violate them to prove they are live. */

#if GLOVE_FLEX_COUNT > GLOVE_FINGER_CHANNELS
#error "GLOVE_FLEX_COUNT cannot exceed GLOVE_FINGER_CHANNELS"
#endif

#if GLOVE_EMA_ALPHA_Q8 < 1 || GLOVE_EMA_ALPHA_Q8 > 255
#error "GLOVE_EMA_ALPHA_Q8 is a Q8 weight and must be 1..255"
#endif

#if GLOVE_IMU_COMP_ALPHA_PCT > 100
#error "GLOVE_IMU_COMP_ALPHA_PCT is a percentage and must be 0..100"
#endif

#if GLOVE_TX_SAMPLE_MS > GLOVE_TX_SEND_MS
#error "GLOVE_TX_SAMPLE_MS should not exceed GLOVE_TX_SEND_MS (stale frames)"
#endif

#if GLOVE_RX_ANGLE_MIN >= GLOVE_RX_ANGLE_MAX
#error "GLOVE_RX_ANGLE_MIN must be below GLOVE_RX_ANGLE_MAX"
#endif

#if GLOVE_RX_ANGLE_MIN < GLOVE_ANGLE_FLOOR || GLOVE_RX_ANGLE_MAX > GLOVE_ANGLE_CEILING
#error "GLOVE_RX_ANGLE_MIN/MAX must fit inside GLOVE_ANGLE_FLOOR..CEILING"
#endif

#if GLOVE_CAL_MIN_SPAN < 1
#error "GLOVE_CAL_MIN_SPAN must be at least 1 ADC count"
#endif

#if GLOVE_RX_POSE_HOLD_MS == 0
#error "GLOVE_RX_POSE_HOLD_MS of 0 would make every preset a no-op"
#endif

#endif /* GLOVE_CONFIG_H */
