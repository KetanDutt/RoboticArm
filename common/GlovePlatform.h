/*
 * GlovePlatform.h -- portability shims for the shared RoboticArm code.
 *
 * The headers in `common/` are deliberately free of any Arduino API so that
 * they can be compiled and unit tested on a desktop machine (see `tests/`).
 * This file provides the tiny amount of glue that is needed for that: pin
 * name aliases that expand to the real Arduino symbols when the code is built
 * by the Arduino toolchain, and to plain integers when it is built natively.
 *
 * IMPORTANT: never use the bare `A0`..`A7` names in `common/` or in the
 * sketches' configuration.  On AVR those names are `static const uint8_t`
 * variables, not macros, so defining a macro called `A0` before
 * <Arduino.h>/<pins_arduino.h> is pulled in would rewrite the core's own
 * declaration and break the build.  Always use GLOVE_PIN_Ax instead.
 *
 * Shared file: edit `common/GlovePlatform.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/ARCHITECTURE.md.
 */
#ifndef GLOVE_PLATFORM_H
#define GLOVE_PLATFORM_H

#include <stdint.h>
#include <stddef.h>

#if defined(ARDUINO)
/* Built by the Arduino toolchain: use the core's own analog pin symbols. */
#define GLOVE_PIN_A0 A0
#define GLOVE_PIN_A1 A1
#define GLOVE_PIN_A2 A2
#define GLOVE_PIN_A3 A3
#define GLOVE_PIN_A4 A4
#define GLOVE_PIN_A5 A5
#define GLOVE_PIN_A6 A6
#define GLOVE_PIN_A7 A7
#else
/* Native/host build (unit tests): ATmega328P numbering, where A0 == D14. */
#define GLOVE_PIN_A0 14
#define GLOVE_PIN_A1 15
#define GLOVE_PIN_A2 16
#define GLOVE_PIN_A3 17
#define GLOVE_PIN_A4 18
#define GLOVE_PIN_A5 19
#define GLOVE_PIN_A6 20
#define GLOVE_PIN_A7 21
#endif

#endif /* GLOVE_PLATFORM_H */
