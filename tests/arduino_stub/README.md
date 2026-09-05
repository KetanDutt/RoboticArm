# Arduino API stubs (host build only)

These headers are **not** the real Arduino core or the real third-party
libraries. They are minimal, signature-faithful stand-ins that let the two
sketches be compiled and linked on a desktop with `g++`, so that typos,
missing prototypes, wrong types and dead configuration branches are caught
before anyone touches an AVR toolchain.

They exist for `make -C tests sketch-check` only. A real firmware build uses
arduino-cli / the Arduino IDE with the genuine libraries -- see
`tools/build.sh` and `.github/workflows/ci.yml`.

Signatures were transcribed from the upstream headers:

| stub | mirrors |
| --- | --- |
| `Arduino.h` | Arduino AVR core 1.8.x (`WProgram.h` API surface used here) |
| `Wire.h` | Arduino AVR core `Wire` (`TwoWire`) |
| `EEPROM.h` | Arduino AVR core `EEPROM` (`get`/`put`/`update`) |
| `VirtualWire.h` | VirtualWire 1.27 (`vw_*` API) |
| `RH_ASK.h` | RadioHead `RH_ASK` / `RHGenericDriver` |
| `ServoTimer2.h` | ServoTimer2 (Michael Margolis / Nick Bontrager) |
| `I2Cdev.h`, `MPU6050.h` | jrowberg/i2cdevlib `MPU6050` |
| `avr/wdt.h` | avr-libc watchdog |
