/* Host stub of the Arduino AVR core API. See tests/arduino_stub/README.md. */
#ifndef ARDUINO_H
#define ARDUINO_H

#define ARDUINO 10819
#define ARDUINO_ARCH_AVR 1
#define WIRE_HAS_SET_CLOCK 1

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#define LOW 0x0
#define HIGH 0x1
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define LSBFIRST 0
#define MSBFIRST 1
#define LED_BUILTIN 13
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#ifndef _BV
#define _BV(b) (1UL << (b))
#endif

/* The AVR core declares the analog pin numbers as constants, not macros. */
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;
static const uint8_t A6 = 20;
static const uint8_t A7 = 21;
static const uint8_t SS = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK = 13;
static const uint8_t SDA = 18;
static const uint8_t SCL = 19;

/* PROGMEM and friends are no-ops on the host. */
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define F(string_literal) (string_literal)

#ifdef __cplusplus
extern "C" {
#endif

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
void analogWrite(uint8_t pin, int value);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder,
              uint8_t val);
void yield(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

#ifdef __cplusplus
}
#endif

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define sq(x) ((x) * (x))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

#ifdef __cplusplus

/* Minimal Serial stand-in: every print/println overload resolves so that
 * formatting calls in the sketches are type-checked. */
class HardwareSerial {
public:
    void begin(unsigned long baud);
    void begin(unsigned long baud, uint8_t config);
    void end();
    int available();
    int read();
    int peek();
    void flush();
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);
    operator bool() const;

    void print(const char *s);
    void print(char c);
    void print(unsigned char v, int base = DEC);
    void print(int v, int base = DEC);
    void print(unsigned int v, int base = DEC);
    void print(long v, int base = DEC);
    void print(unsigned long v, int base = DEC);
    void print(double v, int digits = 2);

    void println();
    void println(const char *s);
    void println(char c);
    void println(unsigned char v, int base = DEC);
    void println(int v, int base = DEC);
    void println(unsigned int v, int base = DEC);
    void println(long v, int base = DEC);
    void println(unsigned long v, int base = DEC);
    void println(double v, int digits = 2);
};

extern HardwareSerial Serial;

#endif /* __cplusplus */

#endif /* ARDUINO_H */
