/*
 * stubs.cpp -- definitions for every symbol declared by the host stub
 * headers, so that a sketch can be compiled AND LINKED on the desktop.
 *
 * Linking matters: it turns "I declared a helper and forgot to define it" and
 * "this library function does not exist" into build errors instead of runtime
 * surprises.  Nothing here simulates hardware behaviour -- see
 * tests/arduino_stub/README.md.
 */
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <VirtualWire.h>
#include <RH_ASK.h>
#include <ServoTimer2.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <avr/wdt.h>

/* ------------------------------------------------------- Arduino core --- */

/* The host clock advances one millisecond per millis() call (and delay() adds
 * to it).  That is enough to make the sketches' millis()-based schedulers run
 * during `make sketch-check`, and it keeps the smoke test deterministic. */
static unsigned long g_fakeMillis = 0;

extern "C" {

unsigned long millis(void) { return ++g_fakeMillis; }
unsigned long micros(void) { return g_fakeMillis * 1000UL; }
void delay(unsigned long ms) { g_fakeMillis += ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(uint8_t pin, uint8_t mode) { (void)pin; (void)mode; }
void digitalWrite(uint8_t pin, uint8_t value) { (void)pin; (void)value; }
int digitalRead(uint8_t pin) { (void)pin; return HIGH; }
int analogRead(uint8_t pin) { (void)pin; return 512; }
void analogReference(uint8_t mode) { (void)mode; }
void analogWrite(uint8_t pin, int value) { (void)pin; (void)value; }
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
    (void)pin; (void)state; (void)timeout; return 0;
}
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder,
              uint8_t val) {
    (void)dataPin; (void)clockPin; (void)bitOrder; (void)val;
}
void yield(void) {}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

} /* extern "C" */

/* ------------------------------------------------------------ Serial --- */

HardwareSerial Serial;

void HardwareSerial::begin(unsigned long baud) { (void)baud; }
void HardwareSerial::begin(unsigned long baud, uint8_t config) {
    (void)baud; (void)config;
}
void HardwareSerial::end() {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
int HardwareSerial::peek() { return -1; }
void HardwareSerial::flush() {}
size_t HardwareSerial::write(uint8_t c) { (void)c; return 1; }
size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
    (void)buffer; return size;
}
HardwareSerial::operator bool() const { return true; }

void HardwareSerial::print(const char *s) { (void)s; }
void HardwareSerial::print(char c) { (void)c; }
void HardwareSerial::print(unsigned char v, int base) { (void)v; (void)base; }
void HardwareSerial::print(int v, int base) { (void)v; (void)base; }
void HardwareSerial::print(unsigned int v, int base) { (void)v; (void)base; }
void HardwareSerial::print(long v, int base) { (void)v; (void)base; }
void HardwareSerial::print(unsigned long v, int base) { (void)v; (void)base; }
void HardwareSerial::print(double v, int digits) { (void)v; (void)digits; }

void HardwareSerial::println() {}
void HardwareSerial::println(const char *s) { (void)s; }
void HardwareSerial::println(char c) { (void)c; }
void HardwareSerial::println(unsigned char v, int base) { (void)v; (void)base; }
void HardwareSerial::println(int v, int base) { (void)v; (void)base; }
void HardwareSerial::println(unsigned int v, int base) { (void)v; (void)base; }
void HardwareSerial::println(long v, int base) { (void)v; (void)base; }
void HardwareSerial::println(unsigned long v, int base) { (void)v; (void)base; }
void HardwareSerial::println(double v, int digits) { (void)v; (void)digits; }

/* -------------------------------------------------------------- Wire --- */

TwoWire Wire;

void TwoWire::begin() {}
void TwoWire::begin(uint8_t address) { (void)address; }
void TwoWire::begin(int address) { (void)address; }
void TwoWire::end() {}
void TwoWire::setClock(uint32_t clock) { (void)clock; }
void TwoWire::beginTransmission(uint8_t address) { (void)address; }
uint8_t TwoWire::endTransmission() { return 0; }
uint8_t TwoWire::endTransmission(uint8_t sendStop) { (void)sendStop; return 0; }
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
    (void)address; return quantity;
}
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
                             uint8_t sendStop) {
    (void)address; (void)sendStop; return quantity;
}
size_t TwoWire::write(uint8_t data) { (void)data; return 1; }
size_t TwoWire::write(const uint8_t *data, size_t quantity) {
    (void)data; return quantity;
}
int TwoWire::available() { return 0; }
int TwoWire::read() { return 0; }

/* ------------------------------------------------------------ EEPROM --- */

EEPROMClass EEPROM;

uint8_t EEPROMClass::read(int index) { (void)index; return 0xFF; }
void EEPROMClass::write(int index, uint8_t value) { (void)index; (void)value; }
void EEPROMClass::update(int index, uint8_t value) { (void)index; (void)value; }
uint16_t EEPROMClass::length() { return 1024; }

/* ------------------------------------------------------- VirtualWire --- */

extern "C" {

void vw_set_ptt_pin(uint8_t pin) { (void)pin; }
void vw_set_tx_pin(uint8_t pin) { (void)pin; }
void vw_set_rx_pin(uint8_t pin) { (void)pin; }
void vw_set_rx_inverted(uint8_t inverted) { (void)inverted; }
void vw_set_ptt_inverted(uint8_t inverted) { (void)inverted; }
void vw_setup(uint16_t speed) { (void)speed; }
void vw_rx_start(void) {}
void vw_rx_stop(void) {}
uint8_t vw_tx_active(void) { return 0; }
void vw_wait_tx(void) {}
void vw_wait_rx(void) {}
uint8_t vw_wait_rx_max(unsigned long milliseconds) { (void)milliseconds; return 0; }
uint8_t vw_send(uint8_t *buf, uint8_t len) { (void)buf; (void)len; return 1; }
uint8_t vw_have_message(void) { return 0; }
uint8_t vw_get_message(uint8_t *buf, uint8_t *len) { (void)buf; (void)len; return 0; }
uint8_t vw_get_rx_good(void) { return 0; }
uint8_t vw_get_rx_bad(void) { return 0; }

} /* extern "C" */

/* -------------------------------------------------------- RadioHead ---- */

RH_ASK::RH_ASK(uint16_t speed, uint8_t rxPin, uint8_t txPin, uint8_t pttPin,
               bool pttInverted) {
    (void)speed; (void)rxPin; (void)txPin; (void)pttPin; (void)pttInverted;
}
bool RH_ASK::init() { return true; }
bool RH_ASK::available() { return false; }
bool RH_ASK::recv(uint8_t *buf, uint8_t *len) { (void)buf; (void)len; return false; }
bool RH_ASK::send(const uint8_t *data, uint8_t len) { (void)data; (void)len; return true; }
bool RH_ASK::waitPacketSent() { return true; }
bool RH_ASK::waitPacketSent(uint16_t timeout) { (void)timeout; return true; }
uint8_t RH_ASK::maxMessageLength() { return RH_ASK_MAX_MESSAGE_LEN; }
uint16_t RH_ASK::rxGood() { return 0; }
uint16_t RH_ASK::rxBad() { return 0; }

/* ------------------------------------------------------- ServoTimer2 --- */

ServoTimer2::ServoTimer2() {}
uint8_t ServoTimer2::attach(int pin) { (void)pin; return 1; }
uint8_t ServoTimer2::attach(int pin, int min, int max) {
    (void)pin; (void)min; (void)max; return 1;
}
void ServoTimer2::detach() {}
void ServoTimer2::write(int pulseWidth) { (void)pulseWidth; }
int ServoTimer2::read() { return DEFAULT_PULSE_WIDTH; }
uint8_t ServoTimer2::attached() { return 1; }

/* ----------------------------------------------------- I2Cdev/MPU6050 --- */

int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                       uint8_t *data, uint16_t timeout) {
    (void)devAddr; (void)regAddr; (void)bitNum; (void)data; (void)timeout;
    return 1;
}
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                        uint8_t length, uint8_t *data, uint16_t timeout) {
    (void)devAddr; (void)regAddr; (void)bitStart; (void)length; (void)data;
    (void)timeout;
    return 1;
}
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data,
                        uint16_t timeout) {
    (void)devAddr; (void)regAddr; (void)data; (void)timeout; return 1;
}
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                         uint8_t *data, uint16_t timeout) {
    (void)devAddr; (void)regAddr; (void)length; (void)data; (void)timeout;
    return static_cast<int8_t>(length);
}
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data,
                        uint16_t timeout) {
    (void)devAddr; (void)regAddr; (void)data; (void)timeout; return 1;
}
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                      uint8_t data) {
    (void)devAddr; (void)regAddr; (void)bitNum; (void)data; return true;
}
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    (void)devAddr; (void)regAddr; (void)data; return true;
}
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    (void)devAddr; (void)regAddr; (void)data; return true;
}
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint8_t *data) {
    (void)devAddr; (void)regAddr; (void)length; (void)data; return true;
}

MPU6050::MPU6050() {}
MPU6050::MPU6050(uint8_t address) { (void)address; }
void MPU6050::initialize() {}
bool MPU6050::testConnection() { return true; }
uint8_t MPU6050::getFullScaleGyroRange() { return MPU6050_GYRO_FS_250; }
void MPU6050::setFullScaleGyroRange(uint8_t range) { (void)range; }
uint8_t MPU6050::getFullScaleAccelRange() { return MPU6050_ACCEL_FS_2; }
void MPU6050::setFullScaleAccelRange(uint8_t range) { (void)range; }
uint8_t MPU6050::getDLPFMode() { return MPU6050_DLPF_BW_20; }
void MPU6050::setDLPFMode(uint8_t bandwidth) { (void)bandwidth; }
uint8_t MPU6050::getRate() { return 0; }
void MPU6050::setRate(uint8_t rate) { (void)rate; }
bool MPU6050::getSleepEnabled() { return false; }
void MPU6050::setSleepEnabled(bool enabled) { (void)enabled; }
void MPU6050::setI2CMasterModeEnabled(bool enabled) { (void)enabled; }
void MPU6050::setClockSource(uint8_t source) { (void)source; }
void MPU6050::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                         int16_t *gy, int16_t *gz) {
    *ax = 0; *ay = 0; *az = 16384; *gx = 0; *gy = 0; *gz = 0;
}
void MPU6050::getMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                         int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my,
                         int16_t *mz) {
    getMotion6(ax, ay, az, gx, gy, gz);
    *mx = 0; *my = 0; *mz = 0;
}
void MPU6050::getAcceleration(int16_t *ax, int16_t *ay, int16_t *az) {
    *ax = 0; *ay = 0; *az = 16384;
}
void MPU6050::getRotation(int16_t *gx, int16_t *gy, int16_t *gz) {
    *gx = 0; *gy = 0; *gz = 0;
}
int16_t MPU6050::getTemperature() { return 0; }

/* ----------------------------------------------------------- watchdog --- */

volatile uint8_t MCUSR = 0;

extern "C" {
void wdt_reset(void) {}
void wdt_enable(uint8_t timeout) { (void)timeout; }
void wdt_disable(void) {}
}
