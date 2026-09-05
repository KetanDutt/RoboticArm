/* Host stub of the Arduino Wire (TWI/I2C) library. */
#ifndef TWO_WIRE_H
#define TWO_WIRE_H

#include <Arduino.h>

class TwoWire {
public:
    void begin();
    void begin(uint8_t address);
    void begin(int address);
    void end();
    void setClock(uint32_t clock);
    void beginTransmission(uint8_t address);
    uint8_t endTransmission();
    uint8_t endTransmission(uint8_t sendStop);
    uint8_t requestFrom(uint8_t address, uint8_t quantity);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
    size_t write(uint8_t data);
    size_t write(const uint8_t *data, size_t quantity);
    int available();
    int read();
};

extern TwoWire Wire;

#endif /* TWO_WIRE_H */
