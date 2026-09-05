/* Host stub of the I2Cdev device library (jrowberg/i2cdevlib). */
#ifndef I2CDEV_H
#define I2CDEV_H

#include <Arduino.h>

class I2Cdev {
public:
    static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                          uint8_t *data, uint16_t timeout = 0);
    static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                           uint8_t length, uint8_t *data, uint16_t timeout = 0);
    static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data,
                           uint16_t timeout = 0);
    static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                            uint8_t *data, uint16_t timeout = 0);
    static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data,
                           uint16_t timeout = 0);
    static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                         uint8_t data);
    static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
    static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                           uint8_t *data);
};

#endif /* I2CDEV_H */
