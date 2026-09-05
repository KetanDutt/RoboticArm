/* Host stub of the Arduino AVR EEPROM library (get/put/update included). */
#ifndef EEPROM_H
#define EEPROM_H

#include <Arduino.h>

class EEPROMClass {
public:
    uint8_t read(int index);
    void write(int index, uint8_t value);
    void update(int index, uint8_t value);
    uint16_t length();

    template <typename T> T &get(int index, T &value) {
        (void)index;
        (void)value;
        return value;
    }
    template <typename T> const T &put(int index, const T &value) {
        (void)index;
        (void)value;
        return value;
    }
};

extern EEPROMClass EEPROM;

#endif /* EEPROM_H */
