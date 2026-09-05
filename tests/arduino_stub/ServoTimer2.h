/* Host stub of ServoTimer2 (Michael Margolis, updated by Nick Bontrager).
 * write() takes a pulse width in MICROSECONDS (750..2250), not degrees. */
#ifndef SERVO_TIMER2_H
#define SERVO_TIMER2_H

#include <stdint.h>

#define MIN_PULSE_WIDTH 750
#define MAX_PULSE_WIDTH 2250
#define DEFAULT_PULSE_WIDTH 1500
#define FRAME_SYNC_PERIOD 20000
#define NBR_CHANNELS 8

class ServoTimer2 {
public:
    ServoTimer2();
    uint8_t attach(int pin);
    uint8_t attach(int pin, int min, int max);
    void detach();
    void write(int pulseWidth);
    int read();
    uint8_t attached();
};

#endif /* SERVO_TIMER2_H */
