/* Host stub of RadioHead's RH_ASK driver (used when GLOVE_RF_DRIVER is set to
 * GLOVE_RF_DRIVER_RH). */
#ifndef RH_ASK_H
#define RH_ASK_H

#include <stdint.h>

#define RH_ASK_MAX_PAYLOAD_LEN 67
#define RH_ASK_MAX_MESSAGE_LEN (RH_ASK_MAX_PAYLOAD_LEN - 7)

class RH_ASK {
public:
    RH_ASK(uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12,
           uint8_t pttPin = 10, bool pttInverted = false);
    bool init();
    bool available();
    bool recv(uint8_t *buf, uint8_t *len);
    bool send(const uint8_t *data, uint8_t len);
    bool waitPacketSent();
    bool waitPacketSent(uint16_t timeout);
    uint8_t maxMessageLength();
    uint16_t rxGood();
    uint16_t rxBad();
};

#endif /* RH_ASK_H */
