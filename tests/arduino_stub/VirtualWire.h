/* Host stub of VirtualWire 1.27.  Signatures follow the published API:
 * vw_send() takes a non-const buffer, vw_get_message()'s length is in/out and
 * it returns non-zero only when the FCS was good. */
#ifndef VIRTUALWIRE_H
#define VIRTUALWIRE_H

#include <stdint.h>

#define VW_PLATFORM_ARDUINO 1
#define VW_MAX_MESSAGE_LEN 80
#define VW_MAX_PAYLOAD (VW_MAX_MESSAGE_LEN - 3)
#define VW_RX_SAMPLES_PER_BIT 8

#ifdef __cplusplus
extern "C" {
#endif

void vw_set_ptt_pin(uint8_t pin);
void vw_set_tx_pin(uint8_t pin);
void vw_set_rx_pin(uint8_t pin);
void vw_set_rx_inverted(uint8_t inverted);
void vw_set_ptt_inverted(uint8_t inverted);
void vw_setup(uint16_t speed);
void vw_rx_start(void);
void vw_rx_stop(void);
uint8_t vw_tx_active(void);
void vw_wait_tx(void);
void vw_wait_rx(void);
uint8_t vw_wait_rx_max(unsigned long milliseconds);
uint8_t vw_send(uint8_t *buf, uint8_t len);
uint8_t vw_have_message(void);
uint8_t vw_get_message(uint8_t *buf, uint8_t *len);
uint8_t vw_get_rx_good(void);
uint8_t vw_get_rx_bad(void);

#ifdef __cplusplus
}
#endif

#endif /* VIRTUALWIRE_H */
