/* Host stub of avr-libc's watchdog header. */
#ifndef AVR_WDT_H
#define AVR_WDT_H

#include <stdint.h>

#define WDTO_15MS 0
#define WDTO_30MS 1
#define WDTO_60MS 2
#define WDTO_120MS 3
#define WDTO_250MS 4
#define WDTO_500MS 5
#define WDTO_1S 6
#define WDTO_2S 7
#define WDTO_4S 8
#define WDTO_8S 9

#define WDRF 3

extern volatile uint8_t MCUSR;

#ifdef __cplusplus
extern "C" {
#endif
void wdt_reset(void);
void wdt_enable(uint8_t timeout);
void wdt_disable(void);
#ifdef __cplusplus
}
#endif

#endif /* AVR_WDT_H */
