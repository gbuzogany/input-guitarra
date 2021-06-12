#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for nRF52840-MDK
#define LEDS_NUMBER    3

#define LED_1          NRF_GPIO_PIN_MAP(0,22)
#define LED_2          NRF_GPIO_PIN_MAP(0,23)
#define LED_3          NRF_GPIO_PIN_MAP(0,24)
#define LED_START      LED_1
#define LED_STOP       LED_3

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      22
#define BSP_LED_1      23
#define BSP_LED_2      24

#define BUTTONS_NUMBER 2

#define BUTTON_1       NRF_GPIO_PIN_MAP(1,0)
#define BUTTON_2       NRF_GPIO_PIN_MAP(1,0)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_1

#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           false

#define BSP_QSPI_SCK_PIN   NRF_GPIO_PIN_MAP(1,3)
#define BSP_QSPI_CSN_PIN   NRF_GPIO_PIN_MAP(1,6)
#define BSP_QSPI_IO0_PIN   NRF_GPIO_PIN_MAP(1,5)
#define BSP_QSPI_IO1_PIN   NRF_GPIO_PIN_MAP(1,4)
#define BSP_QSPI_IO2_PIN   NRF_GPIO_PIN_MAP(1,2)
#define BSP_QSPI_IO3_PIN   NRF_GPIO_PIN_MAP(1,1)


#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
