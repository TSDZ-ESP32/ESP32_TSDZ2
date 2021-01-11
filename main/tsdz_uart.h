/*
 * tsdz_uart.h
 *
 *  Created on: 2 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_UART_H_
#define MAIN_TSDZ_UART_H_

#include "driver/gpio.h"

#define LCD_UART            UART_NUM_1
#define CT_UART             UART_NUM_2

#define CT_TX_PIN           GPIO_NUM_27  // Controller TX and RX2 for ESP32
#define CT_RX_PIN           GPIO_NUM_25  // Controller RX and TX2 for ESP32
#define LCD_TX_PIN          GPIO_NUM_26  // LCD TX and RX1 for ESP32
#define LCD_RX_PIN          GPIO_NUM_33  // LCD RX and TX1 for ESP32

#define CT_MSG_ID           0x43
#define LCD_MSG_ID          0x59

#define LCD_OEM_MSG_BYTES   7
#define CT_OS_MSG_BYTES     29
#define CT_OEM_MSG_BYTES    9
#define LCD_OS_MSG_BYTES    13

void tsdz_uart_init(void);
void tsdz_uart_task(void);

#endif /* MAIN_TSDZ_UART_H_ */
