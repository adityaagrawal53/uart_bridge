#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <MSP432P4xx/gpio.h>


// BOLT
// SPI pins
#define BOLT_SCK_PORT     GPIO_PORT_P1
#define BOLT_SCK_PIN      GPIO_PIN5
#define BOLT_MOSI_PORT    GPIO_PORT_P1
#define BOLT_MOSI_PIN     GPIO_PIN6
#define BOLT_MISO_PORT    GPIO_PORT_P1
#define BOLT_MISO_PIN     GPIO_PIN7


// communication pins
#define BOLT_IND_PORT     GPIO_PORT_P4
#define BOLT_IND_PIN      GPIO_PIN4
#define BOLT_MODE_PORT    GPIO_PORT_P4
#define BOLT_MODE_PIN     GPIO_PIN5
#define BOLT_REQ_PORT     GPIO_PORT_P4
#define BOLT_REQ_PIN      GPIO_PIN6
#define BOLT_ACK_PORT     GPIO_PORT_P4
#define BOLT_ACK_PIN      GPIO_PIN7

// UART
#define UART_TX_PORT      GPIO_PORT_P1
#define UART_TX_PIN       GPIO_PIN3
#define UART_RX_PORT      GPIO_PORT_P1
#define UART_RX_PIN       GPIO_PIN2

#endif 
