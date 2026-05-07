#include <msp432p401r.h>
#include "../include/pi_UART.h"
#include <MSP432P4xx/cs.h>

int main(void) {
    /* Stop watchdog */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /* Configure DCO / SMCLK to 12 MHz (same as original project) */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initialize UART (pi_uart provides configuration) */
    pi_uart_init(1); 

    /* Enable interrupts in case peripherals rely on them */
    __enable_irq();

    /* Main loop: just poll UART and echo bytes (pi_uart_poll handles the echo) */
    while (1) {
        pi_uart_poll(0);
    }

    return 0;
}
