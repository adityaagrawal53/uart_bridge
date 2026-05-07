#include <msp432p401r.h>
#include <bolt.h>
#include <board_config.h>
#include <MSP432P4xx/gpio.h>
#include <MSP432P4xx/spi.h>

// ----------------------------------------------------------------
// Local helper functions
// ----------------------------------------------------------------

// set GPIO high or low
static void pin_set(uint_fast8_t port, uint_fast16_t pin, uint8_t value) {
    if (value) {
        GPIO_setOutputHighOnPin(port, pin);
    } else {
        GPIO_setOutputLowOnPin(port, pin);
    }
}


// read GPIO value
static uint8_t pin_read(uint_fast8_t port, uint_fast16_t pin) {
    return GPIO_getInputPinValue(port, pin);
}

// SPI byte transfer using polling
// Waits until transmit is ready, sends one byte, then waits and returns the received byte.
static uint8_t spi_txrx(uint8_t tx) {
    while (!EUSCI_B_SPI_getInterruptStatus(
        EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, tx);
 
    while (!EUSCI_B_SPI_getInterruptStatus(
        EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT));
    return EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
}

// ----------------------------------------------------------------
// Public functions
// ----------------------------------------------------------------

// BOLT inialization
uint8_t bolt_init(void) {

    // sed pins according o he schemadics

    // BOLD pins
    // MODE, ells cp if wriding or reading, starts LOW reading
    GPIO_setAsOutputPin(BOLT_MODE_PORT, BOLT_MODE_PIN);
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);

    // REQ, requess from cp, starts LOW no request
    GPIO_setAsOutputPin(BOLT_REQ_PORT, BOLT_REQ_PIN);
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);

    // ACK pin, CP sends acknowledge
    GPIO_setAsInputPin(BOLT_ACK_PORT, BOLT_ACK_PIN);

    // IND pin, indicades dad dada is available 
    GPIO_setAsInputPin(BOLT_IND_PORT, BOLT_IND_PIN);

    // SPI pins
    // Oupud pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        BOLT_SCK_PORT,
        BOLT_SCK_PIN | BOLT_MOSI_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );
    
    // Inpud pins
    GPIO_setAsPeripheralModuleFunctionInputPin(
        BOLT_MISO_PORT,
        BOLT_MISO_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // configure SPI module
    eUSCI_SPI_MasterConfig spi_config = {
        .selectClockSource     = EUSCI_SPI_CLOCKSOURCE_SMCLK,
        .clockSourceFrequency  = 12000000,
        .desiredSpiClock       = 4000000,
        .msbFirst              = EUSCI_SPI_MSB_FIRST,
        .clockPhase            = EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
        .clockPolarity         = EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        .spiMode               = EUSCI_SPI_3PIN
    };
    SPI_initMaster(EUSCI_B0_BASE, &spi_config);
    SPI_enableModule(EUSCI_B0_BASE);

    // return 1 if BOLT is ready and ACK should be 0
    return (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 0);
}

// chekcs IND for if data is available
uint8_t bolt_data_available(void) {
    return pin_read(BOLT_IND_PORT, BOLT_IND_PIN);
}

// APP to CP write 
uint8_t bolt_write(uint8_t* data, uint16_t len) {

    // MODE HIGH
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 1);

    // REQ HIGH, requesting transfer
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 1);

    // queue full
    uint32_t timeout = 100000;
    while (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 0) {
        timeout--;

        if (timeout == 0) {
            pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
            pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);
            return 0; 
        }
    }

    spi_txrx((uint8_t)(len & 0xFF));
    spi_txrx((uint8_t)((len >> 8) & 0xFF));

    uint16_t i;
    for (i = 0; i < len; i++) {
        spi_txrx(data[i]);
    }

    while (EUSCI_B_SPI_isBusy(EUSCI_B0_BASE));
 
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);
 
    return 1;

}

// BOLT READ reading from CP to APP
uint8_t bolt_read(uint8_t* buf, uint8_t* len) {
    uint8_t temp[2 + BOLT_MAX_PAYLOAD];

    // MODE LOW, read
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);

    // REQ HIGH, request data
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 1);

    // wait for ACK to go high
    uint32_t timeout = 100000;
    while (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 0) {
        timeout--;

        if (timeout == 0) {
            pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
            return 0;
        }
    }

    // receive bytes until ACK drops LOW
    uint8_t raw_len = 0;
    while (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 1) {
        // check that bytes dont exceed buffer size
        if (raw_len >= sizeof(temp)) {
            pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
            return 0;
        }
        temp[raw_len++] = spi_txrx(0x00);
    }

    // set REQ LOW
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);

    // check length and structure of received data
    if (raw_len < 2) {
        return 0;
    }

    uint16_t msg_len = ((uint16_t)temp[1] << 8) | temp[0];

    if (msg_len == 0 || msg_len > BOLT_MAX_PAYLOAD) {
        return 0;
    }

    if (raw_len < (uint8_t)(2 + msg_len)) {
        return 0;
    }

    uint8_t i;

    for (i = 0; i < msg_len; i++) {
        buf[i] = temp[2 + i];
    }

    *len = (uint8_t)msg_len;
    return 1;
}
