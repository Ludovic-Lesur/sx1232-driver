/*
 * sx1232_hw.c
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#include "sx1232_hw.h"

#ifndef SX1232_DRIVER_DISABLE_FLAGS_FILE
#include "sx1232_driver_flags.h"
#endif
#include "sx1232.h"
#include "types.h"

#ifndef SX1232_DRIVER_DISABLE

/*** SX1232 HW functions ***/

/*******************************************************************/
SX1232_status_t __attribute__((weak)) SX1232_HW_init(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SX1232_status_t __attribute__((weak)) SX1232_HW_de_init(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SX1232_status_t __attribute__((weak)) SX1232_HW_spi_write_read_16(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    /* To be implemented */
    UNUSED(tx_data);
    UNUSED(rx_data);
    UNUSED(transfer_size);
    return status;
}

/*******************************************************************/
void __attribute__((weak)) SX1232_HW_spi_write_16(uint16_t tx_data) {
    /* To be implemented */
    UNUSED(tx_data);
}

/*******************************************************************/
SX1232_status_t __attribute__((weak)) SX1232_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#endif /* SX1232_DRIVER_DISABLE */
