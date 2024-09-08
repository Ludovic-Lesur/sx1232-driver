/*
 * sx1232_hw.h
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#ifndef __SX1232_HW_H__
#define __SX1232_HW_H__

#ifndef SX1232_DRIVER_DISABLE_FLAGS_FILE
#include "sx1232_driver_flags.h"
#endif
#include "sx1232.h"
#include "types.h"

#ifndef SX1232_DRIVER_DISABLE

/*** SX1232 HW functions ***/

/*!******************************************************************
 * \fn SX1232_status_t SX1232_HW_init(void)
 * \brief Init SX1232 hardware interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_HW_init(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_HW_de_init(void)
 * \brief Release SX1232 hardware interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_HW_de_init(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_HW_spi_write_read_16(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size)
 * \brief Write data to transceiver over SPI interface.
 * \param[in]	tx_data: Byte array to send.
 * \param[in]	transfer_size: Number of bytes to send and receive.
 * \param[out] 	rx_data: Pointer to the received bytes.
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_HW_spi_write_read_16(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size);

/*!******************************************************************
 * \fn void SX1232_HW_spi_write_16(uint16_t tx_data)
 * \brief Optimized SPI single short transfer function.
 * \param[in]  	instance: Peripheral instance to use.
 * \param[in]	tx_data: Short to send.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SX1232_HW_spi_write_16(uint16_t tx_data);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]  	delay_ms: Delay to wait in ms.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_HW_delay_milliseconds(uint32_t delay_ms);

#endif /* SX1232_DRIVER_DISABLE */

#endif /* __SX1232_HW_H__ */
