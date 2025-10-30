/*
 * sx1232.c
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#include "sx1232.h"

#ifndef SX1232_DRIVER_DISABLE_FLAGS_FILE
#include "sx1232_driver_flags.h"
#endif
#include "sx1232_hw.h"
#include "types.h"

#ifndef SX1232_DRIVER_DISABLE

/*** SX1232 local macros ***/

// SPI.
#define SX1232_REGISTER_SPI_TRANSFER_SIZE       2
// RF frequency range.
#define SX1232_RF_FREQUENCY_HZ_MIN              862000000
#define SX1232_RF_FREQUENCY_HZ_MAX              1020000000
// FSK deviation range.
#define SX1232_FSK_DEVIATION_MAX                16383
// Output power ranges.
#define SX1232_OUTPUT_POWER_RFO_MIN             0
#define SX1232_OUTPUT_POWER_RFO_MAX             17
#define SX1232_OUTPUT_POWER_PABOOST_MIN         2
#define SX1232_OUTPUT_POWER_PABOOST_MAX         20
// SX1232 oscillator frequency.
#define SX1232_SYNC_WORD_MAXIMUM_SIZE_BYTES     8
// SX1232 minimum and maximum bit rate.
#define SX1232_BIT_RATE_BPS_MIN                 (SX1232_DRIVER_FXOSC_HZ / ((1 << 16) - 1))
#define SX1232_BIT_RATE_BPS_MAX                 300000
// SX1232 maximum preamble size.
#define SX1232_PREAMBLE_SIZE_BYTES_MAX          3
// SX1232 DIOs.
#define SX1232_DIO_NUMBER                       6
#define SX1232_DIO_MAPPING_MAX_VALUE            4
// SX1232 FIFO.
#define SX1232_FIFO_SIZE_BYTES                  64

/*** SX1232 local structures ***/

/*******************************************************************/
typedef enum {
    SX1232_REGISTER_FIFO = 0x00,
    SX1232_REGISTER_OPMODE = 0x01,
    SX1232_REGISTER_BITRATEMSB = 0x02,
    SX1232_REGISTER_BITRATELSB = 0x03,
    SX1232_REGISTER_FDEVMSB = 0x04,
    SX1232_REGISTER_FDEVLSB = 0x05,
    SX1232_REGISTER_FRFMSB = 0x06,
    SX1232_REGISTER_FRFMID = 0x07,
    SX1232_REGISTER_FRFLSB = 0x08,
    SX1232_REGISTER_PACONFIG = 0x09,
    SX1232_REGISTER_PARAMP = 0x0A,
    SX1232_REGISTER_OCP = 0x0B,
    SX1232_REGISTER_LNA = 0x0C,
    SX1232_REGISTER_RXCONFIG = 0x0D,
    SX1232_REGISTER_RSSICONFIG = 0x0E,
    SX1232_REGISTER_RSSICOLLISION = 0x0F,
    SX1232_REGISTER_RSSITHRESH = 0x10,
    SX1232_REGISTER_RSSIVALUE = 0x11,
    SX1232_REGISTER_RXBW = 0x12,
    SX1232_REGISTER_AFCBW = 0x13,
    SX1232_REGISTER_OOKPEAK = 0x14,
    SX1232_REGISTER_OOKFIX = 0x15,
    SX1232_REGISTER_OOKAVG = 0x16,
    SX1232_REGISTER_AFCFEI = 0x1A,
    SX1232_REGISTER_AFCMSB = 0x1B,
    SX1232_REGISTER_AFCLSB = 0x1C,
    SX1232_REGISTER_FEIMSB = 0x1D,
    SX1232_REGISTER_FEILSB = 0x1E,
    SX1232_REGISTER_PREAMBLEDETECT = 0x1F,
    SX1232_REGISTER_RXTIMEOUT1 = 0x20,
    SX1232_REGISTER_RXTIMEOUT2 = 0x21,
    SX1232_REGISTER_RXTIMEOUT3 = 0x22,
    SX1232_REGISTER_RXDELAY = 0x23,
    SX1232_REGISTER_OSC = 0x24,
    SX1232_REGISTER_PREAMBLEMSB = 0x25,
    SX1232_REGISTER_PREAMBLELSB = 0x26,
    SX1232_REGISTER_SYNCCONFIG = 0x27,
    SX1232_REGISTER_SYNCVALUE1 = 0x28,
    SX1232_REGISTER_SYNCVALUE2 = 0x29,
    SX1232_REGISTER_SYNCVALUE3 = 0x2A,
    SX1232_REGISTER_SYNCVALUE4 = 0x2B,
    SX1232_REGISTER_SYNCVALUE5 = 0x2C,
    SX1232_REGISTER_SYNCVALUE6 = 0x2D,
    SX1232_REGISTER_SYNCVALUE7 = 0x2E,
    SX1232_REGISTER_SYNCVALUE8 = 0x2F,
    SX1232_REGISTER_PACKETCONFIG1 = 0x30,
    SX1232_REGISTER_PACKETCONFIG2 = 0x31,
    SX1232_REGISTER_PAYLOADSIZE = 0x32,
    SX1232_REGISTER_NODEADRS = 0x33,
    SX1232_REGISTER_BROADCASTADRS = 0x34,
    SX1232_REGISTER_FIFOTHRESH = 0x35,
    SX1232_REGISTER_SEQCONFIG1 = 0x36,
    SX1232_REGISTER_SEQCONFIG2 = 0x37,
    SX1232_REGISTER_TIMERRESOL = 0x38,
    SX1232_REGISTER_TIMER1COEF = 0x39,
    SX1232_REGISTER_TIMER2COEF = 0x3A,
    SX1232_REGISTER_IMAGECAL = 0x3B,
    SX1232_REGISTER_TEMP = 0x3C,
    SX1232_REGISTER_LOWBAT = 0x3D,
    SX1232_REGISTER_IRQFLAGS1 = 0x3E,
    SX1232_REGISTER_IRQFLAGS2 = 0x3F,
    SX1232_REGISTER_DIOMAPPING1 = 0x40,
    SX1232_REGISTER_DIOMAPPING2 = 0x41,
    SX1232_REGISTER_VERSION = 0x42,
    SX1232_REGISTER_AGCREF = 0x43,
    SX1232_REGISTER_AGCTHRESH1 = 0x44,
    SX1232_REGISTER_AGCTHRESH2 = 0x45,
    SX1232_REGISTER_AGCTHRESH3 = 0x46,
    SX1232_REGISTER_PLLHOP = 0x4B,
    SX1232_REGISTER_PAVALUE = 0x4C,
    SX1232_REGISTER_TCXO = 0x58,
    SX1232_REGISTER_PADAC = 0x5A,
    SX1232_REGISTER_PLL = 0x5C,
    SX1232_REGISTER_PLLLOWPN = 0x5E,
    SX1232_REGISTER_PAMANUAL = 0x63,
    SX1232_REGISTER_FORMERTEMP = 0x6C,
    SX1232_REGISTER_BITRATEFRAC = 0x70,
    SX1232_REGISTER_LAST
} SX1232_register_t;

/*******************************************************************/
typedef struct {
    SX1232_rf_output_pin_t rf_output_pin;
    uint16_t spi_tx_data;
    uint16_t spi_rx_data;
    uint16_t spi_tx_pa_power_value;
} SX1232_context_t;

/*** SX1232 local global variables ***/

static SX1232_context_t sx1232_ctx;

/*** SX1232 local functions ***/

/*******************************************************************/
static SX1232_status_t _SX1232_write_register(SX1232_register_t register_address, uint8_t value) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Build SPI frame.
    sx1232_ctx.spi_tx_data = ((register_address | 0x80) << 8);
    sx1232_ctx.spi_tx_data |= (value << 0);
    // Write access sequence.
    status = SX1232_HW_spi_write_read_16(&sx1232_ctx.spi_tx_data, &sx1232_ctx.spi_rx_data, 1);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SX1232_status_t _SX1232_read_register(SX1232_register_t register_address, uint8_t* value) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Build SPI frame.
    sx1232_ctx.spi_tx_data = ((register_address & 0x7F) << 8);
    // Read access sequence.
    status = SX1232_HW_spi_write_read_16(&sx1232_ctx.spi_tx_data, &sx1232_ctx.spi_rx_data, 1);
    if (status != SX1232_SUCCESS) goto errors;
    // Update value.
    (*value) = (uint8_t) (sx1232_ctx.spi_rx_data & 0x00FF);
errors:
    return status;
}

/*** SX1232 functions ***/

/*******************************************************************/
SX1232_status_t SX1232_init(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Init context.
    sx1232_ctx.rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
    sx1232_ctx.spi_tx_pa_power_value = ((SX1232_REGISTER_PAVALUE | 0x80) << 8);
    // Init hardware interface.
    status = SX1232_HW_init();
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_de_init(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Release hardware interface.
    status = SX1232_HW_de_init();
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_oscillator(SX1232_oscillator_t oscillator) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Select oscillator.
    switch (oscillator) {
    case SX1232_OSCILLATOR_QUARTZ:
        // Enable quartz input.
        _SX1232_write_register(SX1232_REGISTER_TCXO, 0x09);
        break;
    case SX1232_OSCILLATOR_TCXO:
        // Enable TCXO input.
        _SX1232_write_register(SX1232_REGISTER_TCXO, 0x19);
        break;
    default:
        status = SX1232_ERROR_OSCILLATOR;
        goto errors;
    }
    // Wait TS_OSC = 250us typical.
    status = SX1232_HW_delay_milliseconds(SX1232_OSCILLATOR_DELAY_MS);
    if (status != SX1232_SUCCESS) goto errors;
    // Disable output clock.
    status = _SX1232_write_register(SX1232_REGISTER_OSC, 0x07);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_mode(SX1232_mode_t mode) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Read OP mode register.
    status = _SX1232_read_register(SX1232_REGISTER_OPMODE, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Reset bits 0-2.
    reg_value &= 0xF8;
    // Compute register.
    switch (mode) {
    case SX1232_MODE_SLEEP:
        // Already done by previous reset.
        break;
    case SX1232_MODE_STANDBY:
        // Mode = '001'.
        reg_value |= 0x01;
        break;
    case SX1232_MODE_FSTX:
        // Mode = '010'.
        reg_value |= 0x02;
        break;
    case SX1232_MODE_TX:
        // Mode = '011'.
        reg_value |= 0x03;
        break;
    case SX1232_MODE_FSRX:
        // Mode = '100'.
        reg_value |= 0x04;
        break;
    case SX1232_MODE_RX:
        // Mode = '101'.
        reg_value |= 0x05;
        break;
    default:
        status = SX1232_ERROR_MODE;
        goto errors;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_OPMODE, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_modulation(SX1232_modulation_t modulation, SX1232_modulation_shaping_t modulation_shaping) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Read OP mode register.
    status = _SX1232_read_register(SX1232_REGISTER_OPMODE, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Reset bits 3-6.
    reg_value &= 0x87;
    // Compute modulation.
    switch (modulation) {
    case SX1232_MODULATION_FSK:
        // Already done by previous reset.
        break;
    case SX1232_MODULATION_OOK:
        // Modulation type = '01'.
        reg_value |= 0x20;
        break;
    default:
        status = SX1232_ERROR_MODULATION;
        goto errors;
    }
    // Compute modulation shaping.
    switch (modulation_shaping) {
    case SX1232_MODULATION_SHAPING_NONE:
        // Already done by previous reset.
        break;
    case SX1232_MODULATION_SHAPING_FSK_BT_1:
    case SX1232_MODULATION_SHAPING_OOK_BITRATE:
        // Modulation shaping = '01'.
        reg_value |= 0x08;
        break;
    case SX1232_MODULATION_SHAPING_FSK_BT_05:
    case SX1232_MODULATION_SHAPING_OOK_TWO_BITRATE:
        // Modulation shaping = '10'.
        reg_value |= 0x10;
        break;
    case SX1232_MODULATION_SHAPING_FSK_BT_03:
        // Modulation shaping = '11'.
        reg_value |= 0x18;
        break;
    default:
        status = SX1232_ERROR_MODULATION_SHAPING;
        goto errors;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_OPMODE, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_rf_frequency(uint32_t rf_frequency_hz) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint64_t reg_value = 0;
    // Check frequency range.
    if (rf_frequency_hz > SX1232_RF_FREQUENCY_HZ_MAX) {
        status = SX1232_ERROR_RF_FREQUENCY_OVERFLOW;
        goto errors;
    }
    if (rf_frequency_hz < SX1232_RF_FREQUENCY_HZ_MIN) {
        status = SX1232_ERROR_RF_FREQUENCY_UNDERFLOW;
        goto errors;
    }
    // Compute register.
    reg_value = (((uint64_t) rf_frequency_hz) << 19) / ((uint64_t) SX1232_DRIVER_FXOSC_HZ);
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_FRFMSB, (uint8_t) (reg_value >> 16));
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_FRFMID, (uint8_t) (reg_value >> 8));
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_FRFLSB, (uint8_t) (reg_value >> 0));
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_pll_mode(SX1232_pll_mode_t pll_mode) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_PARAMP, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute register.
    switch (pll_mode) {
    case SX1232_PLL_MODE_NORMAL:
        reg_value |= 0x10;
        break;
    case SX1232_PLL_MODE_LOW_PHASE_NOISE:
        reg_value &= 0xEF;
        break;
    default:
        status = SX1232_ERROR_PLL_MODE;
        goto errors;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_PARAMP, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_fsk_deviation(uint32_t fsk_deviation_hz) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint64_t reg_value = 0;
    // Check parameter.
    if (fsk_deviation_hz > SX1232_FSK_DEVIATION_MAX) {
        status = SX1232_ERROR_FSK_DEVIATION;
        goto errors;
    }
    // Compute register.
    reg_value = (((uint64_t) fsk_deviation_hz) << 19) / ((uint64_t) SX1232_DRIVER_FXOSC_HZ);
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_FDEVMSB, (uint8_t) (reg_value >> 8));
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_FDEVLSB, (uint8_t) (reg_value >> 0));
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_bitrate(uint32_t bit_rate_bps) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint32_t reg_value = 0;
    // Check parameter.
    if (bit_rate_bps > SX1232_BIT_RATE_BPS_MAX) {
        status = SX1232_ERROR_BIT_RATE_OVERFLOW;
        goto errors;
    }
    if (bit_rate_bps < SX1232_BIT_RATE_BPS_MIN) {
        status = SX1232_ERROR_BIT_RATE_UNDERFLOW;
        goto errors;
    }
    // Compute register.
    reg_value = (SX1232_DRIVER_FXOSC_HZ / bit_rate_bps);
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_BITRATEFRAC, 0x00);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_BITRATEMSB, (uint8_t) (reg_value >> 8));
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_BITRATELSB, (uint8_t) (reg_value >> 0));
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_data_mode(SX1232_data_mode_t data_mode) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_PACKETCONFIG2, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Reset bit 6.
    reg_value &= 0xBF;
    // Program data mode.
    switch (data_mode) {
    case SX1232_DATA_MODE_CONTINUOUS:
        // Already done by previous reset.
        break;
    case SX1232_DATA_MODE_PACKET:
        // Data mode = '1'.
        reg_value |= 0x40;
        break;
    default:
        status = SX1232_ERROR_DATA_MODE;
        goto errors;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_PACKETCONFIG2, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_data_size(uint8_t data_size_bytes) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Use fixed size, no CRC computation, do not clear FIFO when CRC fails.
    status = _SX1232_write_register(SX1232_REGISTER_PACKETCONFIG1, 0x08);
    if (status != SX1232_SUCCESS) goto errors;
    // Set data size.
    status = _SX1232_read_register(SX1232_REGISTER_PACKETCONFIG2, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Reset MSB.
    reg_value &= 0xF8;
    // Set data size.
    status = _SX1232_write_register(SX1232_REGISTER_PACKETCONFIG2, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_PAYLOADSIZE, data_size_bytes);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_dio_mapping(SX1232_dio_t dio, SX1232_dio_mapping_t dio_mapping) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_addr = 0;
    uint8_t reg_value = 0;
    uint8_t dio_shift = 0;
    // Check parameters.
    if (dio >= SX1232_DIO_LAST) {
        status = SX1232_ERROR_DIO;
        goto errors;
    }
    if (dio_mapping >= SX1232_DIO_MAPPING_LAST) {
        status = SX1232_ERROR_DIO_MAPPING;
        goto errors;
    }
    // Select register and mask.
    reg_addr = (SX1232_REGISTER_DIOMAPPING1 + (dio >> 2));
    // Read register.
    status = _SX1232_read_register(reg_addr, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute register.
    dio_shift = ((3 - (dio % 4)) << 1);
    reg_value &= (~(0b11 << dio_shift));
    reg_value |= (dio_mapping << dio_shift);
    // Write register.
    status = _SX1232_write_register(reg_addr, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX1232_status_t SX1232_get_irq_flag(SX1232_irq_index_t irq_index, uint8_t* irq_flag) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    uint8_t reg_addr_offset = 0;
    uint8_t irq_bit_offset = 0;
    // Check parameters.
    if (irq_index >= SX1232_IRQ_INDEX_LAST) {
        status = SX1232_ERROR_IRQ_INDEX;
        goto errors;
    }
    if (irq_flag == NULL) {
        status = SX1232_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Get register and bit offsets.
    reg_addr_offset = (irq_index >> 3);
    irq_bit_offset = (irq_index % 8);
    // Read register.
    status = _SX1232_read_register((SX1232_REGISTER_IRQFLAGS2 - reg_addr_offset), &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Read bit.
    (*irq_flag) = (reg_value >> irq_bit_offset) & 0x01;
errors:
    return status;
}

#ifdef SX1232_DRIVER_TX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_start_tx(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Start radio.
    status = SX1232_set_mode(SX1232_MODE_FSTX);
    if (status != SX1232_SUCCESS) goto errors;
    // Wait TS_FS=60us typical.
    status = SX1232_HW_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS);
    if (status != SX1232_SUCCESS) goto errors;
    // TX mode.
    status = SX1232_set_mode(SX1232_MODE_TX);
    if (status != SX1232_SUCCESS) goto errors;
    // Wait TS_TR=120us typical.
    status = SX1232_HW_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_TX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_rf_output_pin(SX1232_rf_output_pin_t rf_output_pin) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_PACONFIG, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute register.
    switch (rf_output_pin) {
    case SX1232_RF_OUTPUT_PIN_RFO:
        // PaSelect = '0'.
        reg_value &= 0x7F;
        sx1232_ctx.rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
        break;
    case SX1232_RF_OUTPUT_PIN_PABOOST:
        // PaSelect = '1'.
        reg_value |= 0x80;
        sx1232_ctx.rf_output_pin = SX1232_RF_OUTPUT_PIN_PABOOST;
        break;
    default:
        status = SX1232_ERROR_RF_OUTPUT_PIN;
        goto errors;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_PACONFIG, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_TX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_rf_output_power(int8_t rf_output_power_dbm) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_PACONFIG, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Reset bits 0-3.
    reg_value &= 0xF0;
    // Compute register.
    switch (sx1232_ctx.rf_output_pin) {
    case SX1232_RF_OUTPUT_PIN_RFO:
        // Check parameter.
        if (rf_output_power_dbm > SX1232_OUTPUT_POWER_RFO_MAX) {
            status = SX1232_ERROR_RF_OUTPUT_POWER_OVERFLOW;
            goto errors;
        }
        if (rf_output_power_dbm < SX1232_OUTPUT_POWER_RFO_MIN) {
            status = SX1232_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
            goto errors;
        }
        // Pout = -1 + OutputPower [dBm].
        reg_value |= (rf_output_power_dbm + 1) & 0x0F;
        break;
    case SX1232_RF_OUTPUT_PIN_PABOOST:
        // Check parameter.
        if (rf_output_power_dbm > SX1232_OUTPUT_POWER_PABOOST_MAX) {
            status = SX1232_ERROR_RF_OUTPUT_POWER_OVERFLOW;
            goto errors;
        }
        if (rf_output_power_dbm < SX1232_OUTPUT_POWER_PABOOST_MIN) {
            status = SX1232_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
            goto errors;
        }
        // Pout = 2 + OutputPower [dBm].
        reg_value |= (rf_output_power_dbm - 2) & 0x0F;
        break;
    default:
        status = SX1232_ERROR_RF_OUTPUT_PIN;
        goto errors;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_PACONFIG, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_TX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_enable_manual_pa_control(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Set required registers.
    status = _SX1232_write_register(SX1232_REGISTER_PLLHOP, 0x7B);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(0x4D, 0x03);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_PAMANUAL, 0x60);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_TX_ENABLE
/*******************************************************************/
void __attribute__((optimize("-O0"))) SX1232_set_pa_power_value(uint8_t pa_power_value) {
    // Build SPI frame.
    sx1232_ctx.spi_tx_pa_power_value &= 0xFF00;
    sx1232_ctx.spi_tx_pa_power_value |= pa_power_value;
    // Write access sequence.
    SX1232_HW_spi_write_16(sx1232_ctx.spi_tx_pa_power_value);
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_start_rx(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Start radio.
    status = SX1232_set_mode(SX1232_MODE_FSRX);
    if (status != SX1232_SUCCESS) goto errors;
    // Wait TS_FS=60us typical.
    status = SX1232_HW_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS);
    if (status != SX1232_SUCCESS) goto errors;
    // TX mode.
    status = SX1232_set_mode(SX1232_MODE_RX);
    if (status != SX1232_SUCCESS) goto errors;
    // Wait TS_TR=120us typical.
    status = SX1232_HW_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_calibrate_image(void) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Go to standby mode.
    status = SX1232_set_mode(SX1232_MODE_STANDBY);
    if (status != SX1232_SUCCESS) goto errors;
    // Set RSSI sampling.
    status = SX1232_set_rssi_sampling(SX1232_RSSI_SAMPLING_256);
    if (status != SX1232_SUCCESS) goto errors;
    // Start calibration.
    status = _SX1232_read_register(SX1232_REGISTER_IMAGECAL, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    reg_value &= 0x7F;
    reg_value |= 0x40;
    status = _SX1232_write_register(SX1232_REGISTER_IMAGECAL, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Wait for calibration to complete.
    status = SX1232_HW_delay_milliseconds(SX1232_IMAGE_CALIBRATION_DELAY_MS);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_rx_bandwidth(SX1232_rxbw_mantissa_t rxbw_mantissa, SX1232_rxbw_exponent_t rxbw_exponent) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameters.
    if (rxbw_mantissa >= SX1232_RXBW_MANTISSA_LAST) {
        status = SX1232_ERROR_RXBW_MANTISSA;
        goto errors;
    }
    if (rxbw_exponent >= SX1232_RXBW_EXPONENT_LAST) {
        status = SX1232_ERROR_RXBW_EXPONENT;
        goto errors;
    }
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_RXBW, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute register.
    reg_value &= 0xE0; // Reset bits 0-4.
    reg_value |= ((rxbw_mantissa & 0x03) << 3) | (rxbw_exponent & 0x07);
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_RXBW, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_lna_configuration(SX1232_lna_mode_t lna_mode, SX1232_lna_gain_attenuation_t lna_gain_attenuation, uint8_t agc_enable) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (lna_gain_attenuation >= SX1232_LNA_GAIN_ATTENUATION_LAST) {
        status = SX1232_ERROR_LNA_GAIN_ATTENUATION;
        goto errors;
    }
    // Set AGC configuration.
    reg_value = (agc_enable != 0) ? 0x0E : 0x00;
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_RXCONFIG, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Program default values.
    status = _SX1232_write_register(SX1232_REGISTER_AGCREF, 0x13);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_AGCTHRESH1, 0x0E);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_AGCTHRESH2, 0x5B);
    if (status != SX1232_SUCCESS) goto errors;
    status = _SX1232_write_register(SX1232_REGISTER_AGCTHRESH3, 0xDB);
    if (status != SX1232_SUCCESS) goto errors;
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_LNA, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute register.
    switch (lna_mode) {
    case SX1232_LNA_MODE_NORMAL:
        reg_value &= 0xFC;
        break;
    case SX1232_LNA_MODE_BOOST:
        reg_value |= 0x03;
        break;
    default:
        status = SX1232_ERROR_LNA_MODE;
        goto errors;
    }
    reg_value &= 0x1F;
    reg_value |= (uint8_t) ((lna_gain_attenuation + 1) << 5);
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_LNA, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_preamble_detector(uint8_t preamble_size_bytes, uint8_t preamble_polarity) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (preamble_size_bytes > SX1232_PREAMBLE_SIZE_BYTES_MAX) {
        status = SX1232_ERROR_PREAMBLE_SIZE;
        goto errors;
    }
    // Load default value.
    reg_value = 0x2A;
    // Check size;
    if (preamble_size_bytes > 0) {
        // Enable preamble detector.
        reg_value &= 0x9F;
        reg_value |= (uint8_t) ((preamble_size_bytes - 1) << 5);
        reg_value |= 0x80;
    }
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_PREAMBLEDETECT, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Set polarity.
    status = _SX1232_read_register(SX1232_REGISTER_SYNCCONFIG, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Program register.
    if (preamble_polarity == 0) {
        reg_value &= 0xDF;
    }
    else {
        reg_value |= 0x20;
    }
    status = _SX1232_write_register(SX1232_REGISTER_SYNCCONFIG, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_sync_word(uint8_t* sync_word, uint8_t sync_word_size_bytes) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t reg_value = 0;
    uint8_t idx = 0;
    // Check parameters.
    if (sync_word == NULL) {
        status = SX1232_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (sync_word_size_bytes > SX1232_SYNC_WORD_MAXIMUM_SIZE_BYTES) {
        status = SX1232_ERROR_SYNC_WORD_SIZE;
        goto errors;
    }
    // Read register.
    status = _SX1232_read_register(SX1232_REGISTER_SYNCCONFIG, &reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute register.
    reg_value &= 0x38; // Reset bits 0-2 and disable receiver auto_restart.
    reg_value |= ((sync_word_size_bytes - 1) & 0x07);
    reg_value |= 0x10; // Enable synchronization word detector.
    // Program register.
    status = _SX1232_write_register(SX1232_REGISTER_SYNCCONFIG, reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Fill synchronization word.
    for (idx = 0; idx < sync_word_size_bytes; idx++) {
        // Warning: this loop is working because registers addresses are adjacent.
        status = _SX1232_write_register((SX1232_REGISTER_SYNCVALUE1 + idx), sync_word[idx]);
        if (status != SX1232_SUCCESS) goto errors;
    }
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_set_rssi_sampling(SX1232_rssi_sampling_t rssi_sampling) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    // Check parameter.
    if (rssi_sampling >= SX1232_RSSI_SAMPLING_LAST) {
        status = SX1232_ERROR_RSSI_SAMPLING;
        goto errors;
    }
    // Program register
    status = _SX1232_write_register(SX1232_REGISTER_RSSICONFIG, rssi_sampling);
    if (status != SX1232_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_get_rssi(int16_t* rssi_dbm) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t rssi_reg_value = 0;
    // Check parameter.
    if (rssi_dbm == NULL) {
        status = SX1232_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read RSSI register.
    status = _SX1232_read_register(SX1232_REGISTER_RSSIVALUE, &rssi_reg_value);
    if (status != SX1232_SUCCESS) goto errors;
    // Compute RSSI.
    (*rssi_dbm) = (-1) * ((int16_t) (rssi_reg_value >> 1));
errors:
    return status;
}
#endif

#ifdef SX1232_DRIVER_RX_ENABLE
/*******************************************************************/
SX1232_status_t SX1232_read_fifo(uint8_t* fifo_data, uint8_t fifo_data_size) {
    // Local variables.
    SX1232_status_t status = SX1232_SUCCESS;
    uint8_t idx = 0;
    uint8_t fifo_data_byte = 0;
    // Check parameters.
    if (fifo_data == NULL) {
        status = SX1232_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (fifo_data_size > SX1232_FIFO_SIZE_BYTES) {
        status = SX1232_ERROR_FIFO_SIZE;
        goto errors;
    }
    // Access FIFO byte per byte.
    for (idx = 0; idx < fifo_data_size; idx++) {
        status = _SX1232_read_register(SX1232_REGISTER_FIFO, &fifo_data_byte);
        if (status != SX1232_SUCCESS) goto errors;
        fifo_data[idx] = fifo_data_byte;
    }
errors:
    return status;
}
#endif

#endif /* SX1232_DRIVER_DISABLE */
