 /* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
 //Superlative Semiconductor note: controller terminology changed 122020

#ifndef __SPI_CNTRLR_FAST_H
#define __SPI_CNTRLR_FAST_H

#include <stdbool.h>
#include <stdint.h>

#define SPI_FAST_DEFAULT_CONFIG {.pin_PCK = 1, .pin_COPI = 2, .pin_CIPO = 3, .pin_CSN = 4,  \
                                 .frequency = SPI_FREQ_1MBPS, .config.fields.mode = 0, .config.fields.bit_order = SPI_BITORDER_MSB_LSB}

/**
 *  SPI controller operating frequency
 */
typedef enum
{
    SPI_FREQ_125KBPS = 0x02,           /*!< drive SClk with frequency 125Kbps */
    SPI_FREQ_250KBPS = 0x04,           /*!< drive SClk with frequency 250Kbps */
    SPI_FREQ_500KBPS = 0x08,           /*!< drive SClk with frequency 500Kbps */
    SPI_FREQ_1MBPS = 0x10,             /*!< drive SClk with frequency 1Mbps */
    SPI_FREQ_2MBPS = 0x20,             /*!< drive SClk with frequency 2Mbps */
    SPI_FREQ_4MBPS = 0x40,             /*!< drive SClk with frequency 4Mbps */
    SPI_FREQ_8MBPS = 0x80              /*!< drive SClk with frequency 8Mbps */
} SPI_frequency_t;

/**
 *  SPI controller module number
 */
typedef enum
{
    SPI0 = 0,               /*!< SPI module 0 */
    SPI1                    /*!< SPI module 1 */
} SPI_module_number_t;

/**
 *  SPI mode
 */
typedef enum
{
    //------------------------Clock polarity 0, Clock starts with level 0-------------------------------------------
    SPI_MODE0 = 0,          /*!< Sample data at rising edge of clock and shift serial data at falling edge */
    SPI_MODE1,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    //------------------------Clock polarity 1, Clock starts with level 1-------------------------------------------
    SPI_MODE2,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    SPI_MODE3               /*!< Sample data at rising edge of clock and shift serial data at falling edge */
} SPI_mode_t;

/**
 *  SPI master bit ordering
 */
typedef enum
{
    SPI_BITORDER_MSB_LSB = 0, /*!< Most significant to least significant bit */
    SPI_BITORDER_LSB_MSB      /*!< Least significant to most significant bit */
} SPI_bit_order_t;

/**
 *  Struct containing all parameters necessary to configure the SPI interface
 */
typedef struct
{
    union
    {
        uint8_t SPI_cfg;            /*!< Bit mode and bit order merged, as in the SPI CONFIG register */
        struct
        {
            uint8_t bit_order : 1;  /*!< SPI master bit order */
            uint8_t mode : 2;       /*!< SPI master mode */
            uint8_t : 5;            /*!< Padding */
        }fields;
    }config;    
    uint8_t frequency;              /*!< SPI master frequency */
    uint8_t pin_PCK;                /*!< SPI master PCK pin */
    uint8_t pin_COPI;               /*!< SPI master COPI pin */
    uint8_t pin_CIPO;               /*!< SPI master CIPO pin */
    uint8_t pin_CSN;                /*!< SPI master chip select pin */
} SPI_config_t;

/**
 * Initializes given SPI master with given configuration.
 *
 * After initializing the given SPI master with given configuration, this function also test if the
 * SPI slave is responding with the configurations by transmitting few test bytes. If the slave did not
 * respond then error is returned and contents of the rx_data are invalid.
 *
 * @param module_number SPI master number (SPIModuleNumber) to initialize.
 * @param pointer to a struct of type @ref SPIConfig_t containing the SPI configuration parameters.
 * @return
 * @retval pointer to direct physical address of the requested SPI module if init was successful
 * @retval 0, if either init failed or slave did not respond to the test transfer
 */
uint32_t* spi_cntrlr_init_fast(SPI_module_number_t spi_num, SPI_config_t *spi_config);

/**
 * Transmit/receive data over SPI bus.
 *
 * @note Make sure at least transfer_size number of bytes is allocated in tx_data/rx_data.
 *
 * @param spi_num SPI master number (SPIModuleNumber)
 * @param transfer_size  number of bytes to transmit/receive over SPI master
 * @param tx_data pointer to the data that needs to be transmitted
 * @param rx_data pointer to the data that needs to be received
 * @return
 * @retval true if transmit/reveive of transfer_size were completed.
 * @retval false if transmit/reveive of transfer_size were not complete and tx_data/rx_data points to invalid data.
 */
bool spi_cntrlr_tx_rx(SPI_module_number_t spi_num, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data);

 
#endif
