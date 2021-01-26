////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
// Module : RFIDr NRF driver configurations.                                      //
//                                                                                //
// Filename: nrf_drv_config.c                                                     //
// Creation Date: circa 10/31/2016                                                //
// Author: Edward Keehr                                                           //
//                                                                                //
//    This file was derived from the Nordic Semiconductor example code in their   //
//    SDK 8.0. Specifically, this file was derived from "main.c" in the           //
//    ble_app_uart project.                                                       //
//                                                                                //
//     The required notice to be reproduced for Nordic Semiconductor code is      //
//     given below:                                                               //
//                                                                                //
//    /* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.            //
// *                                                                              //
// * The information contained herein is property of Nordic Semiconductor ASA.    //
// * Terms and conditions of usage are described in detail in NORDIC              //
// * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.                           //
// *                                                                              //
// * Licensees are granted free, non-transferable use of the information. NO      //
// * WARRANTY of ANY KIND is provided. This heading must NOT be removed from      //
// * the file.                                                                    //
// *                                                                              //
// */                                                                             //
//                                                                                //
//    For components of the code modified or authored by Superlative              //
//    Semiconductor LLC, the copyright notice is as follows:                      //
//                                                                                //
//    Copyright 2021 Superlative Semiconductor LLC                                //
//                                                                                //
//    Licensed under the Apache License, Version 2.0 (the "License");             //
//    you may not use this file except in compliance with the License.            //
//    You may obtain a copy of the License at                                     //
//                                                                                //
//       http://www.apache.org/licenses/LICENSE-2.0                               //
//                                                                                //
//    Unless required by applicable law or agreed to in writing, software         //
//    distributed under the License is distributed on an "AS IS" BASIS,           //
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//    See the License for the specific language governing permissions and         //
//    limitations under the License.                                              //
//                                                                                //
// Description:                                                                   //
//                                                                                //
//    This file contains configuration data for the NRF microcontroller used in   //
//    the SDR RFID reader.                                                        //
//                                                                                //
//    Revisions:                                                                  //
//    061819 - Major commentary cleanup.                                          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

//Superlative Semiconductor note: The following template of defines was provided by
//Nordic Semiconductor. Values were modified by Superlative Semiconductor for the 
//UHF RFID reader project.

#ifndef NRF_DRV_CONFIG_H
#define NRF_DRV_CONFIG_H

/* CLOCK */
#define CLOCK_CONFIG_XTAL_FREQ          NRF_CLOCK_XTALFREQ_Default
#define CLOCK_CONFIG_LF_SRC             NRF_CLOCK_LF_SRC_Xtal
#define CLOCK_CONFIG_LF_RC_CAL_INTERVAL RC_2000MS_CALIBRATION_INTERVAL
#define CLOCK_CONFIG_IRQ_PRIORITY       APP_IRQ_PRIORITY_LOW

/* GPIOTE */
#define GPIOTE_ENABLED 1

#if (GPIOTE_ENABLED == 1)
#define GPIOTE_CONFIG_USE_SWI_EGU false
#define GPIOTE_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 4
#endif

#define EN_VDD_PA_PIN 30
#define PWR_TOGL_PIN 0
#define SAMPLE_PIN 1
#define CYCLE_PIN 2
#define DIO3_PIN 6
#define OPA_SPDT1_CTL_PIN 7
#define DIO2_PIN 8
#define RDIO_RST_P_PIN 9
#define XO_ENABLE_PIN 10

//Comment out below for debug of SPI on dev board
#define FPGA_IRQ_PIN 15
//#define FPGA_IRQ_PIN 17
#define FPGA_RST_N_PIN 16
#define ANT0_PIN 21
#define ANT1_PIN 22
#define BAT_LED1_PIN 28
#define BAT_LED0_PIN 29
//Below: for debugging on nrf pca10001 only
//#define BAT_LED0_PIN 18
//#define BAT_LED1_PIN 19

/* TIMER */
#define TIMER0_ENABLED 0

#if (TIMER0_ENABLED == 1)
#define TIMER0_CONFIG_FREQUENCY    NRF_TIMER_FREQ_16MHz
#define TIMER0_CONFIG_MODE         TIMER_MODE_MODE_Timer
#define TIMER0_CONFIG_BIT_WIDTH    TIMER_BITMODE_BITMODE_32Bit
#define TIMER0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TIMER0_INSTANCE_INDEX      0
#endif

#define TIMER1_ENABLED 0

#if (TIMER1_ENABLED == 1)
#define TIMER1_CONFIG_FREQUENCY    NRF_TIMER_FREQ_16MHz
#define TIMER1_CONFIG_MODE         TIMER_MODE_MODE_Timer
#define TIMER1_CONFIG_BIT_WIDTH    TIMER_BITMODE_BITMODE_16Bit
#define TIMER1_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TIMER1_INSTANCE_INDEX      (TIMER0_ENABLED)
#endif
 
#define TIMER2_ENABLED 0

#if (TIMER2_ENABLED == 1)
#define TIMER2_CONFIG_FREQUENCY    NRF_TIMER_FREQ_16MHz
#define TIMER2_CONFIG_MODE         TIMER_MODE_MODE_Timer
#define TIMER2_CONFIG_BIT_WIDTH    TIMER_BITMODE_BITMODE_16Bit
#define TIMER2_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TIMER2_INSTANCE_INDEX      (TIMER1_ENABLED+TIMER0_ENABLED)
#endif

#define TIMER3_ENABLED 0

#if (TIMER3_ENABLED == 1)
#define TIMER3_CONFIG_FREQUENCY    NRF_TIMER_FREQ_16MHz
#define TIMER3_CONFIG_MODE         TIMER_MODE_MODE_Timer
#define TIMER3_CONFIG_BIT_WIDTH    TIMER_BITMODE_BITMODE_16Bit
#define TIMER3_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TIMER3_INSTANCE_INDEX      (TIMER2_ENABLED+TIMER1_ENABLED+TIMER0_ENABLED)
#endif

#define TIMER4_ENABLED 0

#if (TIMER4_ENABLED == 1)
#define TIMER4_CONFIG_FREQUENCY    NRF_TIMER_FREQ_16MHz
#define TIMER4_CONFIG_MODE         TIMER_MODE_MODE_Timer
#define TIMER4_CONFIG_BIT_WIDTH    TIMER_BITMODE_BITMODE_16Bit
#define TIMER4_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TIMER4_INSTANCE_INDEX      (TIMER3_ENABLED+TIMER2_ENABLED+TIMER1_ENABLED+TIMER0_ENABLED)
#endif


#define TIMER_COUNT (TIMER0_ENABLED + TIMER1_ENABLED + TIMER2_ENABLED + TIMER3_ENABLED + TIMER4_ENABLED)

/* RTC */
#define RTC0_ENABLED 0

#if (RTC0_ENABLED == 1)
#define RTC0_CONFIG_FREQUENCY    32678
#define RTC0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#define RTC0_CONFIG_RELIABLE     false

#define RTC0_INSTANCE_INDEX      0
#endif

#define RTC1_ENABLED 0

#if (RTC1_ENABLED == 1)
#define RTC1_CONFIG_FREQUENCY    32768
#define RTC1_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#define RTC1_CONFIG_RELIABLE     false

#define RTC1_INSTANCE_INDEX      (RTC0_ENABLED)
#endif

#define RTC_COUNT                (RTC0_ENABLED+RTC1_ENABLED)

#define NRF_MAXIMUM_LATENCY_US 2000

/* RNG */
#define RNG_ENABLED 0

#if (RNG_ENABLED == 1)
#define RNG_CONFIG_ERROR_CORRECTION true
#define RNG_CONFIG_POOL_SIZE        8
#define RNG_CONFIG_IRQ_PRIORITY     APP_IRQ_PRIORITY_LOW
#endif

/* SPI */
#define SPI0_ENABLED 1

#if (SPI0_ENABLED == 1)
#define SPI0_USE_EASY_DMA 0

#define SPI0_CONFIG_PCK_PIN         14
//Swap out COPI pin to 15 for dev board
#define SPI0_CONFIG_COPI_PIN        11
//#define SPI0_CONFIG_COPI_PIN        15
#define SPI0_CONFIG_CIPO_PIN        12
#define SPI0_CONFIG_PS_PIN        13
#define SPI0_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPI0_INSTANCE_INDEX 0
#endif

#define SPI1_ENABLED 0

#if (SPI1_ENABLED == 1)
#define SPI1_USE_EASY_DMA 0

#define SPI1_CONFIG_PCK_PIN         2
#define SPI1_CONFIG_COPI_PIN        3
#define SPI1_CONFIG_CIPO_PIN        4
#define SPI1_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPI1_INSTANCE_INDEX (SPI0_ENABLED)
#endif

#define SPI2_ENABLED 0

#if (SPI2_ENABLED == 1)
#define SPI2_USE_EASY_DMA 0

#define SPI2_CONFIG_PCK_PIN         2
#define SPI2_CONFIG_COPI_PIN        3
#define SPI2_CONFIG_CIPO_PIN        4
#define SPI2_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPI2_INSTANCE_INDEX (SPI0_ENABLED + SPI1_ENABLED)
#endif

#define SPI_COUNT   (SPI0_ENABLED + SPI1_ENABLED + SPI2_ENABLED)

/* SPIS */
#define SPIS0_ENABLED 0

#if (SPIS0_ENABLED == 1)
#define SPIS0_CONFIG_PCK_PIN         2
#define SPIS0_CONFIG_COPI_PIN        3
#define SPIS0_CONFIG_CIPO_PIN        4
#define SPIS0_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPIS0_INSTANCE_INDEX 0
#endif

#define SPIS1_ENABLED 0

#if (SPIS1_ENABLED == 1)
#define SPIS1_CONFIG_PCK_PIN         2
#define SPIS1_CONFIG_COPI_PIN        3
#define SPIS1_CONFIG_CIPO_PIN        4
#define SPIS1_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPIS1_INSTANCE_INDEX SPIS0_ENABLED
#endif

#define SPIS2_ENABLED 0

#if (SPIS2_ENABLED == 1)
#define SPIS2_CONFIG_PCK_PIN         2
#define SPIS2_CONFIG_COPI_PIN        3
#define SPIS2_CONFIG_CIPO_PIN        4
#define SPIS2_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPIS2_INSTANCE_INDEX (SPIS0_ENABLED + SPIS1_ENABLED)
#endif

#define SPIS_COUNT   (SPIS0_ENABLED + SPIS1_ENABLED + SPIS2_ENABLED)

/* UART */
#define UART0_ENABLED 0

#if (UART0_ENABLED == 1)
#define UART0_CONFIG_HWFC         NRF_UART_HWFC_DISABLED
#define UART0_CONFIG_PARITY       NRF_UART_PARITY_EXCLUDED
#define UART0_CONFIG_BAUDRATE     NRF_UART_BAUDRATE_38400
#define UART0_CONFIG_PSEL_TXD 0
#define UART0_CONFIG_PSEL_RXD 0
#define UART0_CONFIG_PSEL_CTS 0
#define UART0_CONFIG_PSEL_RTS 0
#define UART0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#ifdef NRF52
#define UART0_CONFIG_USE_EASY_DMA false
//Compile time flag
#define UART_EASY_DMA_SUPPORT     1
#define UART_LEGACY_SUPPORT       1
#endif //NRF52
#endif

#define TWI0_ENABLED 0

#if (TWI0_ENABLED == 1)
#define TWI0_CONFIG_FREQUENCY    NRF_TWI_FREQ_100K
#define TWI0_CONFIG_SCL          0
#define TWI0_CONFIG_SDA          1
#define TWI0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TWI0_INSTANCE_INDEX      0
#endif

#define TWI1_ENABLED 0

#if (TWI1_ENABLED == 1)
#define TWI1_CONFIG_FREQUENCY    NRF_TWI_FREQ_100K
#define TWI1_CONFIG_SCL          0
#define TWI1_CONFIG_SDA          1
#define TWI1_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TWI1_INSTANCE_INDEX      (TWI0_ENABLED)
#endif

#define TWI_COUNT                (TWI0_ENABLED+TWI1_ENABLED)

/* TWIS */
#define TWIS0_ENABLED 0

#if (TWIS0_ENABLED == 1)
    #define TWIS0_CONFIG_ADDR0        0
    #define TWIS0_CONFIG_ADDR1        0 /* 0: Disabled */
    #define TWIS0_CONFIG_SCL          0
    #define TWIS0_CONFIG_SDA          1
    #define TWIS0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

    #define TWIS0_INSTANCE_INDEX      0
#endif

#define TWIS1_ENABLED 0

#if (TWIS1_ENABLED ==  1)
    #define TWIS1_CONFIG_ADDR0        0
    #define TWIS1_CONFIG_ADDR1        0 /* 0: Disabled */
    #define TWIS1_CONFIG_SCL          0
    #define TWIS1_CONFIG_SDA          1
    #define TWIS1_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

    #define TWIS1_INSTANCE_INDEX      (TWIS0_ENABLED)
#endif

#define TWIS_COUNT (TWIS0_ENABLED + TWIS1_ENABLED)
/* For more documentation see nrf_drv_twis.h file */
#define TWIS_ASSUME_INIT_AFTER_RESET_ONLY 0
/* For more documentation see nrf_drv_twis.h file */
#define TWIS_NO_SYNC_MODE 0
/**
 * @brief Definition for patching PAN problems
 *
 * Set this definition to nonzero value to patch anomalies
 * from MPW3 - first lunch microcontroller.
 *
 * Concerns:
 * - PAN-29: TWIS: incorrect bits in ERRORSRC
 * - PAN-30: TWIS: STOP task does not work as expected
 */
#define NRF_TWIS_PATCH_FOR_MPW3 1


/* QDEC */
#define QDEC_ENABLED 0

#if (QDEC_ENABLED == 1)
#define QDEC_CONFIG_REPORTPER    NRF_QDEC_REPORTPER_10
#define QDEC_CONFIG_SAMPLEPER    NRF_QDEC_SAMPLEPER_16384us
#define QDEC_CONFIG_PIO_A        1
#define QDEC_CONFIG_PIO_B        2
#define QDEC_CONFIG_PIO_LED      3
#define QDEC_CONFIG_LEDPRE       511
#define QDEC_CONFIG_LEDPOL       NRF_QDEC_LEPOL_ACTIVE_HIGH
#define QDEC_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#define QDEC_CONFIG_DBFEN        false
#define QDEC_CONFIG_SAMPLE_INTEN false
#endif

/* SAADC */
#define SAADC_ENABLED 0

#if (SAADC_ENABLED == 1)
#define SAADC_CONFIG_RESOLUTION      NRF_SAADC_RESOLUTION_10BIT
#define SAADC_CONFIG_OVERSAMPLE      NRF_SAADC_OVERSAMPLE_DISABLED
#define SAADC_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW
#endif

/* LPCOMP */
#define LPCOMP_ENABLED 0

#if (LPCOMP_ENABLED == 1)
#define LPCOMP_CONFIG_REFERENCE    NRF_LPCOMP_REF_SUPPLY_4_8
#define LPCOMP_CONFIG_DETECTION    NRF_LPCOMP_DETECT_DOWN
#define LPCOMP_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#define LPCOMP_CONFIG_INPUT        NRF_LPCOMP_INPUT_0
#endif

/* WDT */
#define WDT_ENABLED 0

#if (WDT_ENABLED == 1)
#define WDT_CONFIG_BEHAVIOUR     NRF_WDT_BEHAVIOUR_RUN_SLEEP
#define WDT_CONFIG_RELOAD_VALUE  2000
#define WDT_CONFIG_IRQ_PRIORITY  APP_IRQ_PRIORITY_HIGH
#endif

/* SWI EGU */
#ifdef NRF52
    #define EGU_ENABLED 0
#endif


#include "nrf_drv_config_validation.h"

#endif // NRF_DRV_CONFIG_H
