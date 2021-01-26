////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
// Module : RFIDr Firmware SPI Functions                                          //
//                                                                                //
// Filename: rfidr_spi.c                                                          //
// Creation Date: circa 10/31/2016                                                //
// Author: Edward Keehr                                                           //
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
//    This file contains code required for operating the SPI link between the MCU //
//    and the FPGA.                                                               //
//    In addition, this file contains code for operating the SPI pass-through     //
//    bridge that is used to communicate with the SX1257 through the FPGA.        //
//                                                                                //
//    Historically throughout the project, the FPGA register map accessible over  //
//    the SPI has changed frequently, so it hasn't been documented.               //
//                                                                                //
//    Each SPI transaction takes 4 bytes for simplicity.                          //
//    The first three bytes contain data sent from the MCU to the FPGA.           //
//    The final byte contains data read back from the FPGA to the MCU.            //
//    This final byte is either the data that was just written, in case of a      //
//    write, or data that was specified to be read back.                          //
//                                                                                //
//    More comprehensive documentation on the SPI packet structure is forthcoming.//
//    The actual packet structure depends on the type of memory to be accessed.   //
//    Much of the SPI packet structure can be inferred from the spi_cntrlr_set_tx //
//    function.                                                                   //
//                                                                                //
//    Note that mentions of the "spi_cntrlr" in function names refer to the SPI   //
//    Master on the MCU, not the one in the FPGA.                                 //
//                                                                                //
//    All of the functions in this file that are named "nrf_drv" can be found in  //
//    SDK\components\drivers_nrf\spi_master\nrf_drv_spi.c                         //
//                                                                                //
//    Revisions:                                                                  //
//    061819 - Major commentary cleanup.                                          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

//This file provides high level abstraction to RFIDR SPI functionality, mimicing
//the SPI driver functions from the top level simulation verilog testbenches.

#ifndef RFIDR_SPI_H__
#define RFIDR_SPI_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "rfidr_error.h"

//SPI Internal Register Read Write type 

typedef enum
{
  RFIDR_SPI_WRITE,
  RFIDR_SPI_READ
} spi_wr_t;

//SPI Internal Register memory type

typedef enum
{
  RFIDR_WVFM_MEM,
  RFIDR_RDIO_MEM,
  RFIDR_TXCN_MEM,
  RFIDR_USER_MEM
} spi_mem_t;

//SPI TX RX Radio RAM select type

typedef enum
{
  RFIDR_SPI_RXRAM,
  RFIDR_SPI_TXRAM
} spi_rxntx_ram_t;

//function for initializing the spi cntrlr
//returns NRF_SUCCESS on successful SPI initialization

uint32_t spi_cntrlr_init(void);

//function for executing the SPI transaction
//returns NRF_SUCCESS on successful SPI transaction

uint32_t spi_cntrlr_send_recv(void);

//function for reading the SPI RX buffer after a transaction
//returns NRF_SUCCESS on successful spi RX buffer read

rfidr_error_t spi_cntrlr_read_rx(uint8_t * p_spi_rx_byte);

//function for setting the SPI TX buffer prior to a transaction
//returns RFIDR_SUCCESS on successful SPI TX buffer set

rfidr_error_t spi_cntrlr_set_tx(spi_mem_t spi_mem, spi_wr_t wr, spi_rxntx_ram_t rxntx, uint16_t addr, uint8_t data);

//function for making a robust SPI write to general memories
//returns RFIDR_SUCCESS on successful SPI TX buffer write

rfidr_error_t spi_cntrlr_write_tx_robust(spi_mem_t spi_mem, spi_rxntx_ram_t rxntx, uint16_t addr, uint8_t data);

//function for making a robust SPI write to the SX1257
//returns NRF_SUCCESS on successful SPI TX buffer set

rfidr_error_t spi_cntrlr_write_sx1257_robust(uint8_t addr, uint8_t data);

//function for making a robust SPI read check to the SX1257
//returns NRF_SUCCESS Successful SPI TX buffer set

rfidr_error_t spi_cntrlr_read_sx1257_robust(uint8_t addr, uint8_t * data);

#endif // RFIDR_SPI_H__

