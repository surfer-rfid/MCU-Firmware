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
//    Controller on the MCU, not the one in the FPGA.                             //
//                                                                                //
//                                                                                //
//    Revisions:                                                                  //
//    061819 - Major commentary cleanup.                                          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_config.h"
//#include "nrf_drv_spi.h"
#include "spi_cntrlr_fast.h"
#include "nordic_common.h"
#include "rfidr_error.h"
#include "rfidr_spi.h"

 // Number of bytes to transmit and receive from the MCU SPI.
 // This amount of bytes will also be tested to see that the received bytes from peripheral are the same as the transmitted bytes from the cntrlr.
#define TX_RX_MSG_LENGTH         4

//#if (SPI0_ENABLED == 1)

static uint8_t m_tx_data_spi[TX_RX_MSG_LENGTH]; // SPI cntrlr TX buffer state variable, required for interacting with the SPI peripheral.
static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; // SPI cntrlr RX buffer state variable, required for interacting with the SPI peripheral.

//Initialize the SPI. 
uint32_t spi_cntrlr_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    SPI_config_t spi_config =  {.pin_PCK                 = SPI0_CONFIG_PCK_PIN,
                                .pin_COPI                = SPI0_CONFIG_COPI_PIN,
                                .pin_CIPO                = SPI0_CONFIG_CIPO_PIN,
                                .pin_CSN                 = SPI0_CONFIG_PS_PIN,
                                .frequency               = SPI_FREQ_4MBPS,
                                .config.fields.mode      = 0,
                                .config.fields.bit_order = SPI_BITORDER_MSB_LSB};

    // Initialize the SPI
    spi_cntrlr_init_fast(SPI0, &spi_config);

    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

//Wrapper for NRF SPI transfer. The internal function takes state variables as arguments, simplifying the interface of this file module to
//other modules in the RFID reader MCU codebase.
uint32_t spi_cntrlr_send_recv(void)
{
    uint32_t err_code = NRF_SUCCESS;
    spi_cntrlr_tx_rx(SPI0, TX_RX_MSG_LENGTH, m_tx_data_spi, m_rx_data_spi);
    
    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

//Recover the byte that was read back from the FPGA SPI.
rfidr_error_t spi_cntrlr_read_rx(uint8_t * p_spi_rx_byte)
{
    *p_spi_rx_byte = m_rx_data_spi[3];
    return RFIDR_SUCCESS;
}

//Construct the SPI TX packet based on: 
// 1) Which FPGA memory to write to, 2) Read/write, 3) RX or TX RAM in the case of the Radio RAM, 4) RAM Address, 5) Data to write
rfidr_error_t spi_cntrlr_set_tx(spi_mem_t spi_mem, spi_wr_t wr, spi_rxntx_ram_t rxntx, uint16_t addr, uint8_t data)
{
    /*Declare a local temp variable to hold the arranged data */
    uint32_t temp         =    0;
    uint32_t addr_mask    =     (1<<3) -1; /*The address should be 3 bits */
    uint32_t byte_mask    =    (1<<8)  -1; /*Byte mask should be 8 bits */

    switch(spi_mem){
        case RFIDR_WVFM_MEM:
            addr_mask    =     (1<<13) -1; /*The address should be 13 bits */
            temp         = temp | (0 << 22); /*Read, not write*/
            temp         = temp | (1 << 21); /*This is a waveform memory operation*/
            temp         = temp | ((addr & addr_mask) << 8); /*Load the address*/
            break;
        case RFIDR_RDIO_MEM:
            addr_mask    =     (1<<9) -1; /*The address should be 9 bits */
            temp         = temp | ((wr == RFIDR_SPI_WRITE) << 22); /*Write, not read*/
            temp         = temp | (0 << 21); /*This is not a waveform memory operation*/
            temp         = temp | (1 << 20); /*This is a radio memory operation*/
            temp         = temp | ((rxntx == RFIDR_SPI_RXRAM) << 17); /*Set to operate on tx or rx half of RAM*/
            temp         = temp | ((addr & addr_mask) << 8); /*Load the address*/
            break;
        case RFIDR_TXCN_MEM:
            addr_mask    =     (1<<10) -1; /*The address should be 10 bits */
            temp         = temp | ((wr == RFIDR_SPI_WRITE) << 22); /*Write, not read*/
            temp         = temp | (0 << 21); /*This is not a waveform memory operation*/
            temp         = temp | (0 << 20); /*This is not a radio memory operation*/
            temp         = temp | (1 << 19); /*This is a txcancel memory operation*/
            temp         = temp | ((addr & addr_mask) << 8); /*Load the address*/
            break;
        case RFIDR_USER_MEM:
            addr_mask    =     (1<<3) -1; /*The address should be 3 bits */
            temp         = temp | ((wr == RFIDR_SPI_WRITE) << 22); /*Write, not read*/
            temp         = temp | (0 << 21); /*This is not a waveform memory operation*/
            temp         = temp | (0 << 20); /*This is not a radio memory operation*/
            temp         = temp | (0 << 19); /*This is not a txcancel memory operation*/
            temp         = temp | (1 << 18); /*This is a user memory operation*/
            temp         = temp | ((addr & addr_mask) << 8); /*Load the address*/
            break;
        default:
            addr_mask    =     (1<<3) -1; /*The address should be 3 bits */
            temp         = temp | ((wr == RFIDR_SPI_WRITE) << 22); /*Write, not read*/
            temp         = temp | (0 << 21); /*This is not a waveform memory operation*/
            temp         = temp | (0 << 20); /*This is not a radio memory operation*/
            temp         = temp | (0 << 19); /*This is not a txcancel memory operation*/
            temp         = temp | (1 << 18); /*This is a user memory operation*/
            temp         = temp | ((addr & addr_mask) << 8); /*Load the address*/
            break;
    }

    temp                = temp | (byte_mask & data);
    temp                = temp << 9;

    //Load the state variable bytes to be used by the NRF SPI function.
    m_tx_data_spi[0]    =    (temp >> 24) & byte_mask;
    m_tx_data_spi[1]    =    (temp >> 16) & byte_mask;
    m_tx_data_spi[2]    =    (temp >> 8) & byte_mask;
    m_tx_data_spi[3]    =    (temp >> 0) & byte_mask;

    return RFIDR_SUCCESS;
}

//A wrapper for robust SPI writing in which data is written and read back several times before giving up and flagging an error.
rfidr_error_t spi_cntrlr_write_tx_robust(spi_mem_t spi_mem, spi_rxntx_ram_t rxntx, uint16_t addr, uint8_t data)
{
    uint8_t        loop_try        =    0;

    while (loop_try < 3)
    {
        spi_cntrlr_set_tx(spi_mem, RFIDR_SPI_WRITE, rxntx, addr, data);    //Set up the TX write packet.
        spi_cntrlr_send_recv();                                            //Perform a write/read transaction over SPI.
        spi_cntrlr_set_tx(spi_mem, RFIDR_SPI_READ, rxntx, addr, 0);        //Set up the TX packet, but this time read back a byte.
        spi_cntrlr_send_recv();                                            //Perform a write/read transaction over SPI.
        if(m_rx_data_spi[3]==data)                                         //If the received byte matches the data sent, it worked!
        {
            return RFIDR_SUCCESS;
        }
        loop_try++;
    }
    return RFIDR_ERROR_SPI_WRITE_TX;
}

//A function for writing registers in the SX1257 through the SPI bridge in the FPGA.
//This one is quite a bit more complicated than a regular SPI write because we have to control another SPI cntrlr with the SPI cntrlr in the MCU.
rfidr_error_t spi_cntrlr_write_sx1257_robust(uint8_t sx1257_addr, uint8_t sx1257_data)
{
    #define USER_MEM_SX1257_STAT_ADDR        2    //Address of FPGA SPI cntrlr status registers in FPGA SPI peripheral user memory.
    #define USER_MEM_SX1257_ADDR_ADDR        4    //Address of FPGA SPI peripheral user memory where we place the SX1257 address to be written to.
    #define USER_MEM_SX1257_DATA_ADDR        5    //Address of FPGA SPI peripheral user memory where we place the SX1257 data to be written.
    #define USER_MEM_SX1257_RTRN_ADDR        5    //Address of FPGA SPI cntrlr return address storage location in FPGA SPI peripheral.

    //Registers for controlling the SPI bridge in the FPGA are in the user memory.
    
    uint8_t        loop_try_outer                =    0;
    uint8_t        loop_try_inner                =    0;
    uint8_t        sx1257_addr_masked_write      =    0;
    uint8_t        sx1257_addr_masked_read       =    0;

    sx1257_addr_masked_read     =    sx1257_addr & 127;                //There is only 7-bit address space in the SX1257. MSB is the read/write bit.
    sx1257_addr_masked_write    =    sx1257_addr_masked_read | 128;    //And there is the write bit.


    while (loop_try_outer < 3)
    {
        //Check to see that pending and done registers are properly set
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
        spi_cntrlr_send_recv();
        if(((m_rx_data_spi[3] >> 5) & 3) != 0 )
        {
            return RFIDR_ERROR_SPI_WRITE_SX1257_1;
        }
        //Write SX1257 WNR bit + ADDR to SPI cntrlr passthrough address byte
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_ADDR_ADDR, sx1257_addr_masked_write);
        spi_cntrlr_send_recv();
        //Write SX1257 DATA to SPI cntrlr passthrough data byte
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_DATA_ADDR, sx1257_data);
        spi_cntrlr_send_recv();
        //Write spi ready address with just a 1
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 2);
        spi_cntrlr_send_recv();

        // Wait for IRQ. Alternatively, can do polling over SPI.
        // If we do get an IRQ callback it will be at main.
        // Main can call into rfidr_spi and set a variable here, but said variable will also need to be polled.
        // Still, said internal polling is quite a bit faster than polling over SPI.
        // If we use the IRQ, the IRQ handler at main will need to be able to differentiate the IRQ events.
        // The problem with doing *this* is that we need to check 3 different SPI registers to see what happened.
        // Perhaps the best thing to do is to poll since it will be done rather quickly
        // If it takes more than 3 polls, give up and flag an error

        loop_try_inner=0;
        while (1)
        {
            //Check to see that the FPGA cntrlr SPI done register indicates that it is done
            spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
            spi_cntrlr_send_recv();
            if(((m_rx_data_spi[3] >> 5) & 1) == 1 )
            {
                break;
            }
            else
            {
                loop_try_inner++;
                if(loop_try_inner > 3)
                {
                    return RFIDR_ERROR_SPI_WRITE_SX1257_2;
                }
            }

        }

        //Write spi ready deassert
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
        spi_cntrlr_send_recv();
        //Set up for a read from the same register
        //Write SX1257 WNR bit + ADDR to SPI cntrlr passthrough address byte
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_ADDR_ADDR, sx1257_addr_masked_read);
        spi_cntrlr_send_recv();
        //Write SX1257 DATA to SPI cntrlr passthrough data byte
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_DATA_ADDR, sx1257_data);
        spi_cntrlr_send_recv();
        //Write spi ready address with just a 2
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 2);
        spi_cntrlr_send_recv();

        loop_try_inner=0;
        while (1)
        {
            //Check to see that FPGA cntrlr SPI done register indicates that it is done
            spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
            spi_cntrlr_send_recv();
            if(((m_rx_data_spi[3] >> 5) & 1) == 1 )
            {
                break;
            }
            else
            {
                loop_try_inner++;
                if(loop_try_inner > 3)
                {
                    return RFIDR_ERROR_SPI_WRITE_SX1257_3;
                }
            }

        }

        //Write spi ready deassert
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
        spi_cntrlr_send_recv();
        //Read back returned data from SX1257
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_RTRN_ADDR, 0);
        spi_cntrlr_send_recv();
        if(m_rx_data_spi[3] != sx1257_data )
        {
            return RFIDR_ERROR_SPI_WRITE_SX1257_4;
        }
        else
        {
            return RFIDR_SUCCESS;
        }

        loop_try_outer++;
    }
    return RFIDR_ERROR_SPI_WRITE_SX1257_5;
}

//A function for reading registers in the SX1257 through the SPI bridge in the FPGA.
//This one is quite a bit more complicated than a regular SPI read because we have to control another SPI cntrlr with the SPI cntrlr in the MCU.
rfidr_error_t spi_cntrlr_read_sx1257_robust(uint8_t sx1257_addr, uint8_t * sx1257_data)
{
    #define USER_MEM_SX1257_STAT_ADDR        2    //Address of FPGA SPI controller status registers in FPGA SPI peripheral user memory.
    #define USER_MEM_SX1257_ADDR_ADDR        4    //Address of FPGA SPI peripheral user memory where we place the SX1257 address to be written to.
    #define USER_MEM_SX1257_DATA_ADDR        5    //Address of FPGA SPI peripheral user memory where we place the SX1257 data to be written.
    #define USER_MEM_SX1257_RTRN_ADDR        5    //Address of FPGA SPI controller return address storage location in FPGA SPI peripheral.

    uint8_t        loop_try_inner                =    0;
    uint8_t        sx1257_addr_masked_read       =    0;

    sx1257_addr_masked_read        =    sx1257_addr & 127;    //There is only 7-bit address space in the SX1257. MSB is the read/write bit.
                                                        //No write bit.

        //Set up for a read from the same register
        //Write SX1257 WNR bit + ADDR to SPI cntrlr passthrough address byte
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_ADDR_ADDR, sx1257_addr_masked_read);
        spi_cntrlr_send_recv();
        //Write SX1257 DATA to SPI cntrlr passthrough data byte
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_DATA_ADDR, 0x00);
        spi_cntrlr_send_recv();
        //Write spi ready address with just a 2
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 2);
        spi_cntrlr_send_recv();

        loop_try_inner=0;
        while (1)
        {
            //Check to see that cntrlr SPI done register indicates that it is indeed done
            spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
            spi_cntrlr_send_recv();
            if(((m_rx_data_spi[3] >> 5) & 1) == 1 )
            {
                break;
            }
            else
            {
                loop_try_inner++;
                if(loop_try_inner > 3)
                {
                    return RFIDR_ERROR_SPI_WRITE_SX1257_3;
                }
            }

        }

        //Write spi ready deassert
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_STAT_ADDR, 0);
        spi_cntrlr_send_recv();
        //Read back returned data from SX1257
        spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)USER_MEM_SX1257_RTRN_ADDR, 0);
        spi_cntrlr_send_recv();
        
        *sx1257_data=m_rx_data_spi[3];
        
        return RFIDR_SUCCESS;

}

//#endif
