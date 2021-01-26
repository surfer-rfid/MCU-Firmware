//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware Waveform Operations                                  //
//                                                                              //
// Filename: rfidr_waveform.c                                                   //
// Creation Date: circa 10/31/2016                                              //
// Author: Edward Keehr                                                         //
//                                                                              //
//    Copyright 2021 Superlative Semiconductor LLC                              //
//                                                                              //
//    Licensed under the Apache License, Version 2.0 (the "License");           //
//    you may not use this file except in compliance with the License.          //
//    You may obtain a copy of the License at                                   //
//                                                                              //
//       http://www.apache.org/licenses/LICENSE-2.0                             //
//                                                                              //
//    Unless required by applicable law or agreed to in writing, software       //
//    distributed under the License is distributed on an "AS IS" BASIS,         //
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  //
//    See the License for the specific language governing permissions and       //
//    limitations under the License.                                            //
//                                                                              //
// Description:                                                                 //
//                                                                              //
//    This file contains code for working with the waveform recovery feature in //
//    the FPGA.                                                                 //
//                                                                              //
//    Revisions:                                                                //
//    061819 - Major commentary cleanup.                                        //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

#include "app_error.h"
#include "app_util_platform.h"
#include "ble_rfidrs.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "rfidr_error.h"
#include "rfidr_spi.h"
#include "rfidr_waveform.h"

#define    WAVEFORM_MEMORY_DEPTH_IN_BYTES    8192

//Set the waveform offset, in clock cycles, from the start of the FPGA radio state machine operation.
//The available memory for this operation is 8192 bytes, which represents about 0.9ms worth of I,Q data.
//While the each FPGA radio state machine run is typically an order of magnitude longer than this, 
//it is hoped that such a snippet of data proves useful for debugging or advanced ranging purposes.
rfidr_error_t    set_waveform_offset(uint8_t offset)
{
    rfidr_error_t    error_code    =    RFIDR_SUCCESS;

    error_code            =    spi_cntrlr_write_tx_robust(RFIDR_USER_MEM,RFIDR_SPI_TXRAM,(uint16_t)3,offset);
    return error_code;
}

//Move the waveform data from the waveform RAM to the iDevice over Bluetooth LE.
//This takes quite some time over BTLE 4 and an iDevice.
//If it is desired to use this feature extensively, we'll need to move to BTLE 5.
rfidr_error_t     rfidr_push_waveform_data_over_ble(ble_rfidrs_t * p_rfidrs)
{
    uint16_t        loop_bytes                                        =    0;
    uint8_t            counter                                            =    0;
    uint8_t            message_buffer[BLE_RFIDRS_WAVFM_DATA_CHAR_LEN]    =    {0};
    uint8_t            recovery_byte                                    =    0;
    uint32_t        error_code                                        =    NRF_SUCCESS;

    for(loop_bytes=0;loop_bytes < WAVEFORM_MEMORY_DEPTH_IN_BYTES;loop_bytes++)
    {
        //Read bytes out of FPGA waveform memory.
        spi_cntrlr_set_tx(RFIDR_WVFM_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, loop_bytes, 0);
        spi_cntrlr_send_recv();
        spi_cntrlr_read_rx(&recovery_byte);
        message_buffer[counter]    =    recovery_byte;

        counter++;
        //If we're done reading the memory, send off a BTLE packet back to the iDevice.
        if(loop_bytes >= WAVEFORM_MEMORY_DEPTH_IN_BYTES-1)
        {
            do
            {
                error_code=ble_rfidrs_wavfm_data_send(p_rfidrs, message_buffer, counter);
            }while(error_code == BLE_ERROR_NO_TX_BUFFERS);

            if (error_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(error_code);
            }
            counter=0;
        }
        //If we've added as much data to the message buffer as will fit in a BTLE packet (20 bytes as of 6/19/2019),
        //send a BTLE packet to the iDevice.
        else if(counter>=BLE_RFIDRS_WAVFM_DATA_CHAR_LEN)
        {
            do
            {
                error_code=ble_rfidrs_wavfm_data_send(p_rfidrs, message_buffer, BLE_RFIDRS_WAVFM_DATA_CHAR_LEN);
            }while(error_code == BLE_ERROR_NO_TX_BUFFERS);

            if (error_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(error_code);
            }
            counter=0;
        }
    }

    return RFIDR_SUCCESS;
}
