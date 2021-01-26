//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware RX Radio RAM loading                                 //
//                                                                              //
// Filename: rfidr_rxradio.c                                                    //
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
//    This file contains code required for loading the RX RAM on the RFIDr      //
//    FPGA. This file also contains code related to EPCs to be transmitted.     //
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
#include "rfidr_state.h"
#include "rfidr_rxradio.h"
#include "rfidr_txradio.h"
#include <math.h>
#include <string.h>

//There exists an offset in FPGA RX RAM for each type of tag reply that the reader can process.
//These offsets represent 16-byte chunks. Some packet types require two address spaces to fit all of the data that may be added to the RX RAM during operation.

//The format of each address space is as follows:
//Section 0 (1 byte): Number of bits of valid data. This value is written by the MCU FW to the FPGA. The FPGA reads this to determine how many bits to read in from the received reply.
//This scheme of reply termination was chosen as opposed to relying on the reply dummy-1 terminator bit for security purposes and to prevent RX RAM overflow.
//Section 1 (up to 17 bytes): Returned data. For example, for a PCEPC reply, this is 128 bits, or 16 bytes.
//Section 2 (1 byte): Exit code. The code provides information on which state of the data recovery state machine the reception process terminated at.
//Section 3 (4 bytes): Main integrator magnitude. Provides information on complex RSSI. Represents either I or Q data depending on how the use_i register is set in the FPGA.
//Section 4 (4 bytes): Alt integrator magnitude. Provides information on complex RSSI. Represents either I or Q data depending on how the use_i register is set in the FPGA.

#define    RX_RAM_ADDR_OFFSET_RN16      0        //Store an RN16 in response to a Req_RN.
#define    RX_RAM_ADDR_OFFSET_RN16_I    1        //This is required because the number of received bits is less in the initial RN16 (RN16_I) in a reader-tag exchange than in a regular RN16.
#define    RX_RAM_ADDR_OFFSET_HANDLE    2        //Store a handle in response to a Req_RN.
#define    RX_RAM_ADDR_OFFSET_WRITE     3        //Store a delayed reply in response to a write.
#define    RX_RAM_ADDR_OFFSET_LOCK      4        //Store a delayed reply in response to a lock.
#define    RX_RAM_ADDR_OFFSET_READ      5        //Store data recovered from a tag during a read operation.
#define    RX_RAM_ADDR_OFFSET_PCEPC     7        //Store PC+EPC+CRC data in response to an ACK. 

#define    RX_BITS_RN16                 32        //Tag replies to Req_RN contain a 16-bit handle or new RN16 followed by a CRC-16.
#define    RX_BITS_RN16_I               16        //Tag replies to Query, QueryRep, or QueryAdjust only send back an RN16 value (RN16_I=RN16 Initial).
#define    RX_BITS_HANDLE               32        //Tag replies to Req_RN contain a 16-bit handle or new RN16 followed by a CRC-16.
#define    RX_BITS_WRITE                41        //Remember that this is a delayed reply - 33 bits for a successful reply, 33+8 bits for an unsuccessful reply.
#define    RX_BITS_LOCK                 41        //Remember that this is a delayed reply - 33 bits for a successful reply, 33+8 bits for an unsuccessful reply.
#define    RX_BITS_READ                 129       //(1 bit header+96 bits+16 bit RN+16 bit CRC).
#define    RX_BITS_PCEPC                128       //See Table 6.17 of spec. We get PC(16b)+EPC(96b)+CRC(16b)=128b back.

//This function writes the first byte of every RX RAM memory space so that the RX Data Recovery state machine knows how many bits to look for in the tag reply.
rfidr_error_t load_rfidr_rxram_default(void)
{
    uint16_t        radio_sram_addr             =    0;
    uint8_t         radio_sram_wdata            =    0;
    rfidr_error_t   error_code                  =    0;

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_RN16 << 4;
    radio_sram_wdata    =    RX_BITS_RN16;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_RN16_I  << 4;
    radio_sram_wdata    =    RX_BITS_RN16_I;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_PCEPC << 4;
    radio_sram_wdata    =    RX_BITS_PCEPC;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_HANDLE  << 4;
    radio_sram_wdata    =    RX_BITS_HANDLE;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_WRITE  << 4;
    radio_sram_wdata    =    RX_BITS_WRITE;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_READ << 4;
    radio_sram_wdata    =    RX_BITS_READ;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    radio_sram_addr     =    RX_RAM_ADDR_OFFSET_LOCK  << 4;
    radio_sram_wdata    =    RX_BITS_LOCK;
    error_code=spi_cntrlr_write_tx_robust(RFIDR_RDIO_MEM,RFIDR_SPI_RXRAM,radio_sram_addr,radio_sram_wdata);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Once we get a successful reader-tag transaction, we need to send data back to the iDevice.
//This needs to happen during both search and inventory.
//It's best to have one do-it-all function for both, but this function needs to support rapid-read situations in which phase calibration data is not sent over to the iDevice
rfidr_error_t rfidr_push_data_over_ble(ble_rfidrs_t * p_rfidrs, rfidr_return_t * search_return_ant, rfidr_return_t * search_return_cal, uint8_t frequency_slot, uint8_t num_failed_runs, uint8_t hopskip_nonce, rfidr_ble_push_t ble_push)
{
    static    uint8_t    data_id                                       =    0;          //This is just a nonce variable counting how many times we have sent packets to the iDevice using this function.
    uint8_t              choose_i_cal                                  =    255;        //255 means both I and Q failed. "0" means send Q data, "1" means send I data.
    uint8_t              choose_i_ant                                  =    255;          //255 means both I and Q failed. "0" means send Q data, "1" means send I data.
    uint8_t              pckt_data1[BLE_RFIDRS_PCKT_DATA1_CHAR_LEN]    =    {0};        //Send back information listed below (only 20 bytes available).
    uint8_t              pckt_data2[BLE_RFIDRS_PCKT_DATA2_CHAR_LEN]    =    {0};        //Send back information listed below (only 20 bytes available).
    uint32_t             error_code                                    =    NRF_SUCCESS;
    uint8_t              loop_bytes                                    =    0;

    //As of 112519, we no longer send the error code out routinely with this function call
    //The new packet structure for sending back data is this:
    
    //Packet 1: Main Data - This is the minimal data needed on a rapid-response basis, typically during inventory.
    //Bytes 0-11:     EPC from Ant Run
    //Byte  12:       Frequency Slot - also MSB=1 indicates that a second packet containing supplementary data is forthcoming.
    //Bytes 13-15:    I Magnitude 3 MSB - Ant
    //Bytes 16-18:    Q Magnitude 3 MSB - Ant
    //Byte  19:       Data ID

    //Packet 2: Supplementary Data - This data is required during searches and PDOA ranging. If PDOA is required in inventory, perhaps send this once every Q iteration.
    //Byte  0:        Was Ant I or Q Data Sent?
    //Byte  1:        I Magnitude 1 LSB - Ant    
    //Byte  2:        Q Magnitude 1 LSB - Ant
    //Byte  3:        Was Cal I or Q Data Sent?
    //Bytes 4-7:      I Magnitude - Cal
    //Bytes 8-11:     Q Magnitude - Cal
    //Byte  12:       Num total failed ant runs (4 bits for now, may increase in future). When this number is 255, that means that this run was the initial PDOA run. When less, it was the second PDOA run.
    //Byte  13:       Byte with Passing Information {<Reserved: 4 bits>, I Cal Pass, Q Cal Pass, I Ant Pass, Q Ant Pass}
    //Byte  14:       Hop/skip nonce. We only want to process hop/skip pairs if they are close to each other in time.
    //Byte  15:       Data ID
    
    
    //num_failed_runs does two things. If this value is 255, it means it is a first search run that passed.
    //If this value is less than 255, it means that we are on a secondary run and the value is the number of failed runs that occurred to get to a final acceptable one.

    //Decide which cal magnitude we want to send. We want to pick one that's passed. If both passed, we want to pick the one with larger "main" value.
    if(search_return_cal->i_pass && search_return_cal->q_pass)
        choose_i_cal=(search_return_cal->i_main_mag) > (search_return_cal->q_main_mag);
    else if(search_return_cal->i_pass && !search_return_cal->q_pass) 
        choose_i_cal=1;
    else if(!search_return_cal->i_pass && search_return_cal->q_pass) 
        choose_i_cal=0;
    else 
        choose_i_cal=255; //We shouldn't be here, but this signifies a failure
    
    //Decide which ant magnitude we want to send. We want to pick one that's passed. If both passed, we want to pick the one with larger "main" value.
    if(search_return_ant->i_pass && search_return_ant->q_pass)
        choose_i_ant=(search_return_ant->i_main_mag) > (search_return_ant->q_main_mag);
    else if(search_return_ant->i_pass && !search_return_ant->q_pass) 
        choose_i_ant=1;
    else if(!search_return_ant->i_pass && search_return_ant->q_pass) 
        choose_i_ant=0;
    else
        choose_i_ant=255; //We shouldn't be here, but this signifies a failure

    //Load EPC bytes into pckt_data1
    for(loop_bytes=0;loop_bytes < MAX_EPC_LENGTH_IN_BYTES;loop_bytes++)
    {
        if(choose_i_ant==1)
            pckt_data1[loop_bytes]=search_return_ant->i_epc[loop_bytes];
        else if(choose_i_ant==0) 
            pckt_data1[loop_bytes]=search_return_ant->q_epc[loop_bytes];
        else 
            pckt_data1[loop_bytes]=0x00; //This should never happen - rfidr_state should report a fail before this happens.
    }
    
    //Load the recovered frequency slot into pckt_data1
    //Since we will ever at most use 6 bits for the frequency slot, we can also use the MSB to communicate whether or not a second packet will follow.
    //Note that the second packet is currently a notification so it may get lost.
    pckt_data1[MAX_EPC_LENGTH_IN_BYTES]= (frequency_slot & 127) | ((ble_push==BLE_PUSH_SUPPLEMENT ? 1 : 0) << 7);
    
    //Send the magnitude MSB for data recovered over the antenna.
    for(loop_bytes=0;loop_bytes < 3;loop_bytes++)
    {
        if(choose_i_ant==1)
        {
            pckt_data1[MAX_EPC_LENGTH_IN_BYTES+1+loop_bytes]=(uint8_t)(((uint32_t)(search_return_ant->i_main_mag) >> ((3-loop_bytes) << 3)) & 255); //I magnitude data
            pckt_data1[MAX_EPC_LENGTH_IN_BYTES+4+loop_bytes]=(uint8_t)(((uint32_t)(search_return_ant->i_alt_mag) >> ((3-loop_bytes) << 3)) & 255); //Q magnitude data
        }
        else if(choose_i_ant==0)
        {
            pckt_data1[MAX_EPC_LENGTH_IN_BYTES+1+loop_bytes]=(uint8_t)(((uint32_t)(search_return_ant->q_alt_mag) >> ((3-loop_bytes) << 3)) & 255); //I magnitude data
            pckt_data1[MAX_EPC_LENGTH_IN_BYTES+4+loop_bytes]=(uint8_t)(((uint32_t)(search_return_ant->q_main_mag) >> ((3-loop_bytes) << 3)) & 255); //Q magnitude data
        }
        else 
        {
            pckt_data1[MAX_EPC_LENGTH_IN_BYTES+1+loop_bytes]=0;    //This should not happen - leave a clue that something odd has happened.
            pckt_data1[MAX_EPC_LENGTH_IN_BYTES+4+loop_bytes]=0;    //This should not happen - leave a clue that something odd has happened.
        }
    }
    //Send the data ID for data tracking purposes on the iDevice.
    pckt_data1[MAX_EPC_LENGTH_IN_BYTES+7]= data_id;
    
    //Send first packet back to the iDevice over BTLE. Keep trying to send the packet if the FIFO buffer for BTLE packets is full.
    do
    {
        error_code=ble_rfidrs_pckt_data1_send(p_rfidrs, pckt_data1, BLE_RFIDRS_PCKT_DATA1_CHAR_LEN);
    } while(error_code == BLE_ERROR_NO_TX_BUFFERS);

    if (error_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(error_code);
    }

    //Increment the data ID nonce variable.
    data_id++;

    if(ble_push==BLE_PUSH_SUPPLEMENT) //Send the second packet as part of sending data over BLE.
    {
        
        //Tell the iDevice which data is being used - I or Q
        pckt_data2[0]=choose_i_ant;
        pckt_data2[3]=choose_i_cal;
        pckt_data2[12]=num_failed_runs; //Yes this is sort of repeated. Oh well.
        pckt_data2[13]= (search_return_cal->i_pass << 3) | (search_return_cal->q_pass << 2) | (search_return_ant->i_pass << 1) | (search_return_ant->q_pass << 0);
        pckt_data2[14]=hopskip_nonce;
        pckt_data2[15]=data_id;
    
        //Send the appropriate magnitude data over to the iDevice
    
        loop_bytes=3;    //Should be this already, but let's make sure.
            
        if(choose_i_ant==1)
        {
            pckt_data2[1]=(uint8_t)(((uint32_t)(search_return_ant->i_main_mag) >> ((3-loop_bytes) << 3)) & 255); //I magnitude data
            pckt_data2[2]=(uint8_t)(((uint32_t)(search_return_ant->i_alt_mag) >> ((3-loop_bytes) << 3)) & 255); //Q magnitude data
        }
        else if(choose_i_ant==0)
        {
            pckt_data2[1]=(uint8_t)(((uint32_t)(search_return_ant->q_alt_mag) >> ((3-loop_bytes) << 3)) & 255); //I magnitude data
            pckt_data2[2]=(uint8_t)(((uint32_t)(search_return_ant->q_main_mag) >> ((3-loop_bytes) << 3)) & 255); //Q magnitude data
        }
        else
        {
            pckt_data2[1]=0; //This should not happen - leave a clue that something odd has happened.
            pckt_data2[2]=0; //This should not happen - leave a clue that something odd has happened.
        }
    
        for(loop_bytes=0;loop_bytes < 4;loop_bytes++)
        {
            if(choose_i_cal==1)
            {
                pckt_data2[loop_bytes+4]=(uint8_t)(((uint32_t)(search_return_cal->i_main_mag) >> ((3-loop_bytes) << 3)) & 255); //I magnitude data
                pckt_data2[loop_bytes+8]=(uint8_t)(((uint32_t)(search_return_cal->i_alt_mag) >> ((3-loop_bytes) << 3)) & 255); //Q magnitude data
            }
            else if(choose_i_cal==0)
            {
                pckt_data2[loop_bytes+4]=(uint8_t)(((uint32_t)(search_return_cal->q_alt_mag) >> ((3-loop_bytes) << 3)) & 255); //I magnitude data
                pckt_data2[loop_bytes+8]=(uint8_t)(((uint32_t)(search_return_cal->q_main_mag) >> ((3-loop_bytes) << 3)) & 255); //Q magnitude data
            }
            else 
            {
                pckt_data2[loop_bytes+4]=0; //This should not happen - leave a clue that something odd has happened.
                pckt_data2[loop_bytes+8]=0; //This should not happen - leave a clue that something odd has happened.
            }
        }
        
        //Send second packet back to the iDevice over BTLE. Keep trying to send the packet if the FIFO buffer for BTLE packets is full.
        do
        {
            error_code=ble_rfidrs_pckt_data2_send(p_rfidrs, pckt_data2, BLE_RFIDRS_PCKT_DATA2_CHAR_LEN);
        } while(error_code == BLE_ERROR_NO_TX_BUFFERS);

        if (error_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(error_code);
        }
        
        //Increment the data ID nonce variable.
        data_id++;
    }
    
    

    return RFIDR_SUCCESS;
}

//051519 - This code was added in late 2017 to add some robustness to the programming operation,
//by allowing a readback of the EPC value that was just programmed to the tag.
//This code was copied over from another function and should be revised at some point, for example it does not require a ble_rfidrs_t argument

rfidr_error_t rfidr_pull_and_check_read_data(rfidr_target_epc_t target)
{
    uint16_t    radio_sram_addr                                 =    (RX_RAM_ADDR_OFFSET_READ << 4);    //Read from the "READ" section of RX RAM - after a read command is issued.
    uint8_t     loop_bytes                                      =    0;                                 //A loop variable.
    uint8_t     recovery_byte                                   =    0;                                 //A storage byte that we use to recover data from memory.
    uint8_t     recovered_epc_bytes[MAX_EPC_LENGTH_IN_BYTES]    =    {0};                               //The EPC data read back from the tag.
    uint8_t     original_epc_bytes[MAX_EPC_LENGTH_IN_BYTES]     =    {0};                               //The EPC data that was intended to be programmed onto the tag.
    uint64_t    recovered_epc_bits_msb                          =    0;                                 //The most significant 64 bits (8 bytes) of the EPC data recovered from the tag.
    uint32_t    recovered_epc_bits_lsb                          =    0;                                 //The least significant 32 bits (4 bytes) of the EPC data recovered from the tag.

    //Get EPC bits, ignoring header, RN and CRC bits
    //The complication with doing with with a read is that there is a 1 bit header in front of the read
    //data. So the bytes that we pull out have to be re-formed.

    radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_READ << 4)+1;    //We require an offset of 1 to start the memory read after the byte containing the #bits to expect in the tag reply.
    spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr++, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);

    //Get the first 7 bits and start assembling the EPC
    recovered_epc_bits_msb |= (uint64_t)(recovery_byte & 127) << 57;

    //Get the next 56 bits (7 bytes) and continue assembling the EPC
    for(loop_bytes=0;loop_bytes < 7;loop_bytes++)
    {
        spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr++, 0);
        spi_cntrlr_send_recv();
        spi_cntrlr_read_rx(&recovery_byte);
        recovered_epc_bits_msb |= (uint64_t)(recovery_byte & 255) << (49-(loop_bytes<<3));
    }

    //Get the final bit to be included in the 64 most significant bits.
    spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr++, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovered_epc_bits_msb |= (uint64_t)(recovery_byte >> 7) & 1;

    //And get the MSB of the 32 LSB and continue assembling the EPC.
    recovered_epc_bits_lsb |= (uint32_t)(recovery_byte & 127) << 25;

    //Get the next 24 bits of the LSB continue assembling the EPC.
    for(loop_bytes=0;loop_bytes < 3;loop_bytes++)
    {
        spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr++, 0);
        spi_cntrlr_send_recv();
        spi_cntrlr_read_rx(&recovery_byte);
        recovered_epc_bits_lsb |= (uint32_t)(recovery_byte & 255) << (17-(loop_bytes<<3));
    }

    //Get the final bit to be included in the 32-bit LSB and finish assembling the EPC.
    spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr++, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovered_epc_bits_lsb |= (uint32_t)(recovery_byte >> 7) & 1;

    //Build the bytes up in an array - msb
    for(loop_bytes=0;loop_bytes < 8;loop_bytes++)
    {
        recovered_epc_bytes[loop_bytes]=(uint8_t)((recovered_epc_bits_msb >> ((7-loop_bytes) << 3)) & 255);
    }
    //Build the bytes up in an array - lsb
    for(loop_bytes=8;loop_bytes < 12;loop_bytes++)
    {
        recovered_epc_bytes[loop_bytes]=(uint8_t)((recovered_epc_bits_lsb >> ((11-loop_bytes) << 3)) & 255);
    }

    //Pull out the stored new_epc from the txradio module
    switch(target)
    {
        case TARGET_APP_SPECD_EPC:    read_app_specd_program_epc(original_epc_bytes); break;
        case TARGET_LAST_INV_EPC:     read_last_inv_epc(original_epc_bytes); break;
        default:                      read_app_specd_program_epc(original_epc_bytes); break;
    }

    if(memcmp(recovered_epc_bytes,original_epc_bytes,MAX_EPC_LENGTH_IN_BYTES))
    {
        return RFIDR_ERROR_READ_CHECK;
    }

    return RFIDR_SUCCESS;

}

rfidr_error_t rfidr_read_main_magnitude(int32_t * main_magnitude, rfidr_read_rxram_type_t read_type)
{
    uint16_t        radio_sram_addr     =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+2; //The address of the main path magnitude in RX RAM when PCEPC data is received.
    uint8_t         loop_bytes          =    0;    //Note - the offset of 2 above is only in the case of initialization - this won't need to change with a dummy
    uint8_t         recovery_byte[4]    =    {0};    //tag provided that we address it with the wrong address during initialization (we do - we use 0 EPC).

    //Get Main magnitude bits
    //Data was loaded into FPGA RX RADIO RAM as MagMain[LSByte] MagMain[LSByte-1] MagMain[MSByte-1] MagMain[MSByte] MagAlt[LSByte] MagAlt[LSByte-1] MagAlt[MSByte-1] MagAlt[MSByte]
    *main_magnitude = 0;
    for(loop_bytes=0;loop_bytes < 4;loop_bytes++)
    {
        //radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+2+loop_bytes+(read_type==READ_RXRAM_REGULAR ? 16 : 0); //Note this offset of 2 is only for the case of initialization where there are no CRC bits
        radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+18+loop_bytes;
        spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr, 0);
        spi_cntrlr_send_recv();
        spi_cntrlr_read_rx(&recovery_byte[loop_bytes]);
    }

    *main_magnitude = (int32_t)(((uint32_t)recovery_byte[0] << 0)+((uint32_t)recovery_byte[1] << 8)+((uint32_t)recovery_byte[2] << 16)+((uint32_t)recovery_byte[3] << 24));

    return RFIDR_SUCCESS;
}

rfidr_error_t rfidr_read_alt_magnitude(int32_t * alt_magnitude, rfidr_read_rxram_type_t read_type)
{
    uint16_t        radio_sram_addr     =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+6; //The address of the alt path magnitude in RX RAM when PCEPC data is received.
    uint8_t         loop_bytes          =    0; //Note - the offset of 6 above is only in the case of initialization - this won't need to change with a dummy
    uint8_t         recovery_byte[4]    =    {0}; //tag provided that we address it with the wrong address during initialization (we do - we use 0 EPC).

    //Get Alt magnitude bits
    //Data was loaded into FPGA RX RADIO RAM as MagMain[LSByte] MagMain[LSByte-1] MagMain[MSByte-1] MagMain[MSByte] MagAlt[LSByte] MagAlt[LSByte-1] MagAlt[MSByte-1] MagAlt[MSByte]
    *alt_magnitude=0;
    for(loop_bytes=0;loop_bytes < 4;loop_bytes++)
    {
        //radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+6+loop_bytes+(read_type==READ_RXRAM_REGULAR ? 16 : 0); //Note this offset of 6 is only for the case of initialization where there are no CRC bits
        radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+22+loop_bytes;
        spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr, 0);
        spi_cntrlr_send_recv();
        spi_cntrlr_read_rx(&recovery_byte[loop_bytes]);
    }

    *alt_magnitude = (int32_t)(((uint32_t)recovery_byte[0] << 0)+((uint32_t)recovery_byte[1] << 8)+((uint32_t)recovery_byte[2] << 16)+((uint32_t)recovery_byte[3] << 24));

    return RFIDR_SUCCESS;
}

rfidr_error_t rfidr_read_epc(uint8_t * epc, rfidr_read_rxram_type_t read_type)
{
    uint16_t         radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+3;
    uint8_t          loop_bytes         =    0;
    uint8_t          recovery_byte      =    0;
    rfidr_error_t    error_code         =    RFIDR_SUCCESS;
    
    //Get EPC bits, ignoring PC and CRC bits, from RX RAM and load them into the buffer for containing the first BTLE packet back to the iDevice.
    for(loop_bytes=0;loop_bytes < MAX_EPC_LENGTH_IN_BYTES;loop_bytes++)
    {
        if(read_type==READ_RXRAM_REGULAR)
        {
            radio_sram_addr    =    (RX_RAM_ADDR_OFFSET_PCEPC << 4)+3+loop_bytes;
            spi_cntrlr_set_tx(RFIDR_RDIO_MEM, RFIDR_SPI_READ, RFIDR_SPI_RXRAM, radio_sram_addr, 0);
            spi_cntrlr_send_recv();
            spi_cntrlr_read_rx(&recovery_byte);
            *(epc+loop_bytes)=recovery_byte;
        }
        else
            *(epc+loop_bytes)=0x00;    
    }
    
    return error_code;
}
