//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware RX Radio RAM loading                                 //
//                                                                              //
// Filename: rfidr_rxradio.h                                                    //
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

#ifndef RFIDR_RXRADIO_H__
#define RFIDR_RXRADIO_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "rfidr_error.h"

typedef struct
{
    bool           i_pass;
    bool           q_pass;
    uint8_t        i_epc[MAX_EPC_LENGTH_IN_BYTES];
    uint8_t        q_epc[MAX_EPC_LENGTH_IN_BYTES];
    uint8_t        i_lna_gain;
    uint8_t        q_lna_gain;
    int32_t        i_main_mag;
    int32_t        i_alt_mag;
    int32_t        q_main_mag;
    int32_t        q_alt_mag;
} rfidr_return_t;

typedef enum
{
    TARGET_APP_SPECD_EPC,
    TARGET_LAST_INV_EPC,
    TARGET_CAL_EPC,
    TARGET_PLL_EPC
} rfidr_target_epc_t;

typedef enum{
    BLE_PUSH_MINIMAL,
    BLE_PUSH_SUPPLEMENT
} rfidr_ble_push_t;

typedef enum{
    READ_RXRAM_PLLCHECK,
    READ_RXRAM_REGULAR
} rfidr_read_rxram_type_t;

//function for initializing state variables in rfidr_txradio
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t load_rfidr_rxram_default(void);

//function for pushing data over bluetooth LE back to the phone
//This will need to happen every time we get a PCEPC done IRQ event
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t rfidr_push_data_over_ble(ble_rfidrs_t * p_rfidrs, rfidr_return_t * search_return_ant, rfidr_return_t * search_return_cal, uint8_t recover_frequency_slot, uint8_t num_failed_runs, uint8_t hopskip_nonce, rfidr_ble_push_t ble_push);

//function for pulling read data back from the tag to compare with the epc we intended to write
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t rfidr_pull_and_check_read_data(rfidr_target_epc_t target);

//function for pulling magnitude data back from the RX and returning the magnitude for the main integrator path
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t rfidr_read_main_magnitude(int32_t * main_magnitude, rfidr_read_rxram_type_t read_type);

//function for pulling magnitude data back from the RX and returning the magnitude for the alternate integrator path
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t rfidr_read_alt_magnitude(int32_t * alt_magnitude, rfidr_read_rxram_type_t read_type);

//function for pulling epc data back from the RX
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t rfidr_read_epc(uint8_t * epc, rfidr_read_rxram_type_t read_type);

#endif
