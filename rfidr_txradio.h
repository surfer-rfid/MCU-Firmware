//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware TX Radio RAM loading                                 //
//                                                                              //
// Filename: rfidr_txradio.h                                                    //
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
//    This file contains code required for loading the TX RAM on the RFIDr      //
//    FPGA. This file also contains code related to EPCs to be transmitted.     //
//                                                                              //
//    Revisions:                                                                //
//    061619 - Major commentary cleanup.                                        //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

//rfidr_txradio RFIDR TX Radio Interface and Driver Functions
//RFIDR TX radio functionality
//This file provides high level abstraction to programming the RFIDR TX radio, mimicking
//the SPI driver functions from the top level simulation verilog testbenches.


#ifndef RFIDR_TXRADIO_H__
#define RFIDR_TXRADIO_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "ble_rfidrs.h"
#include "rfidr_error.h"

//write offset type

typedef enum
{
  WRITE_OFFSET_0 = 0,
  WRITE_OFFSET_1,
  WRITE_OFFSET_2,
  WRITE_OFFSET_3,
  WRITE_OFFSET_4,
  WRITE_OFFSET_5
} rfidr_write_offset_t;

//last write type

typedef enum
{
  LAST_WRITE_NO = 0,
  LAST_WRITE_YES
} rfidr_last_write_t;

//write mode type

typedef enum
{
  WRITE_WRITE_MODE = 0,
  KILL_WRITE_MODE
} rfidr_write_mode_t;

//RFID tag membank type

typedef enum
{
  MEMBANK_0 = 0,
  MEMBANK_1,
  MEMBANK_2,
  MEMBANK_3
} rfidr_membank_t;

//Select epc type

typedef enum
{
  APP_SPECD_EPC,
  FMW_SPECD_EPC,
  LAST_INV_EPC,
  DUMMYTAG_EPC,
  ZERO_EPC
} rfidr_select_epc_type_t;

//Select packet type

typedef enum
{
  SEL_PACKET_NO_1,
  SEL_PACKET_NO_2,
} rfidr_select_packet_type_t;

//Select target type

typedef enum
{
  TARGET_S0,
  TARGET_S1,
  TARGET_S2,
  TARGET_S3,
  TARGET_SL
} rfidr_select_target_t;

//Select action type

typedef enum
{
  ACTION_A0,
  ACTION_A1,
  ACTION_A2,
  ACTION_A3,
  ACTION_A4,
  ACTION_A5,
  ACTION_A6,
  ACTION_A7
} rfidr_select_action_t;

//Query sel type

typedef enum
{
  SEL_ALL,
  SEL_NSL,
  SEL_PSL
} rfidr_query_sel_t;

//Query session type

typedef enum
{
  SESSION_S0,
  SESSION_S1,
  SESSION_S2,
  SESSION_S3
} rfidr_query_session_t;

//Query target type

typedef enum
{
  TARGET_A,
  TARGET_B
} rfidr_query_target_t;

//Query flagswap target type

typedef enum
{
  FLAGSWAP_NO = 0,
  FLAGSWAP_YES = 1
} rfidr_query_flagswap_t;

//RFIDr Select Raw structure
//This structure contains the 28 bits in a select message, exclusive of the 96 EPC bits and the 1 truncation bit

typedef struct
{
    uint8_t                  command;
    rfidr_select_target_t    target;
    rfidr_select_action_t    action;
    uint8_t                  membank;
    uint8_t                    pointer;
//    uint8_t                length; //Deprecated this on 12/3/2019 - we'll feed length into the load_select function at the same time as the epc
} rfidr_select_raw_t;

//RFIDr Query Raw structure
//This structure contains the 22 bits in a query message, exclusive of the 5 CRC bits

typedef struct
{
    uint8_t                    command;
    uint8_t                    dr;
    uint8_t                    mod_index;
    uint8_t                    trext;
    rfidr_query_sel_t          sel;
    rfidr_query_session_t      session;
    rfidr_query_target_t       target;
    uint8_t                    query_q;
} rfidr_query_raw_t;

//function for initializing state variables in rfidr_txradio
//returns RFIDR_SUCCESS on successful field set

uint32_t rfidr_txradio_init(void);

//function for setting the app-specified target epc field. Called by target epc data handler. Also send over length.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_app_specd_target_epc(ble_rfidrs_t * p_rfidrs, const uint8_t * p_app_specd_target_epc, uint8_t length);

//function for setting the software-specified target epc field.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_fmw_specd_target_epc(char * epc);

//function for setting the private new epc field. Called by new epc data handler.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_app_specd_program_epc(ble_rfidrs_t * p_rfidrs, const uint8_t * p_app_specd_program_epc);

//function for setting the last inventoried EPC field. Called in rfidr_state.c.

rfidr_error_t set_last_inv_epc(const uint8_t * p_last_inv_epc);

//function for reading the app-specified target epc field. Called by target epc data handler.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t read_app_specd_target_epc(uint8_t * p_app_specd_target_epc);

//function for reading the length of the app-specified target epc field.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t read_length_app_specd_target_epc(uint8_t * p_length_app_specd_target_epc);

//function for reading the length of the software-specified target epc field.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t read_length_fmw_specd_target_epc(uint8_t * p_length_sfw_specd_target_epc);

//function for reading the private new epc field. Called by new epc data handler.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t read_app_specd_program_epc(uint8_t * p_app_specd_program_epc);

//function for reading the last inventoried EPC field. Called in rfidr_state.c.

rfidr_error_t read_last_inv_epc(uint8_t * p_last_inv_epc);

//function for setting the select target field prior to programming the TX RAM
//returns RFIDR_SUCCESS on successful field set

rfidr_error_t set_select_target(rfidr_select_target_t target);

//function for setting the select action field prior to programming the TX RAM
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_select_action(rfidr_select_action_t action);

//function for setting the query sel field prior to programming the TX RAM
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_query_sel(rfidr_query_sel_t sel);

//function for setting the query session field prior to programming the TX RAM
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_query_session(rfidr_query_session_t session);

//function for setting the query target field prior to programming the TX RAM
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_query_target(rfidr_query_target_t target);

//function for setting the Q field prior to programming the TX RAM
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t set_query_q(uint8_t query_q);

//function for loading the select packet based on one of several EPC stored as state variables in this function.
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_select_packet_only(rfidr_select_epc_type_t epc_type, uint8_t epc_length_in_bytes, rfidr_select_packet_type_t packet_type);

//function for loading the minimum possible select packet in place to make searches go as fast as possible (assuming we even need the select at all)
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_dummy_select_packet_only(void);

//function for loading just the second select packet with the target epc in place
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_query_packet_only(rfidr_query_flagswap_t flagswap);

//function for loading just the query rep packet
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_query_rep_packet(void);

//function for loading a query adjust packet in the query rep FPGA RAM space
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_query_adj_packet(bool up_downb);

//function for loading just the write packet with the new epc in place
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_write_packet_only_program_epc(void);

//function for loading the write packet SRAM with the kill password
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_write_packet_only_kill_password(void);

//function for loading the write packet SRAM with the kill command
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_write_packet_only_kill_command(void);

//function for loading a complete default image of the TX RADIO RAM
//In this case, we will select using blank epcs and have query by default look for selected flags
//returns RFIDR_SUCCESS on successful field buffer set

rfidr_error_t load_rfidr_txram_default(void);

#endif
