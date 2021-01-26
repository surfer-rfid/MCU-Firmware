//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware State                                                //
//                                                                              //
// Filename: rfidr_state.h                                                      //
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
//    This file contains the main application-level behavior of the RFIDr MCU   //
//    firmware, including the state machine that gets executed whenever the     //
//    iDevice software prompts for a state transition.                          //
//                                                                              //
//    Revisions:                                                                //
//    061619 - Major commentary cleanup.                                        //
//    121319 - Major code cleanup to remove repeated code in several "states".  //
//    Now each state core has its own function which is called in the "state".  //
//    Support for frequency hopping, PDOA ranging, and kill functionality are   //
//    added in. Have only one error handling function now that checks state.    //
//    Also, more robust programming and searching/programming last inventoried  //
//    flag functionalities are added in.                                        //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

#ifndef RFIDR_STATE_H__
#define RFIDR_STATE_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "ble_rfidrs.h"
#include "rfidr_error.h"

typedef enum
{
  IDLE_UNCONFIGURED,
  IDLE_CONFIGURED,
  INITIALIZING,
  SEARCHING_APP_SPECD_TAG,
  SEARCHING_LAST_INV_TAG,
  INVENTORYING,
  TESTING_DTC,
  PROGRAMMING_APP_SPECD_TAG,
  PROGRAMMING_LAST_INV_TAG,
  RECOVERING_WAVEFORM_MEMORY,
  RESET_SX1257_AND_FPGA,
  KILL_TAG,
  PROGRAMMING_KILL_PASSWD,
  TRACK_APP_SPECD_TAG,
  TRACK_LAST_INV_TAG
} rfidr_state_t;

typedef enum
{
    RETURN_EPC_NO = 0,
    RETURN_EPC_YES = 1
} return_epc_t;

typedef enum
{
    RETURN_MAG_NO = 0,
    RETURN_MAG_YES = 1
} return_mag_t;

typedef enum
{
    RETURN_LNA_GAIN_NO = 0,
    RETURN_LNA_GAIN_YES = 1
} return_lna_gain_t;

typedef enum
{
    TRACK_APP_SPECD = 0,
    TRACK_LAST_INV = 1
} rfidr_tracking_mode_t;

typedef enum
{
    PROGRAM_NEW_EPC,
    PROGRAM_KILL_PWD,
    KILL_COMMAND
} rfidr_program_content_t;

void        update_adc_sample(int32_t adc_sample);

void        rfidr_state_init(void);

void        rfidr_state_received_irq(void);

void        rfidr_state_received_read_state_confirmation(void);

void        rfidr_state_received_pckt_data1_confirmation(void);

uint32_t    write_rfidr_state_next(ble_rfidrs_t *p_rfidrs, rfidr_state_t    l_rfidr_state_next);

uint32_t    read_rfidr_state(rfidr_state_t * p_rfidr_state);

void        run_rfidr_state_machine(ble_rfidrs_t *p_rfidrs);

#endif
