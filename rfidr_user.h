////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
// Module : RFIDr Firmware User functions                                         //
//                                                                                //
// Filename: rfidr_user.h                                                         //
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
//    This file contains a number of miscellaneous medium level "user" functions  //
//    that can be made accessible across the code space. Each of these functions  //
//    accomplishes a simple task with respect to the FPGA, such as setting        //
//    the CDR circuit to examine I or Q data, or to check whether the radio is    //
//    running or not.                                                             //
//                                                                                //
//    There are also a few secondary GPIO event handlers in here which interact   //
//    directly with the FPGA, such as sample_received_irq, which increments one   //
//    of the TMN DTC state variables during TESTING_DTC.                          //
//                                                                                //
//                                                                                //
//    Revisions:                                                                  //
//    061819 - Major commentary cleanup.                                          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

#ifndef RFIDR_USER_H__
#define RFIDR_USER_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "rfidr_error.h"

rfidr_error_t    enter_dtc_test_mode(void);
rfidr_error_t    exit_dtc_test_mode(void);
rfidr_error_t    pwr_togl_received_irq(void);
rfidr_error_t    sample_received_irq(void);
rfidr_error_t    cycle_received_irq(void);

bool             is_radio_done(void);
bool             is_radio_running(void);
bool             is_clk_36_running(void);
bool             is_clk_36_valid(void);
uint8_t          read_radio_exit_code(void);
uint8_t          read_radio_write_cntr(void);

rfidr_error_t    set_go_radio_oneshot(void);
rfidr_error_t    set_irq_ack_oneshot(void);
rfidr_error_t    set_clk_36_oneshot(void);
rfidr_error_t    set_sw_reset(void);
rfidr_error_t    set_use_i(void);
rfidr_error_t    set_use_q(void);
rfidr_error_t    set_use_kill_pkt(void);
rfidr_error_t    set_use_select_pkt(void);
rfidr_error_t    set_alt_radio_fsm_loop(void);
rfidr_error_t    set_end_radio_fsm_loop(void);
rfidr_error_t    clear_query_inventory(void);
rfidr_error_t    set_radio_mode_search(void);
rfidr_error_t    set_radio_mode_inventory(void);
rfidr_error_t    set_radio_mode_prog_cfm(void);
rfidr_error_t    set_radio_mode_program(void);
rfidr_error_t    set_sx1257_pll_chk_mode(void);
rfidr_error_t    unset_sx1257_pll_chk_mode(void);
rfidr_error_t    set_tx_sdm_offset(uint8_t offset);
rfidr_error_t    set_tx_zgn_offset(uint8_t offset);

#endif