//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware SX1257 SDR ASIC Drivers                              //
//                                                                              //
// Filename: rfidr_sx1257.h                                                     //
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
//    This file contains driver code for operating the SX1257 SDR ASIC chip.    //
//                                                                              //
//    Revisions:                                                                //
//    061919 - Major commentary cleanup.                                        //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

//rfidr_sx1257 RFIDR SX1257 Interface and Driver Functions
//RFIDR SX1257 functionality
//This file provides high level abstraction to programming the RFIDR SX1257

#ifndef RFIDR_SX1257_H__
#define RFIDR_SX1257_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "rfidr_error.h"

//function for a default load of the SX1257
//returns RFIDR_SUCCESS on successful load of the SX1257

rfidr_error_t load_sx1257_default(void);

//function for setting the SX1257 rx lna gain explicitly
//returns RFIDR_SUCCESS on successful set of SX1257 rx lna gain

rfidr_error_t set_sx1257_lna_gain(uint8_t lna_gain);

//function for checking the SX1257 rx lna gain
//returns RFIDR_SUCCESS on successful set of SX1257 rx lna gain

rfidr_error_t get_sx1257_lna_gain(uint8_t * lna_gain);

//function for setting the SX1257 rx gain high
//return RFIDR_SUCCESS on successful set of SX1257 gain high

rfidr_error_t set_sx1257_rx_gain_high(void);

//function for setting the SX1257 rx gain medium
//return RFIDR_SUCCESS on successful set of SX1257 gain medium

rfidr_error_t set_sx1257_rx_gain_med(void);

//function for setting the SX1257 rx gain low
//return RFIDR_SUCCESS on successful set of SX1257 gain low

rfidr_error_t set_sx1257_rx_gain_low(void);

//function for attempting to fix pll lock
//return RFIDR_SUCCESS on successful set of SX1257 pll lock

rfidr_error_t set_sx1257_fix_tones(void);

//function for setting the SX1257 tx power high
//return RFIDR_SUCCESS on successful set of SX1257 tx power high

rfidr_error_t set_sx1257_tx_power_high(void);

//function for setting the SX1257 tx power med
//return RFIDR_SUCCESS on successful set of SX1257 tx power med

rfidr_error_t set_sx1257_tx_power_med(void);

//function for setting the SX1257 tx power low
//return RFIDR_SUCCESS on successful set of SX1257 tx power low

rfidr_error_t set_sx1257_tx_power_low(void);

//function for hopping the SX1257 frequency
//return RFIDR_SUCCESS on successful hop of SX1257 frequency

rfidr_error_t hop_sx1257_frequency(uint8_t * p_sx1257_frequency_slot);

//function for setting the SX1257 frequency
//return RFIDR_SUCCESS on successful hop of SX1257 frequency

rfidr_error_t set_sx1257_frequency(uint8_t sx1257_frequency_slot);

#endif
