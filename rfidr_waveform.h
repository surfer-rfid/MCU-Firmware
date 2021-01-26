//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware Waveform Operations                                  //
//                                                                              //
// Filename: rfidr_waveform.h                                                   //
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

//rfidr_waveform RFIDR Waveform memory interface Interface and Driver Functions
// RFIDR waveform memory access functionality
//This file provides high level abstraction to accessing the RFIDR waveform memory

#ifndef RFIDR_WAVEFORM_H__
#define RFIDR_WAVEFORM_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "rfidr_error.h"
#include "ble_rfidrs.h"

rfidr_error_t    rfidr_push_waveform_data_over_ble(ble_rfidrs_t * p_rfidrs);
rfidr_error_t    set_waveform_offset(uint8_t offset);


#endif