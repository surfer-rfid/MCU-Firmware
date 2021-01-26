//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware Error handling                                       //
//                                                                              //
// Filename: rfidr_error.h                                                      //
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
//    This file contains code dedicated to error handling in the MCU firmware   //
//                                                                              //
//    Revisions:                                                                //
//    061919 - Major commentary cleanup.                                        //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

//rfidr_error RFIDR Error definitions and handling
//RFIDR Error definitions and handling
//This file provides high level abstraction to RFIDR error handling functionality

#ifndef RFIDR_ERROR_H__
#define RFIDR_ERROR_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "ble_rfidrs.h"

#define MAX_EPC_LENGTH_IN_BYTES            12

typedef enum
{
  RFIDR_SUCCESS,
  RFIDR_ERROR_SPI_WRITE_TX,
  RFIDR_ERROR_SPI_WRITE_SX1257_1,
  RFIDR_ERROR_SPI_WRITE_SX1257_2,
  RFIDR_ERROR_SPI_WRITE_SX1257_3,
  RFIDR_ERROR_SPI_WRITE_SX1257_4,
  RFIDR_ERROR_SPI_WRITE_SX1257_5,
  RFIDR_ERROR_BLE_PCKT1_SEND,
  RFIDR_ERROR_BLE_PCKT2_SEND,
  RFIDR_ERROR_READ_CHECK,
  RFIDR_ERROR_WAVE_MEM_1,
  RFIDR_ERROR_WAVE_MEM_2,
  RFIDR_ERROR_USER_MEM,
  RFIDR_ERROR_GENERAL
}rfidr_error_t;

uint32_t    rfidr_error_complete_message_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string);

#endif
