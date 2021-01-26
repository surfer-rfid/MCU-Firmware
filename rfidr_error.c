//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware Error handling                                       //
//                                                                              //
// Filename: rfidr_error.c                                                      //
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

#include "app_error.h"
#include "app_util_platform.h"
#include "ble_rfidrs.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "rfidr_error.h"
#include <string.h>
#include <stdio.h>

//#define NULL 0

static    uint32_t            m_num_failed_attempts                        =    0;
static    bool                m_failed_attempt_flag                        =    false;

//This function sends a string of arbitrary length to the iDevice as an error message.
//This is the accepted method for passing C-string to a function: http://stackoverflow.com/questions/17131863/passing-string-to-a-function-in-c-with-or-without-pointers
uint32_t rfidr_error_complete_message_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string)
{
    //We go through the string looking for the NULL character at the end. When we see it, we send the remainder of the bits

    uint8_t        error_code                                          =    NRF_SUCCESS;
    uint8_t        p_string_segment[BLE_RFIDRS_LOG_MESSGE_CHAR_LEN]    =    {0};
    char           e_string_segment[BLE_RFIDRS_LOG_MESSGE_CHAR_LEN]    =    {0};
    uint8_t        counter                                             =    0;
    uint16_t       overall_counter                                     =    0;

    while(*(p_string+overall_counter) != 0)
    {

        p_string_segment[counter]    =    *(p_string+overall_counter);

        counter++;
        overall_counter++;
        if(counter==BLE_RFIDRS_LOG_MESSGE_CHAR_LEN)
        {
            error_code=ble_rfidrs_log_messge_send(p_rfidrs, p_string_segment, counter);
            if (error_code != NRF_ERROR_INVALID_STATE)
            {
                m_failed_attempt_flag    =    true;
                m_num_failed_attempts++;
            }
            counter    =    0;
        }
    }
    //We've reached a NULL character. Add this character to the array, making sure null character is sent.

    p_string_segment[counter]    =    *(p_string+overall_counter);
    counter++;
    overall_counter++;

    error_code=ble_rfidrs_log_messge_send(p_rfidrs, p_string_segment, counter);
    if (error_code != NRF_ERROR_INVALID_STATE) {
        m_failed_attempt_flag    =    true;
        m_num_failed_attempts++;
    } 
    else 
    {
        if(m_failed_attempt_flag == true) 
        {
            memset(e_string_segment, '\0', sizeof(e_string_segment));
            strncpy(e_string_segment,"Rcvrd. BT ops after",19);
            error_code=ble_rfidrs_log_messge_send(p_rfidrs, (uint8_t *)e_string_segment, 20);
            if (error_code != NRF_ERROR_INVALID_STATE) 
            {
                m_failed_attempt_flag    =    true;
                m_num_failed_attempts++;
            } 
            else 
            {
                memset(e_string_segment, '\0', sizeof(e_string_segment));
                sprintf(e_string_segment,"%010lu tries",m_num_failed_attempts);
                error_code=ble_rfidrs_log_messge_send(p_rfidrs, (uint8_t *)e_string_segment, 17);
                if (error_code != NRF_ERROR_INVALID_STATE) 
                {
                    m_failed_attempt_flag    =    true;
                    m_num_failed_attempts++;
                } 
                else 
                {
                    m_failed_attempt_flag    =    false;
                    m_num_failed_attempts=0;
                }
            }
        }
    }

    return error_code;
}


