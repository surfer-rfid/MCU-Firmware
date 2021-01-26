//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware State                                                //
//                                                                              //
// Filename: rfidr_state.c                                                      //
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
//  This file contains the main application-level behavior of the RFIDr MCU     //
//  firmware, including the state machine that gets executed whenever the       //
//  iDevice software prompts for a state transition.                            //
//                                                                              //
//  Revisions:                                                                  //
//  061619 - Major commentary cleanup.                                          //
//  121219 - Major code cleanup to remove repeated code in several "states".    //
//  Now each state core has its own function which is called in the "state".    //
//  Support for frequency hopping, PDOA ranging, and kill functionality are     //
//  added in. Have only one error handling function now that checks state.      //
//  Also, more robust programming and searching/programming last inventoried    //
//  flag functionalities are added in.                                          //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

#include "app_error.h"
#include "app_util_platform.h"
#include "ble_rfidrs.h"
#include "nordic_common.h"
#include "nrf_adc.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "rfidr_error.h"
#include "rfidr_gpio.h"
#include "rfidr_rxradio.h"
#include "rfidr_spi.h"
#include "rfidr_state.h"
#include "rfidr_sx1257.h"
#include "rfidr_txradio.h"
#include "rfidr_user.h"
#include "rfidr_waveform.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

//Define state variables that need to persist between calls to functions in this file.
//Also, only allow these state variables to be accessible to functions in this file.

static rfidr_state_t    m_rfidr_state                            =    IDLE_UNCONFIGURED;    //Current state
static rfidr_state_t    m_rfidr_state_next                       =    IDLE_UNCONFIGURED;    //Transition to next state when we can
static bool             m_received_irq_flag                      =    false;
static bool             m_received_hvc_read_state_flag           =    false;
static bool             m_received_hvc_pckt_data1_flag           =    false;
static bool             m_dtc_state_flag                         =    false;
static bool             m_track_tag_state_flag                   =    false;
static bool             m_adc_returned_flag                      =    false;                //To be set to true when adc returns data.
static uint16_t         m_num_inv_tags_found                     =    0;
static uint8_t          m_return_state_code                      =    0;
static uint8_t          m_hopskip_nonce                          =    0;    //A number to help the iDevice keep track of which hop/skip pairs are associated.
static uint16_t         m_last_adc_sample                        =    0;

//This function is used for transferring local state information to the iDevice over BTLE.
//After reviewing the code, it seems as if m_return_state code doesn't really need to be a state variable.
//This can be cleaned up in the next major code overhaul.

static uint8_t * decode_rfidr_state(rfidr_state_t rfidr_state)
{

    switch(rfidr_state)
    {
        case(IDLE_UNCONFIGURED):             m_return_state_code=0;    break;
        case(IDLE_CONFIGURED):               m_return_state_code=1;    break;
        case(INITIALIZING):                  m_return_state_code=2;    break;
        case(SEARCHING_APP_SPECD_TAG):       m_return_state_code=3;    break; //These were all renumbered on 11/21 so we need to also update iPhone app.
        case(SEARCHING_LAST_INV_TAG):        m_return_state_code=4;    break;
        case(INVENTORYING):                  m_return_state_code=5;    break;
        case(TESTING_DTC):                   m_return_state_code=6;    break;
        case(PROGRAMMING_APP_SPECD_TAG):     m_return_state_code=7;    break;
        case(PROGRAMMING_LAST_INV_TAG):      m_return_state_code=8;    break;
        case(RECOVERING_WAVEFORM_MEMORY):    m_return_state_code=9;    break;
        case(RESET_SX1257_AND_FPGA):         m_return_state_code=10;    break;
        case(KILL_TAG):                      m_return_state_code=11;    break;
        case(PROGRAMMING_KILL_PASSWD):       m_return_state_code=12;    break;
        case(TRACK_APP_SPECD_TAG):           m_return_state_code=13;    break;
        case(TRACK_LAST_INV_TAG):            m_return_state_code=14;    break;
        default: m_return_state_code=99;                                break;
    }

    return &m_return_state_code;

}

//This function sends a generic log message to the iDevice as a notification.
//Such a notification may take a while to send over a number of packets.
//This notification also stretches out over 3 connection intervals.

static void send_log_message(ble_rfidrs_t *p_rfidrs, char * inputString)
{
    char        rfidr_log_message[256]    =    {0};
    uint16_t    cnt                       =    0;
    uint32_t    nrf_error_code            =    NRF_SUCCESS;

    //Truncate the input string short of its maximum length, leaving room for some of the null characters at the end to guarantee string termination.
    while (*(inputString+cnt) && cnt < 240)
    {
        rfidr_log_message[cnt]=*(inputString+cnt); cnt++;
    }

    nrf_error_code=rfidr_error_complete_message_send(p_rfidrs,(uint8_t *)rfidr_log_message);
    if (nrf_error_code != NRF_ERROR_INVALID_STATE)
    {
        //APP_ERROR_CHECK(nrf_error_code);
    }
}

//This function sends a generic log message to the iDevice as a notification.
//Such a notification will send quickly over one packet (20 bytes).

static void send_short_message(ble_rfidrs_t *p_rfidrs, char * inputString)
{
    char        rfidr_log_message[20]        =    {0};
    uint16_t    cnt                          =    0;
    uint32_t    nrf_error_code               =    NRF_SUCCESS;

    while (*(inputString+cnt) && cnt < 20)
    {
        rfidr_log_message[cnt]=*(inputString+cnt); cnt++;
    }

    nrf_error_code=rfidr_error_complete_message_send(p_rfidrs,(uint8_t *)rfidr_log_message);
    if (nrf_error_code != NRF_ERROR_INVALID_STATE)
    {
        //APP_ERROR_CHECK(nrf_error_code);
    }
}

//This function is called during the startup sequence in main().
//Here, we set all of the state variables to default values.

void    rfidr_state_init(void)
{
    m_rfidr_state                            =    IDLE_UNCONFIGURED;    //State variable to hold the current state information.
    m_rfidr_state_next                       =    IDLE_UNCONFIGURED;    //State variable to hold the state to transition to the next time run_rfidr_state_machine is called.
    m_received_irq_flag                      =    false;                //Indicates when an IRQ has been received from the FPGA.
    m_received_hvc_read_state_flag           =    false;                //Indicates when an indication ACK has been received on the BTLE "Read State" characteristic.
    m_received_hvc_pckt_data1_flag           =    false;                //Indicates when an indication ACK has been received on the BTLE "Data 1" characteristic.
    m_dtc_state_flag                         =    false;                //Indicates when we are being kept in the DTC state to operate the reader in testing mode.
    m_track_tag_state_flag                   =    false;                //Indicates when we are being kept in a tag tracking state to continually track a set of tags.
    m_adc_returned_flag                      =    false;                //Indicates whether the ADC has returned a value or not.
    m_num_inv_tags_found                     =    0;                    //Keep track of how many tags are found in an inventory - to be used while tracking tags.
    m_return_state_code                      =    0;                    //Not really used, we can delete this on the next major code overhaul.
    m_hopskip_nonce                          =    0;                    //We will increment each time we hop frequencies but not skip frequencies.
    m_last_adc_sample                        =    0;                    //Retain the ADC sample for separate processing by IRQ handler and TX offset calibration algorithm.
    //We keep setting of the hopskip nonce in this file to minimize message passing and also to retain flexibility of implementation.
}

void update_adc_sample(int32_t adc_sample)
{
    m_adc_returned_flag                      =    true;
    m_last_adc_sample                         =    (uint16_t)(adc_sample & 0x0000FFFF); //Let's cast this 10-bit unsigned value to something appropriate
}

//This function sets the "received irq" state variable denoting when the MCU has received an IRQ from the FPGA.
//This function should be called from the IRQ handler assigned to the PIN connected to the FPGA IRQ output.

void    rfidr_state_received_irq(void)
{
    m_received_irq_flag    =    true;
}

//This function sets a state variable flag denoting that the NRF BTLE SoftDevice has received an indication ACK from the iDevice for a read state characteristic write.

void    rfidr_state_received_read_state_confirmation(void)
{
    m_received_hvc_read_state_flag            =    true;
}

//This function sets a state variable flag denoting that the NRF BTLE SoftDevice has received an indication ACK from the iDevice for a packet data 1 characteristic write.

void    rfidr_state_received_pckt_data1_confirmation(void)
{
    m_received_hvc_pckt_data1_flag            =    true;
}

//This function informs the iDevice of any state transition and hence serves as a "bookend" function in the run_rfidr_state_machine function
static void rfidr_state_bookend_function(ble_rfidrs_t *p_rfidrs)
{
    uint32_t    nrf_error_code                =    NRF_SUCCESS;

    m_received_hvc_read_state_flag            =    false;
    nrf_error_code=ble_rfidrs_read_state_send(p_rfidrs,decode_rfidr_state(m_rfidr_state),BLE_RFIDRS_READ_STATE_CHAR_LEN);
    if (nrf_error_code != NRF_ERROR_INVALID_STATE){APP_ERROR_CHECK(nrf_error_code);}
    while(m_received_hvc_read_state_flag == false){}
}

//We got an error - send a message to the iDevice, shut down the PA to avoid damage, and return to unconfigured state.
//The intent is to go back to the unconfigured state if we get an error during initialization.

static void handle_error(ble_rfidrs_t *p_rfidrs, char * input_string_outer, char * input_string_inner, rfidr_error_t rfidr_error_code)
{
    char        rfidr_outer_sanitize[128]    =    {0};
    char        rfidr_inner_sanitize[128]    =    {0};
    char        rfidr_error_message[256]     =    {0};
    uint16_t    cnt                          =    0;
    uint32_t    nrf_error_code               =    NRF_SUCCESS;

    //Truncate the input string short of its maximum length, leaving room for some of the null characters at the end to guarantee string termination.
    cnt=0;
    while (*(input_string_outer+cnt) && cnt < 116)
    {
        rfidr_outer_sanitize[cnt]=*(input_string_outer+cnt); cnt++;
    }

    cnt=0;
    while (*(input_string_inner+cnt) && cnt < 116)
    {
        rfidr_inner_sanitize[cnt]=*(input_string_inner+cnt); cnt++;
    }
    
    rfidr_disable_pa(); //Don't check for error, just do it. We already got an error!
    sprintf(rfidr_error_message,"Error at %s: %s: %02d",rfidr_outer_sanitize,rfidr_inner_sanitize,(uint8_t)rfidr_error_code);
    nrf_error_code=rfidr_error_complete_message_send(p_rfidrs,(uint8_t *)rfidr_error_message);
    
    //If we got an error while in tracking or DTC modes, we are exiting said state and need
    //to set the state flag accordingly.
    m_track_tag_state_flag    =    false;
    m_dtc_state_flag          =    false;
    
    if (nrf_error_code != NRF_ERROR_INVALID_STATE)
    {
       //APP_ERROR_CHECK(nrf_error_code);
    }
    if(m_rfidr_state==IDLE_UNCONFIGURED || m_rfidr_state==INITIALIZING){
        rfidr_disable_led1(); //Keep LED disabled to show that the reader not configured.
        m_rfidr_state_next=IDLE_UNCONFIGURED;
        m_rfidr_state=IDLE_UNCONFIGURED;    
    } else {
        rfidr_enable_led1(); //Keep LED enabled so show that the reader is configured.
        m_rfidr_state_next=IDLE_CONFIGURED;
        m_rfidr_state=IDLE_CONFIGURED;
    }
    rfidr_state_bookend_function(p_rfidrs);
}



//This function is called by the BTLE write state characteristic event handler in main.c.
//This function is ensured to be called prior to running the state machine itself, which is picked up by the main loop
//as soon as this function has completed.

uint32_t    write_rfidr_state_next(ble_rfidrs_t *p_rfidrs, rfidr_state_t    l_rfidr_state_next)
{
    //Modus operandi of this function is to check for an illegal state transaction request from the app and
    //ignore it. This will also be enforced on the app side. App side should figure out the problem by reading the state.
    //In other words, this function protects the hardware from a bad app.

    uint32_t nrf_error_code    =    NRF_SUCCESS;

    switch(l_rfidr_state_next){
        case INITIALIZING:
            if(m_rfidr_state==IDLE_UNCONFIGURED)    //If the app tries to initialize the state machine while in the unconfigured state, great! Do it!
            {
                m_rfidr_state_next    =    INITIALIZING;
            }
            else if(m_rfidr_state==IDLE_CONFIGURED) //If the app tries to initialize the state machine while in the configured state, don't change the state.
            {
                m_rfidr_state_next    =    m_rfidr_state;
            }
            else
            {
                m_rfidr_state_next    =    m_rfidr_state; //If the app tries to initialize the state machine while in any other state, don't change the state.
            }
            break;
        case SEARCHING_APP_SPECD_TAG:
        case SEARCHING_LAST_INV_TAG:
        case INVENTORYING:
        case PROGRAMMING_APP_SPECD_TAG:
        case PROGRAMMING_LAST_INV_TAG:
        case KILL_TAG:
        case PROGRAMMING_KILL_PASSWD:
        case RECOVERING_WAVEFORM_MEMORY:
        case TESTING_DTC:
            if(m_rfidr_state==IDLE_CONFIGURED)
            {
                m_rfidr_state_next    =    l_rfidr_state_next;            //Safely transition to another state if we are not in the middle of one.
            }                                                             //Note that the TRACK and DTC states require the app to toggle them in
            else                                                          //and out of the state. These are the only states that "stick" between app accesses.
            {
                m_rfidr_state_next    =    m_rfidr_state;                 //Otherwise (for example we are in the middle of a state), stay in the same state.
            }
            break;
        case TRACK_APP_SPECD_TAG:
            if(m_rfidr_state==IDLE_CONFIGURED)                            //If we are configured, toggle the tracking state true.
            {                                                             //and enter the tracking state when the state machine is run.
                m_track_tag_state_flag    =    true;
                m_rfidr_state_next        =    l_rfidr_state_next;
            }
            else if(m_rfidr_state==TRACK_APP_SPECD_TAG)
            {
                m_track_tag_state_flag    =    false;                     //If we get a state request from the same state while we are in the state,
                m_rfidr_state_next        =    m_rfidr_state;             //we need to toggle out of it. So keep the same state variable, but set
            }                                                             //the toggle flag to false.
            else
            {
                m_rfidr_state_next    =    m_rfidr_state;                 //Otherwise, stay in the same state.
            }
            break;
        case TRACK_LAST_INV_TAG:
            if(m_rfidr_state==IDLE_CONFIGURED)                            //If we are configured, toggle the tracking state true.
            {                                                             //and enter the tracking state when the state machine is run.
                m_track_tag_state_flag    =    true;
                m_rfidr_state_next        =    l_rfidr_state_next;
            }
            else if(m_rfidr_state==TRACK_LAST_INV_TAG)
            {
                m_track_tag_state_flag    =    false;                     //If we get a state request from the same state while we are in the state,
                m_rfidr_state_next        =    m_rfidr_state;             //we need to toggle out of it. So keep the same state variable, but set
            }                                                             //the toggle flag to false.
            else
            {
                m_rfidr_state_next    =    m_rfidr_state;                 //Otherwise, stay in the same state.
            }
            break;
        case RESET_SX1257_AND_FPGA:                                       //Want to be able to get reset if we're in an "IDLE" state.
            if(m_rfidr_state==IDLE_CONFIGURED || m_rfidr_state==IDLE_UNCONFIGURED || m_rfidr_state==TESTING_DTC || m_rfidr_state==TRACK_APP_SPECD_TAG || TRACK_LAST_INV_TAG)
            {
                m_rfidr_state_next    =    l_rfidr_state_next;
            }
            else
            {
                m_rfidr_state_next    =    m_rfidr_state;    //We do not want to reset if the MCU is in the middle of an operational state.
            }
            break;
        default: break; //Do nothing
    }

    return nrf_error_code;
}

//This function is called by the BTLE read state characteristic.

uint32_t    read_rfidr_state(rfidr_state_t    * p_rfidr_state)
{
    *p_rfidr_state    =    m_rfidr_state;
    return NRF_SUCCESS;
}

//These functions below calibrate the transmit offset in the I/Q DACs of the SX1257.
//We do this by setting registers in the FPGA, which then alter the 1-bit waveforms being sent to the DACs.
//We run this routine when we are sure that the FPGA is putting all "zeros" through the DACs.
//In this fashion, minimizing the RF output power is tantamount to maximizing the modulation depth of
//of the reader when we are doing DSB-ASK modulation.

static rfidr_error_t tx_offset_calibration_core(ble_rfidrs_t *p_rfidrs,uint8_t loop_tx_cal_sdm,uint8_t loop_tx_cal_zgn)
{
    rfidr_error_t    rfidr_error_code                        =    RFIDR_SUCCESS;
    
    rfidr_error_code = set_tx_sdm_offset(loop_tx_cal_sdm);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, set sdm offset","",rfidr_error_code); return rfidr_error_code;}
    rfidr_error_code = set_tx_zgn_offset(loop_tx_cal_zgn);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, set zgn offset","",rfidr_error_code); return rfidr_error_code;}
    nrf_delay_us(100);
    rfidr_enable_pa();
    nrf_delay_us(800);
    m_adc_returned_flag = false;
    nrf_adc_start();
    while(m_adc_returned_flag == false){}
    rfidr_disable_pa();
    
    return RFIDR_SUCCESS;
}

//Note that the MAX2204 power detector takes about 1ms to completely settle, which will dictate how fast we can 
//actuate the algorithm.
//Also the FCC requirement to hop every 400ms means that we need to complete this operation within 400ms.
//Luckily it appears we have time to do this, but not with a lot of margin.
//We won't use a full gradient descent scheme since we really need to find the minimum value to ensure maximum modulation depth.
//We'll just brute force the approach since we should have just enough time to do this.

static rfidr_error_t tx_offset_calibration_brute_force(ble_rfidrs_t *p_rfidrs)
{
    rfidr_error_t    rfidr_error_code                        =    RFIDR_SUCCESS;
    uint8_t          loop_tx_cal_sdm                         =    0;
    uint8_t          loop_tx_cal_zgn                         =    0;
    uint8_t          sdm_offset_best                         =    8;
    uint8_t          zgn_offset_best                         =    8;
    uint16_t         meas_power_best                         =    9999;
    uint8_t          recover_frequency_slot                  =    12;
    char             short_message[20]                       =    {0};

    rfidr_error_code=hop_sx1257_frequency(&recover_frequency_slot); m_hopskip_nonce++;
    //rfidr_error_code=set_sx1257_frequency(12); m_hopskip_nonce++;
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, freq. hop.","",rfidr_error_code); return rfidr_error_code;}

    rfidr_disable_pa();

    for (loop_tx_cal_sdm = 0; loop_tx_cal_sdm < 16; loop_tx_cal_sdm++)
    {
        for (loop_tx_cal_zgn = 0; loop_tx_cal_zgn < 16; loop_tx_cal_zgn++)
        {
            rfidr_error_code = tx_offset_calibration_core(p_rfidrs,loop_tx_cal_sdm,loop_tx_cal_zgn);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, tx cal core","",rfidr_error_code); return rfidr_error_code;}

            //sprintf(short_message,"CAL S:%02d Z:%02d O:%04d",loop_tx_cal_sdm,loop_tx_cal_zgn,m_last_adc_sample);
            //send_short_message(p_rfidrs, short_message);
            if(m_last_adc_sample < meas_power_best)
            {
                meas_power_best = m_last_adc_sample;
                sdm_offset_best = loop_tx_cal_sdm;
                zgn_offset_best = loop_tx_cal_zgn;
            }
        }
    }
    rfidr_disable_pa();
    rfidr_error_code = set_tx_sdm_offset(sdm_offset_best);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, final set sdm offset","",rfidr_error_code); return rfidr_error_code;}
    rfidr_error_code = set_tx_zgn_offset(zgn_offset_best);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, final set zgn offset","",rfidr_error_code); return rfidr_error_code;}
    sprintf(short_message,"CAL S:%02d Z:%02d O:%04d",sdm_offset_best,zgn_offset_best,meas_power_best);
    send_short_message(p_rfidrs, short_message);

    return rfidr_error_code;
}

static rfidr_error_t tx_offset_calibration_gradient(ble_rfidrs_t *p_rfidrs)
{
    #define          MAX_GRAD_CAL_ITERS                           96
    
    rfidr_error_t    rfidr_error_code                        =    RFIDR_SUCCESS;
    int8_t           sdm_val_curr                            =    10;
    int8_t           zgn_val_curr                            =    10;
    int8_t           sdm_val_dir                             =    1;
    int8_t           zgn_val_dir                             =    1;
    uint16_t         result_curr                             =    9999;
    bool             sdm_p_improve                           =    false;
    bool             sdm_n_improve                           =    false;
    bool             zgn_p_improve                           =    false;
    bool             zgn_n_improve                           =    false;
    uint8_t          recover_frequency_slot                  =    12;
    uint8_t          iter_curr                               =    0;
    char             short_message[20]                       =    {0};

    rfidr_error_code=hop_sx1257_frequency(&recover_frequency_slot); m_hopskip_nonce++;
    //rfidr_error_code=set_sx1257_frequency(12); m_hopskip_nonce++;
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, freq. hop.","",rfidr_error_code); return rfidr_error_code;}

    while(iter_curr++ <= MAX_GRAD_CAL_ITERS)
    {
         sprintf(short_message,"S:%02d Z:%02d O:%03d I:%02d",(uint8_t)(sdm_val_curr%16),(uint8_t)(sdm_val_curr%16),result_curr,iter_curr);
            send_short_message(p_rfidrs, short_message);
        //Assuming "dir" is still the correct representation of the gradient, continue down the gradient.
        rfidr_error_code = tx_offset_calibration_core(p_rfidrs,(uint8_t)((sdm_val_curr+sdm_val_dir)%16),(uint8_t)((zgn_val_curr+zgn_val_dir)%16));
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, gradient step","",rfidr_error_code); return rfidr_error_code;}
        //If indeed we are still going down the gradient, update values. Don't change "dir" (the representation of the gradient).
        if(m_last_adc_sample < result_curr)
        {
            result_curr    =    m_last_adc_sample;
            sdm_val_curr   =    (sdm_val_curr+sdm_val_dir)%16;
            zgn_val_curr   =    (zgn_val_curr+zgn_val_dir)%16;
            continue; //Move to the next iteration of the loop.
        }
        else //We need to reestablish the gradient.
        {
            rfidr_error_code = tx_offset_calibration_core(p_rfidrs,(uint8_t)((sdm_val_curr+1)%16),(uint8_t)((zgn_val_curr+0)%16));
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, ip grad test","",rfidr_error_code); return rfidr_error_code;}
            sdm_p_improve  =    m_last_adc_sample < result_curr;
            
            rfidr_error_code = tx_offset_calibration_core(p_rfidrs,(uint8_t)((sdm_val_curr-1)%16),(uint8_t)((zgn_val_curr+0)%16));
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, im grad test","",rfidr_error_code); return rfidr_error_code;}
            sdm_n_improve  =    m_last_adc_sample < result_curr;
            
            rfidr_error_code = tx_offset_calibration_core(p_rfidrs,(uint8_t)((sdm_val_curr+0)%16),(uint8_t)((zgn_val_curr+1)%16));
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, qp grad test","",rfidr_error_code); return rfidr_error_code;}
            zgn_p_improve  =    m_last_adc_sample < result_curr;
            
            rfidr_error_code = tx_offset_calibration_core(p_rfidrs,(uint8_t)((sdm_val_curr+0)%16),(uint8_t)((zgn_val_curr-1)%16));
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, qm grad test","",rfidr_error_code); return rfidr_error_code;}
            zgn_n_improve  =    m_last_adc_sample < result_curr;
            
            if(sdm_p_improve)
                sdm_val_dir    =    1;
            else if (sdm_n_improve)
                sdm_val_dir    =    -1;
            else
                sdm_val_dir    =    0;
            
            if(zgn_p_improve)
                zgn_val_dir    =    1;
            else if (zgn_n_improve)
                zgn_val_dir    =    -1;
            else
                zgn_val_dir    =    0;
            
            
            if(sdm_val_dir == 0 && zgn_val_dir == 0)
                break; //Break out of the loop. We've found the local minimum.
            else
            {
                rfidr_error_code = tx_offset_calibration_core(p_rfidrs,(uint8_t)((sdm_val_curr+sdm_val_dir)%16),(uint8_t)((zgn_val_curr+zgn_val_dir)%16));
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, gradient reestablish","",rfidr_error_code); return rfidr_error_code;}

                    result_curr    =    m_last_adc_sample;
                    sdm_val_curr   =    (sdm_val_curr+sdm_val_dir)%16;
                    zgn_val_curr   =    (zgn_val_curr+zgn_val_dir)%16;
                    continue; //Move to the next iteration of the loop.
                    //We assume that this operation did in fact reduce the error.
            }
        }
    }

    rfidr_disable_pa();
    rfidr_error_code = set_tx_sdm_offset((uint8_t)(sdm_val_curr%16));
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, final set sdm offset","",rfidr_error_code); return rfidr_error_code;}
    rfidr_error_code = set_tx_zgn_offset((uint8_t)(zgn_val_curr%16));
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tx cal, final set zgn offset","",rfidr_error_code); return rfidr_error_code;}
    sprintf(short_message,"S:%02d Z:%02d O:%03d I:%02d",(uint8_t)(sdm_val_curr%16),(uint8_t)(sdm_val_curr%16),result_curr,iter_curr);
    send_short_message(p_rfidrs, short_message);

    return rfidr_error_code;
}

//This function performs the core initialization routine as directed by the iDevice app.
static rfidr_error_t initialization_core(ble_rfidrs_t *p_rfidrs, char *error_info)
{
    rfidr_error_t    rfidr_error_code                        =    RFIDR_SUCCESS;
    uint8_t          blank_epc[MAX_EPC_LENGTH_IN_BYTES]      =    {0};
    
    //GPIOTE is not initialized here but is initialized as part of the top level main entry
    //SPI MASTER is not initialized here but is initialized as part of the top level main entry
    //Note to self - we will need some way to debug if we get an error and we are not plugged into

    //Most of this code has always been part of the initialization routine.
    //Here we ensure an orderly power on.

    rfidr_disable_pa();    //Make sure the PA is off until everything else is up.
    //rfidr_disable_xo();
    rfidr_enable_xo();
    nrf_delay_ms(100);
    rfidr_reset_fpga();
    rfidr_reset_radio();    //was just enable the radio but it should come up already
    nrf_delay_ms(100);
    //rfidr_enable_xo();    //Maybe we don't actually want to be enabling the XO after the FPGA is pulled out of reset
    rfidr_txradio_init();    //This function sets up TX RADIO state variables within the MCU firmware.
    set_app_specd_target_epc(p_rfidrs,blank_epc,MAX_EPC_LENGTH_IN_BYTES);    //This function sets up a target EPC within the MCU firmware for SELECT-based tag SEARCH operations.
    set_app_specd_program_epc(p_rfidrs,blank_epc);    //This function sets up a new EPC for programming onto a tag during PROGRAM operations.
    rfidr_error_code=load_sx1257_default();    //This function sets up the registers in the SX1257. We try to shake around the PLL a bit to help it converge properly.
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"load sx1257 default",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    rfidr_sel_ant0();    //Once again we default to ant0 on 11/21/19
    rfidr_error_code=load_rfidr_rxram_default(); //Load RX RAM. At this point, loading data consists of expected receive packet lengths.
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"load rfidr_rxram_default",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    rfidr_error_code=load_rfidr_txram_default(); //Load TX RAM. This loads all of the default TX packet opcodes into the FPGA TX RAM.
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"load_rfidr_txram_default",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    rfidr_error_code=set_sx1257_frequency(12);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set frequency",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    //Check that clk36 from the SX1257 is valid. This may be currently disabled to save LUT, as it never really failed.
    //If it is disabled, this may need to be revisited on account of seeming clk36-related initialization failures on the FPGA.
        if(!is_clk_36_valid()){handle_error(p_rfidrs,error_info,"clk 36 not valid: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    //Set clk36 inside the FPGA to run. This may be currently disabled to save LUT, as it never really failed.
    //If it is disabled, this may need to be revisited on account of seeming clk36-related initialization failures on the FPGA.
    rfidr_error_code=set_clk_36_oneshot();     
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set_clk_36_oneshot",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    //check that clk36 is indeed running, wait a bit before we do this.
        if(!is_clk_36_running()){handle_error(p_rfidrs,error_info,"clk 36 not running",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    //enable the irq - I don't think that we need to do this explicitly
    rfidr_error_code = tx_offset_calibration_brute_force(p_rfidrs);
    //rfidr_error_code = tx_offset_calibration_gradient(p_rfidrs);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"tx offset cal",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    //set_tx_sdm_offset(5); //Set offsets for TX to maximize modulation depth of the reader TX waveforms.
    //set_tx_zgn_offset(10);


    rfidr_enable_led1(); //Enabled LED1 to show that we are entering a configured state.
    send_log_message(p_rfidrs,"Initialization function complete!");

    return rfidr_error_code;
}

static rfidr_error_t search_core(ble_rfidrs_t *p_rfidrs, char *error_info, rfidr_query_session_t session, rfidr_target_epc_t target_epc, return_epc_t return_epc, return_mag_t return_mag, return_lna_gain_t return_lna_gain, rfidr_return_t *return_struct)
{
    //Mode types:
    //SEARCH_TARGET: Use main antenna, use user-specified target epc.
    //SEARCH_LAST: Use main antenna, use last epc obtained via inventorying (useful for searching on tag that hasn't been programmed).
    //SEARCH_CAL: Use alternate antenna, use epc consistent with dummy tag for phase calibration.
    //SEARCH_PLL: Use alternate antenna, use all-zeros epc.
    
    //We offer a-la-carte return of data since querying FPGA memory over the SPI bus takes time and slows down the read rate.
    rfidr_error_t    rfidr_error_code                        =    RFIDR_SUCCESS;
    uint8_t          loop_iq                                 =    0;
    uint8_t          loop_load                               =    0;
    char             short_message[20]                       =    {0};

    //Set return variable to default values
    
    return_struct->i_pass=false;
    return_struct->q_pass=false;
    for(loop_load=0; loop_load<MAX_EPC_LENGTH_IN_BYTES; loop_load++)
    {
        return_struct->i_epc[loop_load]=0;
        return_struct->q_epc[loop_load]=0;
    }
    return_struct->i_lna_gain=0xD4;
    return_struct->q_lna_gain=0xD4;
    return_struct->i_main_mag=0;
    return_struct->i_alt_mag=0;
    return_struct->q_main_mag=0;
    return_struct->q_alt_mag=0;
    
    //Load TX RAM, getting ready for search of specific tag and read back.
    //We assume that the iDevice has set the target epc via callback already
    set_select_target(TARGET_SL);     //No possible error, so don't check. Select packet will make changes to tags' SL flag.
    set_select_action(ACTION_A0);     //No possible error, so don't check. Tags matching/not matching select packet specification will assert/deassert SL flag.
    set_query_sel(SEL_PSL);           //No possible error, so don't check. Only tags with SL flag asserted will respond to query.
    set_query_session(session);       //No possible error, so don't check.
    set_query_target(TARGET_A);       //No possible error, so don't check. Only tags whose inventory flag is set to "A" will participate in the round.
    set_query_q(0);                   //Targeted search requires immediate reply by the tag of interest, so set query Q=0


    //If we are checking the SX1257 PLL behavior, we need to set a flag bit in the FPGA to reconfigure the data recovery to look for tonal behavior in an otherwise benign environment.
    if(target_epc==TARGET_PLL_EPC)
        rfidr_error_code=set_sx1257_pll_chk_mode();
    else
        rfidr_error_code=unset_sx1257_pll_chk_mode();    //Have this in here just in case somehow pll mode was set elsewhere.    
    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting PLL check mode",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Depending on the argument provided in the function all, load the appropriate select packet into the FPGA.
    switch(target_epc)
    {
        case TARGET_APP_SPECD_EPC:    rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
        //Why is the above calling for an epc length of max bytes? Because we are doing Q=0 so we need to uniquely specify the tag.
        //If we have an incomplete EPC, the app specd epc should be appended with all zeros and probably won't find the correct tag.
        case TARGET_LAST_INV_EPC:    rfidr_error_code=load_select_packet_only(LAST_INV_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
        case TARGET_CAL_EPC:         rfidr_error_code=load_select_packet_only(DUMMYTAG_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
        case TARGET_PLL_EPC:         rfidr_error_code=load_select_packet_only(ZERO_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
        default:                     rfidr_error_code=load_select_packet_only(ZERO_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
    }

        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading select packet 1",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //For the select #2 packet, just put in all zeros and make sure this gets added to the selection as part of a union

    //Next we set the second select packet to set the session flag to "A" on all tags that also comply with EPC2
    set_select_action(ACTION_A1);        //No possible error, so don't check.

    rfidr_error_code=load_select_packet_only(ZERO_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_2);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading select packet 2",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Load query packet into TX RAM. This had to be done each time one of query select, session, target, or Q are changed.
    //Since we are doing query Q=0 we don't need to load the Query Rep or Query Adjust packets.
    rfidr_error_code=load_query_packet_only(FLAGSWAP_NO);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading query",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Tell the FPGA to operate its radio state machine in search mode - that is, run the sequence of operation that looks for one particular tag.
    rfidr_error_code=set_radio_mode_search();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set radio mode to search",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //We need to check both I and Q paths of the clock and data recovery circuit.
    for(loop_iq=0;loop_iq<=1;loop_iq++)
    {
        //Enable PA.
        rfidr_error_code=rfidr_enable_pa();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"enabling pa",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        
        //Set use_i or use_q by sending the appropriate command to the FPGA.
        if(loop_iq % 2 == 0)
            rfidr_error_code=set_use_i();
        else
            rfidr_error_code=set_use_q();
            
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set_use_i/q",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        
        //Run the radio. First ensure we specify a select packet and that the SX1257 LNA gain is reset.
        //We also set the flag in the FPGA to use the select packet on the first packet to be sent out.
        rfidr_error_code=set_use_select_pkt();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting select packet",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        set_sx1257_lna_gain((uint8_t)(0xD4)); //Reset the gain
        
        //Set the FPGA IRQ flag to false so that we can wait for it.
        m_received_irq_flag    =    false;
        
        //Now that the FPGA commands have all been loaded up, tell the FPGA to execute a run of the state machine.
        rfidr_error_code=set_go_radio_oneshot();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set go radio",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        
        //Wait for IRQ back from FPGA.
        while(m_received_irq_flag==false){}
        //ACK the IRQ to permit the FPGA state machines to accept another input.
        rfidr_error_code=set_irq_ack_oneshot();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"acking irq",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        
        //Disable the PA to reduce the amount of time we spend eating into our FCC on-time budget.
        rfidr_error_code=rfidr_disable_pa();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"disabling pa",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //Was the operation a success? If so, declare it and dump EPC and magnitude data back to the iDevice.
        //If the operation was not a success, declare a fail, because any information that might be returned would be invalid.

        if(target_epc==TARGET_PLL_EPC || read_radio_exit_code()==0) //We weren't checking the exit code before for PLL check. Order of execution should prevent check of exit code from occurring. If not, not a problem.
        {
            if(loop_iq==0)
            {
                send_short_message(p_rfidrs, "Search I Pass");
                return_struct->i_pass=true;
                if(return_epc==RETURN_EPC_YES)
                {
                    rfidr_error_code=rfidr_read_epc(return_struct->i_epc,(target_epc==TARGET_PLL_EPC) ? READ_RXRAM_PLLCHECK : READ_RXRAM_REGULAR);
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"checking I EPC", rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                }
                if(return_mag==RETURN_MAG_YES)
                {
                    rfidr_error_code=rfidr_read_main_magnitude(&(return_struct->i_main_mag),(target_epc==TARGET_PLL_EPC) ? READ_RXRAM_PLLCHECK : READ_RXRAM_REGULAR);
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"checking I - Main Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    rfidr_error_code=rfidr_read_alt_magnitude(&(return_struct->i_alt_mag),(target_epc==TARGET_PLL_EPC) ? READ_RXRAM_PLLCHECK : READ_RXRAM_REGULAR);
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"checking I - Alt Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

                        sprintf(short_message,"MI(I): %10d",(int)return_struct->i_main_mag);
                        send_short_message(p_rfidrs, short_message);

                        sprintf(short_message,"MQ(I): %10d",(int)return_struct->i_alt_mag);
                        send_short_message(p_rfidrs, short_message);
                }
                if(return_lna_gain==RETURN_LNA_GAIN_YES)
                {
                    rfidr_error_code=get_sx1257_lna_gain(&(return_struct->i_lna_gain)); //Check the gain
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"getting I LNA gain",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                }
            }
            else
            {
                send_short_message(p_rfidrs, "Search Q Pass");
                return_struct->q_pass=true;
                if(return_epc==RETURN_EPC_YES)
                {
                    rfidr_error_code=rfidr_read_epc(return_struct->q_epc,(target_epc==TARGET_PLL_EPC) ? READ_RXRAM_PLLCHECK : READ_RXRAM_REGULAR);
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"checking Q EPC", rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                }
                if(return_mag==RETURN_MAG_YES)
                {
                    rfidr_error_code=rfidr_read_main_magnitude(&(return_struct->q_main_mag),(target_epc==TARGET_PLL_EPC) ? READ_RXRAM_PLLCHECK : READ_RXRAM_REGULAR);
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"checking Q - Main Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    rfidr_error_code=rfidr_read_alt_magnitude(&(return_struct->q_alt_mag),(target_epc==TARGET_PLL_EPC) ? READ_RXRAM_PLLCHECK : READ_RXRAM_REGULAR);
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"checking Q - Alt Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    sprintf(short_message,"MI(Q): %10d",(int)return_struct->q_alt_mag);
                    send_short_message(p_rfidrs, short_message);

                    sprintf(short_message,"MQ(Q): %10d",(int)return_struct->q_main_mag);
                    send_short_message(p_rfidrs, short_message);
                }
                if(return_lna_gain==RETURN_LNA_GAIN_YES)
                {
                    rfidr_error_code=get_sx1257_lna_gain(&(return_struct->q_lna_gain)); //Check the gain
                        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"getting Q LNA gain",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                }
            }
        } 
        else 
        {
            sprintf(short_message,"Search %c Fail",loop_iq==0 ? 'I' : 'Q');
            send_short_message(p_rfidrs, short_message);
            //nrf_delay_ms(60); //I think this is in place to space out the fails on the logic analyzer so we can see then as they were early in the project.
        }
    
        //Set use_q to 1 and repeat the search.
    }   //for loop_iq

    if(target_epc==TARGET_PLL_EPC)
    {
        rfidr_error_code=unset_sx1257_pll_chk_mode();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"unsetting PLL check mode",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    }

    return rfidr_error_code;
} // static void search_core

static rfidr_error_t end_inventory(ble_rfidrs_t *p_rfidrs, char *error_info)
{
    rfidr_error_t    rfidr_error_code    =    RFIDR_SUCCESS;

    //If we bail on the inventory in the middle of it, we need to make sure that the PA is disabled.
    rfidr_error_code    =    rfidr_disable_pa();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"disabling PA",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Also we need to stop LED toggling if that was going on.
    rfidr_enable_led1();

    //Since the FPGA radio state machine will keep issuing query reps forever, we need to tell it that it's time to end.
    //Setting the "inventory end" bit will do this.
    rfidr_error_code    =    set_end_radio_fsm_loop();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting inventory end",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Have the "radio go" in order for the FPGA and FPGA Radio FSMs to get back to their idle states.
    m_received_irq_flag    =    false;
    rfidr_error_code       =    set_go_radio_oneshot();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set go radio for inv. end",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    while(m_received_irq_flag==false){}

    rfidr_error_code=set_irq_ack_oneshot();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"acking irq for inv. end",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    return RFIDR_SUCCESS;
}

static rfidr_error_t inventory_core(ble_rfidrs_t *p_rfidrs, char *error_info, rfidr_query_session_t session, const char *query_q_vector, uint8_t max_tags, char *epc2, rfidr_return_t *return_struct)
{
    //The way inventory is going to run here is that we will only target tags with their inventory tag set to "A".
    //We will exclude tags from inventory by setting their flags to "B".
    //There are two EPC inputs, one implicit (epc1) - from the app, one explicit (epc2) - from calling this function.
    //The inventoried tags are the union of the two epc masks.
    //If one wants to inventory all tags in an area, one can set either of the EPCs to a zero-length EPC.
    //In general, we'll be using Session S2 or S3 so that we can power down the PA and frequency hop in between query rounds.

    #define    QUERY_ROUND_LIMIT    36    //Somewhat arbitrary
    #define    MAX_QUERY_Q          6    //We want to limit this to limit frequency dwell time so we don't have to hop in the middle of a query round.

    rfidr_error_t            rfidr_error_code          =    RFIDR_SUCCESS;    //An output error code.
    uint8_t                  loop_query_q              =    0;                //Loop iteration value between query rounds.
    uint8_t                  loop_iq                   =    0;                //Loop iteration value between I and Q sensing in the reader.
    uint8_t                  loop_load                 =    0;                //Loop iteration value for loading arrays.
    uint16_t                 loop_q_iter               =    0;                //Loop iteration variable for within the query round.
    uint8_t                  q_value                   =    0;                //Create a variable to hold the current value of Query Q so that we can take clear steps to sanitize it.
    uint8_t                  recover_frequency_slot    =    0;                //A variable to fish out what frequency slot we hopped to in rfidr_sx1257.c.
    uint8_t                  epc1_length_in_bytes      =    0;                //Variable to fish out what the application specified EPC length is.
    //uint8_t                epc2_length_in_bytes      =    0;                //Variable to fish out what the software-specified EPC length is.
    char                     short_message[20]         =    {0};              //An array to hold a short message to be sent back to the iDevice.
    bool                     query_adj_burn_flag       =    false;            //We demo the Query Adjacent packet here by using it once. This flag lets us just do it once.
    rfidr_select_target_t    target                    =    TARGET_S2;        //The session flag to be targeted by the select packet.
        
    m_num_inv_tags_found                               =    0;                //Use a state variable for this now, so that other functions can use the info.

    //Utilize a string for epc2 here because they are easy to input while also having a native sentinel value (null).
    //EPC values of less than or equal to 12 bytes can be used here.
    //EPC values of greater than 12 bytes will be truncated by the called function.
    
    //Load TX RAM, getting ready for inventory
    //In this code, we select both tags that comply with both EPC1 and EPC2 (i.e. the union).
    //By setting EPC1 to a zero-length EPC, one ends up inventorying all flags.
    //In the future, these functions can be easily modified to support inventorying different groups of RFID tags
    switch(session)
    {
        case SESSION_S0: target=TARGET_S0; break;
        case SESSION_S1: target=TARGET_S1; break;
        case SESSION_S2: target=TARGET_S2; break;
        case SESSION_S3: target=TARGET_S3; break;
        default: target=TARGET_S0; break;
    }

    set_select_target(target);       //Target inventoried flag of selected session with the Query. //No possible error, so don't check.
    set_select_action(ACTION_A0);    //With the first select packet, set the session flag to A for tags complying with EPC1 and to B for those not complying. //No possible error, so don't check.
    set_query_sel(SEL_PSL);          //121320 - We'll check for select flag to ensure exclusivity of this inventory test.
    set_query_session(session);      //Query packet will target the session specified by the calling function. //No possible error, so don't check.
    set_query_target(TARGET_A);      //Query packet will target flags with their session flag set to A. //No possible error, so don't check.

    //Load a select packet into the FPGA TX RAM with an EPC that we want to act as a select mask.
    //In other words, all of the packets with this tag EPC value will have their session flags set to "A" and all others will have the flags set to "B".

    rfidr_error_code=read_length_app_specd_target_epc(&epc1_length_in_bytes); //No real error, so don't check it.
    rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,epc1_length_in_bytes, SEL_PACKET_NO_1);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading arbitrary select packet",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    
    //Next we set the second select packet to set the session flag to "A" on all tags that also comply with EPC2
    //set_select_action(ACTION_A1);         //No possible error, so don't check.
    //Load the second select packet into TX RAM.
    //Why are we doing this? To inventory all tags in the area and to demonstrate that the system can accommodate 2 select packets for
    //more precise control of tag selecting.
    //Note that currently, the FPGA radio FSM code only supports using two select packets during inventory. The rest of the time, we are trying to find one specific tag of interest.

    //rfidr_error_code=set_fmw_specd_target_epc(epc2); //No real error, so don't check it
    //rfidr_error_code=read_length_fmw_specd_target_epc(&epc2_length_in_bytes); //No real error, so don't check it.
    //121320 - We seem to have problems with inventory specificity, so we'll use the second packet to ensure that
    //absolutely only packets with the correct EPC are included in the inventory round.
    set_select_target(TARGET_SL);
    //Don't need to read the epc length again
    rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,epc1_length_in_bytes, SEL_PACKET_NO_2);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading arbitrary select packet no 2",rfidr_error_code); return RFIDR_ERROR_GENERAL;}


    //Tell the FPGA to operate its state machine as per inventory rules.
    rfidr_error_code=set_radio_mode_inventory();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set radio mode to inventory",rfidr_error_code); return RFIDR_ERROR_GENERAL;}


    //With the information we have so far, we can load the Query Adjust packet into the QUERY REP slot of the FPGA TX RAM.
    //We do this first to demonstrate usage of the Query Adjust packet.
    rfidr_error_code=load_query_adj_packet(true);
    query_adj_burn_flag=true;
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading query rep",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //We also set the flag in the FPGA to use the select packet on the first packet to be sent out.
    rfidr_error_code=set_use_select_pkt();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting select packet",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    set_sx1257_lna_gain((uint8_t)(0xD4));    //Reset the LNA gain. Not sure if we want it here or within the I/Q loop below as of 120719.

    //Set query q and load the appropriate query packet. This needs to be done each time Q is changed.
    //We dynamically check the length of the Q vector instead of hard coding it by waiting for the NULL (0) character in the Q vector string.
    //When we get an error in the loop, we need to end the inventory so that we don't get stuck in the inventory.
    //Also we need to put a hard QUERY_ROUND_LIMIT on this loop so it doesn't get stuck.
    for(loop_query_q=0;(*(query_q_vector+loop_query_q) != 0) || loop_query_q < QUERY_ROUND_LIMIT;loop_query_q++)
    {
        rfidr_toggle_led1(); //Toggle LED to show that the reader is doing something.
        q_value=(uint8_t)(*(query_q_vector+loop_query_q)-'0');      //Supposedly we got an integer char input. Subtract '0' (48) to do a char to integer conversion.
        q_value=(q_value > MAX_QUERY_Q) ? MAX_QUERY_Q : q_value;    //Sanitize q_value. Make sure we don't send through anything smaller than 0 and bigger than MAX_QUERY_Q.
        //The type of q_value should enforce minimum of 0.

        //We need to hop frequencies on a regular basis to comply with FCC section 15.247.
        //We can't transmit on a given frequency for greater than 0.4s in a 10 second period.
        //Each query rep interval lasts about 2ms and during this time we have the PA on almost the whole time.
        //Given that we should not hop between I and Q variants (we want to search both I and Q for a given frequency),
        //we really need to set Q <= 6. In addition, we need to try having the PA on only when absolutely necessary for RFID traffic.
        //In other words, turn off the PA for BTLE transfers and moving data off of the FPGA.
        
        rfidr_error_code=set_query_q(q_value);                    //Make sure we convert Q to an integer only value less than 16.
        rfidr_error_code=hop_sx1257_frequency(&recover_frequency_slot); m_hopskip_nonce++;
            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"hopping frequency",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        rfidr_error_code=load_query_packet_only(FLAGSWAP_NO);
            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"loading query",rfidr_error_code); return RFIDR_ERROR_GENERAL;}


        //Enable PA. - 060120 - We'll want to move this into the loop_q_iter to minimize the time the PA is on by turning it off during SPI and BTLE transfers.
        rfidr_error_code=rfidr_enable_pa();
            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"enabling pa",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //Set check I or check Q
        for(loop_iq=0;loop_iq<=1;loop_iq++)
        {
            if(loop_iq % 2 == 0)
                rfidr_error_code=set_use_i();
            else
                rfidr_error_code=set_use_q();
            
            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"set_use_i/q",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

            //Set new query each time we do a round. This is an FPGA setting that alerts the state machine that a new inventory round is starting.
            //In this case, the query packet is sent during the first set of TX commands, followed by query rep. packets in between tag reads on subsequent TX commands.
            rfidr_error_code=set_alt_radio_fsm_loop();
                if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"setting new query flag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

            //Run through a query round
            //We basically re-run the query round for Q after doing I.
            //Why do we have the query as the inner loop instead of I/Q? Let's say I/Q was the inner loop and the tag was only accessible on Q channel.
            //Then the "I" channel sense would always result in the tag replying but not getting read and then going back to the arbitrate state with the slot counter set to maximum.
            //We'd never read that tag as a result!
            for(loop_q_iter=0;loop_q_iter <= (1 << q_value);loop_q_iter++)
            {   //The <= is to ensure no tag backscatters RN16 again at a query.
                //set_sx1257_lna_gain((uint8_t)(0xD4)); //Don't reset the LNA gain. The idea is that convergence won't change a whole lot in between rounds.

                //Set the FPGA IRQ flag to false so we can wait for the IRQ.
                m_received_irq_flag    =    false;
                //Now that the FPGA is loaded with settings and commands, tell it to execute those commands.
                rfidr_error_code=set_go_radio_oneshot();
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"set go radio",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                //Wait for IRQ back from FPGA.
                while(m_received_irq_flag==false){}
                //ACK the FPGA when we receive the IRQ so the FPGA can transition its state.
                rfidr_error_code=set_irq_ack_oneshot();
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"acking irq",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

                //Was the operation a success? This time it's important because otherwise we don't store the epc.
                //Also, the original plan was to store all of the data in RAM and _then_ send it out.
                //What is the time required to execute one indication transaction?
                //Also in this case we wouldn't care about the stats, so we'd avoid sending ble packet 2.
                //On the other hand, we can't afford a missed notification
                //Actually what we can do is use the indication handler to shut down transactions (through a different state) if we end up missing an indication.
                //This way writing the data back to the phone becomes nonblocking.
                
                //This part is a bit interesting because we are in the middle of the state machine when we get the IRQ.
                //The state machine has stopped at this point and passed back control to the MCU.
                //When the FPGA state machine gets another "go_radio" it will start executing again.
                if(read_radio_exit_code()==0)
                {
                    m_num_inv_tags_found++;
                        if(m_num_inv_tags_found >= max_tags){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"inventoried more than max # tags",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    
                    return_struct->i_pass=false;
                    return_struct->q_pass=false;
                    for(loop_load=0; loop_load < MAX_EPC_LENGTH_IN_BYTES; loop_load++)
                    {
                        return_struct->i_epc[loop_load]=0;
                        return_struct->q_epc[loop_load]=0;
                    }
                    return_struct->i_lna_gain=0xD4;
                    return_struct->q_lna_gain=0xD4;
                    return_struct->i_main_mag=0;
                    return_struct->i_alt_mag=0;
                    return_struct->q_main_mag=0;
                    return_struct->q_alt_mag=0;
                    
                    if(loop_iq==0)
                    {
                        return_struct->i_pass=true;
                        rfidr_error_code=rfidr_read_epc(return_struct->i_epc,READ_RXRAM_REGULAR);
                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking I EPC", rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                        rfidr_error_code=set_last_inv_epc(return_struct->i_epc);
                            //There should be no error here, since this is strictly an MCU internal operation.
                        rfidr_error_code=rfidr_read_main_magnitude(&(return_struct->i_main_mag),READ_RXRAM_REGULAR);
                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking I - Main Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                        rfidr_error_code=rfidr_read_alt_magnitude(&(return_struct->i_alt_mag),READ_RXRAM_REGULAR);
                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking I - Alt Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    }
                    else
                    {
                        return_struct->q_pass=true;
                        rfidr_error_code=rfidr_read_epc(return_struct->q_epc,READ_RXRAM_REGULAR);
                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking Q EPC", rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                        rfidr_error_code=set_last_inv_epc(return_struct->q_epc);
                                //There should be no error here, since this is strictly an MCU internal operation.
                        rfidr_error_code=rfidr_read_main_magnitude(&(return_struct->q_main_mag),READ_RXRAM_REGULAR);
                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking Q - Main Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                        rfidr_error_code=rfidr_read_alt_magnitude(&(return_struct->q_alt_mag),READ_RXRAM_REGULAR);
                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking Q - Alt Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    }
                    
                    m_received_hvc_pckt_data1_flag            =    false;
                    rfidr_error_code=rfidr_push_data_over_ble(p_rfidrs,return_struct,return_struct,recover_frequency_slot,255,m_hopskip_nonce,BLE_PUSH_MINIMAL);
                        if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"pushing pckt data over ble",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    while(m_received_hvc_pckt_data1_flag == false){}
                }

                //If we've just transmitted a Query Adjust packet, reload the Query Rep TX RADIO RAM with a regular Query Rep packet and continue.
                if(query_adj_burn_flag == true && loop_q_iter > 0)
                {
                    rfidr_error_code=load_query_rep_packet();
                    query_adj_burn_flag=false;
                }
            } //for loop_q_iter
        } //for loop_iq
    
        //Disable PA. Can't have too many of these, but PA enable/disable should move within the innermost loop to not leave PA on needlessly during BTLE and SPI transfers.
        rfidr_error_code=rfidr_disable_pa();
        if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"disabling pa: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        
    } //for loop_query_q
    
    rfidr_error_code=end_inventory(p_rfidrs,"End Inv.");
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"ending inventory",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    rfidr_enable_led1(); //Stop LED toggling

    //Finally report the number of tags we found. iDevice software will report how long it took to find them.
    sprintf(short_message,"InventryFnd %03dTags",(uint8_t)(m_num_inv_tags_found & 255));
    send_short_message(p_rfidrs, short_message);
    
    return RFIDR_SUCCESS;
    
}

//tracking_core is the core function for tag-tracking modes.
//This function is similar to inventory core, but runs indefinitely, supports PDOA ranging, and has some intelligence
//on how to set up the query_q vector for the actual tracking operation.
//As such, it's currently the most complicated function in this file.
//We don't allow EPC unions here since we'll need the two select packets for setting both the select and session flags of the tags.

static rfidr_error_t tracking_core(ble_rfidrs_t *p_rfidrs, char *error_info, rfidr_query_session_t session, rfidr_tracking_mode_t mode, rfidr_return_t *return_struct_ant, rfidr_return_t *return_struct_cal)
{
    #define    TRACK_MAX_QUERY_Q                5            //We want to limit this to limit frequency dwell time so we don't have to hop in the middle of a query round.
    #define    TRACK_QUERY_ROUND_LIMIT          16           //We want to be able to run through as many queries as we can before needing to frequency hop.
    //For 11ms per tag query (10ms has PA on), this means we can query 32 tags before we need to hop frequencies (recall the limit is 400ms per frequency).
    //But, we must split this up between I and Q, so we can only do 16 rounds of 0 per I or Q.
    #define    TRACK_MAX_INV_TAGS               32           //We want the maximum number of tags to fit in a query Q of 5.
    #define    NUM_ALLOWED_OUTER_CAL_FAILS      3            //If the cal operation fails, we give a limited number of retries before ending this function.
    #define    NUM_ALLOWED_INNER_CAL_FAILS      5            //If the cal operation fails, we give a limited number of retries before ending this function.

    rfidr_error_t            rfidr_error_code                              =    RFIDR_SUCCESS;    //An output error code.
    uint8_t                  num_track_loops                               =    0;
    uint8_t                  loop_query_q                                  =    0;        //Loop iteration value between query rounds.
    uint8_t                  loop_iq                                       =    0;        //Loop iteration value between I and Q sensing in the reader.
    uint8_t                  loop_load                                     =    0;        //Loop iteration value for loading arrays.
    uint8_t                  loop_cal_fails_outer                          =    0;        //If we fail repeatedly in a calibration the calibration tag may be in a null. Hop frequencies.
    uint8_t                  loop_cal_fails_inner                          =    0;        //Retry each calibration just in case it fails to detect the PDOA calibration tag or tag equivalent.
    uint16_t                 loop_q_iter                                   =    0;        //Loop iteration variable for within the query round.
    char                     query_q_vector[TRACK_QUERY_ROUND_LIMIT+1]     =    {0};    //The +1 is to hold the null value, which we are using as a sentinel value.
    static const char        q_q_vector_def0[TRACK_QUERY_ROUND_LIMIT+1]    =    {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0',0};    //Default query Q vector when we have 1 tag.
    //In theory, expect to hit on all query "reps" for perfect channel above for 1 tag.
    static const char        q_q_vector_def1[TRACK_QUERY_ROUND_LIMIT+1]    =    {'3','2','1','1','1','1','1','1','1','1','1','1','1','1','1','1',0};    //Default query Q vector when we have 2 tags.
    //In theory, expect to hit on 50% on all query reps for perfect channel above for 2 tags.
    //So in this case, we can roughly use 16 query rounds (probably a bit less) and still meet US FCC hopping requirements in the worst case.
    static const char        q_q_vector_def2[TRACK_QUERY_ROUND_LIMIT+1]    =    {'3','3','2','2','2','2','2','2',0,0,0,0,0,0,0,0,0};    //Default query Q vector when we are close to 4 tags.
    //Some very loose hand analysis suggests that the pattern above is the best for 3 or 4 tags.
    //We expect to hit about 2 tags and miss 2 tags for each query round, so we end after 8 query rounds.
    static const char        q_q_vector_def3[TRACK_QUERY_ROUND_LIMIT+1]    =    {'4','4','3','3',0,0,0,0,0,0,0,0,0,0,0,0,0};    //Default query Q vector when we are close to 8 tags.
    //The pattern above is pattern-matched from q_q_vector_def2
    static const char        q_q_vector_def4[TRACK_QUERY_ROUND_LIMIT+1]    =    {'5','5',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //Default query Q vector when we are close to 16 tags.
    static const char        q_q_vector_def5[TRACK_QUERY_ROUND_LIMIT+1]    =    {'6',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //Default query Q vector when we are close to 32 tags.
    uint8_t                  q_value                                       =    0;        //What is the query Q value for a given query round, in bytes.
    uint8_t                  recover_frequency_slot                        =    0;        //A variable to fish out what frequency slot we hopped to in rfidr_sx1257.c.
    uint8_t                  skip_frequency_slot                           =    0;        //A variable to hold what frequency we will skip to for PDOA ranging.
    uint8_t                  epc_length_in_bytes                           =    0;        //Variable to fish out what the application specified EPC length is.
    uint8_t                  session_flag_flip_limit                       =    1;        //Flip the session flag after these numbers of tags have been found.
    uint8_t                  num_tracked_tags_found                        =    0;        //Track how many successes we are having. When we have enough, flip the session flag target.
    uint32_t                 num_total_tags_found                          =    0;        //For preliminary reporting purposes, count how many total tags we access.
    bool                     frequency_skip_flag                           =    false;    //Set this when it is time to skip (by 1) a frequency so that we don't get PDOA aliasing.
    bool                     query_a_flag                                  =    true;    //020520 - This flag represents that the query "A" flag has been transmitted and is in play for the current round.
                                                                                //This is not robust coding practice, but will speed up tags reads for the current UM goals.
    char                    short_message[20]                              =    {0};    //An array to hole a short message back to the iDevice.
    rfidr_select_target_t    target                                        =    TARGET_S0;

    //EPC values of less than or equal to 12 bytes can be used here.
    //EPC values of greater than 12 bytes will be truncated by the called function.

    //First thing that happens is, if we are using an app-specd EPC, we run an inventory on it in case there are multiple tags in the area.
    //We want to determine the number of tags (roughly - assume we get 75% of them) to set the Q accordingly for subsequent tracking.
    //Running this will set m_num_inv_tags_found.

    //Note that the inventory core function natively searches for the app-specd EPC.

    rfidr_sel_ant0(); //Switch to ant0 in order to use the main antenna for operations on actual tags.

    if(mode == TRACK_APP_SPECD)
    {
        //Run an inventory to see how many tags in the area match the app-specd EPC.
        //Put a dummy all-zero value in for EPC2 since none of the tags are likely to match this.
        rfidr_error_code=inventory_core(p_rfidrs, "track-inv-internal", SESSION_S3, "6666655555444444433333333322", TRACK_MAX_INV_TAGS, "000000000000000000000000", return_struct_ant);
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"tracking-inventory",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //Based on the number of tags found, we're going to set up the query_q_vector.
        //We can also set up the query_q_vector adaptively on-the-fly but decline to do so for now.
        //If the inventory core only found 1 tag, the functions we write below need to
        //degenerate to a query q vector length of 16(QUERY_ROUND_LIMIT) with a value of '0'.
        //Similarly, if we are going to track the last inventoried EPC, we want the query q vector to
        //have a length of 16(QUERY_ROUND_LIMIT) with a value of '0' (note we are using NULL (0) as a sentinel value for the vector).

        //We're going to do this in an easy to understand and readable way, not a compact way.
        //We probably aren't saving that many lines of code by doing it this way, either.
        //Vector copying courtesy of: https://stackoverflow.com/questions/3535410/assign-multiple-values-to-array-in-c

        if(m_num_inv_tags_found == 0)
        {
            handle_error(p_rfidrs,error_info,"No tags inv'd for tracking: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;
        }
        else if(m_num_inv_tags_found <= 1)
        {
            memcpy(query_q_vector, q_q_vector_def0, sizeof(q_q_vector_def0));
            session_flag_flip_limit    =    1;
        }
        else if(m_num_inv_tags_found <= 2)
        {
            memcpy(query_q_vector, q_q_vector_def1, sizeof(q_q_vector_def1));
            session_flag_flip_limit    =    2;
        }
        else if(m_num_inv_tags_found <= 4)
        {
            memcpy(query_q_vector, q_q_vector_def2, sizeof(q_q_vector_def2));
            session_flag_flip_limit    =    3;
        }
        else if(m_num_inv_tags_found <= 8)
        {
            memcpy(query_q_vector, q_q_vector_def3, sizeof(q_q_vector_def3));
            session_flag_flip_limit    =    7;
        }
        else if(m_num_inv_tags_found <= 16)
        {
            memcpy(query_q_vector, q_q_vector_def4, sizeof(q_q_vector_def4));
            session_flag_flip_limit    =    14;
        }
        else //We should max out at 32 tags
        {
            memcpy(query_q_vector, q_q_vector_def5, sizeof(q_q_vector_def5));
            session_flag_flip_limit    =    30;
        }
    }
    else //If we are conducting a search for the last inventoried tag
    {
        memcpy(query_q_vector, q_q_vector_def0, sizeof(q_q_vector_def0));
        session_flag_flip_limit    =    1;
    }

    //Second thing is that we enter a while loop.
    //This function can only be broken when we get the tracking flag set to false or if we exit and go to handle_error.
    //When either of these things happen, we need to cleanly exit inventory by using an end_inventory function.

    while(m_track_tag_state_flag == true)
    {
        sprintf(short_message,"NumTrackLoop-%03d",num_track_loops++);
        send_short_message(p_rfidrs, short_message);
        
        //We need to hop frequencies on a regular basis to comply with FCC section 15.247.
        //We can't transmit on a given frequency for greater than 0.4s in a 10 second period.
        //Each query rep interval lasts about 10ms when returning data and during this time we have the PA on almost the whole time.
        //Given that we should not frequency hop between I and Q variants (we want to search both I and Q for a given frequency),
        //we really need to set Q <= 5 since we'll be doing a few query rounds after the main one to pick up any stray tags.
        //In addition, we need to try having the PA on only when absolutely necessary for RFID traffic.
        //In other words, turn off the PA for BTLE transfers and while moving data off of the FPGA.

        //Now in addition to hopping frequencies, we need to "skip" frequencies too.
        //The reason for this is that we can only shift three frequency bin to do PDOA without getting ranging aliasing up to 25 meters.
        //So we ping-pong between pseudo-random hops and skips.
        //The app host software will know whether we are hopping or skipping, since this information will be transmitted over BTLE packets
        //at the end of this function.

        for(loop_cal_fails_outer=0; loop_cal_fails_outer < NUM_ALLOWED_OUTER_CAL_FAILS; loop_cal_fails_outer++)
        {
            if(frequency_skip_flag == false)    //We skipped last time, so now it's time to hop. We assume our hopping algorithm complies with FCC rules.
            {
                rfidr_error_code=hop_sx1257_frequency(&recover_frequency_slot);    m_hopskip_nonce++;//Hop frequency and figure out what slot we hopped to.
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"hopping frequency",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
            }
            else    //Now we frequency skip.
            {
                if(recover_frequency_slot >= 22)    //If we are at the uppermost edge of the frequency band, we can't hop 'up'.
                    skip_frequency_slot = recover_frequency_slot-(3-loop_cal_fails_outer);     //So we hop 3 down instead.
                    //This will result in slots 22, 23, 24 being unfairly weighted, so we'll need a better algorithm later if we want FCC certification.
                    //If we are failing at the skip frequency, we need to try another nearby frequency.
                    //App software should handle the different hop/skip delta frequency.
                else
                    skip_frequency_slot = recover_frequency_slot+(3-loop_cal_fails_outer);    //In general, we will skip by moving to the next third highest frequency slot.
                    //If we are failing at the skip frequency, we need to try another nearby frequency.
                    //App software should handle the different hop/skip delta frequency.

                rfidr_error_code=set_sx1257_frequency(skip_frequency_slot);    //Set the frequency slot.
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"skipping frequency",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
            }
            //The first thing that needs to happen after a frequency hop is to run a search on the dummy tag to calibrate for PDOA.

            //We run a search core on the dummy tag for phase calibration and collect the results
            //We do need to figure out how to modify this so that select packets aren't used in order to cut the overhead of this operation.
            //Select packets really aren't needed since it's just the one tag sitting there, just a query with q=0 is all that's needed.

            //Give this a couple of times to succeed. In the proposed next gen reader, the tracking tag will be a chip on the reader
            //so in that case it should never fail.
            for(loop_cal_fails_inner=0; loop_cal_fails_inner < NUM_ALLOWED_INNER_CAL_FAILS; loop_cal_fails_inner++)
            {
                rfidr_error_code=search_core(p_rfidrs, "track-searching", SESSION_S0, TARGET_CAL_EPC, RETURN_EPC_NO, RETURN_MAG_YES, RETURN_LNA_GAIN_NO, return_struct_cal);
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"PDOA cal",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

                if(return_struct_cal->i_pass == true || return_struct_cal->q_pass == true){break;} //If we passed search, break the loop.
            
                sprintf(short_message,"TrackCalFailInner%01d",loop_cal_fails_inner); //If we didn't pass search, retry and report the error.
                send_short_message(p_rfidrs, short_message);
            }
            
            if(return_struct_cal->i_pass == true || return_struct_cal->q_pass == true){break;} //If we passed search within X tries, break the loop.
            
            sprintf(short_message,"TrackCalFailOuter%01d",loop_cal_fails_outer); //If we failed search after X tries, retry the outer loop and report as much.
            send_short_message(p_rfidrs, short_message);
        }
        
        //If we do end up continuing to fail after trying to succeed, throw and error and exit from the function.
        if(return_struct_cal->i_pass == false && return_struct_cal->q_pass == false)
        {
            end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"tracking calibration",rfidr_error_code); 
            return RFIDR_ERROR_GENERAL;
        }
        
        //Flip frequency skip flag after the big retry loop above. Changing the flag in the retry loop will result in moving between hop and skip in the retry loop.
        if(frequency_skip_flag == false)    //We skipped last time, so now it's time to hop. We assume our hopping algorithm complies with FCC rules.
        {
            frequency_skip_flag = true;    //Next time we are here, we need to do a frequency skip.
        } 
        else 
        {
            frequency_skip_flag = false; //Next time, we do a real frequency hop.
        }
        
        rfidr_toggle_led1(); //Toggle LED slowly to show that the reader is doing something.

        //Load TX RAM, getting ready for actual tracking
        //In this code, we select only tags that comply with one EPC (truncated or not)
        //By setting the EPC to a zero-length EPC, one ends up tracking all flags. Please don't do this for tracking mode.
        switch(session)
        {
            case SESSION_S0: target=TARGET_S0; break;
            case SESSION_S1: target=TARGET_S1; break;
            case SESSION_S2: target=TARGET_S2; break;
            case SESSION_S3: target=TARGET_S3; break;
            default: target=TARGET_S0; break;
        }

        set_select_target(target);       //Target "inventoried" flag of selected session with the Query. //No possible error, so don't check.
        set_select_action(ACTION_A0);    //With the first select packet, set the session flag to A for tags complying with EPC and to B for those not complying. //No possible error, so don't check.
        set_query_sel(SEL_PSL);          //Query packet will target only tags with their select flag set. //No possible error, so don't check.
        set_query_session(session);      //Query packet will target the session specified by the calling function. //No possible error, so don't check.
        set_query_target(TARGET_A);      //Query packet will target flags with their session flag set to A. //No possible error, so don't check.
        set_query_q(0);                  //Set query q to a fixed value for now, this will be overwritten later.
        query_a_flag = true;             //Set this to true to initialize this at the beginning of every loop.

        //Load a full query packet
        rfidr_error_code=load_query_packet_only(FLAGSWAP_NO);
            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"loading full query",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //Load a select packet into the FPGA TX RAM with an EPC that we want to act as a select mask.
        //In other words, all of the packets with this tag EPC value will have their session flags set to "A" and all others will have the flags set to "B".
        //We need to rewrite FPGA TX RAM because we've just overwritten the select packet EPC field for the calibration tag search.
        //In the future, we can take the select packet RAM writes outside of the loop and just do them once, since
        //the calibration tag won't be singulated with a select packet (it's connectorized, and the only tag in its channel, so we can just do a query with q=0 to read it).

        if(mode == TRACK_APP_SPECD)
        {
            rfidr_error_code=read_length_app_specd_target_epc(&epc_length_in_bytes); //No real error, so don't check it.
            rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,epc_length_in_bytes, SEL_PACKET_NO_1);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading app specd epc no 1",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        }
        else //We are tracking the last inventoried tag
        {
            rfidr_error_code=load_select_packet_only(LAST_INV_EPC,MAX_EPC_LENGTH_IN_BYTES, SEL_PACKET_NO_1);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading last inv epc no 1",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        }

        //Next we set the second select packet to set the select flag to "selected" on all tags that also comply with EPC
        set_select_target(TARGET_SL);
        //Load the second select packet into TX RAM.
        //Why are we doing this? Because we're not going to be able to move all tags into one session flag before switching session flags, we'll
        //usually have a mix of tags in both "A" and "B" session flags. In order to ensure that Query is targeting only the tags of interest, therefore,
        //we also need to use the select flag to activate the set of tags to be tracked.

        if(mode == TRACK_APP_SPECD)
        {
            //Don't need to read epc length again.
            rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,epc_length_in_bytes, SEL_PACKET_NO_2);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading app specd epc no 2",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        }
        else //We are tracking the last inventoried tag
        {
            rfidr_error_code=load_select_packet_only(LAST_INV_EPC,MAX_EPC_LENGTH_IN_BYTES, SEL_PACKET_NO_2);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading last inv epc no 2",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
        }

        //Tell the FPGA to operate its state machine as per inventory rules.
        rfidr_error_code=set_radio_mode_inventory();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set radio mode to inventory",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //Load the Query Rep packet into the QUERY REP slot of the FPGA TX RAM.
        rfidr_error_code=load_query_rep_packet();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading query rep",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //We also set the flag in the FPGA to use the select packet on the first packet to be sent out.
        rfidr_error_code=set_use_select_pkt();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting select packet",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        set_sx1257_lna_gain((uint8_t)(0xD4));                     //Reset the LNA gain. Not sure if we want it here or within the I/Q loop below as of 120719.

        //Set query q and load the appropriate query packet. This needs to be done each time Q is changed.
        //In this case, we want to set Q to be consistent with the number of tags from the last inventory.
        //Or, in the case where we are targeting the last tag of last inventory, we use Q=0;

        for(loop_query_q=0;(*(query_q_vector+loop_query_q) != 0) || loop_query_q < TRACK_QUERY_ROUND_LIMIT;loop_query_q++)
        {
            q_value=(uint8_t)(*(query_q_vector+loop_query_q)-'0');                  //Supposedly we got an integer char input. Subtract '0' (48) to do a char to integer conversion.
            q_value=(q_value > TRACK_MAX_QUERY_Q) ? TRACK_MAX_QUERY_Q : q_value;    //Sanitize q_value. Make sure we don't send through anything smaller than 0 and bigger than MAX_QUERY_Q.
            //The type of q_value should enforce minimum of 0.

            rfidr_error_code=set_query_q(q_value);                                  //Set the query Q in the query packet to be sent out.

            //Enable PA. - 060120 - We'll want to move this into the loop_q_iter to minimize the time the PA is on by turning it off during SPI and BTLE transfers.
            //120920 - Actually let's keep it here for now. SPI can be sped up and BTLE will speed up with move to 5.0. Powering off the tags during an inventory round
            //will kick them all out of the round.
            rfidr_error_code=rfidr_enable_pa();
                if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"enabling pa",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

            //Set check I or check Q
            for(loop_iq=0;loop_iq<=1;loop_iq++)
            {
                if(loop_iq % 2 == 0)
                    rfidr_error_code=set_use_i();
                else
                    rfidr_error_code=set_use_q();

                if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"set_use_i/q",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

                //Load a query packet, updating only the A/B inventory flag and the CRC bits (061720 - Also do the query Q).
                rfidr_error_code=load_query_packet_only(FLAGSWAP_YES);
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"loading query a/b flagswap",rfidr_error_code); return RFIDR_ERROR_GENERAL;}


                //Set new query each time we do a round. This is an FPGA setting that alerts the state machine that a new inventory round is starting.
                //In this case, the query packet is sent during the first set of TX commands, followed by query rep. packets in between tag reads on subsequent TX commands.
                rfidr_error_code=set_alt_radio_fsm_loop();
                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"setting new query flag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

                //Run through a query round
                //We basically re-run the query round for Q after doing I.
                //Why do we have the query as the inner loop instead of I/Q? Let's say I/Q was the inner loop and the tag was only accessible on Q channel.
                //Then the "I" channel sense would always result in the tag replying but not getting read and then going back to the arbitrate state with the slot counter set to maximum.
                //We'd never read that tag as a result!
                for(loop_q_iter=0;loop_q_iter <= (1 << q_value);loop_q_iter++)
                {   //The <= is to ensure no tag backscatters RN16 again at a query.
                    //set_sx1257_lna_gain((uint8_t)(0xD4)); //Don't reset the LNA gain. The idea is that convergence won't change a whole lot in between rounds.

                    //Set the FPGA IRQ flag to false so we can wait for the IRQ.
                    m_received_irq_flag    =    false;
                    //Now that the FPGA is loaded with settings and commands, tell it to execute those commands.
                    rfidr_error_code=set_go_radio_oneshot();
                        if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"set go radio",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                    //Wait for IRQ back from FPGA.
                    while(m_received_irq_flag==false){}
                    //ACK the FPGA when we receive the IRQ so the FPGA can transition its state.
                    rfidr_error_code=set_irq_ack_oneshot();
                        if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"acking irq",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

                    //Was the operation a success? This time it's important because otherwise we don't store the epc.
                    //Also, the original plan was to store all of the data in RAM and _then_ send it out.
                    //What is the time required to execute one indication transaction?
                    //Also in this case we wouldn't care about the stats, so we'd avoid sending ble packet 2.
                    //On the other hand, we can't afford a missed notification
                    //Actually what we can do is use the indication handler to shut down transactions (through a different state) if we end up missing an indication.
                    //This way writing the data back to the phone becomes nonblocking.

                    //This part is a bit interesting because we are in the middle of the state machine when we get the IRQ.
                    //The state machine has stopped at this point and passed back control to the MCU.
                    //When the FPGA state machine gets another "go_radio" it will start executing again.
                    if(read_radio_exit_code()==0)
                    {
                        num_tracked_tags_found++;    //Cool beans, we singulated a tag and flipped its session tag.
                        num_total_tags_found++;

                        return_struct_ant->i_pass=false;
                        return_struct_ant->q_pass=false;
                        for(loop_load=0; loop_load < MAX_EPC_LENGTH_IN_BYTES; loop_load++)
                        {
                            return_struct_ant->i_epc[loop_load]=0;
                            return_struct_ant->q_epc[loop_load]=0;
                        }
                        return_struct_ant->i_lna_gain=0xD4;
                        return_struct_ant->q_lna_gain=0xD4;
                        return_struct_ant->i_main_mag=0;
                        return_struct_ant->i_alt_mag=0;
                        return_struct_ant->q_main_mag=0;
                        return_struct_ant->q_alt_mag=0;

                        if(loop_iq==0)
                        {
                                return_struct_ant->i_pass=true;
                                rfidr_error_code=rfidr_read_epc(return_struct_ant->i_epc,READ_RXRAM_REGULAR);
                                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking I EPC", rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                                rfidr_error_code=rfidr_read_main_magnitude(&(return_struct_ant->i_main_mag),READ_RXRAM_REGULAR);
                                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking I - Main Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                                rfidr_error_code=rfidr_read_alt_magnitude(&(return_struct_ant->i_alt_mag),READ_RXRAM_REGULAR);
                                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking I - Alt Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                            }
                            else
                            {
                                return_struct_ant->q_pass=true;
                                rfidr_error_code=rfidr_read_epc(return_struct_ant->q_epc,READ_RXRAM_REGULAR);
                                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking Q EPC", rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                                rfidr_error_code=rfidr_read_main_magnitude(&(return_struct_ant->q_main_mag),READ_RXRAM_REGULAR);
                                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking Q - Main Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                                rfidr_error_code=rfidr_read_alt_magnitude(&(return_struct_ant->q_alt_mag),READ_RXRAM_REGULAR);
                                    if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"checking Q - Alt Mag",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                            }

                            m_received_hvc_pckt_data1_flag            =    false;
                            //We need to tell the iDevice whether the data being sent over corresponds to a hop (first PDOA value) or a skip (second PDOA value).
                            //Note that if we are getting data from a frequency hop, the skip flag is "true" because the next time around it will
                            //be time to skip.
                            
                            if(frequency_skip_flag == true) //Remember if we are at a hop we will have already set this flag to true
                            {
                                rfidr_error_code=rfidr_push_data_over_ble(p_rfidrs,return_struct_ant,return_struct_cal,recover_frequency_slot,255,m_hopskip_nonce,BLE_PUSH_SUPPLEMENT);
                            }
                            else
                            {
                                rfidr_error_code=rfidr_push_data_over_ble(p_rfidrs,return_struct_ant,return_struct_cal,skip_frequency_slot,loop_cal_fails_outer,m_hopskip_nonce,BLE_PUSH_SUPPLEMENT);
                            }

                            if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"pushing pckt data over ble",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
                            while(m_received_hvc_pckt_data1_flag == false){}
                        }

                    } //for loop_q_iter

                    //After each Query round, we want to ask ourselves if we've flipped enough tags to justify switching the session flag target value.
                    //If so, we're going to change the query target. Also reset the number of tracked tags.
                    if(num_tracked_tags_found >= session_flag_flip_limit)
                    {
                        num_tracked_tags_found = 0;

                        if(query_a_flag == true)
                        {
                            set_query_target(TARGET_B);
                            query_a_flag = false;
                        }
                        else
                        {
                            set_query_target(TARGET_A);
                            query_a_flag = true;
                        }
                    }

                } //for loop_iq

                //Disable PA. Can't have too many of these, but PA enable/disable should move within the innermost loop to not leave PA on needlessly during BTLE and SPI transfers.
                rfidr_error_code=rfidr_disable_pa();
                if(rfidr_error_code != RFIDR_SUCCESS){end_inventory(p_rfidrs,"End Inv."); handle_error(p_rfidrs,error_info,"disabling pa: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

            } //for loop_query_q

        //Need to end inventory so that we can do search after we hop frequencies.
        rfidr_error_code=end_inventory(p_rfidrs,"End Inv.");
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"ending tracking",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    } //For while(m_track_tag_state_flag == true) If we're here, that means the iDevice set the flag to false and it's time to exit this function.

    rfidr_enable_led1(); //Stop LED toggling.

    //Finally report the number of tags we found. iDevice software will report how long it took to find them.
    sprintf(short_message,"Fnd %08lu Tags",(uint32_t)(num_total_tags_found & ((1 << 24)-1)));
    send_short_message(p_rfidrs, short_message);

    return RFIDR_SUCCESS;

}

static rfidr_error_t program_core(ble_rfidrs_t *p_rfidrs, char *error_info, rfidr_query_session_t session, rfidr_target_epc_t target, rfidr_program_content_t content, rfidr_return_t *return_struct)
{
    #define    MAX_PROG_RETRIES        5    //Define the maximum number of program retries once we get into the tag open/secured state.
    
    rfidr_error_t    rfidr_error_code                            =    RFIDR_SUCCESS;

    uint8_t          program_q                                   =    0;    //1 if q, 0 if i
    uint8_t          loop_prog_retry                             =    0;
    uint8_t          write_cntr                                  =    0;
    char             short_message[20]                           =    {0};
    
    //Load TX RAM, getting ready for program of specific tag and read back.
    set_select_target(TARGET_SL);    //No possible error, so don't check.
    set_select_action(ACTION_A0);    //No possible error, so don't check.
    set_query_sel(SEL_PSL);          //No possible error, so don't check.
    set_query_session(session);
    set_query_target(TARGET_A);
    set_query_q(0);                  //Targeted search requires immediate reply by the tag of interest, so set query Q=0
    
    //Next, determine which of I and Q we will attempt the program operation on.
    //We will go with the one that has the higher magnitude.
    //These actions assume that a search_core or inventory_core function was executed right before this and a valid return_struct was filled out.

    if(return_struct->i_pass==true)
        program_q    =    (return_struct->i_main_mag > return_struct->i_alt_mag) ? 0 : 1;
    else if(return_struct->q_pass==true)
        program_q    =    (return_struct->q_main_mag > return_struct->q_alt_mag) ? 1 : 0;
    else{ //We should not be here, but if we are, then bail with an error.
        handle_error(p_rfidrs,error_info,"determining which of I and Q channel to receive on",RFIDR_ERROR_GENERAL); return RFIDR_ERROR_GENERAL;
    }
    
    if(program_q==0)
        rfidr_error_code=set_use_i();
    else
        rfidr_error_code=set_use_q();
    
    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set which of I or Q to use during programming",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    
    switch(target)
    {
        case TARGET_APP_SPECD_EPC:    rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
        case TARGET_LAST_INV_EPC:     rfidr_error_code=load_select_packet_only(LAST_INV_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
        default:                      rfidr_error_code=load_select_packet_only(APP_SPECD_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_1); break;
    }

        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading select packet 1",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Next we set the second select packet to set the session flag to "A" on all tags that also comply with EPC2
    set_select_action(ACTION_A1);         //No possible error, so don't check.

    //Set the zero packet as the second select packet EPC so that we have known behavior here from the reader.
    rfidr_error_code=load_select_packet_only(ZERO_EPC,MAX_EPC_LENGTH_IN_BYTES,SEL_PACKET_NO_2);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading select packet 2",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    //Load query packet into TX RAM. This had to be done each time one of query select, session, target, or Q are changed.
    //Since we are doing query Q=0 we don't need to load the Query Rep or Query Adjust packets.
    rfidr_error_code=load_query_packet_only(FLAGSWAP_NO);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading query",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //Tell the FPGA to conduct its state machine as per its search sequence of packets.
    rfidr_error_code=set_radio_mode_program();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set radio mode to program",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    //We also set the flag in the FPGA to use the select packet on the first packet to be sent out.
    rfidr_error_code=set_use_select_pkt();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting select packet",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    
    //Load the write packet with a new EPC from the phone.
    
    switch(content)
    {
        case PROGRAM_NEW_EPC:            rfidr_error_code=load_write_packet_only_program_epc();        break; //Load the new user specified EPC into the Write portion of the TX RAM.
        case PROGRAM_KILL_PWD:           rfidr_error_code=load_write_packet_only_kill_password();      break; //Load the program kill password into the Write portion of the TX RAM.
        case KILL_COMMAND:               rfidr_error_code=load_write_packet_only_kill_command();       break; //Load the kill command into the Write portion of the TX RAM.
    }
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"loading write packet: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    
    if(content==KILL_COMMAND){rfidr_error_code=set_use_kill_pkt();}
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"setting kill packet bit: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    
    //Note - we aren't hopping frequencies here since we want to operate at the last frequency we operated at.
    //This way, we program at a frequency and i/q phase that is known-good for accessing the tag.
    //Enable PA.
    rfidr_error_code=rfidr_enable_pa();
        if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"enabling pa: ",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
    
    //Run the radio. Wait for IRQ. If it comes back with an error, exit and tell this to user.
    set_sx1257_lna_gain((uint8_t)(0xD4)); //Reset the LNA gain

    //120620 - Add retry capability to programming so that the program operation is more robust.
    //We seem to have an issue (and will always in principle have this issue) where the programming fails on one of the writes, read, or lock commands.
    //Depending on where we fail, different things need to happen:
    //Fail prior to the tag entering the open/secured states (i.e. after the handle is received): quit and start over.
    //Fail while we are in the open/secured states and we are within the allotted number of retries: keep trying, but issue a notification of fail and where.
    //Fail while we are in the open/secured states and we are not within the alotted number of retries: quit and issue notification of where the last failure occurred.
    //After a partial programming failure and removing power to the tag, the tag will have an intermediate EPC between the old and new one.
    //In prinicple we can deal with that also, knowing where the write operation failed, we can construct the actual EPC remaining on the tag.
    //and try to program it again.

    for(loop_prog_retry = 0; loop_prog_retry <= MAX_PROG_RETRIES+1; loop_prog_retry++)
        {
        m_received_irq_flag    =    false;

        rfidr_error_code=set_go_radio_oneshot();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"set go radio",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        //Wait for FPGA IRQ. Check to make sure that we didn't get back an error during the programming sequence.
        while(m_received_irq_flag==false){}
        //ACK the FPGA IRQ.
        rfidr_error_code=set_irq_ack_oneshot();
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"acking irq",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

        if(read_radio_exit_code() != 0 && loop_prog_retry < MAX_PROG_RETRIES)
        {
            write_cntr=read_radio_write_cntr();
            sprintf(short_message,"Prg.FailAt%01d-Retry",write_cntr);
            send_short_message(p_rfidrs, short_message);
        }
        else if (read_radio_exit_code() != 0 && loop_prog_retry == MAX_PROG_RETRIES)
        {
            write_cntr=read_radio_write_cntr();
            sprintf(short_message,"Prg.FailAt%03d-End",write_cntr);
            send_short_message(p_rfidrs, short_message);
            set_end_radio_fsm_loop();
            //If we fail, we want to exit the rfidr radio state machine.
            //For this to work, we need to run this loop one more time to run the go radio.
        }
        else if (read_radio_exit_code() != 0 && loop_prog_retry >= MAX_PROG_RETRIES)
        {
            //This condition should not happen - peripheral should return an exit code here.
            //We'll flag this an an error.
            handle_error(p_rfidrs,error_info,"prog. undef'd condt'n",rfidr_error_code); return RFIDR_ERROR_GENERAL;
        }
        else if (read_radio_exit_code() == 0 && loop_prog_retry >= MAX_PROG_RETRIES)
        {
            //We failed to program the tag but at least were able to exit out OK.
            //Let's send a message saying as much and break.
            write_cntr=read_radio_write_cntr();
            sprintf(short_message,"Prg.FailAt%03d-Exit",write_cntr);
            send_short_message(p_rfidrs, short_message);
            break;
        }
        else
        {
            //We succeeded in programming the tag! Yay!
            sprintf(short_message,"Prog.Pass!!!");
            send_short_message(p_rfidrs, short_message);
            
            //Pull the read register and check its results since everything is going good so far.
            //This is only if we are doing an actual program (not a kill).
            //Perhaps in the future we can also read back if we are writing to the kill password.
            if(content==PROGRAM_NEW_EPC){
                rfidr_error_code     = rfidr_pull_and_check_read_data(target);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"actual readback epc check failed",rfidr_error_code); return RFIDR_ERROR_GENERAL;}
            }
            break;
        }

    }
    //Now that we are finally done with the actual RFID over-the-air operations, power down the PA.
    rfidr_error_code=rfidr_disable_pa();
    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,error_info,"disabling pa",rfidr_error_code); return RFIDR_ERROR_GENERAL;}

    return RFIDR_SUCCESS;
}

//This is the main operational state machine of the reader firmware.
//It is called in the main loop after the "write state" characteristic event handler sets a flag which stops the power_manage() function from running
//and allows the function below to run.
//A lot of this code is a prime candidate for condensing into sub-functions, but we wanted to wait until the system was fully debugged
//before embarking on that task.

void run_rfidr_state_machine(ble_rfidrs_t *p_rfidrs)
{
    #define    MAX_INV_TAGS          51                //In inventory mode, how many tags will we inventory before we exit the loop?
    #define    PLL_GOOD_THRESHOLD    2400
    #define    CORRECT_LNA_GAIN      0x34

    rfidr_error_t            rfidr_error_code                        =    RFIDR_SUCCESS;
    //uint32_t               nrf_error_code                          =    NRF_SUCCESS; //Supposedly unused
    uint8_t                  recover_frequency_slot                  =    0;
    char                     short_message[20]                       =    {0};
    uint8_t                  spi_return_byte                         =    0;
    //bool                   found_good_sx1257_pll_cvg               =    false;    //Added 051519 to find PLL good convergence
    //uint8_t                loop_pll_cvg                            =    0;        //Added 051519 to put a limit on how many good PLL convergence checks we are going to do. Probably ~10.
    rfidr_return_t           return_struct_cal;                      //No need to initialize a value, this is initialized on each run of a core function
    rfidr_return_t           return_struct_ant;                      //No need to initialize a value, this is initialized on each run of a core function
    uint8_t                  search_hop_vector[6]                    =    {255,255,255,255,255,255};    //Added 112519 to hold a vector of valid hop values for PDOA ranging. 255 is our "null" value.
    uint8_t                  loop_hop                                =    0;        //Added 112519 to index vector of valid hop values for PDOA ranging.
    rfidr_program_content_t  content                                 =    PROGRAM_NEW_EPC;
    rfidr_tracking_mode_t    tracking_mode                           =    TRACK_LAST_INV;

    m_rfidr_state=m_rfidr_state_next;

    switch(m_rfidr_state)
    {

        case IDLE_UNCONFIGURED:
            rfidr_disable_led1();
            //Do nothing but wait for a state change
            //Put the below bookend function in as a precaution so that we can notify the iDevice that we are at this state.
            rfidr_state_bookend_function(p_rfidrs);
            break;


        case IDLE_CONFIGURED:
            rfidr_enable_led1();
            send_short_message(p_rfidrs, "Got here");
            //Do nothing but wait for a state change
            //111020 - Handling the situation when we are in IDLE_CONFIGURED and get a request to go to INITIALIZED requires no state change
            //and a notification to the iDevice that the state is remaining in IDLE_CONFIGURED.
            //Having the bookend function in the write state sanitizing function results in a lockup, presumably because
            //nested interrupts aren't working - we never process the hvc confirmation from the iDevice.

            rfidr_state_bookend_function(p_rfidrs);
            break;


        case INITIALIZING:
            //Initialize all components on the reader to the extent possible.
            //This case was heavily modified on 5/15/2019 to deal with bad SX1257 convergence for the PLL setting required to get good phase noise results for 1W operation.
            //For this function to work, the antenna 1 port needs to have a 50 ohm termination tightly screwed onto it with a torque wrench.
            //In future revisions we will use an SP3T or something similar to have a 50 ohm test resistance on the board in the event we can't solve this problem another way.
            //Furthermore, it was found that the more functionality we added onto the FPGA to deal with issues such as this, the FPGA sometimes would not power
            //up correctly.
            //This condition can be detected using the same signal quality measurement

            //Send indication that we are in the initialization state to the iDevice app.
            //Wait for the indication to be ACKed before proceeding.

            rfidr_state_bookend_function(p_rfidrs);

            #if 0

            //This is the new section of code added on 051519 which checks the total baseband power received.
            //If the power is too high, then we suspect bad PLL convergence and we reset the SX1257 to try again
            //We need to make sure that before evaluating the baseband power that the TMN has has time to converge

            while((found_good_sx1257_pll_cvg==false) && loop_pll_cvg < 1)
            {

                rfidr_error_code=initialization_core(p_rfidrs,"initializing-core");
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"initializing, error within initialization_core",rfidr_error_code); break;}

                rfidr_sel_ant1(); //Switch to ant1 in order to do testing on the dummy tag - to see if we are getting tones out of the PLL.

                rfidr_error_code=search_core(p_rfidrs,"initializing-search", SESSION_S0, TARGET_PLL_EPC, RETURN_EPC_NO, RETURN_MAG_YES, RETURN_LNA_GAIN_YES, &return_struct_cal);
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"initializing, error within search_core",rfidr_error_code); break;}

                if((return_struct_cal.i_main_mag < PLL_GOOD_THRESHOLD) && (return_struct_cal.i_lna_gain == CORRECT_LNA_GAIN) && (return_struct_cal.q_main_mag < PLL_GOOD_THRESHOLD) && (return_struct_cal.q_lna_gain == CORRECT_LNA_GAIN))
                        found_good_sx1257_pll_cvg=true;
                //Exit loop if the total in-band error (roughly calculated) is below the threshold we established in lab.
                //Also need to reset if we see both I and Q magnitude equal to 0. This means likely that the FPGA got into a bad state.

                //Need to send a report back to the iDevice based on what happened.

                if(found_good_sx1257_pll_cvg==true)
                    sprintf(short_message,"SX1257 PLL %01d Pass",loop_pll_cvg);
                else
                    sprintf(short_message,"SX1257 PLL %01d Fail",loop_pll_cvg);
                send_short_message(p_rfidrs, short_message);

                sprintf(short_message,"LNA Gain I: %02x",return_struct_cal.i_lna_gain);
                send_short_message(p_rfidrs, short_message);

                sprintf(short_message,"LNA Gain Q: %02x",return_struct_cal.q_lna_gain);
                send_short_message(p_rfidrs, short_message);

                sprintf(short_message,"Chk I: %10d",(int)return_struct_cal.i_main_mag);
                send_short_message(p_rfidrs, short_message);

                sprintf(short_message,"Chk Q: %10d",(int)return_struct_cal.q_main_mag);
                send_short_message(p_rfidrs, short_message);

                loop_pll_cvg++;
            }

            //Assuming that the signal quality metric comes back with an acceptable result, we proceed with normal initialization.
            rfidr_error_code=unset_sx1257_pll_chk_mode();
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"initializing, unsetting PLL check mode","",rfidr_error_code); break;}
            //We set the antenna switch back to antenna 0, which is the antenna port on the front of the reader.
            rfidr_sel_ant0();

            #endif
            //This picks up with the original code prior to 051519

            rfidr_error_code=initialization_core(p_rfidrs,"initializing-core");
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"initializing, error within initialization_core","",rfidr_error_code); break;}

            m_rfidr_state_next=IDLE_CONFIGURED;
            m_rfidr_state=IDLE_CONFIGURED;

            //Communicate to the iDevice that the reader has entered the IDLE_CONFIGURED state.
            rfidr_state_bookend_function(p_rfidrs);
            //m_dtc_state_flag=false; //Why is this here? A candidate for deletion.

            break;

        case SEARCHING_APP_SPECD_TAG:
        case SEARCHING_LAST_INV_TAG:
            //The intent of this file is to search for a known tag.
            //This would be the primary function of a personal RFID reader.
            //The modus operandi would be to send out a packet sequence with SELECT and QUERY with Q=0 to get the tag of interest to respond ASAP.
            //The tag complex RSSI would then be collected and returned to the reader in a somewhat raw form (it would still need to be normalized to the EPC length).

            //As of 11/22/19 things are going to get a little bit interesting because we are going to collect all of the information needed for ranging here

            //First, we send an indication to the iDevice that a SEARCHING state has been entered.
            rfidr_state_bookend_function(p_rfidrs);

            //Second, we frequency hop randomly. No loop here - if the user wants to try again, they can press the search button again and get another hop.

            rfidr_error_code=hop_sx1257_frequency(&recover_frequency_slot); m_hopskip_nonce++;
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, randomly hopping frequency","",rfidr_error_code); break;}

            //Third, we run a search core on the dummy tag for phase calibration and collect the results

            //rfidr_sel_ant1(); //Switch to ant1 in order to do RFID operations on the dummy tag. 080620 - Realized this is not a good idea.

            rfidr_error_code=search_core(p_rfidrs, "searching", SESSION_S0, TARGET_CAL_EPC, RETURN_EPC_NO, RETURN_MAG_YES, RETURN_LNA_GAIN_NO, &return_struct_cal);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, error at first calibration search","",rfidr_error_code); break;}

            if(return_struct_cal.i_pass == false && return_struct_cal.q_pass == false){handle_error(p_rfidrs,"searching, failure at first calibration search","",rfidr_error_code); break;}

            //Fourth, we run a search core through the main antenna. 080620 - Realized this is not a good idea.

            //rfidr_sel_ant0(); //Switch to ant0 in order to use the main antenna

            //Why Session S0? Tag flag state resets when output power is cut off.
            //This allows us to access the tag using flag A on multiple occasions in quick succession.

            rfidr_error_code=search_core(p_rfidrs, "searching", SESSION_S0, m_rfidr_state==SEARCHING_APP_SPECD_TAG ? TARGET_APP_SPECD_EPC : TARGET_LAST_INV_EPC, RETURN_EPC_YES, RETURN_MAG_YES, RETURN_LNA_GAIN_NO, &return_struct_ant);
            if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, error at first antenna search","",rfidr_error_code); break;}

            //This is not the nicest way to break out of the function, but it seems acceptable and makes the code clean.

            if(return_struct_ant.i_pass == false && return_struct_ant.q_pass == false){handle_error(p_rfidrs,"searching, failure at first antenna search:","",rfidr_error_code); break;}

            sprintf(short_message,"FreqSlot1: %3d",(int)recover_frequency_slot);
                    send_short_message(p_rfidrs, short_message);

            m_received_hvc_pckt_data1_flag            =    false;
            rfidr_error_code=rfidr_push_data_over_ble(p_rfidrs,&return_struct_ant,&return_struct_cal,recover_frequency_slot,255,m_hopskip_nonce,BLE_PUSH_SUPPLEMENT);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, pushing first run pckt data over ble","",rfidr_error_code); break;}
            while(m_received_hvc_pckt_data1_flag == false){}

            //Fifth, if the last run passed, we run a small loop where we check adjacent frequencies so that we can run PDOA.
            //For the moment we will only hop +/- 1MHz (1 code) from the frequency we just hopped to.
            //In order to resolve distances up to 25 meters, we can only hop up to 1.5MHz in this step, otherwise we will get ranging aliasing.

            if(recover_frequency_slot<3)
                search_hop_vector[0]=recover_frequency_slot+3;    //Only hop 3 frequencies up - we are not allowed to hop one frequency down
            else if (recover_frequency_slot>21)
                search_hop_vector[0]=recover_frequency_slot-3;    //Only hop 3 frequencies down - we are not allowed to hop one frequency up
            else
            {
                search_hop_vector[0]=recover_frequency_slot+3;
                search_hop_vector[1]=recover_frequency_slot-3;
            }

            //Here, we try to obtain tag reads at frequencies adjacent to the original one so that we can perform PDOA without distance aliasing up to a bit more than 25 meters.
            //Use for loop instead of while so that the first and last successful loop hop can be saved. We use 255 as a sentinel value in search_hop_vector but anything over 24 is invalid so we test against that.
            //Note that while in principle we can center 26 1MHz channels from 902.5MHz to 927.5MHz, we chose 25 channels in rfidr_sx1257.c.
            for(loop_hop=0;search_hop_vector[loop_hop] < 25;loop_hop++)
            {
                return_struct_cal.i_pass =  return_struct_cal.q_pass = return_struct_ant.i_pass =  return_struct_ant.q_pass = false; //Set these so that we fail through if we don't get a pass value.

                rfidr_error_code=set_sx1257_frequency(search_hop_vector[loop_hop]);
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, error at second frequency hop","",rfidr_error_code); break;}

                //rfidr_sel_ant1(); //Switch to ant1 in order to do RFID operations on the dummy tag. 080720 - No need to do this with dummy tag on a dir. coupler.

                sprintf(short_message,"Cal2Srch: %3d",(int)search_hop_vector[loop_hop]);
                send_short_message(p_rfidrs, short_message);

                rfidr_error_code=search_core(p_rfidrs, "searching", SESSION_S0, TARGET_CAL_EPC, RETURN_EPC_NO, RETURN_MAG_YES, RETURN_LNA_GAIN_NO, &return_struct_cal);
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, error at second calibration search","",rfidr_error_code); break;}

                //rfidr_sel_ant0(); //Switch to ant0 in order to do RFID operations on the main antenna. 080720 - No need to do this with dummy tag on a dir. coupler.

                if(return_struct_cal.i_pass == false && return_struct_cal.q_pass == false){continue;} //Just move on to the next loop if this one fails.

                sprintf(short_message,"Ant2Srch: %3d",(int)search_hop_vector[loop_hop]);
                send_short_message(p_rfidrs, short_message);

                //080720 - May not want to return EPC in the future to save RAM time if we can handle this in the app differently
                rfidr_error_code=search_core(p_rfidrs, "searching", SESSION_S0, m_rfidr_state==SEARCHING_APP_SPECD_TAG ? TARGET_APP_SPECD_EPC : TARGET_LAST_INV_EPC, RETURN_EPC_YES, RETURN_MAG_YES, RETURN_LNA_GAIN_NO, &return_struct_ant);
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, error at second antenna search","",rfidr_error_code); break;}

                if(return_struct_ant.i_pass == false && return_struct_ant.q_pass == false){continue;} else {break;} //If we get a pass, break out of the loop and report data

            }
            if(rfidr_error_code != RFIDR_SUCCESS){break;} //If there was an error in the loop, we need to break again.

            //If we get here, the functions in the last iteration of the PDOA loop have all completed. The last run either passed or it didn't.
            //080720 - We don't send data in case of a failure since it will show up as a nonexistent tag EPC. But in future we can and solve error another way.
            //We also need to guard against the case when the calibration search fails each time and the search_core on the return_struct_ant never resets return_struct_ant.
            //We do this when we manually reset return_struct_ant.x_pass above.

            if(return_struct_ant.i_pass == false && return_struct_ant.q_pass == false)
            {
                send_short_message(p_rfidrs, "PDOA srch2 fail");
            }
            else
            {
                sprintf(short_message,"FreqSlot2: %3d",(int)search_hop_vector[loop_hop]);
                send_short_message(p_rfidrs, short_message);

                m_received_hvc_pckt_data1_flag            =    false;
                rfidr_error_code=rfidr_push_data_over_ble(p_rfidrs,&return_struct_ant,&return_struct_cal,search_hop_vector[loop_hop],loop_hop, m_hopskip_nonce, BLE_PUSH_SUPPLEMENT);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"searching, pushing second run pckt data over ble","",rfidr_error_code); break;}
                while(m_received_hvc_pckt_data1_flag == false){}
            }

            //Next state is IDLE_CONFIGURED, transition automatically and immediately.
            m_rfidr_state_next=IDLE_CONFIGURED;
            m_rfidr_state=IDLE_CONFIGURED;

            //Send state change information to iDevice and wait for the indication ACK.
            rfidr_state_bookend_function(p_rfidrs);

            break;


        case INVENTORYING:

            //Send state change information to iDevice and wait for the indication ACK.
            rfidr_state_bookend_function(p_rfidrs);

            rfidr_error_code=inventory_core(p_rfidrs, "inventorying", SESSION_S2, "6666655555444444433333333322", MAX_INV_TAGS, "A0B1C2D3E4F5A6B7C8D9E0F1", &return_struct_ant);

            //Why Session S2? Tag flag persistence remains over 2 seconds past supplying power.
            //We need to power down the PA while we are doing SPI and BTLE operations without losing tag inventory flag state.

                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"inventorying, error within inventory_core","",rfidr_error_code); break;}
            //Change state back to IDLE_CONFIGURED and report this to the iDevice with an indication.
            m_rfidr_state_next=IDLE_CONFIGURED;
            m_rfidr_state=IDLE_CONFIGURED;

            rfidr_state_bookend_function(p_rfidrs);

            break;

        case TRACK_APP_SPECD_TAG:
        case TRACK_LAST_INV_TAG:

            switch(m_rfidr_state)
            {
                case TRACK_APP_SPECD_TAG:       tracking_mode=TRACK_APP_SPECD;       break;
                case TRACK_LAST_INV_TAG:        tracking_mode=TRACK_LAST_INV;        break;
                default:                        tracking_mode=TRACK_LAST_INV;        break;
            }

            //Send state change information to iDevice and wait for the indication ACK.
            rfidr_state_bookend_function(p_rfidrs);

            rfidr_error_code=tracking_core(p_rfidrs, "tracking", SESSION_S2, tracking_mode, &return_struct_ant, &return_struct_cal);

                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"tracking state, error within tracking_core","",rfidr_error_code); break;}
            //Change state back to IDLE_CONFIGURED and report this to the iDevice with an indication.
            m_rfidr_state_next=IDLE_CONFIGURED;
            m_rfidr_state=IDLE_CONFIGURED;

        rfidr_state_bookend_function(p_rfidrs);

        break;


        case TESTING_DTC:
        //This state is required for performing an evaluation of the TMN on the VNA.
        //Currently we need to reflash the FPGA with slightly different options enabled for this to work in order to fit everything inside the FPGA.
        //Earlier on, before a number of bug fixes, the FPGA image could transition to this test mode and work without being recoded and reflashed.
        //When this state is selected, the MCU accepts inputs on 3 of its 4 buttons for incrementing the two DTC state variables, and for resetting the DTC state variables.
        //Normally, the buttons are electronically "pressed" by a LabJack with wires soldered to the button PCB connections to do this automatically.
        //Maybe in the future a better way to handle this is to build the reader PCB with a dedicated test header for this purpose.

        //Tell the iDevice that the state has changed and wait for the ACK.
            rfidr_state_bookend_function(p_rfidrs);

        //Check the DTC state flag. Why do we have a DTC state flag? Aren't we in the DTC state already?
        //The reason is that this is a bit of a hack since the state has already been set in the write state characteristic event handler.
        //And also because we can only do state actions when the DTC Test button is pressed in the iDevice.
        //However, we want to capture the meat of the state behavior here.
        //So we use another flag to check if the iDevice button press is bringing us into the state or out of the state.

            if(m_dtc_state_flag    ==    false)
            {
                //If we are coming into the DTC testing state, set the flag saying we are in the state.
                m_dtc_state_flag    =    true;
                sprintf(short_message,"Enter Test DTC State");
                send_short_message(p_rfidrs, short_message);

                //No idea why this is here since the PA will be off.
                rfidr_error_code=set_sx1257_tx_power_high();
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"DTC, setting tx power high","",rfidr_error_code); break;}

                //Tell the FPGA we will be entering the DTC test mode.
                rfidr_error_code=enter_dtc_test_mode();
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"DTC, enter_dtc_test_mode","",rfidr_error_code); break;}

                //Keep the PA off during this test.

                //rfidr_error_code=rfidr_enable_pa();
                //if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"DTC, enabling pa:",rfidr_error_code); break;}

                rfidr_error_code=rfidr_disable_pa();
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"DTC, disabling pa","",rfidr_error_code); break;}

                //This state setting code below is just for completeness. In fact, these values are already set by the write state event handler
                //And by the state transition at the beginning of the state machine function.
                m_rfidr_state_next=TESTING_DTC;
                m_rfidr_state=TESTING_DTC;

            }
            else
            {
                //If we are exiting the DTC state, send a short message to the iDevice to that effect.
                m_dtc_state_flag    =    false;
                sprintf(short_message,"Exit Test DTC State");
                send_short_message(p_rfidrs, short_message);

                //Set the FPGA back to its original state.
                rfidr_error_code=exit_dtc_test_mode();
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"DTC, exit_dtc_test_mode","",rfidr_error_code); break;}

                rfidr_error_code=rfidr_disable_pa();
                    if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"DTC, disabling pa","",rfidr_error_code); break;}

                //Move back to the IDLE_CONFIGURED state.
                m_rfidr_state_next=IDLE_CONFIGURED;
                m_rfidr_state=IDLE_CONFIGURED;
            }

            //Report change of state (or lack thereof) back to the iDevice and wait for the ACK.
            rfidr_state_bookend_function(p_rfidrs);

            break;


        case PROGRAMMING_APP_SPECD_TAG:
        case PROGRAMMING_LAST_INV_TAG:
        case KILL_TAG:
        case PROGRAMMING_KILL_PASSWD:

            switch(m_rfidr_state)
            {
                case PROGRAMMING_APP_SPECD_TAG:    content=PROGRAM_NEW_EPC;     break;
                case PROGRAMMING_LAST_INV_TAG:     content=PROGRAM_NEW_EPC;     break;
                case KILL_TAG:                     content=KILL_COMMAND;        break;
                case PROGRAMMING_KILL_PASSWD:      content=PROGRAM_KILL_PWD;    break;
                default:                           content=PROGRAM_NEW_EPC;     break;
            }

            //This state is intended to program an "existing" tag, namely, one with a EPC code corresponding to one with the 11 MSBytes that this reader software uses to program a new tag.
            //The "existing" tag EPC MSB are stored in the MCU firmware.

            //First, we send an indication to the iDevice that we have entered one of the program modes

            rfidr_state_bookend_function(p_rfidrs);

            //Second, we frequency hop randomly. No loop here - if the user wants to try again, they can press the program button again and get another hop.

            rfidr_error_code=hop_sx1257_frequency(&recover_frequency_slot); m_hopskip_nonce++;
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"programming, randomly hopping frequency","",rfidr_error_code); break;}

            //Third, we run a search core on the tag to collect I and Q data to see if we are getting back a strong signal.

            rfidr_error_code=search_core(p_rfidrs, "programming", SESSION_S0, m_rfidr_state==PROGRAMMING_LAST_INV_TAG ? TARGET_LAST_INV_EPC : TARGET_APP_SPECD_EPC, RETURN_EPC_NO, RETURN_MAG_YES, RETURN_LNA_GAIN_NO, &return_struct_ant);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"programming, error at first search","",rfidr_error_code); break;}

            if(return_struct_ant.i_pass == false && return_struct_ant.q_pass == false){handle_error(p_rfidrs,"programming, failure at first search","",rfidr_error_code); break;}

            rfidr_error_code=program_core(p_rfidrs, "programming", SESSION_S0, m_rfidr_state==PROGRAMMING_LAST_INV_TAG ? TARGET_LAST_INV_EPC : TARGET_APP_SPECD_EPC, content, &return_struct_ant);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"programming, error at programming","",rfidr_error_code); break;}

            m_rfidr_state_next=IDLE_CONFIGURED;
            m_rfidr_state=IDLE_CONFIGURED;

            //Assuming that the code finally made it all the way down here, inform the iDevice of the state change via notification and ACK the result.
            rfidr_state_bookend_function(p_rfidrs);

            break;

        case RECOVERING_WAVEFORM_MEMORY:
        //This function is pretty simple. We enter the state, inform the iDevice of doing so, then push the waveform data over BTLE to the iDevice.
            rfidr_state_bookend_function(p_rfidrs);

            rfidr_error_code=rfidr_push_waveform_data_over_ble(p_rfidrs);
                if(rfidr_error_code != RFIDR_SUCCESS){handle_error(p_rfidrs,"recovering waveform memory","",rfidr_error_code); break;}

            m_rfidr_state_next=IDLE_CONFIGURED;
            m_rfidr_state=IDLE_CONFIGURED;

            rfidr_state_bookend_function(p_rfidrs);

            break;


        case RESET_SX1257_AND_FPGA:
            //This function performs software resets of the radio and FPGA.
            //It also reads back the SX1257 PLL lock register. This lock check register can probably be removed.
            //The register reads back good even when the PLL has settled into a condition where it generates too much noise.
            rfidr_state_bookend_function(p_rfidrs);

            rfidr_reset_fpga();
            rfidr_reset_radio();    //These always return success, so don't check them

            rfidr_error_code=spi_cntrlr_read_sx1257_robust(0x11, &spi_return_byte); //Changed to 2-argument version on 5/23/19 to clean up code
            if(rfidr_error_code == RFIDR_SUCCESS)
            {
                sprintf(short_message,"PLL Lock Status: %02x",spi_return_byte);
                send_short_message(p_rfidrs, short_message);
            }
            else
            {
                sprintf(short_message,"PLL Lock Status: Er");
                send_short_message(p_rfidrs, short_message);
            }
            //set_sx1257_fix_tones();

            rfidr_disable_led1(); //Turn off the LED to show that we are unconfigured.

            m_rfidr_state_next=IDLE_UNCONFIGURED;
            m_rfidr_state=IDLE_UNCONFIGURED;

            rfidr_state_bookend_function(p_rfidrs);
            m_dtc_state_flag=false;

            break;

        default:
            //This is the catchall case just in case something unexpected or a glitch happens.
            //If this happens, return to the IDLE_UNCONFIGURED state.
            m_rfidr_state_next=IDLE_UNCONFIGURED;
            m_rfidr_state=IDLE_UNCONFIGURED;

            rfidr_state_bookend_function(p_rfidrs);
            m_dtc_state_flag=false;

            break;
        }
    }
    




