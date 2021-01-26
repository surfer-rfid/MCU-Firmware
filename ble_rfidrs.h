//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
// Module : RFIDr Firmware BTLE setup and drivers                                   //
//                                                                                  //
// Filename: ble_rfidrs.h                                                           //
// Creation Date: circa 10/31/2016                                                  //
// Author: Edward Keehr                                                             //
//                                                                                  //
//    This file was derived from the Nordic Semiconductor example code in their     //
//    SDK 8.0. Specifically, this file was derived from "ble_nus.h" in the          //
//    SDK components/ble/ble_services folder                                        //
//                                                                                  //
//     The required notice to be reproduced for Nordic Semiconductor code is        //
//     given below:                                                                 //
//                                                                                  //
//    /* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.              //
// *                                                                                //
// * The information contained herein is property of Nordic Semiconductor ASA.      //
// * Terms and conditions of usage are described in detail in NORDIC                //
// * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.                             //
// *                                                                                //
// * Licensees are granted free, non-transferable use of the information. NO        //
// * WARRANTY of ANY KIND is provided. This heading must NOT be removed from        //
// * the file.                                                                      //
// *                                                                                //
// */                                                                               //
//                                                                                  //
//    For components of the code modified or authored by Superlative                //
//    Semiconductor LLC, the copyright notice is as follows:                        //
//                                                                                  //
//    Copyright 2021 Superlative Semiconductor LLC                                  //
//                                                                                  //
//    Licensed under the Apache License, Version 2.0 (the "License");               //
//    you may not use this file except in compliance with the License.              //
//    You may obtain a copy of the License at                                       //
//                                                                                  //
//       http://www.apache.org/licenses/LICENSE-2.0                                 //
//                                                                                  //
//    Unless required by applicable law or agreed to in writing, software           //
//    distributed under the License is distributed on an "AS IS" BASIS,             //
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.      //
//    See the License for the specific language governing permissions and           //
//    limitations under the License.                                                //
//                                                                                  //
//    Description:                                                                  //
//    Most of this code is adapted from NRF SDK example code.                       //
//    Most of the comments also come from Nordic SDK example code.                  //
//    In addition to setting up the BTLE RFIDr service, this is where               //
//    BTLE characteristics are defined.                                             //
//    There are also event handlers from the S110 SoftDevice and drivers related    //
//    to BTLE operations.                                                           //
//                                                                                  //
//    Revisions:                                                                    //
//    061919 - Major commentary cleanup.                                            //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////

#ifndef BLE_RFIDRS_H__
#define BLE_RFIDRS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

//Superlative Semiconductor note: The following template of defines was provided by
//Nordic Semiconductor. Values were modified by Superlative Semiconductor for the 
//UHF RFID reader project.

#define BLE_UUID_RFIDRS_SERVICE 0x0001                      //The UUID of the RFIDR Service.
#define BLE_RFIDRS_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) //Maximum length of data (in bytes) that can be transmitted to the peer by the RFIDR service module.

#define BLE_RFIDRS_WRTE_STATE_CHAR_LEN    1                 //There will be less than 16 states so we need 1 byte only to cover this
#define BLE_RFIDRS_TARGET_EPC_CHAR_LEN    12                //We will use a maximum of 12 EPC bytes (96b) here
#define BLE_RFIDRS_PROGRAM_EPC_CHAR_LEN   12                //We will strictly use 12 EPC bytes (96b) here
#define BLE_RFIDRS_READ_STATE_CHAR_LEN    1                 //There will be less than 16 states so we need 1 byte only to cover this
#define BLE_RFIDRS_PCKT_DATA1_CHAR_LEN    20                //See rfidr_radio.c for new definitions
#define BLE_RFIDRS_PCKT_DATA2_CHAR_LEN    16                //See rfidr_radio.c for new definitions
#define BLE_RFIDRS_WAVFM_DATA_CHAR_LEN    20                //Send over the maximum number of data bits in a BTLE packet
#define BLE_RFIDRS_LOG_MESSGE_CHAR_LEN    20                //Send over the maximum number of data bits in a BTLE packet

//Forward declaration of the ble_rfidrs_t type.
typedef struct ble_rfidrs_s ble_rfidrs_t;

//Superlative Semiconductor note: Declaration unchanged from Nordic SDK v8.0.
//HVC event type
typedef enum
{
    BLE_HVC_EVT_INDICATION_ENABLED,                         //HVC indication enabled event.
    BLE_HVC_EVT_INDICATION_DISABLED,                        //HVC indication disabled event.
    BLE_HVC_EVT_INDICATION_CONFIRMED                        //Confirmation that an indication has been received.
} ble_hvc_evt_type_t;

//Superlative Semiconductor note: Declaration unchanged from Nordic SDK v8.0.
//RFIDRs HVC event.
typedef struct
{
    ble_hvc_evt_type_t evt_type;                            //Type of event.
} ble_rfidrs_hvc_evt_t;

//RFIDR Service event handler type (data).
typedef void (*ble_rfidrs_wrte_state_handler_t) (ble_rfidrs_t * p_rfidrs, uint8_t * p_data, uint16_t length);

//RFIDR Service event handler type (data).
typedef void (*ble_rfidrs_target_epc_handler_t) (ble_rfidrs_t * p_rfidrs, uint8_t * p_data, uint16_t length);

//RFIDR Service event handler type (data).
typedef void (*ble_rfidrs_program_epc_handler_t) (ble_rfidrs_t * p_rfidrs, uint8_t * p_data, uint16_t length);

//RFIDR Service event handler type (hvc).
typedef void (*ble_rfidrs_read_state_handler_t) (ble_rfidrs_t * p_rfidrs, ble_rfidrs_hvc_evt_t * p_evt);

//RFIDR Service event handler type (hvc).
typedef void (*ble_rfidrs_pckt_data1_handler_t) (ble_rfidrs_t * p_rfidrs, ble_rfidrs_hvc_evt_t * p_evt);

//Superlative Semiconductor note: Declaration and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//RFIDr Service initialization structure.
//This structure contains the initialization information for the service. The application
//must fill this structure and pass it to the service using the @ref ble_rfidr_init
//function.

typedef struct
{
    ble_rfidrs_wrte_state_handler_t        wrte_state_handler; //Event handler to be called for handling received data.
    ble_rfidrs_target_epc_handler_t        target_epc_handler;
    ble_rfidrs_program_epc_handler_t       program_epc_handler;
    ble_rfidrs_read_state_handler_t        read_state_handler;
    ble_rfidrs_pckt_data1_handler_t        pckt_data1_handler;
} ble_rfidrs_init_t;

//Superlative Semiconductor note: Declaration and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//RFIDr Service structure.
//This structure contains status information related to the service.

struct ble_rfidrs_s
{
    uint8_t                            uuid_type;                           //UUID type for RFIDR Service Base UUID.
    uint16_t                           service_handle;                      //Handle of RFIDR Service (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           wrte_state_handles;                  //Handles related to the write state characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           target_epc_handles;                  //Handles related to the target_epc characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           program_epc_handles;                 //Handles related to the program_epc characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           read_state_handles;                  //Handles related to the read state characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           pckt_data1_handles;                  //Handles related to the pckt_data1 characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           pckt_data2_handles;                  //Handles related to the pckt_data2 characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           wavfm_data_handles;                  //Handles related to the wavfm_data characteristic (as provided by the S110 SoftDevice).
    ble_gatts_char_handles_t           log_messge_handles;                  //Handles related to the log message characteristic (as provided by the S110 SoftDevice). 
    uint16_t                           conn_handle;                         //Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection.
    bool                               is_target_epc_indication_enabled;    //Variable to indicate if the peer has enabled indication of the target epc characteristic.
    bool                               is_program_epc_indication_enabled;   //Variable to indicate if the peer has enabled indication of the program epc characteristic.
    bool                               is_read_state_indication_enabled;    //Variable to indicate if the peer has enabled indication of the read state characteristic.
    bool                               is_pckt_data1_indication_enabled;    //Variable to indicate if the peer has enabled indication of the packet data 1 characteristic.
    bool                               is_pckt_data2_notification_enabled;  //Variable to indicate if the peer has enabled notification of the packet data 2 characteristic.
    bool                               is_wavfm_data_notification_enabled;  //Variable to indicate if the peer has enabled notification of the waveform data characteristic.
    bool                               is_log_messge_notification_enabled;  //Variable to indicate if the peer has enabled notification of the log message characteristic.
    ble_rfidrs_wrte_state_handler_t    wrte_state_handler;                  //Event handler to be called for handling received write state request.
    ble_rfidrs_target_epc_handler_t    target_epc_handler;                  //Event handler to be called for handling received app-specified target epc infromation.
    ble_rfidrs_program_epc_handler_t   program_epc_handler;                 //Event handler to be called for handling received app-specified program epc information.
    ble_rfidrs_read_state_handler_t    read_state_handler;                  //Event handler to be called for handling a received indication confirmation for read state.
    ble_rfidrs_pckt_data1_handler_t    pckt_data1_handler;                  //Event handler to be called for handling a received indication confirmation for read state.
};

//Superlative Semiconductor note: Declaration and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function for initializing the RFIDr Service.
//
// output parameter: p_rfidrs    RFIDR Service structure. This structure must be supplied
//                        by the application. It is initialized by this function and will
//                        later be used to identify this particular service instance.
// input parameter:  p_rfidrs_init  Information needed to initialize the service.
//
// returns NRF_SUCCESS if the service was successfully initialized. Otherwise, an error code is returned.
// returns NRF_ERROR_NULL if either of the pointers p_rfidr or p_rfidr_init is NULL.
//
uint32_t ble_rfidrs_init(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init);

//Superlative Semiconductor note: Declaration and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function for handling the RFIDR Service's BLE events.
//
// The RFIDR Service expects the application to call this function each time an
// event is received from the S110 SoftDevice. This function processes the event if it
// is relevant and calls the RFIDR Service event handler of the
// application if necessary.
//
// input parameter: p_rfidrs       RFIDR Service structure.
// input parameter: p_ble_evt   Event received from the S110 SoftDevice.
//
void ble_rfidrs_on_ble_evt(ble_rfidrs_t * p_rfidrs, ble_evt_t * p_ble_evt);

//Superlative Semiconductor note: Declaration and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

// Functions for sending a string to the peer.
//
// This function sends the input string as an RX characteristic notification or indication to the peer.
//
//
// input parameter: p_rfidrs       Pointer to the RFIDR Service structure.
// input parameter: p_string    String to be sent.
// input parameter: length      Length of the string.
//
// returns NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
//
uint32_t ble_rfidrs_target_epc_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_target_epc, uint16_t length);
uint32_t ble_rfidrs_program_epc_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_program_epc, uint16_t length);
uint32_t ble_rfidrs_read_state_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_state, uint16_t length);
uint32_t ble_rfidrs_pckt_data1_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_data1, uint16_t length);
uint32_t ble_rfidrs_pckt_data2_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_data2, uint16_t length);
uint32_t ble_rfidrs_wavfm_data_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_wdata, uint16_t length);
uint32_t ble_rfidrs_log_messge_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_ldata, uint16_t length);

#endif // BLE_RFIDRS_H__
