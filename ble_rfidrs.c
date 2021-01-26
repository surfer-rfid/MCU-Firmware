//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
// Module : RFIDr Firmware BTLE setup and drivers.                                  //
//                                                                                  //
// Filename: ble_rfids.c                                                            //
// Creation Date: circa 10/31/2016                                                  //
// Author: Edward Keehr                                                             //
//                                                                                  //
//    This file was derived from the Nordic Semiconductor example code in their     //
//    SDK 8.0. Specifically, this file was derived from "ble_nus.c" in the          //
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

#include "ble_rfidrs.h"
#include "rfidr_error.h" //Just for MAX_EPC_LENGTH_IN_BYTES
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"

//Define characteristics for the RFID Reader Service - defined by Superlative Semiconductor
#define BLE_UUID_RFIDRS_WRTE_STATE_CHAR     0x0002        //The UUID of the state writing Characteristic.
#define BLE_UUID_RFIDRS_TARGET_EPC_CHAR     0x0003        //The UUID of the target EPC characteristic.
#define BLE_UUID_RFIDRS_PROGRAM_EPC_CHAR    0x0004        //The UUID of the new EPC characteristic top be programmed.
#define BLE_UUID_RFIDRS_READ_STATE_CHAR     0x0005        //The UUID of the state reading Characteristic.
#define BLE_UUID_RFIDRS_PCKT_DATA1_CHAR     0x0006        //The UUID of the packet data - section 1 Characteristic.
#define BLE_UUID_RFIDRS_PCKT_DATA2_CHAR     0x0007        //The UUID of the packet data - section 2 Characteristic.
#define BLE_UUID_RFIDRS_WAVFM_DATA_CHAR     0x0008        //The UUID of the waveform data characteristic.
#define BLE_UUID_RFIDRS_LOG_MESSGE_CHAR     0x0009        //The UUID of the log message characteristic.

#define RFIDRS_BASE_UUID                    {{0x15, 0x59, 0x3B, 0x84, 0xE5, 0x26, 0x46, 0xAD, 0xB5, 0x8D, 0x1D, 0xFC, 0x00, 0x00, 0x56, 0xE7}}

//Superlative Semiconductor note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic.

//Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
//
// param[in] p_rfidrs     RFIDr Service structure.
// param[in] p_ble_evt Pointer to the event received from BLE stack.
//
static void on_connect(ble_rfidrs_t * p_rfidrs, ble_evt_t * p_ble_evt)
{
    p_rfidrs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

//Superlative Semiconductor note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic.

//Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
//
// param[in] p_rfidrs     RFIDr Service structure.
// param[in] p_ble_evt Pointer to the event received from BLE stack.
//
static void on_disconnect(ble_rfidrs_t * p_rfidrs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_rfidrs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

//Superlative Semiconductor note: Function template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.
//Comments immediately below are originally from Nordic.

// Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
//
// param[in] p_rfidrs     RFIDr Service structure.
// param[in] p_ble_evt Pointer to the event received from BLE stack.
//
static void on_write(ble_rfidrs_t * p_rfidrs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if (
         (p_evt_write->handle == p_rfidrs->wrte_state_handles.value_handle)
         &&
         (p_rfidrs->wrte_state_handler != NULL)
       )
    {
        p_rfidrs->wrte_state_handler(p_rfidrs, p_evt_write->data, p_evt_write->len);
    }
    else if (
         (p_evt_write->handle == p_rfidrs->target_epc_handles.value_handle)
         &&
         (p_rfidrs->target_epc_handler != NULL)
       )
    {
        p_rfidrs->target_epc_handler(p_rfidrs, p_evt_write->data, p_evt_write->len);
    }
    else if (
        (p_evt_write->handle == p_rfidrs->target_epc_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {

        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_rfidrs->is_target_epc_indication_enabled = true;
            //evt.evt_type = BLE_HVC_EVT_INDICATION_ENABLED;
        }
        else
        {
            p_rfidrs->is_target_epc_indication_enabled = false;
            //evt.evt_type = BLE_HVC_EVT_INDICATION_DISABLED;
        }

    }
    else if (
         (p_evt_write->handle == p_rfidrs->program_epc_handles.value_handle)
         &&
         (p_rfidrs->program_epc_handler != NULL)
       )
    {
        p_rfidrs->program_epc_handler(p_rfidrs, p_evt_write->data, p_evt_write->len);
    }
    else if (
        (p_evt_write->handle == p_rfidrs->program_epc_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {

        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_rfidrs->is_program_epc_indication_enabled = true;
            //evt.evt_type = BLE_HVC_EVT_INDICATION_ENABLED;
        }
        else
        {
            p_rfidrs->is_program_epc_indication_enabled = false;
            //evt.evt_type = BLE_HVC_EVT_INDICATION_DISABLED;
        }

    }
    else if (
        (p_evt_write->handle == p_rfidrs->read_state_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
        &&
        (p_rfidrs->read_state_handler != NULL)
       )
    {
        ble_rfidrs_hvc_evt_t evt;

        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_rfidrs->is_read_state_indication_enabled = true;
            evt.evt_type = BLE_HVC_EVT_INDICATION_ENABLED;
        }
        else
        {
            p_rfidrs->is_read_state_indication_enabled = false;
            evt.evt_type = BLE_HVC_EVT_INDICATION_DISABLED;
        }

        p_rfidrs->read_state_handler(p_rfidrs, &evt);

    }
    else if (
        (p_evt_write->handle == p_rfidrs->pckt_data1_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
        &&
        (p_rfidrs->pckt_data1_handler != NULL)
       )
    {
        ble_rfidrs_hvc_evt_t evt;

        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_rfidrs->is_pckt_data1_indication_enabled = true;
            evt.evt_type = BLE_HVC_EVT_INDICATION_ENABLED;
        }
        else
        {
            p_rfidrs->is_pckt_data1_indication_enabled = false;
            evt.evt_type = BLE_HVC_EVT_INDICATION_DISABLED;
        }

        p_rfidrs->pckt_data1_handler(p_rfidrs, &evt);

    }
    else if (
        (p_evt_write->handle == p_rfidrs->pckt_data2_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_rfidrs->is_pckt_data2_notification_enabled = true;
        }
        else
        {
            p_rfidrs->is_pckt_data2_notification_enabled = false;
        }
    }
    else if (
        (p_evt_write->handle == p_rfidrs->wavfm_data_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_rfidrs->is_wavfm_data_notification_enabled = true;
        }
        else
        {
            p_rfidrs->is_wavfm_data_notification_enabled = false;
        }
    }
    else if (
        (p_evt_write->handle == p_rfidrs->log_messge_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_rfidrs->is_log_messge_notification_enabled = true;
        }
        else
        {
            p_rfidrs->is_log_messge_notification_enabled = false;
        }
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

//Superlative Semiconductor note: Function template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.
//Comments immediately below are originally from Nordic.

//Function for handling the HVC event.
//
// details Handles HVC events from the BLE stack.
//
// param[in]   p_rfidrs       RFIDr Service structure.
// param[in]   p_ble_evt   Event received from the BLE stack.
//

static void on_hvc(ble_rfidrs_t * p_rfidrs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_rfidrs->read_state_handles.value_handle)
    {
        ble_rfidrs_hvc_evt_t evt;

        evt.evt_type = BLE_HVC_EVT_INDICATION_CONFIRMED;
        p_rfidrs->read_state_handler(p_rfidrs, &evt);
    }
    else if (p_hvc->handle == p_rfidrs->pckt_data1_handles.value_handle)
    {
        ble_rfidrs_hvc_evt_t evt;

        evt.evt_type = BLE_HVC_EVT_INDICATION_CONFIRMED;
        p_rfidrs->pckt_data1_handler(p_rfidrs, &evt);
    }
    else
    {
    // Do nothing
    }
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function for adding Write State characteristic.
//
// param[in] p_rfidrs       RFIDR Service structure.
// param[in] p_rfidrs_init  Information needed to initialize the service.
//
// return NRF_SUCCESS on success, otherwise an error code.
//
static uint32_t wrte_state_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_WRTE_STATE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_WRTE_STATE_CHAR_LEN;
    attr_char_value.p_value   = 0;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->wrte_state_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

// Function for adding Target EPC characteristic.
//
// param[in] p_rfidrs       RFIDR Service structure.
// param[in] p_rfidrs_init  Information needed to initialize the service.
//
// return NRF_SUCCESS on success, otherwise an error code.
//
static uint32_t target_epc_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_value[MAX_EPC_LENGTH_IN_BYTES]    =    {0};
    //Note: this is set to match initial state in rfidr_txradio.c
    //It assumes the compiler will insert all zeros into the array.

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.indicate      = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_TARGET_EPC_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_EPC_LENGTH_IN_BYTES;  //Note: this is set to match initial state in rfidr_txradio.c
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_TARGET_EPC_CHAR_LEN;
    attr_char_value.p_value   = initial_value;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->target_epc_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

// Function for adding Target EPC characteristic.
//
// param[in] p_rfidrs       RFIDR Service structure.
// param[in] p_rfidrs_init  Information needed to initialize the service.
//
// return NRF_SUCCESS on success, otherwise an error code.
//
static uint32_t program_epc_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    //Adding proprietary characteristic to S110 SoftDevice
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;
    uint8_t                initial_value[MAX_EPC_LENGTH_IN_BYTES]    =    {0};
    //Note: this is set to match initial state in rfidr_txradio.c
    //It assumes the compiler will insert all zeros into the array.

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.indicate      = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_PROGRAM_EPC_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = MAX_EPC_LENGTH_IN_BYTES;  //Note: this is set to match initial state in rfidr_txradio.c
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_PROGRAM_EPC_CHAR_LEN;
    attr_char_value.p_value   = initial_value;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->program_epc_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

// Function for adding Read state characteristic.
//
// param[in] p_rfidrs       RFIDR Service structure.
// param[in] p_rfidrs_init  Information needed to initialize the service.
//
// return NRF_SUCCESS on success, otherwise an error code.
//
static uint32_t read_state_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    //Adding proprietary characteristic to S110 SoftDevice
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_value    =    0; //This is important since it matches the initialized state in rfidr_state.c.

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read     = 1;
    char_md.char_props.indicate = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_READ_STATE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_READ_STATE_CHAR_LEN;
    attr_char_value.p_value   = &initial_value;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->read_state_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

static uint32_t pckt_data1_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    //Adding proprietary characteristic to S110 SoftDevice
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.indicate = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_PCKT_DATA1_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_PCKT_DATA1_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->pckt_data1_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

static uint32_t pckt_data2_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    //Adding proprietary characteristic to S110 SoftDevice
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_PCKT_DATA2_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_PCKT_DATA2_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->pckt_data2_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

static uint32_t wavfm_data_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    //Adding proprietary characteristic to S110 SoftDevice
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_WAVFM_DATA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm); /*Unclear if this allows a write to state. Disable for now*/

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_WAVFM_DATA_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->wavfm_data_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

static uint32_t log_messge_char_add(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    //Adding proprietary characteristic to S110 SoftDevice
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_LOG_MESSGE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_RFIDRS_LOG_MESSGE_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_rfidrs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rfidrs->log_messge_handles);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

void ble_rfidrs_on_ble_evt(ble_rfidrs_t * p_rfidrs, ble_evt_t * p_ble_evt)
{
    if ((p_rfidrs == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_rfidrs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_rfidrs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_rfidrs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_rfidrs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

uint32_t ble_rfidrs_init(ble_rfidrs_t * p_rfidrs, const ble_rfidrs_init_t * p_rfidrs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t rfidrs_base_uuid = RFIDRS_BASE_UUID;

    if ((p_rfidrs == NULL) || (p_rfidrs_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    // Initialize the service structure.
    p_rfidrs->conn_handle                         = BLE_CONN_HANDLE_INVALID;
    p_rfidrs->wrte_state_handler                  = p_rfidrs_init->wrte_state_handler;
    p_rfidrs->target_epc_handler                  = p_rfidrs_init->target_epc_handler;
    p_rfidrs->program_epc_handler                 = p_rfidrs_init->program_epc_handler;
    p_rfidrs->read_state_handler                  = p_rfidrs_init->read_state_handler;
    p_rfidrs->pckt_data1_handler                  = p_rfidrs_init->pckt_data1_handler;
    p_rfidrs->is_target_epc_indication_enabled    = false;
    p_rfidrs->is_program_epc_indication_enabled   = false;
    p_rfidrs->is_read_state_indication_enabled    = false;
    p_rfidrs->is_pckt_data1_indication_enabled    = false;
    p_rfidrs->is_pckt_data2_notification_enabled  = false;
    p_rfidrs->is_wavfm_data_notification_enabled  = false;
    p_rfidrs->is_log_messge_notification_enabled  = false;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&rfidrs_base_uuid, &p_rfidrs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_rfidrs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RFIDRS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_rfidrs->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the write state Characteristic.
    err_code = wrte_state_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the target epc Characteristic.
    err_code = target_epc_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the new epc Characteristic.
    err_code = program_epc_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the read state Characteristic.
    err_code = read_state_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the packet data 1 Characteristic.
    err_code = pckt_data1_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the packet data 2 Characteristic.
    err_code = pckt_data2_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the waveform data Characteristic.
    err_code = wavfm_data_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the log message Characteristic.
    err_code = log_messge_char_add(p_rfidrs, p_rfidrs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "target epc" characteristic.

uint32_t ble_rfidrs_target_epc_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_target_epc_indication_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RFIDRS_TARGET_EPC_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->target_epc_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "program epc" characteristic.

uint32_t ble_rfidrs_program_epc_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_program_epc_indication_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RFIDRS_PROGRAM_EPC_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->program_epc_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "read state" characteristic.

uint32_t ble_rfidrs_read_state_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_read_state_indication_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length != BLE_RFIDRS_READ_STATE_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->read_state_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "packet data 1" characteristic.

uint32_t ble_rfidrs_pckt_data1_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_pckt_data1_indication_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length != BLE_RFIDRS_PCKT_DATA1_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->pckt_data1_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "packet data 2" characteristic.

uint32_t ble_rfidrs_pckt_data2_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_pckt_data2_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length != BLE_RFIDRS_PCKT_DATA2_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->pckt_data2_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "waveform data" characteristic.

uint32_t ble_rfidrs_wavfm_data_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_wavfm_data_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RFIDRS_WAVFM_DATA_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->wavfm_data_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}

//Superlative Semiconductor note: Function and comments template from Nordic SDK v8.0.
//Function modified by Superlative Semiconductor to fit the UHF RFID reader project.

//Function call to send a log message to the iDevice over the "log message" characteristic.

uint32_t ble_rfidrs_log_messge_send(ble_rfidrs_t * p_rfidrs, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_rfidrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_rfidrs->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rfidrs->is_log_messge_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RFIDRS_LOG_MESSGE_CHAR_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_rfidrs->log_messge_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_rfidrs->conn_handle, &hvx_params);
}
