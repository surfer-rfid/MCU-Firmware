////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
// Module : RFIDr Main                                                            //
//                                                                                //
// Filename: main.c                                                               //
// Creation Date: circa 10/31/2016                                                //
// Author: Edward Keehr                                                           //
//                                                                                //
//                                                                                //
//    This file was derived from the Nordic Semiconductor example code in their   //
//    SDK 8.0. Specifically, this file was derived from "main.c" in the           //
//    ble_app_uart project.                                                       //
//                                                                                //
//     The required notice to be reproduced for Nordic Semiconductor code is      //
//     given below:                                                               //
//                                                                                //
//    /* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.            //
// *                                                                              //
// * The information contained herein is property of Nordic Semiconductor ASA.    //
// * Terms and conditions of usage are described in detail in NORDIC              //
// * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.                           //
// *                                                                              //
// * Licensees are granted free, non-transferable use of the information. NO      //
// * WARRANTY of ANY KIND is provided. This heading must NOT be removed from      //
// * the file.                                                                    //
// *                                                                              //
// */                                                                             //
//                                                                                //
//    For components of the code modified or authored by Superlative              //
//    Semiconductor LLC, the copyright notice is as follows:                      //
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
//    Description:                                                                //
//                                                                                //
//    This file contains the main() function for the RFIDr firmware.              //
//    In addition, it contains the BTLE characteristic event handlers as well as  //
//    several functions and settings governing the BTLE connection to the iDevice.//
//                                                                                //
//    Revisions:                                                                  //
//    061619 - Major commentary cleanup.                                          //
//    122720 - Added ADC support.                                                 //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_adc.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_rfidrs.h"
#include "app_util_platform.h"
#include "rfidr_spi.h"
#include "rfidr_gpio.h"
#include "rfidr_state.h"
#include "rfidr_txradio.h"

//Superlative Semiconductor note: The following template of defines was provided by
//Nordic Semiconductor. Values were modified by Superlative Semiconductor for the 
//UHF RFID reader project.

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                           // Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device.

#define DEVICE_NAME                     "RFIDr"                                     // Name of device. Will be included in the advertising data.
#define RFIDR_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN                  // UUID type for the RFIDR Service (vendor specific).

#define APP_ADV_INTERVAL                64                                          // The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           // The advertising timeout (in units of seconds).

#define APP_TIMER_PRESCALER             0                                           // Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE         4                                           // Size of timer operation queues.

#define MIN_CONN_INTERVAL               20                                          // Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL               40                                          // Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units.
#define SLAVE_LATENCY                   0                                           // Slave latency.
#define CONN_SUP_TIMEOUT                400                                         // Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  // Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) // Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           // Number of attempts before giving up the connection parameter negotiation.

#define DEAD_BEEF                       0xDEADBEEF                                  // Value used as error code on stack dump, can be used to identify stack location on stack unwind.

//Superlative Semiconductor note: The following set of definitions was provided
//by Nordic Semiconductor in their SDK peripheral->adc_simple example.


#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH 1
#endif

volatile int32_t adc_sample;

//Superlative Semiconductor note: The following set of definitions was modified
//from Nordic Semiconductor code to fit nomenclature in the RFID reader project.

static ble_rfidrs_t                     m_rfidrs;                                   // Structure to identify the RFIDr Service.
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    // Handle of the current connection.

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_RFIDRS_SERVICE, RFIDR_SERVICE_UUID_TYPE}};  // Universally unique service identifier.

static bool                             received_write_state_event;

//Superlative Semiconductor note: Function unchanged from Nordic SDK v8.0.
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    if(error_code != NRF_SUCCESS){
        NVIC_SystemReset();
    }
}

//Superlative Semiconductor note: Function unchanged from Nordic SDK v8.0.
void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();

    adc_sample = nrf_adc_result_get();
    update_adc_sample(adc_sample);
}

//Superlative Semiconductor note: Function unchanged from Nordic SDK 8.0.
void adc_config(void)
{
    const nrf_adc_config_t nrf_adc_config = { NRF_ADC_CONFIG_RES_10BIT,
                                 NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD,
                                 NRF_ADC_CONFIG_REF_VBG};

    // Initialize and configure ADC
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_6);
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    NVIC_EnableIRQ(ADC_IRQn);
}

//Superlative Semiconductor note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic.
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//Superlative Semiconductor note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

//The function below was written by Superlative Semiconductor LLC
//Event handler for the write state characteristic.
//This function calls the write_rfidr_state_next() function in rfidr_state.c to ensure that a proper state transition is being requested,
//then sets a flag so run the actual state machine code on the next iteration of the main while(1) loop.

static void rfidrs_wrte_state_handler(ble_rfidrs_t * p_rfidrs, uint8_t * p_data, uint16_t length)
{
    rfidr_state_t    request_rfidr_state_t;

    switch(p_data[0])
    {
        case(0):    request_rfidr_state_t    =    IDLE_UNCONFIGURED;          break;
        case(1):    request_rfidr_state_t    =    IDLE_CONFIGURED;            break;
        case(2):    request_rfidr_state_t    =    INITIALIZING;               break;
        case(3):    request_rfidr_state_t    =    SEARCHING_APP_SPECD_TAG;    break;
        case(4):    request_rfidr_state_t    =    SEARCHING_LAST_INV_TAG;     break;
        case(5):    request_rfidr_state_t    =    INVENTORYING;               break;
        case(6):    request_rfidr_state_t    =    TESTING_DTC;                break;
        case(7):    request_rfidr_state_t    =    PROGRAMMING_APP_SPECD_TAG;  break;
        case(8):    request_rfidr_state_t    =    PROGRAMMING_LAST_INV_TAG;   break;
        case(9):    request_rfidr_state_t    =    RECOVERING_WAVEFORM_MEMORY; break;
        case(10):   request_rfidr_state_t    =    RESET_SX1257_AND_FPGA;      break;
        case(11):   request_rfidr_state_t    =    KILL_TAG;                   break;
        case(12):   request_rfidr_state_t    =    PROGRAMMING_KILL_PASSWD;    break;
        case(13):   request_rfidr_state_t    =    TRACK_APP_SPECD_TAG;        break;
        case(14):   request_rfidr_state_t    =    TRACK_LAST_INV_TAG;         break;
        default:    request_rfidr_state_t    =    IDLE_UNCONFIGURED;          break;
    }

    write_rfidr_state_next(p_rfidrs, request_rfidr_state_t);
    received_write_state_event    =    true;
    //One question to ask here is why didn't we just call run_rfidr_state_machine right here instead using a flag to trigger it to run in the main loop.
    //We suppose this might be possible, but the intent was to ensure that the power_manage() function that typically keeps the reader in a low-power state
    //was indeed fully exited before doing any complicated operations.
}

//The function below was written by Superlative Semiconductor LLC

//Event handler for the user specified target EPC characteristic.
//Generate temporary memory for the EPC, copy the BTLE data to the temporary data, then 
//copy the temporary array into the user specified target EPC state variable.
//In this case, the target EPC can be any length because we may wish to target
//A wide range of tags with the same EPC prefix during inventory and tracking.
//As of right now, we are not sending character data with a null sentinel value from the iDevice.
//Rather, we are relying on the iDevice to transmit the correct length of the data, with a maximum number of bytes equal to 12.
//Also, sanitization here relies on BLE_RFIDRS_TARGET_EPC_CHAR_LEN being equal to MAX_EPC_LENGTH_IN_BYTES.
static void rfidrs_target_epc_handler(ble_rfidrs_t * p_rfidrs, uint8_t * p_data, uint16_t length)
{
    uint8_t    loop_copy                                    =    0;
    uint8_t    target_epc[BLE_RFIDRS_TARGET_EPC_CHAR_LEN]   =    {0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,0x89,0xAB,0xCD,0xEF};
    uint8_t    sanitized_length                             =    0;

    sanitized_length=MIN(BLE_RFIDRS_TARGET_EPC_CHAR_LEN,length);

    for(loop_copy=0;loop_copy < sanitized_length;loop_copy++)
    {
        target_epc[loop_copy]    =    *(p_data+loop_copy);
    }

    set_app_specd_target_epc(p_rfidrs,&target_epc[0],sanitized_length);
}

//The function below was written by Superlative Semiconductor LLC

//Event handler for the user specified EPC-to-be-programmed characteristic.
//Generate temporary memory for the EPC, copy the BTLE data to the temporary data, then 
//copy the temporary array into the target EPC state variable.
//FOr the purposes of this reader, we will only consider programming 12-byte EPCs
static void rfidrs_program_epc_handler(ble_rfidrs_t * p_rfidrs, uint8_t * p_data, uint16_t length)
{
    uint8_t    loop_copy                                                =    0;
    uint8_t    program_epc[BLE_RFIDRS_PROGRAM_EPC_CHAR_LEN]        =    {0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,0x89,0xAB,0xCD,0xEF};

    for(loop_copy=0;loop_copy < MIN(BLE_RFIDRS_PROGRAM_EPC_CHAR_LEN,length);loop_copy++)
    {
        program_epc[loop_copy]    =    *(p_data+loop_copy);
    }

    set_app_specd_program_epc(p_rfidrs,&program_epc[0]);
}

//The function below was written by Superlative Semiconductor LLC

//Event handler for the read state characteristic, i.e. when the MCU pushes a state change to the iDevice.
//This handler is for the indication ACK; the action is to get the indication confirmation flag state variable
//in the rfidr_state.c module.
static void rfidrs_read_state_handler(ble_rfidrs_t * p_rfidrs, ble_rfidrs_hvc_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HVC_EVT_INDICATION_CONFIRMED:
            rfidr_state_received_read_state_confirmation();
            break;

        default:
            // No implementation needed.
        break;
    }
}

//The function below was written by Superlative Semiconductor LLC

//Event handler for the data1 characteristic, i.e. when the MCU pushes a tag EPC to the iDevice.
//This handler is for the indication ACK; the action is to get the indication confirmation flag state variable
//in the rfidr_state.c module.
static void rfidrs_pckt_data1_handler(ble_rfidrs_t * p_rfidrs, ble_rfidrs_hvc_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HVC_EVT_INDICATION_CONFIRMED:
            rfidr_state_received_pckt_data1_confirmation();
            break;

        default:
            // No implementation needed.
        break;
    }
}


//Superlative Semiconductor Note: Function template unchanged from Nordic SDK v8.0.
//Function internals modified by Superlative Semiconductor to meet RFID reader project requirements.
//Comments originally from Nordic
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t               err_code;
    ble_rfidrs_init_t      rfidrs_init;
    
    memset(&rfidrs_init, 0, sizeof(rfidrs_init));

    rfidrs_init.wrte_state_handler                 = rfidrs_wrte_state_handler;
    rfidrs_init.target_epc_handler                 = rfidrs_target_epc_handler; //App-specified target EPC
    rfidrs_init.program_epc_handler                = rfidrs_program_epc_handler; //App-specified program EPC
    rfidrs_init.read_state_handler                 = rfidrs_read_state_handler;
    rfidrs_init.pckt_data1_handler                 = rfidrs_pckt_data1_handler;
    
    err_code = ble_rfidrs_init(&m_rfidrs, &rfidrs_init);
    APP_ERROR_CHECK(err_code);
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //Do nothing for now
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code    =    NRF_SUCCESS;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            rfidr_enable_led0();
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            rfidr_disable_led0();
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

//Superlative Semiconductor Note: Function template unchanged from Nordic SDK v8.0.
//Function internals modified by Superlative Semiconductor to meet RFID reader project requirements.
//Comments originally from Nordic
/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_rfidrs_on_ble_evt(&m_rfidrs, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    #if (defined(S130) || defined(S132))
        ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    #endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

//Superlative Semiconductor Note: Function unchanged from Nordic SDK v8.0.
//Comments originally from Nordic
///**@brief Function for placing the application in low power state while waiting for events.

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


//Application main function.
//Superlative Semiconductor Note: Function template derived from Nordic SDK v8.0 ble_app_uart project main.c.
//Function internals modified by Superlative Semiconductor to meet RFID reader project requirements.

int main(void)
{
    uint32_t err_code;
    
    //Initialize the MCU
    rfidr_gpiote_init(&m_rfidrs);
    spi_cntrlr_init();
    rfidr_txradio_init();
    rfidr_state_init();
    //We don't initialize the FPGA here, it's done as part of the state machine initialization state.

    //Initialize the Bluetooth LE aspects of the MCU and the SoftDevice
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    adc_config(); //See https://devzone.nordicsemi.com/f/nordic-q-a/9567/application-never-gets-into-adc_irqhandler

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    received_write_state_event=false;
    // Enter main loop.
    for(;;)
    {
        //If we get a message from the iDevice to change state, enter the state machine function.
        //Otherwise, wait in a low power state.
        if(received_write_state_event)
		{
            run_rfidr_state_machine(&m_rfidrs);
            received_write_state_event=false;
        }
        power_manage();
    }

}
