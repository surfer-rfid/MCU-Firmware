//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware GPIO setup and control                               //
//                                                                              //
// Filename: rfidr_gpio.c                                                       //
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
//    This file contains code required for initializing GPIOs on the RFIDr MCU  //
//    and also for performing short operations with the GPIOs (e.g. forcing     //
//    a reset on the SDR or the FPGA.                                           //
//    This file makes extensive reference to nrf_drv_config.h, which contains   //
//    the mapping of the pin firmware names to the actual GPIO assignments.     //
//                                                                              //
//    Revisions:                                                                //
//    061819 - Major commentary cleanup.                                        //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

#include "app_error.h"
#include "app_util_platform.h"
#include "ble_rfidrs.h"
#include "nrf_drv_config.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nordic_common.h"
#include "rfidr_error.h"
#include "rfidr_gpio.h"
#include "rfidr_state.h"
#include "rfidr_sx1257.h"
#include "rfidr_user.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

//
//    Define static variables for the software to (hopefully) mirror the internal state of the FPGA
//    At least, after reset on the pwr_togl line we hope to see agreement.
//

static    uint16_t            m_dtc_cap_val_1            =    512;    //This is the state variable for the first TMN resonator.
static    uint16_t            m_dtc_cap_val_2            =    512;    //This is the state variable for the second TMN resonator.
static    uint8_t             m_sx1257_lna_gain_state    =    1;      //This never got used, it was supposed to be a state variable for the SDR LNA gain.
static    uint8_t             m_sx1257_bba_gain_state    =    1;      //This never got used, it was supposed to be a state variable for the SDR baseband amplifier gain.
static     ble_rfidrs_t       *mp_rfidrs;                             //Static pointer to structure to identify the RFIDr BTLE Service.

//    Initialize all GPIOs. 
//    Pass as an argument to this function the address of the BTLE service structure 
//    so we can get error messages out of this scope to the iDevice over the BTLE connection.

uint32_t rfidr_gpiote_init(ble_rfidrs_t *p_rfidrs)
{
    uint32_t err_code = NRF_SUCCESS;
    
    mp_rfidrs=p_rfidrs;

    //Configuration structure for the output GPIO responsible for powering the PA up and down by collapsing its VDD.
    nrf_drv_gpiote_out_config_t config_vdd_pa_en_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for powering the PA up and down by enabling its bias voltages.
    nrf_drv_gpiote_out_config_t config_opa_spdt1_ctl_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for resetting the SX1257 SDR ASIC.
    nrf_drv_gpiote_out_config_t config_rdio_rst_p_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for enabling and disabling the SX1257 crystal oscillator.
    nrf_drv_gpiote_out_config_t config_xo_enable_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for resetting the FPGA.
    nrf_drv_gpiote_out_config_t config_fpga_rst_n_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for setting the antenna diversity switch to select the front antenna.
    nrf_drv_gpiote_out_config_t config_ant0_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_HIGH,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for setting the antenna diversity switch to select the side antenna.
    nrf_drv_gpiote_out_config_t config_ant1_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for turning LED 0 on and off.
    nrf_drv_gpiote_out_config_t config_bat_led0_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the output GPIO responsible for turning LED 1 on and off.
    nrf_drv_gpiote_out_config_t config_bat_led1_pin =
    {
            .init_state    =    NRF_GPIOTE_INITIAL_VALUE_LOW,
            .task_pin      =    false,
    };

    //Configuration structure for the input GPIO responsible for receiving an interrupt request (IRQ) from the FPGA.
    nrf_drv_gpiote_in_config_t config_fpga_irq_pin =
    {
            .sense          =    NRF_GPIOTE_POLARITY_LOTOHI,
            .pull           =    NRF_GPIO_PIN_NOPULL,
            .is_watcher     =    false,
            .hi_accuracy    =    true,
    };
    
    //Configuration structure for the input GPIO responsible for receiving a "power toggle" button press.
    nrf_drv_gpiote_in_config_t config_pwr_togl_pin =
    {
            .sense          =    NRF_GPIOTE_POLARITY_HITOLO,
            .pull           =    NRF_GPIO_PIN_NOPULL,
            .is_watcher     =    false,
            .hi_accuracy    =    true,
    };

    //Configuration structure for the input GPIO responsible for receiving a "sample" button press.
    nrf_drv_gpiote_in_config_t config_sample_pin =
    {
            .sense          =    NRF_GPIOTE_POLARITY_HITOLO,
            .pull           =    NRF_GPIO_PIN_NOPULL,
            .is_watcher     =    false,
            .hi_accuracy    =    true,
    };
    
    //Configuration structure for the input GPIO responsible for receiving a "cycle" button press.
    nrf_drv_gpiote_in_config_t config_cycle_pin =
    {
            .sense          =    NRF_GPIOTE_POLARITY_HITOLO,
            .pull           =    NRF_GPIO_PIN_NOPULL,
            .is_watcher     =    false,
            .hi_accuracy    =    true,
    };

    //Configuration structure for the input GPIO responsible for receiving information from the SX1257 SDR ASIC DIO3 (digital I/O #3) pin.
    nrf_drv_gpiote_in_config_t config_dio3_pin =
    {
            .sense          =    NRF_GPIOTE_POLARITY_HITOLO,
            .pull           =    NRF_GPIO_PIN_NOPULL,
            .is_watcher     =    false,
            .hi_accuracy    =    false,
    };

    //Configuration structure for the input GPIO responsible for receiving information from the SX1257 SDR ASIC DIO2 (digital I/O #2) pin.
    nrf_drv_gpiote_in_config_t config_dio2_pin =
    {
            .sense          =    NRF_GPIOTE_POLARITY_HITOLO,
            .pull           =    NRF_GPIO_PIN_NOPULL,
            .is_watcher     =    false,
            .hi_accuracy    =    false,
    };


    if(!nrf_drv_gpiote_is_init())    //This is an SDK function found in \SDK\components\drivers_nrf\gpiote\nrf_drv_gpiote.c. It checks to see if general GPIO handling has been initialized on the MCU.
    {
        err_code = nrf_drv_gpiote_init();    //This is an SDK function found in \SDK\components\drivers_nrf\gpiote\nrf_drv_gpiote.c. It sets up the general GPIO handling on the MCU.
        APP_ERROR_CHECK(err_code);
    }

    //A series of SDK functions from \SDK\components\drivers_nrf\gpiote\nrf_drv_gpiote.c are called to initialize each of the GPIO pins.
    //These functions take as arguments the initialization structures defined at the beginning of this function and the pin GPIO assignments defined in nrf_config_drv.h.
    
    //Output GPIO configuration.
    err_code = nrf_drv_gpiote_out_init(EN_VDD_PA_PIN, &config_vdd_pa_en_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(OPA_SPDT1_CTL_PIN, &config_opa_spdt1_ctl_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(RDIO_RST_P_PIN, &config_rdio_rst_p_pin);
    APP_ERROR_CHECK(err_code);
    //Special configuration is required for the SX1257 reset pin. It needs to float during POR of the chip. See the SX1257 data sheet for more details.
    nrf_gpio_cfg(RDIO_RST_P_PIN,NRF_GPIO_PIN_DIR_OUTPUT,NRF_GPIO_PIN_INPUT_DISCONNECT,NRF_GPIO_PIN_NOPULL,NRF_GPIO_PIN_D0S1,NRF_GPIO_PIN_NOSENSE);
    err_code = nrf_drv_gpiote_out_init(XO_ENABLE_PIN, &config_xo_enable_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(FPGA_RST_N_PIN, &config_fpga_rst_n_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(ANT0_PIN, &config_ant0_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(ANT1_PIN, &config_ant1_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(BAT_LED1_PIN, &config_bat_led1_pin);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(BAT_LED0_PIN, &config_bat_led0_pin);
    APP_ERROR_CHECK(err_code);
    
    //Input GPIO configuration. Note that event handlers need to be specified and enabled.
    err_code = nrf_drv_gpiote_in_init(FPGA_IRQ_PIN, &config_fpga_irq_pin, rfidr_fpga_irq_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(FPGA_IRQ_PIN, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(PWR_TOGL_PIN, &config_pwr_togl_pin, rfidr_pwr_togl_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(PWR_TOGL_PIN, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(SAMPLE_PIN, &config_sample_pin, rfidr_sample_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(SAMPLE_PIN, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(CYCLE_PIN, &config_cycle_pin, rfidr_cycle_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(CYCLE_PIN, true);
    APP_ERROR_CHECK(err_code);    
    err_code = nrf_drv_gpiote_in_init(DIO3_PIN, &config_dio3_pin, rfidr_dio3_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(DIO3_PIN, false);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(DIO2_PIN, &config_dio2_pin, rfidr_dio2_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(DIO2_PIN, false);
    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

//Event handler for the IRQ coming from the FPGA.
//The action is to just call the subsequent event handler in rfidr_state.c
void rfidr_fpga_irq_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    rfidr_state_received_irq();
}

//Event handler for the SX1257 DIO3 digital I/O pin. We don't use this for now, so the event handler does nothing.
void rfidr_dio3_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
}

//Event handler for the SX1257 DIO2 digital I/O pin. We don't use this for now, so the event handler does nothing.
void rfidr_dio2_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
}

//If we get an error in this file, turn off the PA and send a message back to the iDevice over BTLE.
void handle_gpio_error(ble_rfidrs_t *p_rfidrs, char * inputString, rfidr_error_t rfidr_error_code){
    char        rfidr_error_message[256]    =    {0};
    uint16_t    cnt                         =    0;
    uint32_t    nrf_error_code              =    NRF_SUCCESS;

    while (*(inputString+cnt) && cnt < 250)
    {
        rfidr_error_message[cnt]=*(inputString+cnt);
        cnt++;
    }

    rfidr_disable_pa(); //Don't check for error, just do it.
    sprintf(rfidr_error_message,"%s %d",rfidr_error_message,(uint8_t)rfidr_error_code);
    nrf_error_code=rfidr_error_complete_message_send(p_rfidrs,(uint8_t *)rfidr_error_message);
    if (nrf_error_code != NRF_ERROR_INVALID_STATE)
    {
       // APP_ERROR_CHECK(nrf_error_code);
    }

}

//Send a short message to the iDevice about things we are doing with the TMN DTC state counters.
//Yes, this is very similar to a function in rfidr_state.c. We suppose this is a candidate for cleaning up later, therefore.

void send_dtc_message_gpio(ble_rfidrs_t *p_rfidrs, char * inputString){
    char        rfidr_log_message[20]        =    {0};
    uint16_t    cnt                          =    0;
    uint32_t    nrf_error_code               =    NRF_SUCCESS;

    while (*(inputString+cnt) && cnt < 20)
    {
        rfidr_log_message[cnt]=*(inputString+cnt);
        cnt++;
    }

    nrf_error_code=rfidr_error_complete_message_send(p_rfidrs,(uint8_t *)rfidr_log_message);
    if (nrf_error_code != NRF_ERROR_INVALID_STATE)
    {
       // APP_ERROR_CHECK(nrf_error_code);
    }
}

//The event handlers for the Power Toggle, Sample, and Cycle buttons don't yet perform any user functions.
//Instead, they are used to help characterize the SX1257 and TMN.

//In this case, the buttons have the following actions:
// POWER_TOGGLE: If the state machine is in the TESTING_DTC state: Clear state variables. If the state machine is not in the TESTING_DTC state: increment SX1257 LNA/BBA gain.
// SAMPLE: Increment state variable 2.
// CYCLE: Increment state variable 1.

//There are actually two sets of state variables that need to be explicitly kept in sync.
//One set is in the MCU firmware (m_dtc_cap_val_X) and one set is in the FPGA.
//There is no loop-closing to ensure these values are the same - we rely on the robustness of MCU-FPGA communications to ensure this.

void rfidr_pwr_togl_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    char             dtc_message[20]         =    {0};
    rfidr_error_t    rfidr_error_code        =    RFIDR_SUCCESS;
    uint8_t          sx1257_lna_gain;
    rfidr_state_t    l_rfidr_state;

    read_rfidr_state(&l_rfidr_state);

    if(l_rfidr_state==TESTING_DTC)
    {
        //Reset MCU TMN/DTC state variables.
        m_dtc_cap_val_1    =    512;
        m_dtc_cap_val_2    =    512;
        rfidr_error_code=pwr_togl_received_irq();    //Call a function in rfidr_user.c that eventually results in an SPI write to the FPGA.
        //Apparently returned error code is always good.
        if(rfidr_error_code != RFIDR_SUCCESS){handle_gpio_error(mp_rfidrs,"At gpio, power toggle failure in tdtc:",rfidr_error_code);}
        //Compose a short message to the iDevice and send it over BTLE as a notification.
        sprintf(dtc_message,"CTC C1:%4d C2:%4d",m_dtc_cap_val_1,m_dtc_cap_val_2);
        send_dtc_message_gpio(mp_rfidrs, dtc_message);
    } 
    else 
    {
        //Cycle through the LNA and BBA gains of the SX1257 so that we can characterize its small signal gain and saturation point at each gain level.
        m_sx1257_bba_gain_state-=1;

        if(m_sx1257_bba_gain_state==0)
        {
            m_sx1257_bba_gain_state=4;
        }

        if((m_sx1257_bba_gain_state & 1) == 0)
        {
            m_sx1257_lna_gain_state-=1;
            if(m_sx1257_lna_gain_state==0)
            {
                m_sx1257_lna_gain_state=6;
            }
        }

        //Compile the gain setting for the SX1257 and send it over the SPI.
        sx1257_lna_gain=(uint8_t)((((m_sx1257_lna_gain_state << 5) + (m_sx1257_bba_gain_state & 1)*10) + 20) & 255 );
        set_sx1257_lna_gain(sx1257_lna_gain);
        if(rfidr_error_code != RFIDR_SUCCESS){handle_gpio_error(mp_rfidrs,"At gpio, power toggle failure out of tdtc:",rfidr_error_code);}
        //Compose a short message to the iDevice and send it over BTLE as a notification.
        sprintf(dtc_message,"LNA Gain is 0x%02X",sx1257_lna_gain);
        send_dtc_message_gpio(mp_rfidrs, dtc_message);
    }
}

void rfidr_sample_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    char             dtc_message[20]    =    {0};
    rfidr_error_t    rfidr_error_code   =    RFIDR_SUCCESS;
    //Possibly check for TESTING DTC mode here as we do for resetCaps
    m_dtc_cap_val_2    =    (m_dtc_cap_val_2+1) % 1024;
    rfidr_error_code=sample_received_irq(); //Call a function in rfidr_user.c that eventually results in an SPI write to the FPGA.
    //Apparently returned error code is always good.
    if(rfidr_error_code != RFIDR_SUCCESS){handle_gpio_error(mp_rfidrs,"At gpio, sample failure:",rfidr_error_code);}
    sprintf(dtc_message,"2TC C1:%4d C2:%4d",m_dtc_cap_val_1,m_dtc_cap_val_2);
    send_dtc_message_gpio(mp_rfidrs, dtc_message);
}

void rfidr_cycle_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    char             dtc_message[20]    =    {0};
    rfidr_error_t    rfidr_error_code  =    RFIDR_SUCCESS;
    //Possibly check for TESTING DTC mode here as we do for resetCaps
    m_dtc_cap_val_1    =    (m_dtc_cap_val_1+1) % 1024;
    rfidr_error_code=cycle_received_irq(); //Call a function in rfidr_user.c that eventually results in an SPI write to the FPGA.
    //Apparently returned error code is always good.
    if(rfidr_error_code != RFIDR_SUCCESS){handle_gpio_error(mp_rfidrs,"At gpio, cycle failure:",rfidr_error_code);}
    sprintf(dtc_message,"1TC C1:%4d C2:%4d",m_dtc_cap_val_1,m_dtc_cap_val_2);
    send_dtc_message_gpio(mp_rfidrs, dtc_message);
}

//A wrapper for selecting the front antenna port for connection to the radio.
//This wrapper is important because the antenna diversity switch has two control inputs which must be kept antipolar to each other.
//The software wrapper helps to enforce this antipolarity.
rfidr_error_t rfidr_sel_ant0(void)
{
    nrf_drv_gpiote_out_set(ANT0_PIN);
    nrf_drv_gpiote_out_clear(ANT1_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for selecting the side antenna port for connection to the radio.
//This wrapper is important because the antenna diversity switch has two control inputs which must be kept antipolar to each other.
//The software wrapper helps to enforce this antipolarity.
rfidr_error_t rfidr_sel_ant1(void)
{
    nrf_drv_gpiote_out_set(ANT1_PIN);
    nrf_drv_gpiote_out_clear(ANT0_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for enabling the PA. First we enable the bias, and then the VDD.
rfidr_error_t rfidr_enable_pa(void)
{
    nrf_drv_gpiote_out_set(OPA_SPDT1_CTL_PIN);
    nrf_drv_gpiote_out_set(EN_VDD_PA_PIN);
    nrf_delay_us(250); //Set a mandatory delay after powering up the PA.
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the PA. First we disable the VDD, then disable the bias.
rfidr_error_t rfidr_disable_pa(void)
{
    nrf_drv_gpiote_out_clear(EN_VDD_PA_PIN);
    nrf_drv_gpiote_out_clear(OPA_SPDT1_CTL_PIN);
    nrf_delay_us(250); //Set a mandatory delay after powering down the PA.
    return RFIDR_SUCCESS;
}

//A wrapper for enabling the FPGA.
//We also add a 100ms delay in case we disabled the FPGA immediately before this to comply with recommended reset wait times.
rfidr_error_t rfidr_enable_fpga(void)
{
    nrf_delay_ms(100);
    nrf_drv_gpiote_out_set(FPGA_RST_N_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the FPGA.
//We also add a 100ms delay to ensure we don't get into trouble disabling the PA too quickly after enabling it.
rfidr_error_t rfidr_disable_fpga(void)
{
    nrf_delay_ms(100);
    nrf_drv_gpiote_out_clear(FPGA_RST_N_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for resetting the FPGA.
//We also add 100ms delays to comply with recommended reset wait times.
rfidr_error_t rfidr_reset_fpga(void)
{
    nrf_delay_ms(100);
    nrf_drv_gpiote_out_clear(FPGA_RST_N_PIN);
    nrf_delay_ms(100);
    nrf_drv_gpiote_out_set(FPGA_RST_N_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for enabling the SX1257 SDR ASIC.
//We also add 20ms delays in case we disabled the SDR ASIC immediately before this to comply with recommended reset wait times.
rfidr_error_t rfidr_enable_radio(void)
{
    nrf_delay_ms(20);
    nrf_drv_gpiote_out_clear(RDIO_RST_P_PIN);
    nrf_delay_ms(20);
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the SX1257 SDR ASIC.
//We also add 20ms delays in case we enabled the SDR ASIC immediately before this to comply with recommended reset wait times.
rfidr_error_t rfidr_disable_radio(void)
{
    nrf_delay_ms(20);
    nrf_drv_gpiote_out_set(RDIO_RST_P_PIN);
    nrf_delay_ms(20);
    return RFIDR_SUCCESS;
}

//A wrapper for resetting the SX1257 SDR ASIC.
//We also add 20ms delays to comply with recommended reset wait times.
rfidr_error_t rfidr_reset_radio(void)
{
    nrf_delay_ms(20);
    nrf_drv_gpiote_out_set(RDIO_RST_P_PIN);
    nrf_delay_ms(20);
    nrf_drv_gpiote_out_clear(RDIO_RST_P_PIN);
    nrf_delay_ms(20);
    return RFIDR_SUCCESS;
}

//A wrapper for enabling the SX1257 crystal oscillator
rfidr_error_t rfidr_enable_xo(void)
{
    nrf_drv_gpiote_out_set(XO_ENABLE_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the SX1257 crystal oscillator
rfidr_error_t rfidr_disable_xo(void)
{
    nrf_drv_gpiote_out_clear(XO_ENABLE_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for enabling the LED 0
rfidr_error_t rfidr_enable_led0(void)
{
    nrf_drv_gpiote_out_set(BAT_LED0_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the LED 0
rfidr_error_t rfidr_disable_led0(void)
{
    nrf_drv_gpiote_out_clear(BAT_LED0_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the LED 0
rfidr_error_t rfidr_toggle_led0(void)
{
    nrf_drv_gpiote_out_toggle(BAT_LED0_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for enabling the LED 1
rfidr_error_t rfidr_enable_led1(void)
{
    nrf_drv_gpiote_out_set(BAT_LED1_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for disabling the LED 1
rfidr_error_t rfidr_disable_led1(void)
{
    nrf_drv_gpiote_out_clear(BAT_LED1_PIN);
    return RFIDR_SUCCESS;
}

//A wrapper for toggling the LED 1
rfidr_error_t rfidr_toggle_led1(void)
{
    nrf_drv_gpiote_out_toggle(BAT_LED1_PIN);
    return RFIDR_SUCCESS;
}
