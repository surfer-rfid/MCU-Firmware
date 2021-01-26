//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware GPIO setup and control                               //
//                                                                              //
// Filename: rfidr_gpio.h                                                       //
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

//RFIDR GPIO Interface and Driver Functions
//This file provides high level abstraction to RFIDR GPIO functionality

#ifndef RFIDR_GPIO_H__
#define RFIDR_GPIO_H__

#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf_drv_gpiote.h"
#include "rfidr_error.h"

// function for initializing the RFIDR GPIO
// returns NRF_SUCCESS on successful GPIO initialization

uint32_t rfidr_gpiote_init(ble_rfidrs_t *p_rfidrs);

//Event handler for processing an IRQ coming back from the FPGA

void rfidr_fpga_irq_event_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

//Event handler for processing an IRQ coming from the PWR_TOGL pin

void rfidr_pwr_togl_event_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

//Event handler for processing an IRQ coming from the SAMPLE pin

void rfidr_sample_event_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

//Event handler for processing an IRQ coming from the CYCLE pin

void rfidr_cycle_event_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

//@brief Event handler for processing an IRQ coming from the DIO3 pin

void rfidr_dio3_event_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

//Event handler for processing an IRQ coming from the DIO2 pin

void rfidr_dio2_event_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

//function for selecting ANT0
//returns RFIDR_SUCCESS on successful selection of ANT0

rfidr_error_t rfidr_sel_ant0(void);

//function for selecting ANT1
//returns RFIDR_SUCCESS on successful selection of ANT1

rfidr_error_t rfidr_sel_ant1(void);

//function for enabling the PA and corresponding circuitry
//returns RFIDR_SUCCESS on successful enabling of the PA

rfidr_error_t rfidr_enable_pa(void);

//function for disabling the PA and corresponding circuitry
//returns RFIDR_SUCCESS on successful disabling of the PA

rfidr_error_t rfidr_disable_pa(void);

//function for enabling the FPGA
//returns RFIDR_SUCCESS on successful enabling of the FPGA

rfidr_error_t rfidr_enable_fpga(void);

//function for disabling the FPGA
//returns RFIDR_SUCCESS on successful disabling of the FPGA

rfidr_error_t rfidr_disable_fpga(void);

//function for resetting the FPGA
//returns RFIDR_SUCCESS on successful resetting of the FPGA

rfidr_error_t rfidr_reset_fpga(void);

//function for enabling the SX1257
//returns RFIDR_SUCCESS on successful enabling of the SX1257

rfidr_error_t rfidr_enable_radio(void);

//function for disabling the SX1257
//returns RFIDR_SUCCESS on successful disabling of the SX1257

rfidr_error_t rfidr_disable_radio(void);

//function for resetting the SX1257
//returns RFIDR_SUCCESS on successful resetting of the SX1257

rfidr_error_t rfidr_reset_radio(void);

//function for enabling the crystal oscillator
//returns RFIDR_SUCCESS on successful enabling of the oscillator

rfidr_error_t rfidr_enable_xo(void);

//function for disabling the crystal oscillator
//returns RFIDR_SUCCESS on successful disabling of the oscillator

rfidr_error_t rfidr_disable_xo(void);

//function for enabling led0
//returns RFIDR_SUCCESS on successful enabling of led0

rfidr_error_t rfidr_enable_led0(void);

//function for disabling led0
//returns RFIDR_SUCCESS on successful disabling led0

rfidr_error_t rfidr_disable_led0(void);

//function for toggling led0
//returns RFIDR_SUCCESS Successful toggling led0

rfidr_error_t rfidr_toggle_led0(void);

//function for enabling led1
//returns RFIDR_SUCCESS Successful enabling of led1

rfidr_error_t rfidr_enable_led1(void);

//function for disabling led1
//returns RFIDR_SUCCESS Successful disabling led1

rfidr_error_t rfidr_disable_led1(void);

//function for toggling led1
//returns RFIDR_SUCCESS Successful toggling led1

rfidr_error_t rfidr_toggle_led1(void);


#endif
