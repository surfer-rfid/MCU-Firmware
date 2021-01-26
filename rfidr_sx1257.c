//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
// Module : RFIDr Firmware SX1257 SDR ASIC Drivers                              //
//                                                                              //
// Filename: rfidr_sx1257.c                                                     //
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

#include "app_error.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "rfidr_error.h"
#include "rfidr_spi.h"
#include <math.h>
#include <string.h>
#include <unistd.h>

#define    FREQUENCY_CODE_BASE    0x00CB5555

static    uint8_t    m_sx1257_frequency_slot    =    12;    //Start at 915MHz (13th slot out of 25)

//In order to ease coding, we predefine a number of operational frequencies that the reader can hop to.
//As of 6/19/2019, we haven't done any hopping, so this hasn't been tested yet, although we do know that
//the reader operates at 915MHz when we command it to.

//As of 11/25/2019, we've tested hopping and it seems to work OK.

//Frequency resolution is 68.66455Hz per bit, so each slot is about 1MHz away from its adjacent slots.
static uint32_t    sx1257_frequency_decode(uint8_t slot)
{
    switch(slot)
    {
        case(0):    return    FREQUENCY_CODE_BASE-0x0002AAAA;    break;
        case(1):    return    FREQUENCY_CODE_BASE-0x000271C7;    break;
        case(2):    return    FREQUENCY_CODE_BASE-0x000238E3;    break;
        case(3):    return    FREQUENCY_CODE_BASE-0x00020000;    break;
        case(4):    return    FREQUENCY_CODE_BASE-0x0001C71C;    break;
        case(5):    return    FREQUENCY_CODE_BASE-0x00018E38;    break;
        case(6):    return    FREQUENCY_CODE_BASE-0x00015555;    break;
        case(7):    return    FREQUENCY_CODE_BASE-0x00011C71;    break;
        case(8):    return    FREQUENCY_CODE_BASE-0x0000E38E;    break;
        case(9):    return    FREQUENCY_CODE_BASE-0x0000AAAA;    break;
        case(10):   return    FREQUENCY_CODE_BASE-0x000071C7;    break;
        case(11):   return    FREQUENCY_CODE_BASE-0x000038E3;    break;
        case(12):   return    FREQUENCY_CODE_BASE+0x00000000;    break;
        case(13):   return    FREQUENCY_CODE_BASE+0x000038E3;    break;
        case(14):   return    FREQUENCY_CODE_BASE+0x000071C7;    break;
        case(15):   return    FREQUENCY_CODE_BASE+0x0000AAAA;    break;
        case(16):   return    FREQUENCY_CODE_BASE+0x0000E38E;    break;
        case(17):   return    FREQUENCY_CODE_BASE+0x00011C71;    break;
        case(18):   return    FREQUENCY_CODE_BASE+0x00015555;    break;
        case(19):   return    FREQUENCY_CODE_BASE+0x00018E38;    break;
        case(20):   return    FREQUENCY_CODE_BASE+0x0001C71C;    break;
        case(21):   return    FREQUENCY_CODE_BASE+0x00020000;    break;
        case(22):   return    FREQUENCY_CODE_BASE+0x000238E3;    break;
        case(23):   return    FREQUENCY_CODE_BASE+0x000271C7;    break;
        case(24):   return    FREQUENCY_CODE_BASE+0x0002AAAA;    break;
        default:    return    FREQUENCY_CODE_BASE+0x00000000;    break;
    }
    return    FREQUENCY_CODE_BASE+0x00000000;
}

//This is code for loading the registers of the SX1257 after it is reset.
//The objective with some of the interesting sequence of operations is to avoid tonal behavior within the PLL
//at the setting that we found was required to be used for 1W (+30dBm) operation.
//It was found that touching the PLL XO input terminals could result in proper PLL convergence, so
//here too it was hoped that 'jiggling' the PLL would allow it to converge to a state which did not produce tonal behavior.
rfidr_error_t    load_sx1257_default(void)
{
    rfidr_error_t    error_code            =    RFIDR_SUCCESS;
    uint32_t         sx1257_freq_code      =    0x00CB5555;    //915MHz
    uint32_t         sx1257_freq_code_rx   =    0x00CB5555;    //915MHz

    m_sx1257_frequency_slot    =    12;    //915MHz
    sx1257_freq_code=sx1257_frequency_decode(m_sx1257_frequency_slot);    //Should return 0x00CB5555

    error_code    =    spi_cntrlr_write_sx1257_robust(0x00,0x00);    //Turn off everything on the SX1257.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x00,0x01);    //Turn on the SX1257 PDS and oscillator.
    if(error_code != RFIDR_SUCCESS){return error_code;}

    error_code    =    spi_cntrlr_write_sx1257_robust(0x08,0x28);    //-18dBFS (0x36) is high gain (1W out). This is just a random value we use during initialization.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x0A,0x00);    //Set TX PLL BW to 75kHz and set TX ANA BW to 213kHz (min) to improve emissions mask performance as much as possible. Was 0x60 for 26dBm operation.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x0B,0x05);    //Use 64 TX FIR_DAC taps. This minimizes the bandwidth of the TX digital filter and improves emissions mask performance as much as possible.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x0C,0xD4);    //Set LNA/RX gain to the minimum useful value (0xD4). Other useful values are 0x94 (medium) and 0x34 (high). Make sure Zin=50 ohms.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x0D,0xF5);    //Maximize RX ADC bandwidth so allow the highest possible BLF. Set oscillator freq. to 36MHz.  Set RX roofing filter BW to 500kHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x0E,0x06);    //RX PLL bandwidth to the min at 75kHz. Was 0x06 for +26dBm operation. Disable RX ADC temp measurement mode.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x10,0x00);    //Disable CLK_OUT, use XTAL (this means an XTAL or an OSC on the XTAL port), no loopback.
    if(error_code != RFIDR_SUCCESS){return error_code;}

    error_code    =    spi_cntrlr_write_sx1257_robust(0x01,(uint8_t)((sx1257_freq_code_rx>>16) & 255));//Set RX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x02,(uint8_t)((sx1257_freq_code_rx>>8) & 255));    //Set RX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x03,(uint8_t)((sx1257_freq_code_rx>>0) & 255));    //Set RX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x04,(uint8_t)((sx1257_freq_code>>16) & 255));    //Set TX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x05,(uint8_t)((sx1257_freq_code>>8) & 255));    //Set TX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x06,(uint8_t)((sx1257_freq_code>>0) & 255));    //Set TX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}


    //Turn on gradually - try to avoid tonal behavior from arising

    error_code    =    spi_cntrlr_write_sx1257_robust(0x00,0x03);    //Turn on the SX1257 RX front end and RX PLL.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    nrf_delay_ms(100);
    error_code    =    spi_cntrlr_write_sx1257_robust(0x00,0x07);    //Turn on the SX1257 TX front end and PLL (except TX PA driver).
    if(error_code != RFIDR_SUCCESS){return error_code;}
    nrf_delay_ms(100);
    error_code    =    spi_cntrlr_write_sx1257_robust(0x10,0x02);    //Enable CLK_OUT, use XTAL (this means an XTAL or an OSC on the XTAL port), no loopback.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    nrf_delay_ms(100);
    error_code    =    spi_cntrlr_write_sx1257_robust(0x00,0x0F);    //Turn on the SX1257 TX PA driver LAST.
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Set the SX1257 LNA gain using an SPI write.
rfidr_error_t    set_sx1257_lna_gain(uint8_t lna_gain)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_write_sx1257_robust(0x0C,lna_gain);    //The three values found to be useful during characterization of the SX1257
    if(error_code != RFIDR_SUCCESS){return error_code;}                  //are 0xD4 (low gain - needed to fit max. TX leakage through SX1257 receiver), 
                                                                         //0x94 (med gain), and 0x34 (high gain - needed for min. sensitivity).
    return RFIDR_SUCCESS;
}

//Get the SX1257 LNA gain using an SPI write.
rfidr_error_t    get_sx1257_lna_gain(uint8_t * lna_gain)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_read_sx1257_robust(0x0C,lna_gain);    //The three values found to be useful during characterization of the SX1257
    if(error_code != RFIDR_SUCCESS){return error_code;}                 //are 0xD4 (low gain - needed to fit max. TX leakage through SX1257 receiver), 
                                                                        //0x94 (med gain), and 0x34 (high gain - needed for min. sensitivity).
    return RFIDR_SUCCESS;
}

//Set the SX1257 upconverter chain gain to a predefined 'low' value.
rfidr_error_t    set_sx1257_tx_power_low(void)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_write_sx1257_robust(0x08,0x34);        //-18dBFS (0x36) is high gain and results in ~30dBm output power. 0x34 results in ~26dBm output power.
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Set the SX1257 upconverter chain gain to a predefined 'medium' value.
rfidr_error_t    set_sx1257_tx_power_med(void)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_write_sx1257_robust(0x08,0x34);        //-18dBFS (0x36) is high gain and results in ~30dBm output power. 0x34 results in ~26dBm output power.
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Set the SX1257 upconverter chain gain to a predefined 'high' value.
rfidr_error_t    set_sx1257_tx_power_high(void)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;
    error_code    =    spi_cntrlr_write_sx1257_robust(0x08,0x36);        //-18dBFS (0x36) is high gain and results in ~30dBm output power. 0x34 results in ~26dBm output power.

    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Set the SX1257 downconverter chain gain to a predefined 'low' value.
rfidr_error_t    set_sx1257_rx_gain_low()
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_write_sx1257_robust(0x0C,0xD4);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Set the SX1257 downconverter chain gain to a predefined 'medium' value.
rfidr_error_t    set_sx1257_rx_gain_med(void)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_write_sx1257_robust(0x0C,0x94);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Set the SX1257 downconverter chain gain to a predefined 'high' value.
rfidr_error_t    set_sx1257_rx_gain_high(void)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;

    error_code    =    spi_cntrlr_write_sx1257_robust(0x0C,0x34);
    if(error_code != RFIDR_SUCCESS){return error_code;}

    return RFIDR_SUCCESS;
}

//Try to fix tonal behavior of SX1257 PLL by rewriting registers and forcing a relock.
//122319 - Try to do with with RX while PA is active, while PA is driving dummy load.
//If TX PA and/or predriver is pulling on the PLL, it may pull it into a good phase relationship if it needs to reconverge
rfidr_error_t    set_sx1257_fix_tones(void)
{
    rfidr_error_t error_code            =    RFIDR_SUCCESS;
    uint32_t        sx1257_freq_code    =    0x00CB5565;

    sx1257_freq_code=sx1257_frequency_decode(m_sx1257_frequency_slot);
    error_code    =    spi_cntrlr_write_sx1257_robust(0x01,(uint8_t)((sx1257_freq_code>>16) & 255));   //Set RX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x02,(uint8_t)((sx1257_freq_code>>8) & 255));    //Set RX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x03,(uint8_t)((sx1257_freq_code>>0) & 255));    //Set RX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    
    nrf_delay_us(250);

    return RFIDR_SUCCESS;
}


//Perform a pseudo-random frequency hop. Make sure to do this when the PA is off.
//Maybe do this when the TX predriver is off?
rfidr_error_t    hop_sx1257_frequency(uint8_t * p_sx1257_frequency_slot)
{
    rfidr_error_t    error_code            =    RFIDR_SUCCESS;
    uint32_t         sx1257_freq_code      =    0x00CB5555;

    //For debugging, comment these out
    m_sx1257_frequency_slot    = (m_sx1257_frequency_slot+7) % 25;
    //m_sx1257_frequency_slot = 12;
    sx1257_freq_code=sx1257_frequency_decode(m_sx1257_frequency_slot);

    *p_sx1257_frequency_slot=m_sx1257_frequency_slot;

    //Rewrite SX1257 PLL-related registers.
    error_code    =    spi_cntrlr_write_sx1257_robust(0x01,(uint8_t)((sx1257_freq_code>>16) & 255));    //Set RX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x02,(uint8_t)((sx1257_freq_code>>8) & 255));    //Set RX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x03,(uint8_t)((sx1257_freq_code>>0) & 255));    //Set RX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x04,(uint8_t)((sx1257_freq_code>>16) & 255));    //Set TX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x05,(uint8_t)((sx1257_freq_code>>8) & 255));    //Set TX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x06,(uint8_t)((sx1257_freq_code>>0) & 255));    //Set TX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    
    nrf_delay_us(250);

    return RFIDR_SUCCESS;
}

//Set a particular frequency slot for use. Make sure to do this when the PA is off.
//Maybe do this when the TX predriver is off?
rfidr_error_t    set_sx1257_frequency(uint8_t sx1257_frequency_slot)
{
    rfidr_error_t    error_code            =    RFIDR_SUCCESS;
    uint32_t         sx1257_freq_code      =    0x00CB5555;

    //For debugging, just examine the central frequency
    sx1257_freq_code=sx1257_frequency_decode(sx1257_frequency_slot);
    
    //Rewrite SX1257 PLL-related registers.
    error_code    =    spi_cntrlr_write_sx1257_robust(0x01,(uint8_t)((sx1257_freq_code>>16) & 255));    //Set RX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x02,(uint8_t)((sx1257_freq_code>>8) & 255));    //Set RX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x03,(uint8_t)((sx1257_freq_code>>0) & 255));    //Set RX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x04,(uint8_t)((sx1257_freq_code>>16) & 255));    //Set TX frequency MSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x05,(uint8_t)((sx1257_freq_code>>8) & 255));    //Set TX frequency MidSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    error_code    =    spi_cntrlr_write_sx1257_robust(0x06,(uint8_t)((sx1257_freq_code>>0) & 255));    //Set TX frequency LSB - start at 915MHz.
    if(error_code != RFIDR_SUCCESS){return error_code;}
    
    nrf_delay_us(250);

    return RFIDR_SUCCESS;
}
