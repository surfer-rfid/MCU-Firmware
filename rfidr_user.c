////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
// Module : RFIDr Firmware User functions                                         //
//                                                                                //
// Filename: rfidr_user.c                                                         //
// Creation Date: circa 10/31/2016                                                //
// Author: Edward Keehr                                                           //
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
// Description:                                                                   //
//                                                                                //
//    This file contains a number of miscellaneous medium level "user" functions  //
//    that can be made accessible across the code space. Each of these functions  //
//    accomplishes a simple task with respect to the FPGA, such as setting        //
//    the CDR circuit to examine I or Q data, or to check whether the radio is    //
//    running or not.                                                             //
//                                                                                //
//    There are also a few secondary GPIO event handlers in here which interact   //
//    directly with the FPGA, such as sample_received_irq, which increments one   //
//    of the TMN DTC state variables during TESTING_DTC.                          //
//                                                                                //
//                                                                                //
//    Revisions:                                                                  //
//    061819 - Major commentary cleanup.                                          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

#include "app_error.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "rfidr_error.h"
#include "rfidr_gpio.h"
#include "rfidr_spi.h"
#include "rfidr_txradio.h"
#include "rfidr_user.h"

// Function used in rfidr_state.c that actually makes a write to FPGA memory to enter the DTC (TMN) test mode.
rfidr_error_t    enter_dtc_test_mode(void)
{
    uint8_t          recovery_byte     =    0;
    rfidr_error_t    error_code        =    NRF_SUCCESS;

    //Here, we are setting a sticky bit in FPGA SPI SLAVE user memory space (which is a bunch of registers).
    //First, do a read of the register to see what settings are currently in place.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)240;        //Retain all information at this user memory address except for three LSB one shot registers and fourth register, which is the DTC state control.
    recovery_byte    |=    (uint8_t)(1 << 3);    //Set DTC mode, don't perform a reset of the DTC state counter registers while we're at it.
    //Now, write the register with the new bit added in.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//     Function used in rfidr_state.c that actually makes a write to FPGA memory to exit the DTC (TMN) test mode.
rfidr_error_t    exit_dtc_test_mode(void)
{
    uint8_t          recovery_byte     =    0;
    rfidr_error_t    error_code        =    NRF_SUCCESS;

    //Here, we are setting a sticky bit in FPGA SPI SLAVE user memory space (which is a bunch of registers).
    //First, do a read of the register to see what settings are currently in place.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)240;    //Retain all information except for three LSB one shot registers and fourth register, which is the DTC state control.
    //Don't do anything here - clear out bit [3]
    //Now, write the register with the bit in question removed.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//     Function used in rfidr_gpio.c that actually makes a write to FPGA memory to clear the DTC state variables.
rfidr_error_t    pwr_togl_received_irq(void)
{
    uint8_t          recovery_byte    =    0;
    rfidr_error_t    error_code       =    NRF_SUCCESS;

    //Here, we are setting a one-shot bit in FPGA SPI SLAVE user memory space (which is a bunch of registers).
    //First, do a read of the register to see what settings are currently in place.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)248;    //Retain all information except for three LSB one shot registers.
    recovery_byte    |=    (uint8_t)(1 << 2);
    //Now, write the register with the new bit added in.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//     Function used in rfidr_gpio.c that actually makes a write to FPGA memory to increment the second DTC state variable.
rfidr_error_t    sample_received_irq(void)
{
    uint8_t          recovery_byte     =    0;
    rfidr_error_t    error_code        =    NRF_SUCCESS;

    //Here, we are setting a one-shot bit in FPGA SPI SLAVE user memory space (which is a bunch of registers).
    //First, do a read of the register to see what settings are currently in place.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)248;    //Retain all information except for three LSB one shot registers.
    recovery_byte    |=    (uint8_t)(1 << 1);
    //Now, write the register with the new bit added in.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//     Function used in rfidr_gpio.c that actually makes a write to FPGA memory to increment the first DTC state variable.
rfidr_error_t    cycle_received_irq(void)
{
    uint8_t          recovery_byte     =    0;
    rfidr_error_t    error_code        =    NRF_SUCCESS;

    //Here, we are setting a one-shot bit in FPGA SPI SLAVE user memory space (which is a bunch of registers).
    //First, do a read of the register to see what settings are currently in place.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)248;    //Retain all information except for three LSB one shot registers.
    recovery_byte    |=    (uint8_t)(1 << 0);
    //Now, write the register with the new bit added in.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//General function used to read from the FPGA user memory to see if it is done with a radio operation.
//Actually this "done" signal is only valid for one 4.5MHz clock cycle, so it will be difficult to catch.
bool    is_radio_done(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register in user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);

    //Interpret the register read-back bit.
    if(((recovery_byte >> 0) & 1) == 1 )
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}


//General function used to read from the FPGA user memory to see if the radio is running.
//This is useful to know because this is a bad time in general to make SPI writes to the FPGA.
bool    is_radio_running(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register in user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);

    //Interpret the register read-back bit.
    if(((recovery_byte >> 1) & 1) == 1 )
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

//Function what is used to check if the 36MHz clock from the SX1257 is toggling.
//The 36MHz clock is pretty robust, so the FPGA hardware to perform this detection was removed to save LUT.
//Currently this always returns matched the 36MHz clock valid bit.
bool    is_clk_36_running(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register in user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);

    //Interpret the register read-back bit.
    if(((recovery_byte >> 0) & 1) == 1 )
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

//Function that is used to check if the retimed 36MHz clock in the FPGA is valid.
bool    is_clk_36_valid(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register in user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);

    //Interpret the register read-back bit.
    if(((recovery_byte >> 1) & 1) == 1 )
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

//This function reads an exit code for the entire radio operation, as opposed to the exit code from just the data recovery circuit.
//Removing any hardware related to this functionality is a candidate for LUT savings.
uint8_t    read_radio_exit_code(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register in user memory and report back the byte that was read, masking only the relevant bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    return ((recovery_byte >> 5) & 7);
}

//This function reads the write counter from the radio_fsm.v to see how much progress we have made programming a tag.
uint8_t    read_radio_write_cntr(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register in user memory and report back the byte that was read, masking only the relevant bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)2, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    return ((recovery_byte >> 0) & 7);
}

//This function sets the user memory one-shot register to start the FPGA state machines performing an RFID operation.
rfidr_error_t    set_go_radio_oneshot(void)
{
    uint8_t          recovery_byte     =    0;
    rfidr_error_t    error_code        =    NRF_SUCCESS;

    //Read the register in user memory since we have to write back a byte with the other memory bits unchanged.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)252;        //Retain all information except for clock status signals (2LSB).
    recovery_byte    |=    (uint8_t)(1 << 0);
    //Write the register with the old data and the new one-shot bit added in.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//This function sets the user memory acknowledgement register to reset the FPGA top level state machine after it has reported itself done with the IRQ.
rfidr_error_t    set_irq_ack_oneshot(void)
{
    uint8_t          recovery_byte     =    0;
    rfidr_error_t    error_code        =    NRF_SUCCESS;

    //Read the register in user memory since we have to write back a byte with the other memory bits unchanged.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)252;    //Retain all information except for clock status signals.
    recovery_byte    |=    (uint8_t)(1 << 1);
    //Write the register with the old data and the new one-shot bit added in.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();

    return error_code;
}

//This function was used to start the FPGA-internal retimed 36MHz clock, but this feature has been disabled.
rfidr_error_t    set_clk_36_oneshot(void)
{
    //Note that a side effect of this command is to clear the alt_radio_fsm_loop and end_radio_fsm_loop registers
    //This should be OK since this command should only be used during startup of the FPGA

    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)2, (uint8_t)(1 << 7));
    spi_cntrlr_send_recv();
    return RFIDR_SUCCESS;
}

//Set a reset through the FPGA by setting a register bit which feeds into the FPGA reset tree.
//This isn't currently used within the code base, but perhaps represents a faster and more robust method of resetting the FPGA than toggling the FPGA external reset pin.
//Nevertheless, removing this feature represents an opportunity to save LUT.
rfidr_error_t    set_sw_reset(void)
{
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)2, 1);
    spi_cntrlr_send_recv();
    //No need to check anything on the FPGA because we just reset it
    //Actually maybe we should have had a register indicating this - maybe later
    //This does end up resetting the mster_spi_rdy explicitly though.
    return RFIDR_SUCCESS;
}

//This function is used extensively in rfidr_state.c to cause the RX CDR circuit to recover clock and data from the I signal path.
rfidr_error_t    set_use_i(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the register byte in which this bit is contained so that when we write back, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x0C;            //Retain radio mode signal only.
    recovery_byte    |=    (uint8_t)0x10;            //Or-in the bit to use I signaling path.
    //Write the new byte with the sticky use_i bit.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back the data to ensure that the bit was correctly written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & 1) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function is used extensively in rfidr_state.c to cause the RX CDR circuit to recover clock and data from the Q signal path.
rfidr_error_t    set_use_q(void)
{
    uint8_t    recovery_byte    =    0;

    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x0C;            //Retain radio mode signal only.
    recovery_byte    |=    (uint8_t)0x00;            //Or-in the bit to use Q signaling path (actually don't OR-in anything at all).
    //Write the new byte with the sticky use_q bit.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back the data to ensure that the bit was correctly written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & 1) == 0)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

rfidr_error_t    set_use_kill_pkt(void)
{
    uint8_t    recovery_byte    =    0;

    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    |=    (uint8_t)0x20;

    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 5) & 1) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function writes a bit into FPGA SPI peripheral user memory that tells the FPGA Radio state machine to transmit a select command on its next operation.
//While this bit is sticky, it is reset within the SPI peripheral module when the radio is done with a given reader-tag exchange.
rfidr_error_t    set_use_select_pkt(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    |=    (uint8_t)0x40;    //Or-in the bit that corresponds to requesting a select command to be transmitted.

    //Write the byte back to SPI user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)1, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back the byte to ensure that it was successfully written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 6) & 1) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function writes a bit into FPGA SPI peripheral user memory that tells the FPGA Radio state machine to transmit a new Query command at the next opportunity.
//While this bit is sticky, it is reset within the SPI peripheral module when the radio is done with a given reader-tag exchange.
rfidr_error_t    set_alt_radio_fsm_loop(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    |=    (uint8_t)0x20; //Or-in the bit that corresponds to requesting a select command to be transmitted.

    //Write the byte back to SPI user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)1, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back the byte to ensure that it was successfully written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 5) & 1) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function writes a bit into FPGA SPI peripheral user memory that tells the FPGA Radio state machine to end the inventory.
//While this bit is sticky, it is reset within the SPI peripheral module when the radio is done with a given reader-tag exchange.
rfidr_error_t    set_end_radio_fsm_loop(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    |=    (uint8_t)0x10;

    //Write the byte back to SPI user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)1, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back the byte to ensure that it was successfully written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & 1) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function clears all of byte 1 in the user memory, which as of 061819, consists of clk_36_start_next, use_select_pkt_next, alt_radio_fsm_loop_next, end_radio_fsm_loop_next.
//So if any of these bits have been written and they all need to be cleared at once, this function will do it.

rfidr_error_t    clear_query_inventory(void)
{
    uint8_t    recovery_byte    =    0;

    //Clear byte 1 in the SPI peripheral user memory.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)1, (0 << 4));
    spi_cntrlr_send_recv();
    //Check to see that the query/inventory bits have indeed been cleared.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)1, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & 3) == 0)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the radio mode register in the SPI peripheral user memory to the "search" value.
//Which means that the FPGA Radio FSM will do a search for a tag (or tags) specified with a select packet.
//With Query Q=0.
rfidr_error_t    set_radio_mode_search(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x10;            //Retain use_i bit only.
    recovery_byte    |=    (uint8_t)0x00;            //Write the code ('00') for search mode.
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 2) & 3) == 0)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the radio mode register in the SPI peripheral user memory to the "inventory" value.
//Which means that the FPGA Radio FSM will search for a large number of tags until the MCU firmware tells it to stop by setting the end_radio_fsm_loop bit.
rfidr_error_t    set_radio_mode_inventory(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x10;            //Retain use_i signal only.
    recovery_byte    |=    (uint8_t)0x04;
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 2) & 3) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the radio mode register in the SPI peripheral user memory to the "program confirm" value.
//This is essentially the same as the "search" mode and we should get rid of it to save LUT in the FPGA.
rfidr_error_t    set_radio_mode_prog_cfm(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x10;            //Retain use_i signal only.
    recovery_byte    |=    (uint8_t)0x08;
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 2) & 3) == 2)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the radio mode register in the SPI peripheral user memory to the "program" value.
//Which means that the FPGA Radio FSM will go through all of the reader-tag exchanges required to program a tag, lock its EPC, and read back the EPC value.
rfidr_error_t    set_radio_mode_program(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x10;            //Retain use_i signal only.
    recovery_byte    |=    (uint8_t)0x0C;
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)0, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)0, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 2) & 3) == 3)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the register in the SPI peripheral user memory which instructs the data recovery module to enter the PLL check mode.
//Currently, the SX1257 does not reliably lock its PLLs at the PLL bandwidth setting required to get good SNR during +30dBm (1W) output power operation.
//This condition can be detected by performing a sort of RSSI test within the data recovery circuit when the radio is connected to a 50 ohm dummy load.
rfidr_error_t    set_sx1257_pll_chk_mode(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x00;    //Retain nothing for the time being. Currently other bits relate to DTC test. This should be cleared here if it was set.
    recovery_byte    |=    (uint8_t)0x10;
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & 1) == 1)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function unsets the register in the SPI peripheral user memory which instructs the data recovery module to enter the PLL check mode.
//In other words, this function causes the data recovery module to resume regular operations.
rfidr_error_t    unset_sx1257_pll_chk_mode(void)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x00;    //Retain nothing for the time being. Currently other bits relate to DTC test. This should be cleared here if it was set.
    recovery_byte    |=    (uint8_t)0x00;
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)6, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)6, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & 1) == 0)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the offset going to the sigma-delta modulator to minimize TX output power on a logic "0"
rfidr_error_t    set_tx_sdm_offset(uint8_t offset)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)7, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0x0F;                    //Retain zero gen offset in the LSB position only.
    recovery_byte    |=    (offset << 4) & (uint8_t)0xF0;    //Write the MSB for this register.
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)7, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)7, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 4) & (uint8_t)0x0F) == offset)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}

//This function sets the offset going to the zero gen to minimize TX output power on a logic "0"
rfidr_error_t    set_tx_zgn_offset(uint8_t offset)
{
    uint8_t    recovery_byte    =    0;

    //Read the target user memory byte so that when we re-write it, we don't change any of the other bits.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)7, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    recovery_byte    &=    (uint8_t)0xF0;                    //Retain sdm offset in the MSB position only.
    recovery_byte    |=    (offset << 0) & (uint8_t)0x0F;    //Write the MSB for this register.
    //Write to the register
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_WRITE, RFIDR_SPI_TXRAM, (uint16_t)7, recovery_byte);
    spi_cntrlr_send_recv();
    //Read back from the register to ensure that the correct data was written.
    spi_cntrlr_set_tx(RFIDR_USER_MEM, RFIDR_SPI_READ, RFIDR_SPI_TXRAM, (uint16_t)7, 0);
    spi_cntrlr_send_recv();
    spi_cntrlr_read_rx(&recovery_byte);
    if(((recovery_byte >> 0) & (uint8_t)0x0F) == offset)
    {
        return RFIDR_SUCCESS;
    }
    else
    {
        return RFIDR_ERROR_USER_MEM;
    }
    return RFIDR_SUCCESS;
}
