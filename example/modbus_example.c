/*!
* @file modbus_local.c
* @author Jarrod Cook
*
* @brief Simple interface routines to the MODBUS driver
*
* This file just contains functions that allow main to interface with
* the MODBUS driver.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includes
#include "modbus_module.h"
#include "parameters.h"             //-----!!! NEED TO DEFINE A FILE TO PULL MB PARAMS FROM

//Local Variables
static bool mbIsErrored;                //!< Tracks whether the current MODBUS settings are an errored set or not
static modbus_modes curMbMode;          //!< Tracks the current mode for updating
static uint8_t curMbAddress;            //!< Tracks the current address for updating
static uint32_t curBaudRate;            //!< Tracks the current baudrate for updating
static modbus_parity curMbParity;       //!< Tracks the current parity for updating
static uint8_t curMbStopBits;           //!< Tracks the current number of stop bits for updating

/*!
* @brief Intializes the MODBUS driver for the board and pulls current settings
*
* This function should be called to initialize the MODBUS driver as
* necessary according to current settings.
*
* @return Boolean indicating if init was successful or not
*/
bool modbus_local_init(void)
{
    //Load in all the current values
    curMbMode = getModbusMode();
    curMbAddress = getModbusAddress();
    curBaudRate = getModbusBaudRate();
    curMbParity = getModbusParity();
    curMbStopBits = getModbusStopBits();

    //Initialize the MODBUS driver
    if((modbus_module_init(MB_PORT_NUM, curMbMode, MODBUS_SLAVE_DEVICE, curMbAddress, curBaudRate, curMbParity, curMbStopBits) != MODBUS_STAT_SUCCESS) || (modbus_module_enable(MB_PORT_NUM, true) != MODBUS_STAT_SUCCESS)) {
        mbIsErrored = true;
        return false;
    }

    return true;
}

/*!
* @brief Handles servicing of the MODBUS to handle packets
*
* This function should be called routinely in the main program loop
* in order to handle packets and also check for and update setting
* changes.
*/
void modbus_local_service(void)
{
    //See if any settings changed
    if(curMbMode != getModbusMode() || curBaudRate != getModbusBaudRate() || curMbParity != getModbusParity() || curMbStopBits != getModbusStopBits()) {
        //We had a setting change while enabled, so reinit everything
        curMbMode = getModbusMode();
        curMbAddress = getModbusAddress();
        curBaudRate = getModbusBaudRate();
        curMbParity = getModbusParity();
        curMbStopBits = getModbusStopBits();
        modbus_module_enable(MB_PORT_NUM, false);
        if((modbus_module_init(MB_PORT_NUM, curMbMode, MODBUS_SLAVE_DEVICE, curMbAddress, curBaudRate, curMbParity, curMbStopBits) != MODBUS_STAT_SUCCESS) || (modbus_module_enable(MB_PORT_NUM, true) != MODBUS_STAT_SUCCESS)) {
            mbIsErrored = true;
            error_module_flag_error(ERROR_MODBUS_INIT_FAIL);
        } else {
            mbIsErrored = false;
            error_module_clear_error(ERROR_MODBUS_INIT_FAIL);
        }
    } else if(curMbAddress != getModbusAddress()) {
        //We can do an address change on the fly
        curMbAddress = getModbusAddress();
        modbus_modbule_change_address(MB_PORT_NUM, curMbAddress);
    }

    //Now just call the service routine
    if(!mbIsErrored) {
        modbus_module_service();
    }
}


int main (void)
{

    modbus_local_init();

    while(1) {
        modbus_local_service();
    }

    return 0;
}