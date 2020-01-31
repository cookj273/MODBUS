/*!
* @file modbus_callbacks.h
* @author Jarrod Cook
*
* @brief Callback functions for retrieving program data for MODBUS
*
* This file contains implementation of stubbed functions in modbus_callbacks.h
* the intent is for these functions to retrieve or set requested program
* registers according to MODBUS commands.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includes
#include <time.h>
#include "modbus_callbacks.h"
#include "real_time_clock.h"                //------!!!! NEED TO DEFINE FILE TO PULL AND KEEP TIME FROM
#include "temp_sensor.h"                    //------!!!! NEED TO DEFINE FILE TO PULL TEMP FROM

//Defines
#define COIL_START_ADDRESS              0       //!< Start Address of MODBUS coils
#define DISCRETE_START_ADDRESS          0       //!< Start Address of MODBUS discrete inputs
#define INPUT_START_ADDRESS             0       //!< Start Address of MODBUS input registers
#define HOLDING_START_ADDRESS           0       //!< Start Address of MODBUS holding registers

//Typedefs
/*!
* The following types defines all available coils
*/
typedef enum {
    COIL_PULSE_OUT1 = 0,        //!< Output Pulse for channel 1 current state
    COIL_PULSE_OUT2,            //!< Output Pulse for channel 2 current state
    COIL_LAST_ONE               //!< Marker of end of coil range
} board_coils;

/*!
* The following types defines all available discrete inputs
*/
typedef enum {
    DISCRETE_LOW_BATTERY = 0,   //!< Low battery indicator from monitor chip
    DISCRETE_SD_CARD_PRESENT,   //!< SD Card present indicator from socket
    DISCRETE_LAST_ONE           //!< Marker of end of discrete range
} board_discretes;

/*!
* The following types defines all available input registers
*/
typedef enum {
    INPUT_HARDWARE_VERSION = 0,         //!< The Hardware version (MSB is Major, and LSB is Minor)
    INPUT_SOFTWARE_VERSION,             //!< The software version (MSB is Major, and LSB is Minor)
    INPUT_TEMPERATURE_MSW,              //!< Current Temperature MSW (float)
    INPUT_TEMPERATURE_LSW,              //!< Current Temperature LSW (float)
    INPUT_LAST_ONE                      //!< Marking of the end of the input registers
} board_inputs;

/*!
* The following types defines all available holding registers
*/
typedef enum {
    HOLDING_MONTH = 0,                      //!< Current Month (uint)
    HOLDING_DAY,                        //!< Current Day (uint)
    HOLDING_YEAR,                       //!< Current Year (uint)
    HOLDING_HOUR,                       //!< Current Hour (uint)
    HOLDING_MINUTE,                     //!< Current Minute (uint)
    HOLDING_SECOND,                     //!< Current Second (uint)
    HOLDING_LAST_ONE                    //!< Marker of the end of the holding registers
} board_holding;

/*!
* The following structure is used for tracking item changes during
* modbus writes.
*/
typedef struct {
    bool changed;       //!< Indicates if the value has changed or not
    uint32_t val;       //!< The current value of the setting
} reg_stat;

//Local Variables
uint8_t pulseOut1 = 0;
uint8_t pulseOut2 = 0;
/*!
* The following structure tracks all of the holding registers so that we
* can verify before change on all of them.
*/
static struct temp_hold_struct {
    reg_stat month;                                             //!< Stores changes to the current month
    reg_stat day;                                               //!< Stores changes to the current day
    reg_stat year;                                              //!< Stores changes to the current year
    reg_stat hour;                                              //!< Stores changes to the current hour
    reg_stat minute;                                            //!< Stores changes to the current minute
    reg_stat second;                                            //!< Stores changes to the current second
} tempHolding;

/*!
* @brief The following function is called when a frame fully completes
*
* This function is intended to handle tasks that need to happen post frame
* completion.  Basically things like resetting the controller and changing
* settings where we need to issue a response before performing the
* changes.
*
* @param port The port the frame was completed on
* @param completedFunc The function that was just completed
*/
void modbus_frame_complete_callback(uint8_t port, modbus_function_codes completedFunc)
{
    uint8_t idx;
    time_t systemTime;
    struct tm *timePtr;

    //We only have one port so if its not it then we shouldn't be here
    if(port != MB_PORT_NUM) {
        return;
    }

    //Time Settings
    if(tempHolding.month.changed || tempHolding.day.changed || tempHolding.year.changed ||
            tempHolding.hour.changed || tempHolding.minute.changed || tempHolding.second.changed) {
        systemTime = real_time_clock_get_time();
        timePtr = gmtime(&systemTime);

        //Make sure the day is in range if month or year changed
        if(tempHolding.year.changed || tempHolding.month.changed) {
            if(tempHolding.day.val > (NUM_MONTH_DAYS[tempHolding.month.val] + (((tempHolding.year.val%4 == 0) && (tempHolding.month.val == 1))?1:0))) {
                tempHolding.day.val = (NUM_MONTH_DAYS[tempHolding.month.val] + (((tempHolding.year.val%4 == 0) && (tempHolding.month.val == 1))?1:0));
                tempHolding.day.changed = true;
            }
        }

        //Set all the new vals
        if(tempHolding.month.changed) {
            timePtr->tm_mon = tempHolding.month.val;
        }
        if(tempHolding.day.changed) {
            timePtr->tm_mday = tempHolding.day.val;
        }
        if(tempHolding.year.changed) {
            timePtr->tm_year = (tempHolding.year.val-1900);
        }
        if(tempHolding.hour.changed) {
            timePtr->tm_hour = tempHolding.hour.val;
        }
        if(tempHolding.minute.changed) {
            timePtr->tm_min = tempHolding.minute.val;
        }
        if(tempHolding.second.changed) {
            timePtr->tm_sec = tempHolding.second.val;
        }

        real_time_clock_set_time(mktime(timePtr));
    }

    //Save any changes to EEPROM
    parameters_save_changes();
}
}

/*!
* @brief The following function gets device info for this board
*
* This function handles the get device info MODBUS command.  It simply
* returns the additional device information as defined within the manual.
*
* @param port The port this request came over
* @param devInfo Pointer to the frame to place data into
*
* @return The number of bytes written
*/
uint8_t modbus_device_info_callback(uint8_t port, uint8_t *devInfo)
{
    //We only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return 0;
    }

    devInfo[0] = 0x00; //SERVER ID (can be longer)
    devInfo[1] = 0xFF; //Run Status 0xFF = on, ox00 = off
    devInfo[2] = 0x00; //Additional info (can be longer)

    return 3;
}

/*!
* @brief Gets the start address of coils for a port
*
* This function simply returns the address at which coils start
* for this port.
*
* @param port The port to get the start address for
*
* @return The coil start address
*/
uint16_t modbus_coil_start_address(uint8_t port)
{
    return COIL_START_ADDRESS;
}

/*!
* @brief Gets the total number of available coils
*
* This function simply returns the number of coils available in the
* system.
*
* @param port The port this request came over
*
* @return The number of available coils
*/
uint16_t modbus_number_of_coils(uint8_t port)
{
    return COIL_LAST_ONE;
}

/*!
* @brief Reads and returns the specified coil
*
* This function reads the coil number given and returns
* it's value.
*
* @param port The port this request came over
* @param coil The number of the coil to read
*
* @return True(0xFF) if coil is on, otherwise False(0x00)
*/
uint8_t modbus_coil_read_callback(uint8_t port, uint16_t coil)
{
    //We only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return 0x00;
    }

    //Get the coil
    switch((board_coils)coil) {
    case COIL_PULSE_OUT1:
        return pulseOut1;
    case COIL_PULSE_OUT2:
        return pulseOut2;
    default:
        break;
    }

    return 0x00;
}

/*!
* @brief Writes the specified coil
*
* This function writes the coil number given.
*
* @param port The port this request came over
* @param coil The number of the coil to write
* @param on Whether to turn the coil on(true/0xFF) or off(false/0x00)
*
* @return Exception code indicating if coil was successfully written.
*/
modbus_exceptions modbus_coil_write_callback(uint8_t port, uint16_t coil, uint8_t on)
{
    //We only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    //Write the coil
    switch((board_coils)coil) {
    case COIL_PULSE_OUT1:
        pulseOut1 = on;
        break;
    case COIL_PULSE_OUT2:
        pulseOut2 = on;
        break;
    default:
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        break;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief Gets the start address of the discrete inputs
*
* This function simply gets the address the discretes for this port start at
*
* @param port The port this request came over
*
* @return The start address of the discretes
*/
uint16_t modbus_discrete_start_address(uint8_t port)
{
    return DISCRETE_START_ADDRESS;
}

/*!
* @brief Gets the total number of available discrete inputs
*
* This function simply returns the number of discrete inputs available in
* the system.
*
* @param port The port this request came over
*
* @return The number of available discrete inputs
*/
uint16_t modbus_number_of_discretes(uint8_t port)
{
    return DISCRETE_LAST_ONE;
}

/*!
* @brief Reads and returns the specified discrete input
*
* This function reads the discrete input number given and returns
* it's value.
*
* @param port The port this request came over
* @param coil The number of the discrete input to read
*
* @return True(0xFF) if input is on, otherwise False(0x00)
*/
uint8_t modbus_discrete_read_callback(uint8_t port, uint16_t discrete)
{
    //We only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return 0x00;
    }

    switch((board_discretes)discrete) {
    case DISCRETE_LOW_BATTERY:
        //Active-low
        return 1;
    case DISCRETE_SD_CARD_PRESENT:
        //Active-low
        return 0;
    default:
        break;
    }

    return 0x00;
}

/*!
* @brief Gets the address that the input registers start at
*
* This function simply returns the address that the input registers
* begin at for this port.
*
* @param port The port this request came over
*
* @return The start address of the input registers
*/
uint16_t modbus_input_start_address(uint8_t port)
{
    return INPUT_START_ADDRESS;
}

/*!
* @brief Gets the total number of available input registers
*
* This function simply returns the number of input registers available
* in the system.
*
* @param port The port this request came over
*
* @return The number of available input registers
*/
uint16_t modbus_number_of_inputs(uint8_t port)
{
    return INPUT_LAST_ONE;
}

/*!
* @brief Reads and returns the specified input register
*
* This function reads the input register given and returns
* it's value.
*
* @param port The port this request came over
* @param reg The number of the input register to read
*
* @return The value of the input register
*/
uint16_t modbus_input_read_callback(uint8_t port, uint16_t reg)
{
    float tempFloat;
    pulse_input_channel chn = PULSE_INPUT_2;

    //We only have one port so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return 0x0000;
    }

    //Get the input register
    switch((board_inputs)reg) {
    case INPUT_HARDWARE_VERSION:
        return ((uint16_t)(HW_VERSION_MAJOR<<8) & 0xFF00) | ((uint16_t)HW_VERSION_MINOR & 0x00FF);
    case INPUT_SOFTWARE_VERSION:
        return ((uint16_t)(SW_VERSION_MAJOR<<8) & 0xFF00) | ((uint16_t)SW_VERSION_MINOR & 0x00FF);
    case INPUT_TEMPERATURE_MSW:
        tempFloat = temp_sensor_get_temp();
        return (uint16_t)(((*((uint32_t *)&tempFloat))>>16) & 0x0000FFFF);
    case INPUT_TEMPERATURE_LSW:
        tempFloat = temp_sensor_get_temp();
        return (uint16_t)((*((uint32_t *)&tempFloat)) & 0x0000FFFF);
    default:
        break;
    }

    return 0x0000;
}

/*!
* @brief Gets the start address of the holding registers
*
* This function simply returns the starting address for the holding
* registers on this port.
*
* @param port The port this request came over
*
* @return The start address of the port holding registers
*/
uint16_t modbus_holding_start_address(uint8_t port)
{
    return HOLDING_START_ADDRESS;
}

/*!
* @brief Gets the total number of available holding registers
*
* This function simply returns the number of holding registers available
* in the system.
*
* @param port The port this request came over
*
* @return The number of available holding registers
*/
uint16_t modbus_number_of_holding(uint8_t port)
{
    return HOLDING_LAST_ONE;
}

/*!
* @brief Reads and returns the specified holding register
*
* This function reads the holding register given and returns
* it's value.
*
* @param port The port this request came over
* @param reg The number of the holding register to read
* @param isWriteRead Indicates if this is part of a dual write/read command
*
* @return The value of the holding register
*/
uint16_t modbus_holding_read_callback(uint8_t port, uint16_t reg, bool isWriteRead)
{
    pulse_input_channel inChn = PULSE_INPUT_2;
    pulse_output_channel outChn = PULSE_OUTPUT_2;
    time_t systemTime;
    struct tm *timePtr;

    //We only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return 0x0000;
    }

    switch((board_holding)reg) {
    case HOLDING_MONTH:
        if(isWriteRead) {
            return (uint16_t)tempHolding.month.val+1;
        } else {
            systemTime = real_time_clock_get_time();
            timePtr = gmtime(&systemTime);
            return (uint16_t)(timePtr->tm_mon+1);
        }
    case HOLDING_DAY:
        if(isWriteRead) {
            return (uint16_t)tempHolding.day.val;
        } else {
            systemTime = real_time_clock_get_time();
            timePtr = gmtime(&systemTime);
            return (uint16_t)timePtr->tm_mday;
        }
    case HOLDING_YEAR:
        if(isWriteRead) {
            return (uint16_t)tempHolding.year.val;
        } else {
            systemTime = real_time_clock_get_time();
            timePtr = gmtime(&systemTime);
            return (uint16_t)(1900 + timePtr->tm_year);
        }
    case HOLDING_HOUR:
        if(isWriteRead) {
            return (uint16_t)tempHolding.hour.val;
        } else {
            systemTime = real_time_clock_get_time();
            timePtr = gmtime(&systemTime);
            return (uint16_t)timePtr->tm_hour;
        }
    case HOLDING_MINUTE:
        if(isWriteRead) {
            return (uint16_t)tempHolding.minute.val;
        } else {
            systemTime = real_time_clock_get_time();
            timePtr = gmtime(&systemTime);
            return (uint16_t)timePtr->tm_min;
        }
    case HOLDING_SECOND:
        if(isWriteRead) {
            return (uint16_t)tempHolding.second.val;
        } else {
            systemTime = real_time_clock_get_time();
            timePtr = gmtime(&systemTime);
            return (uint16_t)timePtr->tm_sec;
        }
    default:
        break;
    }

    return 0x0000;
}

/*!
* @brief Initializes temporary storage for writing registers
*
* Since there are restrictions on many of the registers we allow to be
* written we actually store all writes to a temporary place first, then
* verify the contents before deciding on success and performing the writes.
*
* @param port The port this request came over
*/
void modbus_holding_start_write_callback(uint8_t port)
{
    uint8_t idx;
    time_t systemTime;
    struct tm *timePtr;

    //We only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return;
    }

    //Get the current Time settings
    systemTime = real_time_clock_get_time();
    timePtr = gmtime(&systemTime);
    tempHolding.month.changed = false;
    tempHolding.month.val = timePtr->tm_mon;
    tempHolding.day.changed = false;
    tempHolding.day.val = timePtr->tm_mday;
    tempHolding.year.changed = false;
    tempHolding.year.val = (1900 + timePtr->tm_year);
    tempHolding.hour.changed = false;
    tempHolding.hour.val = timePtr->tm_hour;
    tempHolding.minute.changed = false;
    tempHolding.minute.val = timePtr->tm_min;
    tempHolding.second.changed = false;
    tempHolding.second.val = timePtr->tm_sec;
}

/*!
* @brief Verifies if the holding register writes just performed were valid
*
* This function is called immediately after all the write values are
* applied using modbus_holding_write_callback.  It looks at all the changed
* values in the temp aread to ensure they are valid.  It will then return
* a status indicating if the write is good or not.
*
* @param port The port this request came over
*
* @return Exception status that indicates if the write values are valid or not
*/
modbus_exceptions modbus_holding_verify_write_callback(uint8_t port)
{
    uint8_t idx;

    //We only have one port so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    //Time Settings
    if((tempHolding.month.changed && (tempHolding.month.val > 11)) ||
            (tempHolding.year.changed && ((tempHolding.year.val < 2014) || (tempHolding.year.val > 2037))) ||
            (tempHolding.day.changed && ((tempHolding.month.val > 11) || (tempHolding.day.val < 1) ||
                                         (tempHolding.day.val > (NUM_MONTH_DAYS[tempHolding.month.val] + (((tempHolding.year.val%4 == 0) && (tempHolding.month.val == 1))?1:0))))) ||
            (tempHolding.hour.changed && (tempHolding.hour.val > 23)) ||
            (tempHolding.minute.changed && (tempHolding.minute.val > 59)) ||
            (tempHolding.second.changed && (tempHolding.second.val > 59))) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief Writes the specified holding register
*
* This function writes the holding register number given.
*
* @param port The port this request came over
* @param reg The number of the holding register to write
* @param val The value to write into the register
*
* @return Exception code indicating if the holding register was successfully written.
*/
modbus_exceptions modbus_holding_write_callback(uint8_t port, uint16_t reg, uint16_t val)
{
    pulse_input_channel inChn = PULSE_INPUT_2;
    pulse_output_channel outChn = PULSE_OUTPUT_2;

    //WE only have one port, so if not it we shouldn't be here
    if(port != MB_PORT_NUM) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    switch((board_holding)reg) {
    case HOLDING_MONTH:
        tempHolding.month.val = val-1;
        tempHolding.month.changed = true;
        break;
    case HOLDING_DAY:
        tempHolding.day.val = val;
        tempHolding.day.changed = true;
        break;
    case HOLDING_YEAR:
        tempHolding.year.val = val;
        tempHolding.year.changed = true;
        break;
    case HOLDING_HOUR:
        tempHolding.hour.val = val;
        tempHolding.hour.changed = true;
        break;
    case HOLDING_MINUTE:
        tempHolding.minute.val = val;
        tempHolding.minute.changed = true;
        break;
    case HOLDING_SECOND:
        tempHolding.second.val = val;
        tempHolding.second.changed = true;
        break;
    default:
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    return MODBUS_EX_NONE;
}