/*!
* @file conf_modbus.h
* @author Jarrod Cook
*
* @brief Configuration for the modbus module
*
* This file contains all the configuration settings for the MODBUS
* module to select what features will be included
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

#ifndef CONF_MODBUS_H_
#define CONF_MODBUS_H_
//Includes
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

//Defines
#define NUM_MODBUS_PORTS                            1       //!< Number of MODBUS Ports available for use
#define MODBUS_RTU_ENABLED                          1       //!< If Modbus RTU support is enabled
#define MODBUS_ASCII_ENABLED                        1       //!< If Modbus ASCII support is enabled
#define MODBUS_TCP_ENABLED                          0       //!< If Modbus TCP support is enabled

#if MODBUS_ASCII_ENABLED > 0 && MODBUS_RTU_ENABLED < 1
#error MODBUS ASCII Mode requires RTU mode to also be enabled!
#endif

#define MODBUS_ADDRESS_BROADCAST    0                       //!< Modbus broadcast address.
#define MODBUS_ADDRESS_MIN          1                       //!< Smallest possible slave address.
#define MODBUS_ADDRESS_MAX          247                     //!< Biggest possible slave address.
#define MODBUS_NUM_BAUD_DIGITS      6                       //!< Number of digits in the max baud rate
#define MODBUS_BAUDRATE_MIN         300                     //!< The minimum allowable modbus baud rate
#define MODBUS_BAUDRATE_MAX         256000                  //!< The maximum allowable modbus baud rate

#define MODBUS_ASCII_TIMEOUT_SEC                    1       //!< Character Timeout for MODBUS ASCII, it is not fixed like RTU, so set it to max expected network delay
#define MODBUS_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS    0       //!< ASCII Timeout to wait before sending response packet, used to prevent the slave from responding so fast that the master doesn't have time to swap

#define MODBUS_FUNC_OTHER_REP_SLAVEID_BUF           32      //!< The max size of the additional segments for the report slave id response.
#define MODBUS_FUNC_HANDLERS_MAX                    16      //!< Max number or function supported (must be >= the number of functions listed in this file below)
#define MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED       1       //!< If the Report Slave ID function should be enabled
#define MODBUS_FUNC_READ_INPUT_ENABLED              1       //!< If the <em>Read Input Registers</em> function should be enabled
#define MODBUS_FUNC_READ_HOLDING_ENABLED            1       //!< If the <em>Read Holding Registers</em> function should be enabled
#define MODBUS_FUNC_WRITE_HOLDING_ENABLED           1       //!< If the <em>Write Single Register</em> function should be enabled
#define MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED  1       //!< If the <em>Write Multiple registers</em> function should be enabled
#define MODBUS_FUNC_READ_COILS_ENABLED              1       //!< If the <em>Read Coils</em> function should be enabled
#define MODBUS_FUNC_WRITE_COIL_ENABLED              1       //!< If the <em>Write Coils</em> function should be enabled
#define MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED    1       //!< If the <em>Write Multiple Coils</em> function should be enabled
#define MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED    1       //!< If the <em>Read Discrete Inputs</em> function should be enabled
#define MODBUS_FUNC_READWRITE_HOLDING_ENABLED       1       //!< If the <em>Read/Write Multiple Registers</em> function should be enabled

//Typdefs
/*!
* The following are status values that can be returned from functions
* internally.
*/
typedef enum {
    MODBUS_STAT_SUCCESS = 0,            //!< All is well
    MODBUS_STAT_INVALID_ADDRESS,        //!< The given address was invalid
    MODBUS_STAT_INVALID_VALUE,          //!< A value given was invalid
    MODBUS_STAT_ILLEGAL_STATE,          //!< The system is in an illegal state
    MODBUS_STAT_PORT_ERROR,             //!< A Port error has occurred
    MODBUS_STAT_IO_ERROR,               //!< An IO Error has occurred
    MODBUS_STAT_BUFFER_OVERFLOW,        //!< A buffer overflow occurred
    MODBUS_STAT_NO_DATA                 //!< No data currently available
} modbus_status;

/*!
* This type defines the available modbus parities
*/
typedef enum {
    MODBUS_PARITY_NONE=0,                //!< No parity.
    MODBUS_PARITY_ODD=1,                 //!< Odd parity.
    MODBUS_PARITY_EVEN=2,                //!< Even parity.
    MODBUS_PARITY_LAST_ONE=3             //!< Marker for last parity option
} modbus_parity;


#endif /* CONF_MODBUS_H_ */
