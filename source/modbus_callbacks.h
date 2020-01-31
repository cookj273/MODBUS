/*!
* @file modbus_callbacks.h
* @author Jarrod Cook
*
* @brief Stubs for the function handlers of the MODBUS.
*
* This file contains all the stubs of the different function handlers
* that must be implemented by the user code.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

#ifndef MODBUS_CALLBACKS_H_
#define MODBUS_CALLBACKS_H_

#include "modbus_module.h"

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
void modbus_frame_complete_callback(uint8_t port, modbus_function_codes completedFunc);

#if MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
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
uint8_t modbus_device_info_callback(uint8_t port, uint8_t *devInfo);
#endif

#if (MODBUS_FUNC_WRITE_COIL_ENABLED > 0) || (MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0) || (MODBUS_FUNC_READ_COILS_ENABLED > 0)
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
uint16_t modbus_coil_start_address(uint8_t port);

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
uint16_t modbus_number_of_coils(uint8_t port);
#endif

#if MODBUS_FUNC_READ_COILS_ENABLED > 0
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
uint8_t modbus_coil_read_callback(uint8_t port, uint16_t coil);
#endif

#if (MODBUS_FUNC_WRITE_COIL_ENABLED > 0) || (MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0)
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
modbus_exceptions modbus_coil_write_callback(uint8_t port, uint16_t coil, uint8_t on);
#endif

#if MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
/*!
* @brief Gets the start address of the discrete inputs
*
* This function simply gets the address the discretes for this port start at
*
* @param port The port this request came over
*
* @return The start address of the discretes
*/
uint16_t modbus_discrete_start_address(uint8_t port);

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
uint16_t modbus_number_of_discretes(uint8_t port);

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
uint8_t modbus_discrete_read_callback(uint8_t port, uint16_t discrete);
#endif

#if MODBUS_FUNC_READ_INPUT_ENABLED > 0
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
uint16_t modbus_input_start_address(uint8_t port);

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
uint16_t modbus_number_of_inputs(uint8_t port);

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
uint16_t modbus_input_read_callback(uint8_t port, uint16_t reg);
#endif

#if (MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0) || (MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0) || (MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0) || (MODBUS_FUNC_READ_HOLDING_ENABLED > 0)
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
uint16_t modbus_holding_start_address(uint8_t port);

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
uint16_t modbus_number_of_holding(uint8_t port);
#endif

#if MODBUS_FUNC_READ_HOLDING_ENABLED > 0
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
uint16_t modbus_holding_read_callback(uint8_t port, uint16_t reg, bool isWriteRead);
#endif

#if (MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0) || (MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0) || (MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0)
//Not a huge fan of having to perform 3 steps to write, but what can ya do?

/*!
* @brief Initializes temporary storage for writing registers
*
* Since there are restrictions on many of the registers we allow to be
* written we actually store all writes to a temporary place first, then
* verify the contents before deciding on success and performing the writes.
*
* @param port The port this request came over
*/
void modbus_holding_start_write_callback(uint8_t port);

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
modbus_exceptions modbus_holding_verify_write_callback(uint8_t port);

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
modbus_exceptions modbus_holding_write_callback(uint8_t port, uint16_t reg, uint16_t val);
#endif

#endif /* MODBUS_CALLBACKS_H_ */