/*!
* @file modbus_rtu.h
* @author Jarrod Cook
*
* @brief Implementation of the modbus RTU functionality
*
* This file is the implementation of routines to handle modbus
* RTU communications with external equipment.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/


#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_

#include "conf_modbus.h"
#include "modbus_module.h"

/*!
* @brief Initializes RTU functionality for receiving MODBUS packets
*
* Sets up our state variables and initialize the timers and UART ports
* for communicating MODBUS.
*
* @param mode Whether we are in ASCII for RTU mode
* @param port The port number to used (arbitrary, passed on to the board modbus init function)
* @param baudRate The baud rate to use for communication
* @param parity The parity to use
* @param stopBits The number of stop bits to use
*
* @return Status value indicating the result of the init
*/
modbus_status modbus_rtu_init(modbus_modes mode, uint8_t port, uint32_t baudRate, modbus_parity parity, uint8_t stopBits);

/*!
* @brief Enables/Disables the RTU communications
*
* This will enable and disable RTU communications by starting up and
* shutting down ports and timers and setting settings as necessary.
*
* @param port The port to use for function
* @param enable True implies enable, false implies disable
*/
void modbus_rtu_enable(uint8_t port, bool enable);

/*!
* @brief Closes out the port when shut off
*
* This will close out the port and shut everything off when we decide to
* turn MODBUS off.
*
* @param port The port to use for function
*/
void modbus_rtu_close(uint8_t port);

/*!
* @brief Checks for and retrieves a received frame
*
* This will check to see if a full frame has been successfuly received
* and place it into the given pointers and return the status.
*
* @param port The port to use for function
* @param slaveAddress Pointer to set to the slave address received
* @param frame Pointer to set to the start of the received frame
* @param length Pointer to set to the length of the received frame
*
* @return The status indicating if a frame was received succesfully or not
*/
modbus_status modbus_rtu_receive(uint8_t port, uint8_t *slaveAddress, uint8_t **frame, uint16_t *length);

/*!
* @brief Transmits a frame of data in RTU format
*
* This start the transmission of an RTU frame which will be interrupt
* driven after the start.
*
* @param port The port to use for function
* @param slaveAddress Address to use in frame
* @param frame Pointer to the start of the frame (must be 1 byte available in buffer before start of address!)
* @param length Length of the frame to send
*
* @return The status indicating if transmit was started succesfully or not
*/
modbus_status modbus_rtu_transmit(uint8_t port, uint8_t slaveAddress, const uint8_t *frame, uint16_t length);

/*!
* @brief Lets the RTU system know there is no response to the last receive
*
* This simply flags that there will be no response to the last receive which
* will allow the receive to go back into idle mode and get a new packet.
*
* @param port The port to use for function
*/
void modbus_rtu_no_response(uint8_t port);

/*!
* @brief Gets the TX buffer out of context of a slave response
*
* When in master mode we need to be able to grab the buffer whenever we need
* it, so this function allows that by verifying it is free and putting its
* address in the buf value.
*
* @param port The port to retrieve the buffer for
* @param buf Pointer to the point to put the buffer address in
*
* @return The status indicating if buffer was available or not
*/
bool modbus_rtu_get_tx_buffer(uint8_t port, uint8_t **buf);

/*!
* @brief Releases a buffer from the transmit state
*
* Used when in master mode to release the buffer after a response is
* received and handled.  It does so without flagging the frame complete
* since we do not care in master mode.
*
* @param port The port to free the buffer for
*/
void modbus_rtu_release_tx_buffer(uint8_t port);

/*!
* @brief Gets and clears the transmission complete flag
*
* This allows determination of when a started transmission has actually
* completed.  The flag will be cleared after return assuming and completion
* activities will then be completed.
*
* @param port The port to use for function
*
* @return Boolean being true if the last frame has finished transmitting
*/
bool modbus_rtu_transmit_complete(uint8_t port);

/*!
* @brief Tells whether or not there are frames that need handled
*
* This function will return true if either a full frame has been received
* or a full frame has been transmitted.  It will indicate that the system
* needs serviced to handle said frames before more can be done.
*
* @param port The port to use for function
*
* @return Boolean being true if there is a frame to handle
*/
bool modbus_rtu_has_frames(uint8_t port);


#endif /* MODBUS_RTU_H_ */
