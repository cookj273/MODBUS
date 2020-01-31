/*!
* @file modbus_module.c
* @author Jarrod Cook
*
* @brief Implementation of the modbus module
*
* This file is the implementation of routines to handle modbus
* communications with external equipment.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includes
#include "modbus_module.h"
#include "modbus_rtu.h"
#include "modbus_callbacks.h"

//Defines
#define MODBUS_PDU_FUNC_OFF         0       //!< Offset of function code in PDU.

//Typdefs
/*!
* Defines the potential states of the system
*/
typedef enum {
    MODBUS_STATE_NOT_INITIALIZED = 0,   //!< MODBUS System has not been initialized
    MODBUS_STATE_DISABLED,              //!< MODBUS System has been initialized but is disabled
    MODBUS_STATE_ENABLED,               //!< MODBUS System has been initialized and is enabled
} modbus_states;

/*!
* Defines a structure for holding handling function jump points
*/
typedef struct {
    modbus_function_codes functionCode;
    modbus_exceptions (*handler) (uint8_t port, uint8_t *frame, uint16_t *length);
} modbus_function_handlers;

//Function Prototypes
bool modbus_command_response_handler(uint8_t port, uint8_t slaveAddress, uint16_t address, uint8_t *frame, uint16_t length);
#if MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
modbus_exceptions modbus_report_slave_id_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_READ_INPUT_ENABLED > 0
modbus_exceptions modbus_read_input_register_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_READ_HOLDING_ENABLED > 0
modbus_exceptions modbus_read_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
modbus_exceptions modbus_write_multi_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0
modbus_exceptions modbus_write_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0
modbus_exceptions modbus_readwrite_multi_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_READ_COILS_ENABLED > 0
modbus_exceptions modbus_read_coils_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_WRITE_COIL_ENABLED > 0
modbus_exceptions modbus_write_coil_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
modbus_exceptions modbus_write_multiple_coils_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif
#if MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
modbus_exceptions modbus_read_discrete_inputs_handler(uint8_t port, uint8_t *frame, uint16_t *length);
#endif

//Local Variables
/*!
* The following table simply builds a pointer to all valid function handlers
*/
static modbus_function_handlers funcHandlers[MODBUS_FUNC_HANDLERS_MAX] = {
#if MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MODBUS_FUNC_OTHER_REPORT_SLAVEID, modbus_report_slave_id_handler},
#endif
#if MODBUS_FUNC_READ_INPUT_ENABLED > 0
    {MODBUS_FUNC_READ_INPUT_REGISTER, modbus_read_input_register_handler},
#endif
#if MODBUS_FUNC_READ_HOLDING_ENABLED > 0
    {MODBUS_FUNC_READ_HOLDING_REGISTER, modbus_read_holding_register_handler},
#endif
#if MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS, modbus_write_multi_holding_register_handler},
#endif
#if MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0
    {MODBUS_FUNC_WRITE_REGISTER, modbus_write_holding_register_handler},
#endif
#if MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MODBUS_FUNC_READWRITE_MULTIPLE_REGISTERS, modbus_readwrite_multi_holding_register_handler},
#endif
#if MODBUS_FUNC_READ_COILS_ENABLED > 0
    {MODBUS_FUNC_READ_COILS, modbus_read_coils_handler},
#endif
#if MODBUS_FUNC_WRITE_COIL_ENABLED > 0
    {MODBUS_FUNC_WRITE_SINGLE_COIL, modbus_write_coil_handler},
#endif
#if MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MODBUS_FUNC_WRITE_MULTIPLE_COILS, modbus_write_multiple_coils_handler},
#endif
#if MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MODBUS_FUNC_READ_DISCRETE_INPUTS, modbus_read_discrete_inputs_handler},
#endif
};

/*!
* The following structure contains info for the currently running modbus instance
*/
static struct {
    uint8_t unitAddress;                                                                                        //!< The Address of this unit if a slave
    modbus_modes mode;                                                                                          //!< The Mode of communication (RTU, ASCII)
    modbus_device_types deviceType;                                                                             //!< Device Type (MASTER or SLAVE)
    modbus_states state;                                                                                        //!< The current state of the instance
    modbus_function_codes lastFunctionCode;                                                                     //!< The last function code preformed
    uint8_t lastSlaveAddress;                                                                                   //!< Tracks the slave address of the last command frame
    uint16_t lastCmdRegAddress;                                                                                 //!< The register address the last called command affected
    void (*commsEnable)(uint8_t port, bool enable);                                                             //!< This function will be bound to the mode function for enabling/disabling comms
    modbus_status (*frameTransmit)(uint8_t port, uint8_t slaveAddress, const uint8_t *frame, uint16_t length);  //!< This function will be bound to the mode function for sending a frame
    modbus_status (*frameReceive)(uint8_t port, uint8_t *rcvAddress, uint8_t **frame, uint16_t *length);        //!< This function will be bound to the mode function for getting a received frame
    void (*frameReleaseBuffer)(uint8_t port);                                                                   //!< Releases a grabbed buffer whether that be manually in MASTER mode or by virtue of a received packet completion
    bool (*frameComplete)(uint8_t port);                                                                        //!< This function will be bound to the mode function for determining when a response is complete
    bool (*hasFrames)(uint8_t port);                                                                            //!< This function will be bound to the mode function for determining if there are frames to handle
    void (*commsClose)(uint8_t port);                                                                           //!< This function will be bound to the mode function for closing out a port when done
    bool (*getTxBuffer)(uint8_t port, uint8_t **buf);                                                           //!< Retrieves and locks a buffer for transmitting outside the context of a response (MASTER MODE)
    bool (*masterGetDataCallback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint8_t *data, uint8_t length);                         //!< Callback function to handle simple data getting command responses
    bool (*masterMultiRegReadCallback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty);   //!< Callback to handle multiple register reading command responses
    bool (*masterWriteCallback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t qty);                          //!< Callback to handle single/multiple registers write and multiple coil writes
    bool (*masterWriteSingleCoilCallback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, bool val);                    //!< Callback to handle single coil writes
    bool (*masterReadCoilsCallback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint8_t *vals, uint16_t qty);       //!< Callback to handler coil/discrete reads
} modbusInfo[NUM_MODBUS_PORTS];

/*!
* @brief Initializes modbus functionality so that we are ready for communication
*
* This function will setup modbus communication so that the device is ready
* to perform communication according to user settings.
*
* @param port The port number to used (arbitrary, passed on to the board modbus init function)
* @param mode The desired mode of communication (RTU, ASCII)
* @param deviceType The type of device this is master or slave
* @param slaveAddress The address to respond to
* @param baudRate The baud rate to use for communication
* @param parity The parity to use when using UART
* @param stopBits The number of stop bits to use when using UART
*
* @return Status value indicating the result of the init
*/
modbus_status modbus_module_init(uint8_t port, modbus_modes mode, modbus_device_types deviceType, uint8_t slaveAddress, uint32_t baudRate, modbus_parity parity, uint8_t stopBits)
{
    modbus_status status;

    // check preconditions
    if((deviceType == MODBUS_MASTER_DEVICE) || ((deviceType == MODBUS_SLAVE_DEVICE) && (slaveAddress != MODBUS_ADDRESS_BROADCAST)
            && (slaveAddress >= MODBUS_ADDRESS_MIN) && (slaveAddress <= MODBUS_ADDRESS_MAX))) {
        modbusInfo[port].unitAddress = slaveAddress;
        modbusInfo[port].deviceType = deviceType;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_NONE;

        //Setup the correct mode
        switch (mode) {
        case MODBUS_RTU:
            modbusInfo[port].commsEnable = modbus_rtu_enable;
            modbusInfo[port].frameTransmit = modbus_rtu_transmit;
            modbusInfo[port].frameReceive = modbus_rtu_receive;
            modbusInfo[port].frameComplete = modbus_rtu_transmit_complete;
            modbusInfo[port].hasFrames = modbus_rtu_has_frames;
            modbusInfo[port].commsClose = modbus_rtu_close;
            if(modbusInfo[port].deviceType == MODBUS_SLAVE_DEVICE) {
                modbusInfo[port].frameReleaseBuffer = modbus_rtu_no_response;
                modbusInfo[port].getTxBuffer = NULL;
            } else {
                modbusInfo[port].frameReleaseBuffer = modbus_rtu_release_tx_buffer;
                modbusInfo[port].getTxBuffer = modbus_rtu_get_tx_buffer;
            }

            status = modbus_rtu_init(port, baudRate, parity, stopBits);
            break;
        //TODO: IMPLEMENT ASCII DRIVERS!!!!
        /*case MB_ASCII:
            modbusInfo.frameStart = eMBASCIIStart;
            modbusInfo.frameStop = eMBASCIIStop;
            modbusInfo.frameSend = eMBASCIISend;
            modbusInfo.frameReceive = eMBASCIIReceive;
            modbusInfo.frameClose = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            modbusInfo.frameByteReceived = xMBASCIIReceiveFSM;
            modbusInfo.frameTransmitterEmpty = xMBASCIITransmitFSM;
            modbusInfo.timerExpired = xMBASCIITimerT1SExpired;
            modbusInfo.isStateIdle = xMBASCIIStateIdle;

            status = eMBASCIIInit(ucPort, ulBaudRate, uParity, uStopBits );
            break;*/
        default:
            status = MODBUS_STAT_INVALID_VALUE;
            break;
        }

        //Setup our mode
        if(status == MODBUS_STAT_SUCCESS) {
            modbusInfo[port].mode = mode;
            modbusInfo[port].state = MODBUS_STATE_DISABLED;
        }
    } else {
        status = MODBUS_STAT_INVALID_ADDRESS;
    }

    return status;
}

/*!
* @brief Changes the slave address of the unit's port
*
* This function should be used to change the slave address of
* the unit on the fly.
*
* @param port The port to change the address for
* @param slaveAddress The new address to give the port
*
* @return Boolean indicating if address was valid and changed or not
*/
bool modbus_modbule_change_address(uint8_t port, uint8_t slaveAddress)
{
    if((slaveAddress == MODBUS_ADDRESS_BROADCAST) || (slaveAddress < MODBUS_ADDRESS_MIN) || (slaveAddress > MODBUS_ADDRESS_MAX)) {
        return false;
    }

    modbusInfo[port].unitAddress = slaveAddress;
    return true;
}

/*!
* @brief Enables/Disabled the modbus module
*
* This function will both enable and disabled the MODBUS module.
* disabling will prevent any communications and shutdown to save power.
*
* @param port The port to enable
* @param enable Indicate the desired enable status
*
* @return Status value indicating the result of the enable
*/
modbus_status modbus_module_enable(uint8_t port, bool enable)
{
    if(enable && (modbusInfo[port].state == MODBUS_STATE_DISABLED) && modbusInfo[port].commsEnable) {
        // Activate the protocol stack.
        modbusInfo[port].commsEnable(port, true);
        modbusInfo[port].state = MODBUS_STATE_ENABLED;
    } else if(!enable && (modbusInfo[port].state == MODBUS_STATE_ENABLED) && modbusInfo[port].commsEnable) {
        // Deactivate the protocol stack.
        modbusInfo[port].commsEnable(port, false);
        modbusInfo[port].state = MODBUS_STATE_DISABLED;
    } else if((modbusInfo[port].state != MODBUS_STATE_ENABLED) && (modbusInfo[port].state != MODBUS_STATE_ENABLED)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    return MODBUS_STAT_SUCCESS;
}

/*!
* @brief Closes down the MODBUS module if we choose to stop
*
* This function will both completely close out the MODBUS driver, saving
* power by shutting stuff off.
*
* @param port the port to close out
*/
void modbus_module_close(uint8_t port)
{
    //First Disable everything
    modbus_module_enable(port, false);

    //Now close out the port
    if(modbusInfo[port].commsClose) {
        modbusInfo[port].commsClose(port);
    }

    //We need a reinit to come back from this
    modbusInfo[port].state = MODBUS_STATE_NOT_INITIALIZED;
}

/*!
* @brief Simply indicates whether a frame was received or transmitted
*
* This function will check to see if there a send or received frame
* that still needs handling.  It can be used to determine if the polling
* function needs called or not.
*
* @return True if there are frames that need handled
*/
bool modbus_module_needs_serviced(void)
{
    uint8_t idx;
    for(idx = 0; idx < NUM_MODBUS_PORTS; idx++) {
        if((modbusInfo[idx].state == MODBUS_STATE_ENABLED) && modbusInfo[idx].hasFrames && modbusInfo[idx].hasFrames(idx)) {
            return true;
        }
    }

    return false;
}

/*!
* @brief Indicates whether or not MODBUS is currently running
*
* This function simply inform the user whether or not MODBUS
* is currently running, so they can save power if not.
*
* @return True if modbus is currently enabled and running
*/
bool modbus_module_is_running(void)
{
    uint8_t idx;
    for(idx = 0; idx < NUM_MODBUS_PORTS; idx++) {
        if(modbusInfo[idx].state == MODBUS_STATE_ENABLED) {
            return true;
        }
    }

    return false;
}

/*!
* @brief Function that services the MODBUS module and handles frames
*
* This function should be called routinely or when there is a frame to
* handle.  It will process any received and transmitted frames.
*/
void modbus_module_service(void)
{
    uint8_t *packFrame;
    uint16_t packLength;
    uint8_t rcvAddress, idx;
    modbus_exceptions err;
    int i;

    for(idx = 0; idx < NUM_MODBUS_PORTS; idx++) {
        // Check if the protocol stack is ready.
        if((modbusInfo[idx].state == MODBUS_STATE_ENABLED) && modbusInfo[idx].frameReceive) {
            //HANDLE DIFFERENTLY BASED ON TYPE (MASTER OF SLAVE)!!!!
            if(modbusInfo[idx].deviceType == MODBUS_MASTER_DEVICE) {
                //********MASTER SERVICING********//
                //Receive a frame if there is one available and handle it
                if(modbusInfo[idx].frameReceive(idx, &rcvAddress, &packFrame, &packLength) == MODBUS_STAT_SUCCESS) {
                    //If we have a callback registered and the response matches then call the callback
                    if(modbusInfo[idx].lastSlaveAddress == rcvAddress &&
                            modbusInfo[idx].lastFunctionCode == (packFrame[MODBUS_PDU_FUNC_OFF] & 0x7F)) {
                        //Call our callback, true will indicate stop waiting for responses on this callback
                        if(modbus_command_response_handler(idx, rcvAddress, modbusInfo[idx].lastCmdRegAddress, packFrame, packLength)) {
                            modbusInfo[idx].lastFunctionCode = MODBUS_FUNC_NONE;
                            modbusInfo[idx].lastCmdRegAddress = 0;
                            modbusInfo[idx].lastSlaveAddress = 0;
                        }
                    }

                    //Release the buffer
                    if(modbusInfo[idx].frameReleaseBuffer) {
                        modbusInfo[idx].frameReleaseBuffer(idx);
                    }
                }

                //Check for complete transmissions
                if(modbusInfo[idx].frameComplete) {
                    modbusInfo[idx].frameComplete(idx);
                }
            } else {
                //********SLAVE SERVICING********//
                //See if our last response if finished and fire the callback
                if(modbusInfo[idx].frameComplete && modbusInfo[idx].frameComplete(idx)) {
                    modbus_frame_complete_callback(idx, modbusInfo[idx].lastFunctionCode);
                }

                //Receive a frame if there is one available and handle it
                if(modbusInfo[idx].frameReceive(idx, &rcvAddress, &packFrame, &packLength) == MODBUS_STAT_SUCCESS) {
                    // Check if the frame is for us. If not ignore the frame.
                    if((rcvAddress == modbusInfo[idx].unitAddress) || (rcvAddress == MODBUS_ADDRESS_BROADCAST)) {
                        modbusInfo[idx].lastFunctionCode = packFrame[MODBUS_PDU_FUNC_OFF];
                        err = MODBUS_EX_ILLEGAL_FUNCTION;
                        for(i = 0; i < MODBUS_FUNC_HANDLERS_MAX; i++) {
                            // No more function handlers registered. Abort.
                            if(funcHandlers[i].functionCode == 0) {
                                break;
                            } else if(funcHandlers[i].functionCode == modbusInfo[idx].lastFunctionCode) {
                                err = funcHandlers[i].handler(idx, packFrame, &packLength);
                                break;
                            }
                        }

                        if(err != MODBUS_EX_NONE) {
                            // An exception occured, so set the error bit in the function code.
                            modbusInfo[idx].lastFunctionCode |= MODBUS_FUNC_ERROR;
                        }

                        // If the request was not sent to the broadcast address we return a reply.
                        if((rcvAddress != MODBUS_ADDRESS_BROADCAST) && modbusInfo[idx].frameTransmit) {
                            if(err != MODBUS_EX_NONE) {
                                // An exception occured. Build an error frame.
                                packLength = 0;
                                packFrame[packLength++] = (uint8_t)modbusInfo[idx].lastFunctionCode;
                                packFrame[packLength++] = err;
                            }

                            //TODO: MOVE THIS DELAY INTO THE ASCII FILE!!!!!
                            //if((modbusInfo[idx].mode == MODBUS_ASCII ) && MODBUS_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS) {
                            //  delay_ms(MODBUS_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS);
                            //}

                            //Send the frame
                            modbusInfo[idx].frameTransmit(idx, modbusInfo[idx].unitAddress, packFrame, packLength);
                        } else if(modbusInfo[idx].frameReleaseBuffer) {
                            //No response so flag as so
                            modbusInfo[idx].frameReleaseBuffer(idx);
                        }
                    }
                }
            }
        }
    }
}


//****************************************************************************FUNCTION HANDLERS**************************************************************//


#if MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
/*!
* @brief This function is the handler for reading the slave id
*
* This function returns the slave id and additional information into
* a modbus frame.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_report_slave_id_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint8_t infoLength;

    if(*length == MODBUS_PDU_SIZE_MIN) {
        *length = MODBUS_PDU_FUNC_OFF;
        frame[*length] = MODBUS_FUNC_OTHER_REPORT_SLAVEID;
        *length += 1;

        infoLength = modbus_device_info_callback(port, &frame[(*length+1)]);

        if(infoLength > 0) {
            frame[*length] = infoLength;
            *length += infoLength + 1;
        } else {
            return MODBUS_EX_ILLEGAL_DATA_VALUE;
        }

    } else {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to get its slave id
*
* This function causes a command to be sent out of the port that will
* request a slave to send its slave id back.  The callback given will
* be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_report_slave_id_command(uint8_t port, uint8_t slaveAddress, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint8_t *data, uint8_t length))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_OTHER_REPORT_SLAVEID;

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 1);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = 0;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_OTHER_REPORT_SLAVEID;
        modbusInfo[port].masterGetDataCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_READ_INPUT_ENABLED > 0
/*!
* @brief This function is the handler for reading an input register
*
* This function must simply return the value of the addressed 16-bit
* input register.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_read_input_register_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress, regCount, tempReg;
    uint8_t  *frameCur;

    if(*length == (MODBUS_PDU_FUNC_READ_SIZE + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        regCount = (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF] << 8);
        regCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1]);

        // Check if the number of registers to read is valid. If not return Modbus illegal data value exception.
        if((regCount > 0) && (regCount < MODBUS_PDU_FUNC_READ_REGCNT_MAX)
                && (regAddress >= modbus_input_start_address(port)) && (regAddress + regCount <= (modbus_input_start_address(port) + modbus_number_of_inputs(port)))) {
            // Set the current PDU data pointer to the beginning.
            frameCur = &frame[MODBUS_PDU_FUNC_OFF];
            *length = MODBUS_PDU_FUNC_OFF;

            // First byte contains the function code.
            *frameCur++ = MODBUS_FUNC_READ_INPUT_REGISTER;

            //But in the byte length based on number of regs
            *frameCur++ = (regCount * 2);

            //Increment length for 2 header bytes and number of reg bytes
            *length += (2 + *(frameCur-1));

            //Get to the address in memory of the first input
            regAddress = regAddress-modbus_input_start_address(port);

            //Read all registers
            while((regCount > 0) && (regAddress < modbus_number_of_inputs(port))) {
                tempReg = modbus_input_read_callback(port, regAddress++);
                *frameCur++ = (uint8_t)((tempReg>>8) & 0x00FF);
                *frameCur++ = (uint8_t)(tempReg & 0x00FF);
                regCount--;
            }

            //Shouldn't get here, but just in case
            if(regCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }
        } else {
            // Out of range coils requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid read coil register request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to get input registers
*
* This function causes a command to be sent out of the port that will
* request a slave to send the value of input registers.  The callback given will
* be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param readAddress The address to start reading input registers at
* @param readQty The quantity of input registers to read
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_read_input_register_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if(readQty > MODBUS_PDU_FUNC_READ_REGCNT_MAX) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_READ_INPUT_REGISTER;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(readAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)readAddress;
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF] = (uint8_t)(readQty>>8);
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1] = (uint8_t)(readQty);


    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 5);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = readAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_READ_INPUT_REGISTER;
        modbusInfo[port].masterMultiRegReadCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_READ_HOLDING_ENABLED > 0
/*!
* @brief This function is the handler for reading a holding register
*
* This function must return the value of the addressed 16-bit holding
* register.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_read_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress, regCount, tempReg;
    uint8_t  *frameCur;

    if(*length == (MODBUS_PDU_FUNC_READ_SIZE + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        regCount = (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF] << 8);
        regCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1]);

        // Check if the number of registers to read is valid. If not return Modbus illegal data value exception.
        if((regCount > 0) && (regCount < MODBUS_PDU_FUNC_READ_REGCNT_MAX)
                && (regAddress >= modbus_holding_start_address(port)) && (regAddress + regCount <= (modbus_holding_start_address(port) + modbus_number_of_holding(port)))) {
            // Set the current PDU data pointer to the beginning.
            frameCur = &frame[MODBUS_PDU_FUNC_OFF];
            *length = MODBUS_PDU_FUNC_OFF;

            // First byte contains the function code.
            *frameCur++ = MODBUS_FUNC_READ_HOLDING_REGISTER;

            //But in the byte length based on number of regs
            *frameCur++ = (regCount * 2);

            //Increment length for 2 header bytes and number of reg bytes
            *length += (2 + *(frameCur-1));

            //Get to the address in memory of the first holding register
            regAddress = regAddress-modbus_holding_start_address(port);

            //Read all registers
            while((regCount > 0) && (regAddress < modbus_number_of_holding(port))) {
                tempReg = modbus_holding_read_callback(port, regAddress++, false);
                *frameCur++ = (uint8_t)((tempReg>>8) & 0x00FF);
                *frameCur++ = (uint8_t)(tempReg & 0x00FF);
                regCount--;
            }

            //Shouldn't get here, but just in case
            if(regCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }
        } else {
            // Out of range coils requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid read coil register request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to get holding registers
*
* This function causes a command to be sent out of the port that will
* request a slave to send the value of holding registers.  The callback given will
* be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param readAddress The address to start reading holding registers at
* @param readQty The quantity of holding registers to read
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_read_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if(readQty > MODBUS_PDU_FUNC_READ_REGCNT_MAX) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_READ_HOLDING_REGISTER;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(readAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)readAddress;
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF] = (uint8_t)(readQty>>8);
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1] = (uint8_t)(readQty);

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 5);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = readAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_READ_HOLDING_REGISTER;
        modbusInfo[port].masterMultiRegReadCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
/*!
* @brief This function is the handler for writing multiple holding registers
*
* This function should write multiple 16-bit holding registers starting at
* the address given with the data given.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_write_multi_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress, regCount, tempReg;
    uint8_t regByteCount;
    uint8_t *frameCur;
    modbus_exceptions ex;

    if( *length >= (MODBUS_PDU_FUNC_WRITE_MUL_SIZE_MIN + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        regCount = (uint16_t)(frame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF] << 8);
        regCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF + 1]);

        regByteCount = frame[MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        // Check if the number of registers to read is valid. If not return Modbus illegal data value exception.
        if((regCount > 0) && (regCount < MODBUS_PDU_FUNC_WRITE_MUL_REGCNT_MAX) && (regByteCount == (uint8_t)(2 * regCount))
                && (regAddress >= modbus_holding_start_address(port)) && (regAddress + regCount <= (modbus_holding_start_address(port) + modbus_number_of_holding(port)))) {
            //Call our start callback in case we have structs to init
            modbus_holding_start_write_callback(port);

            //Get to the address in memory of the first holding register
            regAddress = regAddress-modbus_holding_start_address(port);
            frameCur = &frame[MODBUS_PDU_FUNC_WRITE_MUL_VALUES_OFF];

            //Read all registers
            while((regCount > 0) && (regAddress < modbus_number_of_holding(port))) {
                tempReg = (((uint16_t)*frameCur<<8) & 0xFF00) | ((uint16_t)*(frameCur+1) & 0x00FF);
                if((ex = modbus_holding_write_callback(port, regAddress++,tempReg))) {
                    return ex;
                }
                frameCur += 2;
                regCount--;
            }

            //Shouldn't get here, but just in case
            if(regCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }

            // The response contains the function code, the starting address and the quantity of registers.
            // We reuse the old values in the buffer because they are still valid. /
            *length = MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;

            //Verify the write that just occurred
            return modbus_holding_verify_write_callback(port);
        } else {
            // Out of range requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to write holding registers
*
* This function causes a command to be sent out of the port that will
* request a slave to write the value of holding registers.  The callback given will
* be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param writeAddress The address to start writing holding registers at
* @param writeQty The quantity of holding registers to write
* @param vals Pointer to a buffer of 16 bit values to write to the registers
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_write_multi_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, uint16_t writeQty, uint16_t *vals, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t qty))
{
    modbus_status stat;
    uint8_t *packFrame;
    uint8_t len, idx;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if(writeQty > MODBUS_PDU_FUNC_WRITE_MUL_REGCNT_MAX) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(writeAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)writeAddress;
    packFrame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF] = (uint8_t)(writeQty>>8);
    packFrame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF + 1] = (uint8_t)(writeQty);
    packFrame[MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF] = (uint8_t)(writeQty<<1);

    //Put the values in
    len = MODBUS_PDU_FUNC_WRITE_MUL_VALUES_OFF;
    for(idx = 0; idx < writeQty; idx++) {
        packFrame[len++] = (uint8_t)((*vals)>>8);
        packFrame[len++] = (uint8_t)(*vals++);
    }

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, len);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = writeAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
        modbusInfo[port].masterWriteCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0
/*!
* @brief This function is the handler for writing a holding register
*
* This function should write a single 16-bit holding register as the
* address given with the data given.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_write_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    modbus_exceptions ex;
    uint16_t regAddress, tempReg;

    if( *length == (MODBUS_PDU_FUNC_WRITE_SIZE + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        // Check if the number of registers to read is valid. If not return Modbus illegal data value exception.
        if((regAddress >= modbus_holding_start_address(port)) && (regAddress < (modbus_holding_start_address(port) + modbus_number_of_holding(port)))) {

            //Call our start callback in case we have structs to init
            modbus_holding_start_write_callback(port);

            //Get to the address in memory of the first holding register
            regAddress = regAddress-modbus_holding_start_address(port);

            //Write the register
            tempReg = (((uint16_t)frame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF]<<8) & 0xFF00) | ((uint16_t)frame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF+1] & 0x00FF);
            if((ex = modbus_holding_write_callback(port, regAddress,tempReg))) {
                return ex;
            }

            //Verify the write that just occurred
            return modbus_holding_verify_write_callback(port);
        } else {
            // Out of range requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to write 1 holding register
*
* This function causes a command to be sent out of the port that will
* request a slave to write the value of a single holding register.  The
* callback given will be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param writeAddress The address of the holding register to write
* @param val The value to write to the holding register
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_write_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, uint16_t val, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t val))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_WRITE_REGISTER;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(writeAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)writeAddress;
    packFrame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF] = (uint8_t)(val>>8);
    packFrame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF + 1] = (uint8_t)val;

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 5);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = writeAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_WRITE_REGISTER;
        modbusInfo[port].masterWriteCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0
/*!
* @brief This function is the handler for reading and writing multiple holding registers
*
* This function should first perform a write to multiple 16-bit holding
* registers starting at the given address with the given data.  It should then
* read the registers back as the response.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_readwrite_multi_holding_register_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t readRegAddress, readRegCount, writeRegAddress, writeRegCount, tempReg;
    uint8_t writeRegByteCount;
    uint8_t *readCur, *writeCur;
    modbus_exceptions ex;

    if( *length >= (MODBUS_PDU_FUNC_READWRITE_SIZE_MIN + MODBUS_PDU_SIZE_MIN)) {
        readRegAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_READ_ADDR_OFF] << 8);
        readRegAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1]);

        readRegCount = (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_OFF] << 8);
        readRegCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1]);

        writeRegAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] << 8);
        writeRegAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1]);

        writeRegCount = (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] << 8);
        writeRegCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1]);

        writeRegByteCount = frame[MODBUS_PDU_FUNC_READWRITE_BYTECNT_OFF];

        // Check if all the addresses and register counts are valid
        if((readRegCount > 0) && (readRegCount < MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_MAX)
                && (writeRegCount > 0) && (writeRegCount < MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_MAX) && (writeRegByteCount == (uint8_t)(2 * writeRegCount))
                && (readRegAddress >= modbus_holding_start_address(port)) && (readRegAddress + readRegCount <= (modbus_holding_start_address(port) + modbus_number_of_holding(port)))
                && (writeRegAddress >= modbus_holding_start_address(port)) && (writeRegAddress + writeRegCount <= (modbus_holding_start_address(port) + modbus_number_of_holding(port)))) {

            //WRITE
            //Call our start callback in case we have structs to init
            modbus_holding_start_write_callback(port);

            // Set the current PDU data pointer to the beginning.
            writeCur = &frame[MODBUS_PDU_FUNC_READWRITE_WRITE_VALUES_OFF];

            //Get to the address in memory of the first holding register
            writeRegAddress = writeRegAddress-modbus_holding_start_address(port);

            //Write all registers
            while((writeRegCount > 0) && (writeRegAddress < modbus_number_of_holding(port))) {
                tempReg = (((uint16_t)*writeCur<<8) & 0xFF00) | ((uint16_t)*(writeCur+1) & 0x00FF);
                if((ex = modbus_holding_write_callback(port, writeRegAddress++, tempReg))) {
                    return ex;
                }
                writeCur += 2;
                writeRegCount--;
            }

            //Shouldn't get here, but just in case
            if(writeRegCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }

            //Verify the write that just occurred
            if((ex = modbus_holding_verify_write_callback(port))) {
                return ex;
            }

            //READ
            // Set the current PDU data pointer to the beginning.
            readCur = &frame[MODBUS_PDU_FUNC_OFF];
            *length = MODBUS_PDU_FUNC_OFF;

            // First byte contains the function code.
            *readCur++ = MODBUS_FUNC_READWRITE_MULTIPLE_REGISTERS;

            //But in the byte length based on number of regs
            *readCur++ = (readRegCount * 2);

            //Increment length for 2 header bytes and number of reg bytes
            *length += (2 + *(readCur-1));

            //Get to the address in memory of the first holding register
            readRegAddress = readRegAddress-modbus_holding_start_address(port);

            //Read all registers
            while((readRegCount > 0) && (readRegAddress < modbus_number_of_holding(port))) {
                tempReg = modbus_holding_read_callback(port, readRegAddress++, true);
                *readCur++ = (uint8_t)((tempReg>>8) & 0x00FF);
                *readCur++ = (uint8_t)(tempReg & 0x00FF);
                readRegCount--;
            }

            //Shouldn't get here, but just in case
            if(readRegCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }
        } else {
            // Out of range requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to read and write holding registers
*
* This function causes a command to be sent out of the port that will
* request a slave to read and write the value of holding registers.  The
* callback given will be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param readAddress The address to start reading holding registers at
* @param readQty The quantity of holding registers to read
* @param writeAddress The address to start writing holding registers at
* @param writeQty The quantity of holding registers to write
* @param vals Pointer to a buffer of 16 bit values to write to the registers
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_readwrite_multi_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, uint16_t writeAddress, uint16_t writeQty, uint16_t *vals, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty))
{
    modbus_status stat;
    uint8_t *packFrame;
    uint8_t len, idx;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if((writeQty > MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_MAX) || (readQty > MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_MAX)) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_READWRITE_MULTIPLE_REGISTERS;
    packFrame[MODBUS_PDU_FUNC_READWRITE_READ_ADDR_OFF] = (uint8_t)(readAddress>>8);
    packFrame[MODBUS_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1] = (uint8_t)readAddress;
    packFrame[MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_OFF] = (uint8_t)(readQty>>8);
    packFrame[MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1] = (uint8_t)readQty;
    packFrame[MODBUS_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] = (uint8_t)(writeAddress>>8);
    packFrame[MODBUS_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1] = (uint8_t)writeAddress;
    packFrame[MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] = (uint8_t)(writeQty>>8);
    packFrame[MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1] = (uint8_t)writeQty;
    packFrame[MODBUS_PDU_FUNC_READWRITE_BYTECNT_OFF] = (uint8_t)(writeQty<<1);

    //Put the values in
    len = MODBUS_PDU_FUNC_READWRITE_WRITE_VALUES_OFF;
    for(idx = 0; idx < writeQty; idx++) {
        packFrame[len++] = (uint8_t)((*vals)>>8);
        packFrame[len++] = (uint8_t)(*vals++);
    }

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, len);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = readAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_READWRITE_MULTIPLE_REGISTERS;
        modbusInfo[port].masterMultiRegReadCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_READ_COILS_ENABLED > 0
/*!
* @brief This function is the handler for reading coils
*
* This function should read multiple 1-bit coils starting at the given
* address.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_read_coils_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress, coilCount;
    uint8_t curBit;
    uint8_t  *frameCur;

    if(*length == (MODBUS_PDU_FUNC_READ_SIZE + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        coilCount = (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF] << 8);
        coilCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1]);

        // Check if the number of registers to read is valid. If not return Modbus illegal data value exception.
        if((coilCount > 0) && (coilCount < MODBUS_PDU_FUNC_READ_COILCNT_MAX)
                && (regAddress >= modbus_coil_start_address(port)) && (regAddress + coilCount <= (modbus_coil_start_address(port) + modbus_number_of_coils(port)))) {
            // Set the current PDU data pointer to the beginning.
            frameCur = &frame[MODBUS_PDU_FUNC_OFF];
            *length = MODBUS_PDU_FUNC_OFF;

            // First byte contains the function code.
            *frameCur++ = MODBUS_FUNC_READ_COILS;

            //But in the byte length based on number of coils
            *frameCur++ = (coilCount + 7)/8;

            //Increment length for 2 header bytes and number of coil bytes
            *length += (2 + *(frameCur-1));

            //Get to the address in memory of the first coil and set our current output as 0
            regAddress = regAddress-modbus_coil_start_address(port);
            *frameCur = 0x00;

            //Read all coils
            curBit = 0;
            while((coilCount > 0) && (regAddress < modbus_number_of_coils(port))) {
                //Goto next byte if we filled one
                if(curBit > 7) {
                    curBit = 0;
                    frameCur++;
                    *frameCur = 0x00;
                }

                //Put this coil in
                *frameCur |= ((modbus_coil_read_callback(port, regAddress++))?(0x01<<curBit):0);
                coilCount--;
                curBit++;
            }

            //Shouldn't get here, but just in case
            if(coilCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }
        } else {
            // Out of range coils requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid read coil register request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to read coils
*
* This function causes a command to be sent out of the port that will
* request a slave to read the value of coils.  The callback given will be
* used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param readAddress The address to start reading coils at
* @param readQty The quantity of coils to read
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_read_coils_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint8_t *vals, uint16_t byteCnt))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if(readQty > MODBUS_PDU_FUNC_READ_COILCNT_MAX) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_READ_COILS;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(readAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)readAddress;
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF] = (uint8_t)(readQty>>8);
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1] = (uint8_t)readQty;

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 5);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = readAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_READ_COILS;
        modbusInfo[port].masterReadCoilsCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_WRITE_COIL_ENABLED > 0
/*!
* @brief This function is the handler for writing a single coil
*
* This function should write a single 1-bit coil at the address given with
* the value given.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_write_coil_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress;

    if(*length == (MODBUS_PDU_FUNC_WRITE_SIZE + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        if((frame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00 ) && ((frame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF ) || (frame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF] == 0x00))
                && (regAddress >= modbus_coil_start_address(port)) && (regAddress < (modbus_coil_start_address(port) + modbus_number_of_coils(port)))) {
            //Get to the address in memory of the first coil
            regAddress = regAddress-modbus_coil_start_address(port);

            return modbus_coil_write_callback(port, regAddress, frame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF]);
        } else {
            // Out of range coil requested or bad data value
            return MODBUS_EX_ILLEGAL_DATA_VALUE;
        }
    } else {
        // Can't be a valid write coil register request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to write a single coil
*
* This function causes a command to be sent out of the port that will
* request a slave to write the value of a single coil.  The callback given
* will be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param writeAddress The address of the coil to write
* @param val The value to write to the coil (on/off)
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_write_coil_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, bool val, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, bool val))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_WRITE_SINGLE_COIL;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(writeAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)writeAddress;
    packFrame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF] = (val)?0xFF:0x00;
    packFrame[MODBUS_PDU_FUNC_WRITE_VALUE_OFF + 1] = 0x00;

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 5);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = writeAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_WRITE_SINGLE_COIL;
        modbusInfo[port].masterWriteSingleCoilCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
/*!
* @brief This function is the handler for writing multiple coils
*
* This function should write multiple 1-bit coils starting at the address
* given with the values given.  See spec for response structure.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_write_multiple_coils_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress, coilCount;
    uint8_t curBit, regByteCount;
    uint8_t *frameCur;
    modbus_exceptions ex;

    if(*length > (MODBUS_PDU_FUNC_WRITE_SIZE + MODBUS_PDU_SIZE_MIN )) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        coilCount = (uint16_t)(frame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF] << 8);
        coilCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF + 1]);

        regByteCount = frame[MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        if((coilCount > 0) && (coilCount <= MODBUS_PDU_FUNC_WRITE_MUL_COILCNT_MAX) && (((coilCount+7)/8) == regByteCount)
                && (regAddress >= modbus_coil_start_address(port)) && (regAddress < (modbus_coil_start_address(port) + modbus_number_of_coils(port)))) {
            //Get to the address in memory of the first coil
            regAddress = regAddress-modbus_coil_start_address(port);
            frameCur = &frame[MODBUS_PDU_FUNC_WRITE_MUL_VALUES_OFF];

            //Write all the coils
            curBit = 0;
            while((coilCount > 0) && (regAddress < modbus_number_of_coils(port))) {
                //Goto next byte if we filled one
                if(curBit > 7) {
                    curBit = 0;
                    frameCur++;
                }

                //Write the coil
                if((ex = modbus_coil_write_callback(port, regAddress++, ((*frameCur & (0x01<<curBit))?0xFF:0x00)))) {
                    return ex;
                }

                //Next
                coilCount--;
                curBit++;
            }

            //Shouldn't get here, but just in case
            if(coilCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }

            // The response contains the function code, the starting address and the quantity of registers.
            // We reuse the old values in the buffer because they are still valid.
            *length = MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
        } else {
            // Can't be a valid write coil register request because the length is incorrect.
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid write coil register request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to write coils
*
* This function causes a command to be sent out of the port that will
* request a slave to write the value of coils.  The callback given will be
* used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param writeAddress The address to start writing coils at
* @param writeQty The quantity of coils to write
* @param vals Pointer to the values to write to the coils (uint8_t packed, bit 0 of byte 1 = writeAddress, bit 1 = writeAddress+1, etc., on/off)
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_write_multiple_coils_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, uint16_t writeQty, uint8_t *vals, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t qty))
{
    modbus_status stat;
    uint8_t *packFrame;
    uint8_t len, idx;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if(writeQty > MODBUS_PDU_FUNC_WRITE_MUL_COILCNT_MAX) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_WRITE_MULTIPLE_COILS;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(writeAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)writeAddress;
    packFrame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF] = (uint8_t)(writeQty>>8);
    packFrame[MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF + 1] = (uint8_t)(writeQty);
    packFrame[MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF] = (uint8_t)((writeQty+7)/8);

    //Put the values in
    len = MODBUS_PDU_FUNC_WRITE_MUL_VALUES_OFF;
    for(idx = 0; idx < ((writeQty+7)/8); idx++) {
        packFrame[len++] = vals[idx];
    }

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, len);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = writeAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_WRITE_MULTIPLE_COILS;
        modbusInfo[port].masterWriteCallback = callback;
    }

    return stat;
}
#endif

#if MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
/*!
* @brief This function is the handler for reading discrete inputs
*
* This function needs to be capable of reading multiple 1-bit discrete
* inputs starting at the address given.
*
* @param port The port the request came through
* @param frame Pointer to the start of the received frame
* @param length Then length of the received frame
*
* @return The exception code of the function call (none or otherwise)
*/
modbus_exceptions modbus_read_discrete_inputs_handler(uint8_t port, uint8_t *frame, uint16_t *length)
{
    uint16_t regAddress, discreteCount;
    uint8_t curBit;
    uint8_t  *frameCur;

    if(*length == (MODBUS_PDU_FUNC_READ_SIZE + MODBUS_PDU_SIZE_MIN)) {
        regAddress = (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF] << 8);
        regAddress |= (uint16_t)(frame[MODBUS_PDU_FUNC_ADDR_OFF + 1]);

        discreteCount = (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF] << 8);
        discreteCount |= (uint16_t)(frame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1]);

        // Check if the number of registers to read is valid. If not return Modbus illegal data value exception.
        if((discreteCount > 0) && (discreteCount < MODBUS_PDU_FUNC_READ_COILCNT_MAX)
                && (regAddress >= modbus_discrete_start_address(port)) && (regAddress + discreteCount <= (modbus_discrete_start_address(port) + modbus_number_of_discretes(port)))) {
            // Set the current PDU data pointer to the beginning.
            frameCur = &frame[MODBUS_PDU_FUNC_OFF];
            *length = MODBUS_PDU_FUNC_OFF;

            // First byte contains the function code.
            *frameCur++ = MODBUS_FUNC_READ_DISCRETE_INPUTS;

            //But in the byte length based on number of coils
            *frameCur++ = (discreteCount + 7)/8;

            //Increment length for 2 header bytes and number of coil bytes
            *length += (2 + *(frameCur-1));

            //Get to the address in memory of the first coil and set our current output as 0
            regAddress = regAddress-modbus_discrete_start_address(port);
            *frameCur = 0x00;

            //Read all coils
            curBit = 0;
            while((discreteCount > 0) && (regAddress < modbus_number_of_discretes(port))) {
                //Goto next byte if we filled one
                if(curBit > 7) {
                    curBit = 0;
                    frameCur++;
                    *frameCur = 0x00;
                }

                //Put this coil in
                *frameCur |= ((modbus_discrete_read_callback(port, regAddress++))?(0x01<<curBit):0);
                discreteCount--;
                curBit++;
            }

            //Shouldn't get here, but just in case
            if(discreteCount > 0) {
                return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
            }
        } else {
            // Out of range discrete input requested
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    } else {
        // Can't be a valid read discrete input request because the length is incorrect.
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief This function sends a command to a slave to read discrete inputs
*
* This function causes a command to be sent out of the port that will
* request a slave to read the value of discrete inputs.  The callback given
* will be used to provide the retrieved data.
*
* @param port The port to send the command out of
* @param slaveAddress The slave to send the command to
* @param readAddress The address to start reading discrete inputs at
* @param readQty The quantity of discrete inputs to read
* @param callback Function to call with the returned results (callback should return true to indicate stop waiting for response for this command)
*
* @return Status that indicates if command was successfully sent or not
*/
modbus_status modbus_read_discrete_inputs_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint8_t *vals, uint16_t byteCnt))
{
    modbus_status stat;
    uint8_t *packFrame;

    //Ensure we have valid addresses
    if(slaveAddress > MODBUS_ADDRESS_MAX) {
        return MODBUS_STAT_INVALID_ADDRESS;
    }

    //Ensure we are within allowable range
    if(readQty > MODBUS_PDU_FUNC_READ_COILCNT_MAX) {
        return MODBUS_STAT_INVALID_VALUE;
    }

    //Make sure we are in a valid state to give commands and get our buffer
    if((modbusInfo[port].state != MODBUS_STATE_ENABLED) || (modbusInfo[port].deviceType != MODBUS_MASTER_DEVICE) || !modbusInfo[port].getTxBuffer || !modbusInfo[port].frameTransmit
            || !modbusInfo[port].getTxBuffer(port, &packFrame)) {
        return MODBUS_STAT_ILLEGAL_STATE;
    }

    //Pack our buffer
    packFrame[MODBUS_PDU_FUNC_OFF] = MODBUS_FUNC_READ_DISCRETE_INPUTS;
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF] = (uint8_t)(readAddress>>8);
    packFrame[MODBUS_PDU_FUNC_ADDR_OFF + 1] = (uint8_t)readAddress;
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF] = (uint8_t)(readQty>>8);
    packFrame[MODBUS_PDU_FUNC_READ_CNT_OFF + 1] = (uint8_t)readQty;

    //Send our data
    stat = modbusInfo[port].frameTransmit(port, slaveAddress, packFrame, 5);
    if((stat != MODBUS_STAT_SUCCESS) && modbusInfo[port].frameReleaseBuffer) {
        //We couldn't transmit so let our buffer go
        modbusInfo[port].frameReleaseBuffer(port);
    } else {
        modbusInfo[port].lastCmdRegAddress = readAddress;
        modbusInfo[port].lastSlaveAddress = slaveAddress;
        modbusInfo[port].lastFunctionCode = MODBUS_FUNC_READ_DISCRETE_INPUTS;
        modbusInfo[port].masterReadCoilsCallback = callback;
    }

    return stat;
}
#endif

/*!
* @brief This function handles the response to a master command
*
* This function will basically take in the response frame, format it
* properly, and send it on to the appropriate callback.  The return value
* will indicate if the response was a proper one to the command or not.
*
* @param port The port the response came on
* @param slaveAddress The slave that sent the response
* @param address The address that the command was supposed to read/write too
* @param frame Pointer to the response packet starting at the function code
* @param length The length of the response frame
*
* @return Boolean indicating if the response was correct for the command (completed)
*/
bool modbus_command_response_handler(uint8_t port, uint8_t slaveAddress, uint16_t address, uint8_t *frame, uint16_t length)
{
    modbus_function_codes funcCode = frame[MODBUS_PDU_FUNC_OFF];
    uint16_t regVals[MODBUS_PDU_FUNC_READ_REGCNT_MAX];
    uint16_t idx;
    uint16_t regLength = (length-1);    //Forget function code

    switch(funcCode & 0x7F) {
#if MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    case MODBUS_FUNC_OTHER_REPORT_SLAVEID:
        if(modbusInfo[port].masterGetDataCallback) {
            if(funcCode & 0x80) {
                return modbusInfo[port].masterGetDataCallback(port, slaveAddress, funcCode, &frame[MODBUS_PDU_DATA_OFF], 1);
            } else if((regLength-1) == frame[MODBUS_PDU_DATA_OFF]) {
                return modbusInfo[port].masterGetDataCallback(port, slaveAddress, funcCode, &frame[MODBUS_PDU_DATA_OFF+1], frame[MODBUS_PDU_DATA_OFF]);
            }
        }
        break;
#endif
#if (MODBUS_FUNC_READ_INPUT_ENABLED + MODBUS_FUNC_READ_HOLDING_ENABLED + MODBUS_FUNC_READWRITE_HOLDING_ENABLED) > 0
#if MODBUS_FUNC_READ_INPUT_ENABLED > 0
    case MODBUS_FUNC_READ_INPUT_REGISTER:
#endif
#if MODBUS_FUNC_READ_HOLDING_ENABLED > 0
    case MODBUS_FUNC_READ_HOLDING_REGISTER:
#endif
#if MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0
    case MODBUS_FUNC_READWRITE_MULTIPLE_REGISTERS:
#endif
        if(modbusInfo[port].masterMultiRegReadCallback) {
            if(funcCode & 0x80) {
                regVals[0] = frame[MODBUS_PDU_DATA_OFF];
                return modbusInfo[port].masterMultiRegReadCallback(port, slaveAddress, funcCode, 0, regVals, 1);
            } else if(((regLength-1) == frame[MODBUS_PDU_DATA_OFF]) && ((frame[MODBUS_PDU_DATA_OFF]%2) == 0)) {
                //Do the Endianess here so it is transparent
                idx = MODBUS_PDU_DATA_OFF+1;
                regLength = 0;
                while((idx < (frame[MODBUS_PDU_DATA_OFF] + MODBUS_PDU_DATA_OFF + 1)) && (regLength < MODBUS_PDU_FUNC_READ_REGCNT_MAX)) {
                    regVals[regLength] = (((uint16_t)frame[idx++])<<8);
                    regVals[regLength++] |= frame[idx++];
                }
                return modbusInfo[port].masterMultiRegReadCallback(port, slaveAddress, funcCode, address, regVals, regLength);
            }
        }
        break;
#endif
#if (MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED + MODBUS_FUNC_WRITE_HOLDING_ENABLED + MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED) > 0
#if MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS:
#endif
#if MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0
    case MODBUS_FUNC_WRITE_REGISTER:
#endif
#if MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    case MODBUS_FUNC_WRITE_MULTIPLE_COILS:
#endif
        if(modbusInfo[port].masterWriteCallback) {
            if(funcCode & 0x80) {
                return modbusInfo[port].masterWriteCallback(port, slaveAddress, funcCode, frame[MODBUS_PDU_DATA_OFF], 1);
            } else {
                return modbusInfo[port].masterWriteCallback(port, slaveAddress, funcCode, ((frame[MODBUS_PDU_DATA_OFF]<<8) | frame[MODBUS_PDU_DATA_OFF+1]), ((frame[MODBUS_PDU_DATA_OFF+2]<<8) | frame[MODBUS_PDU_DATA_OFF+3]));
            }
        }
        break;
#endif
#if MODBUS_FUNC_WRITE_COIL_ENABLED > 0
    case MODBUS_FUNC_WRITE_SINGLE_COIL:
        if(modbusInfo[port].masterWriteSingleCoilCallback) {
            if(funcCode & 0x80) {
                return modbusInfo[port].masterWriteSingleCoilCallback(port, slaveAddress, funcCode, frame[MODBUS_PDU_DATA_OFF], true);
            } else {
                return modbusInfo[port].masterWriteSingleCoilCallback(port, slaveAddress, funcCode, ((frame[MODBUS_PDU_DATA_OFF]<<8) | frame[MODBUS_PDU_DATA_OFF+1]), (((frame[MODBUS_PDU_DATA_OFF+2] == 0xFF) && (frame[MODBUS_PDU_DATA_OFF+3] == 0x00))?true:false));
            }
        }
        break;
#endif
#if (MODBUS_FUNC_READ_COILS_ENABLED + MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED) > 0
#if MODBUS_FUNC_READ_COILS_ENABLED > 0
    case MODBUS_FUNC_READ_COILS:
#endif
#if MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    case MODBUS_FUNC_READ_DISCRETE_INPUTS:
#endif
        if(modbusInfo[port].masterReadCoilsCallback) {
            if(funcCode & 0x80) {
                return modbusInfo[port].masterReadCoilsCallback(port, slaveAddress, funcCode, 0, &frame[MODBUS_PDU_DATA_OFF], 1);
            } else if((regLength-1) == frame[MODBUS_PDU_DATA_OFF]) {
                return modbusInfo[port].masterReadCoilsCallback(port, slaveAddress, funcCode, address, &frame[MODBUS_PDU_DATA_OFF+1], frame[MODBUS_PDU_DATA_OFF]);
            }
        }
        break;
#endif
    default:
        break;
    }

    return false;
}