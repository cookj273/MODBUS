/*!
* @file modbus_module.h
* @author Jarrod Cook
*
* @brief Implementation of the modbus module
*
* This file is the implementation of routines to handle modbus
* communications with external equipment.
*
* @copyright Copyright 2024 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/
#ifndef MODBUS_MODULE_H_
#define MODBUS_MODULE_H_

//Includes
#include "conf_modbus.h"

//Version
#define MODBUS_LIBRARY_MAJOR_VERSION    1
#define MODBUS_LIBRARY_MINOR_VERSION    10

//Defines
#define MODBUS_FUNC_ERROR       128                                         //!< FUNCTION ERROR MASK (OR WITH ORIGINAL FUNCTION CODE TO INDICATE ERROR)
//General frame limits
#define MODBUS_PDU_SIZE_MIN     1                                           //!< Minimum frame PDU size just a function code
#define MODBUS_PDU_SIZE_MAX     254                                         //!< Maximum size of a PDU.
#define MODBUS_TCP_PDU_SIZE_MAX 261                                         //!< Maximum size of a PDU for TCP/IP.
#define MODBUS_PDU_FUNC_OFF     0                                           //!< Offset of function code in PDU.
#define MODBUS_PDU_DATA_OFF     1                                           //!< Offset for response data in PDU.
//Read Limits
#define MODBUS_PDU_FUNC_READ_SIZE               4                           //!< The size of a MODBUS Read Command
#define MODBUS_PDU_FUNC_ADDR_OFF                MODBUS_PDU_DATA_OFF         //!< The location of the address in a read command
#define MODBUS_PDU_FUNC_READ_CNT_OFF            MODBUS_PDU_DATA_OFF + 2     //!< The location of the coil count in a read command
#define MODBUS_PDU_FUNC_READ_COILCNT_MAX        0x07D0                      //!< The max coils that can be read in a read command
#define MODBUS_PDU_FUNC_READ_REGCNT_MAX         0x007D                      //!< The max registers that can be read in a read command
//Write Limits
#define MODBUS_PDU_FUNC_WRITE_VALUE_OFF         MODBUS_PDU_DATA_OFF + 2     //!< The Offset of the start of values to write
#define MODBUS_PDU_FUNC_WRITE_SIZE              4                           //!< The minimum size of the write command
//Multi-Write Limits
#define MODBUS_PDU_FUNC_WRITE_MUL_CNT_OFF       MODBUS_PDU_DATA_OFF + 2     //!< Offset of the length byte in the MODBUS packet for multi-write
#define MODBUS_PDU_FUNC_WRITE_MUL_BYTECNT_OFF   MODBUS_PDU_DATA_OFF + 4     //!< Offset of the byte count byte in the MODBUS packet for multi-write
#define MODBUS_PDU_FUNC_WRITE_MUL_VALUES_OFF    MODBUS_PDU_DATA_OFF + 5     //!< Start of the list of values for multi-write
#define MODBUS_PDU_FUNC_WRITE_MUL_SIZE_MIN      5                           //!< The minimum size of a multi-write packet
#define MODBUS_PDU_FUNC_WRITE_MUL_COILCNT_MAX   0x07B0                      //!< The maximum number of coils that can be written at once
#define MODBUS_PDU_FUNC_WRITE_MUL_REGCNT_MAX    0x007B                      //!< The maximum number of registers that can be written at once
//Multi-ReadWrite Limits
#define MODBUS_PDU_FUNC_READWRITE_READ_ADDR_OFF     MODBUS_PDU_DATA_OFF + 0 //!< The offset of the read address in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_OFF   MODBUS_PDU_DATA_OFF + 2 //!< The offset of the read register count in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_WRITE_ADDR_OFF    MODBUS_PDU_DATA_OFF + 4 //!< The offset of the write address in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF  MODBUS_PDU_DATA_OFF + 6 //!< The offset of the write register count in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_BYTECNT_OFF       MODBUS_PDU_DATA_OFF + 8 //!< The offset of the write byte count in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_WRITE_VALUES_OFF  MODBUS_PDU_DATA_OFF + 9 //!< The start of the list of values to write in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_SIZE_MIN          9                       //!< The minimum length of a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_READ_REGCNT_MAX   0x007D                  //!< The maximum number of registers that can be read in a readwrite command
#define MODBUS_PDU_FUNC_READWRITE_WRITE_REGCNT_MAX  0x0079                  //!< The maximum number of registers that can be written in a readwrite command

//Typedefs
/*!
* Defines the modbus function codes
*/
typedef enum {
    MODBUS_FUNC_NONE = 0,                           //!< No Function
    MODBUS_FUNC_READ_COILS = 1,                     //!< Read Coils Function
    MODBUS_FUNC_READ_DISCRETE_INPUTS = 2,           //!< Read Discrete Inputs Function
    MODBUS_FUNC_WRITE_SINGLE_COIL = 5,              //!< Write Single Coils Function
    MODBUS_FUNC_WRITE_MULTIPLE_COILS = 15,          //!< Write Multiple Coils Function
    MODBUS_FUNC_READ_HOLDING_REGISTER = 3,          //!< Read Holding Register Function
    MODBUS_FUNC_READ_INPUT_REGISTER = 4,            //!< Read Input Register Function
    MODBUS_FUNC_WRITE_REGISTER = 6,                 //!< Write Single Register Function
    MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS = 16,      //!< Write Multiple Registers Function
    MODBUS_FUNC_READWRITE_MULTIPLE_REGISTERS = 23,  //!< Read/Write Multiple Registers Function
    MODBUS_FUNC_DIAG_READ_EXCEPTION = 7,            //!< Diagnostic Exception Read Function
    MODBUS_FUNC_DIAG_DIAGNOSTIC = 8,                //!< Diagnostic Function
    MODBUS_FUNC_DIAG_GET_COM_EVENT_CNT = 11,        //!< Diagnostic Get Event Count Function
    MODBUS_FUNC_DIAG_GET_COM_EVENT_LOG = 12,        //!< Diagnostic Get Event Log Function
    MODBUS_FUNC_OTHER_REPORT_SLAVEID = 17           //!< Diagnostic Report Additional Slave ID Function
} modbus_function_codes;

/*!
* Defines the potential exception result of functions
*/
typedef enum {
    MODBUS_EX_NONE = 0x00,                  //!< No exception, handled successfully
    MODBUS_EX_ILLEGAL_FUNCTION = 0x01,      //!< Illegal function given (not handled)
    MODBUS_EX_ILLEGAL_DATA_ADDRESS = 0x02,  //!< Illegal data address (out of range read/write)
    MODBUS_EX_ILLEGAL_DATA_VALUE = 0x03,    //!< Illegal data value (value not acceptable)
    MODBUS_EX_SLAVE_DEVICE_FAILURE = 0x04,  //!< Slave device failure
    MODBUS_EX_ACKNOWLEDGE = 0x05,           //!< Acknowledge error
    MODBUS_EX_SLAVE_BUSY = 0x06,            //!< Slave is busy error
    MODBUS_EX_MEMORY_PARITY_ERROR = 0x08,   //!< Memory Parity Error
    MODBUS_EX_GATEWAY_PATH_FAILED = 0x0A,   //!< Gateway Path Failed
    MODBUS_EX_GATEWAY_TGT_FAILED = 0x0B     //!< Gateway TGT Failed
} modbus_exceptions;

/*!
* The following defines the different communication modes that are
* available.  TCP has not been implemented in this driver but may be
* in future version so it was left in.
*/
typedef enum {
    MODBUS_RTU=0,       //!< RTU transmission mode.
    MODBUS_ASCII=1,     //!< ASCII transmission mode.
    MODBUS_TCP=2        //!< TCP mode.
} modbus_modes;

/*!
* The following defines the different device types a port can be
* this is basically either master or slave.
*/
typedef enum {
    MODBUS_SLAVE_DEVICE=0,      //!< Slave device, respond to commands
    MODBUS_MASTER_DEVICE=1      //!< Master device, transmit commands and wait
} modbus_device_types;

/*!
* The following defines the different RTU receive states
*/
typedef enum {
    STATE_RX_INIT,              //!< Receiver is in initial state.
    STATE_RX_IDLE,              //!< Receiver is in idle state.
    STATE_RX_RCV,               //!< Frame is beeing received.
#if MODBUS_ASCII_ENABLED > 0
    STATE_RX_WAIT_EOF,          //!< Wait for EOF in ASCII mode.
#endif
    STATE_RX_ERROR,             //!< If the frame is invalid.
    STATE_RX_COMPLETE,          //!< A full frame was received
    STATE_RX_OFF
} mb_rx_states;

/*!
* The following defines the different RTU transmit states
*/
typedef enum {
    STATE_TX_IDLE,              //!< Transmitter is in idle state.
    STATE_TX_XMIT,              //!< Transmitter is in transfer state.
#if MODBUS_ASCII_ENABLED > 0
    STATE_TX_ASCII_H,           //!< Send the high nibble of the next character
    STATE_TX_ASCII_L,           //!< Send the low nibble of the next character
    STATE_TX_ASCII_END,         //!< Send the EOF for ASCII mode
#endif
    STATE_TX_WAIT               //!< After last byte transfer we need to wait for hardware to send before going back to receive
} mb_tx_states;

extern volatile mb_tx_states txState[NUM_MODBUS_PORTS];              //!< Tracks the current tx state
extern volatile mb_rx_states rxState[NUM_MODBUS_PORTS];              //!< Tracks the current rx state
extern volatile bool frameTxComplete[NUM_MODBUS_PORTS];               //!< Indicates when a full frame has finished transmitting
extern volatile uint8_t mbDataBuf[NUM_MODBUS_PORTS][MODBUS_PDU_SIZE_MAX];   //!< Store both data received and transmitted
extern volatile uint16_t rxBufferPos[NUM_MODBUS_PORTS];               //!< RX position (what byte is next)
extern volatile uint8_t *txBufferCur[NUM_MODBUS_PORTS];               //!< TX position (what byte to send next)
extern volatile uint16_t txBufferCount[NUM_MODBUS_PORTS];             //!< Total number of bytes to transmit
extern volatile modbus_modes portMode[NUM_MODBUS_PORTS];              //!< Stores the mode ID for each port

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
* @param tcpPort The port number to use if this is a TCP connection (default is 502)
*
* @return Status value indicating the result of the init
*/
modbus_status modbus_module_init(uint8_t port, modbus_modes mode, modbus_device_types deviceType, uint8_t slaveAddress, uint32_t baudRate, modbus_parity parity, uint8_t stopBits, uint16_t tcpPort);

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
bool modbus_modbule_change_address(uint8_t port, uint8_t slaveAddress);

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
modbus_status modbus_module_enable(uint8_t port, bool enable);

/*!
* @brief Closes down the MODBUS module if we choose to stop
*
* This function will both completely close out the MODBUS driver, saving
* power by shutting stuff off.
*
* @param port the port to close out
*/
void modbus_module_close(uint8_t port);

/*!
* @brief Simply indicates whether a frame was received or transmitted
*
* This function will check to see if there a send or received frame
* that still needs handling.  It can be used to determine if the polling
* function needs called or not.
*
* @return True if there are frames that need handled
*/
bool modbus_module_needs_serviced(void);

/*!
* @brief Indicates whether or not MODBUS is currently running
*
* This function simply inform the user whether or not MODBUS
* is currently running, so they can save power if not.
*
* @return True if modbus is currently enabled and running
*/
bool modbus_module_is_running(void);

/*!
* @brief Function that services the MODBUS module and handles frames
*
* This function should be called routinely or when there is a frame to
* handle.  It will process any received and transmitted frames.
*/
void modbus_module_service(void);



//****************************************************************************MASTER COMMAND FUNCTIONS**************************************************************//

//**********EXAMPLE USAGE:
/*
//Callback function
static bool resultCallback(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty) {
    for(uint8_t idx=0; idx < qty; idx++) {
        varStore[idx]=*vals++;
    }

    return true;
}

....
//Call somewhere in code (read input registers on given port from unit with address 247 and read 2 registers starting with address 2016, callback is above func)
modbus_read_input_register_command(MB_PORT_NUM, 247, 2016, 2, resultCallback);
....
*/

#if MODBUS_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
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
modbus_status modbus_report_slave_id_command(uint8_t port, uint8_t slaveAddress, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint8_t *data, uint8_t length));
#endif

#if MODBUS_FUNC_READ_INPUT_ENABLED > 0
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
modbus_status modbus_read_input_register_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty));
#endif

#if MODBUS_FUNC_READ_HOLDING_ENABLED > 0
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
modbus_status modbus_read_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty));
#endif

#if MODBUS_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
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
modbus_status modbus_write_multi_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, uint16_t writeQty, uint16_t *vals, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t qty));
#endif

#if MODBUS_FUNC_WRITE_HOLDING_ENABLED > 0
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
modbus_status modbus_write_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, uint16_t val, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t val));
#endif

#if MODBUS_FUNC_READWRITE_HOLDING_ENABLED > 0
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
modbus_status modbus_readwrite_multi_holding_register_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, uint16_t writeAddress, uint16_t writeQty, uint16_t *vals, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t *vals, uint16_t qty));
#endif

#if MODBUS_FUNC_READ_COILS_ENABLED > 0
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
modbus_status modbus_read_coils_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint8_t *vals, uint16_t byteCnt));
#endif

#if MODBUS_FUNC_WRITE_COIL_ENABLED > 0
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
modbus_status modbus_write_coil_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, bool val, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, bool val));
#endif

#if MODBUS_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
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
modbus_status modbus_write_multiple_coils_command(uint8_t port, uint8_t slaveAddress, uint16_t writeAddress, uint16_t writeQty, uint8_t *vals, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t qty));
#endif

#if MODBUS_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
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
modbus_status modbus_read_discrete_inputs_command(uint8_t port, uint8_t slaveAddress, uint16_t readAddress, uint16_t readQty, bool (*callback)(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint8_t *vals, uint16_t byteCnt));
#endif

#endif /* MODBUS_MODULE_H_ */
