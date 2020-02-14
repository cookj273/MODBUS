/*!
* @file modbus_rtu.c
* @author Jarrod Cook
*
* @brief Implementation of the modbus RTU functionality
*
* This file is the implementation of routines to handle modbus
* RTU communications with external equipment.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includes
#include "modbus_rtu.h"
#include "modbus_port.h"
#include "crc16.h"

//Defines
#define MODBUS_SER_PDU_SIZE_MIN     4       //!< Minimum size of a Modbus RTU frame.
#define MODBUS_SER_PDU_SIZE_MAX     256     //!< Maximum size of a Modbus RTU frame.
#define MODBUS_SER_PDU_SIZE_CRC     2       //!< Size of CRC field in PDU.
#define MODBUS_SER_PDU_ADDR_OFF     0       //!< Offset of slave address in Ser-PDU.
#define MODBUS_SER_PDU_PDU_OFF      1       //!< Offset of Modbus-PDU in Ser-PDU.
#if MODBUS_ASCII_ENABLED > 0
#define MODBUS_SER_PDU_SIZE_LRC     1       // Size of LRC field in PDU.
#define MODBUS_ASCII_DEFAULT_CR     '\r'    // Default CR character for Modbus ASCII.
#define MODBUS_ASCII_DEFAULT_LF     '\n'    // Default LF character for Modbus ASCII.
#endif

//Typedefs
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
} rtu_rx_states;

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
} rtu_tx_states;

//Local Variables
static volatile rtu_tx_states txState[NUM_MODBUS_PORTS];              //!< Tracks the current tx state
static volatile rtu_rx_states rxState[NUM_MODBUS_PORTS];              //!< Tracks the current rx state
static volatile bool frameTxComplete[NUM_MODBUS_PORTS];               //!< Indicates when a full frame has finished transmitting

volatile uint8_t RTUBuf[NUM_MODBUS_PORTS][MODBUS_SER_PDU_SIZE_MAX];   //!< Store both data received and transmitted
static volatile uint16_t rxBufferPos[NUM_MODBUS_PORTS];               //!< RX position (what byte is next)
static volatile uint8_t *txBufferCur[NUM_MODBUS_PORTS];               //!< TX position (what byte to send next)
static volatile uint16_t txBufferCount[NUM_MODBUS_PORTS];             //!< Total number of bytes to transmit
#if MODBUS_ASCII_ENABLED > 0
static volatile uint8_t rxBytePos[NUM_MODBUS_PORTS];                  //!< RX nibble for ASCII mode
#endif
static volatile modbus_modes portMode[NUM_MODBUS_PORTS];              //!< Stores the mode ID for each port

#if MODBUS_ASCII_ENABLED > 0
/*!
* @brief Simply converts a number represented in ASCII to a binary number (0-F)
*
* This should be used in ASCII mode to convert incoming data from ASCII
* to numerical
*
* @param chr The character to convert
*/
static uint8_t char2Binary(char chr)
{
    if((chr >= '0') && (chr <= '9')) {
        return (uint8_t)(chr - '0');
    } else if((chr >= 'A') && (chr <= 'F')) {
        return (uint8_t)(chr - 'A' + 0x0A);
    } else {
        return 0xFF;
    }
}

/*!
* @brief Simply converts a numerical nibble to it's ASCII representation
*
* This should be used in ASCII mode to convert outgoing data into
* ASCII format.
*
* @param bin The nibble to convert
*/
static char binary2Char(uint8_t bin)
{
    if((bin & 0x0F) <= 0x09) {
        return (char)('0' + bin);
    } else {
        return (char)(((bin & 0x0F) - 0x0A) + 'A');
    }

    return '0';
}

/**
 * @brief Compute the updated CRC16 value.
 *
 * Compute the checksum value based on the current checksum value and the additional data.
 *
 * @param chk the current checksum value
 * @param data the new data
 * @param length the number of bytes of new data
 *
 * @return the updated checksum value
 */
uint8_t chksum_calculate(uint8_t chk, uint8_t* data, uint16_t length)
{
    while( length-- ) {
        chk += *data++;
    }

    /* Return twos complement */
    return (uint8_t)(-((int8_t)chk));
}
#endif

/*!
* @brief The receive FSM for receiving in the Frame bytes one-by-one
*
* This should be called from the receive interrupt to place received bytes
* into the buffer and handle running the timer.
*
* @param port The port to use for function
* @param newByte The new byte that was received.
*/
static void modbus_rtu_receive_fsm(uint8_t port)
{
    uint8_t newByte;

    //Critical Section enter
    modbus_port_enable_interrupts(port, false);

    //Get the byte and ensure we don't have an error (if we are in complete or off state then don't change state)
    if(!modbus_port_get_byte(port, &newByte) && (rxState[port] != STATE_RX_COMPLETE) && (rxState[port] != STATE_RX_OFF)) {
        //We had some sort of error
        rxState[port] = STATE_RX_ERROR;
    }

#if MODBUS_ASCII_ENABLED > 0
    //ASCII doesn't rely on timeouts so don't use certain states
    if((portMode[port] == MODBUS_ASCII) && ((rxState[port] == STATE_RX_INIT) || (rxState[port] == STATE_RX_ERROR))) {
        rxState[port] = STATE_RX_IDLE;
    }
#endif

    switch (rxState[port]) {
    // If we have received a character in the init state we have to wait until the frame is finished.
    case STATE_RX_INIT:
        modbus_port_timer_enable(port, true);
        break;
    // In the error state we wait until all characters in the damaged frame are transmitted.
    case STATE_RX_ERROR:
        modbus_port_timer_enable(port, true);
        break;
    // In the idle state we wait for a new character. If a character is received the t1.5 and t3.5 timers are started and the receiver is in the state STATE_RX_RECEIVCE.
    case STATE_RX_IDLE:
#if MODBUS_ASCII_ENABLED > 0
        if(portMode[port] == MODBUS_ASCII) {
            if(newByte == ':') {
                /* Reset the input buffers to store the frame. */
                rxBufferPos[port] = 0;
                rxState[port] = STATE_RX_RCV;
                rxBytePos[port] = 0;

                //Enable timeout
                modbus_port_timer_enable(port, true);
            }
        } else {
#endif
            rxBufferPos[port] = 0;
            RTUBuf[port][rxBufferPos[port]++] = newByte;
            rxState[port] = STATE_RX_RCV;

            // Enable t3.5 timers.
            modbus_port_timer_enable(port, true);
#if MODBUS_ASCII_ENABLED > 0
        }
#endif
        break;
    // We are currently receiving a frame. Reset the timer after every character received. If more than the maximum possible number of bytes in a modbus frame is received the frame is ignored.
    case STATE_RX_RCV:
        modbus_port_timer_enable(port, true);

        if(rxBufferPos[port] < MODBUS_SER_PDU_SIZE_MAX) {
#if MODBUS_ASCII_ENABLED > 0
            if(portMode[port] == MODBUS_ASCII) {
                if(newByte == ':') {
                    //Start character so reset
                    rxBytePos[port] = 0;
                    rxBufferPos[port] = 0;
                } else if(newByte == MODBUS_ASCII_DEFAULT_CR) {
                    rxState[port] = STATE_RX_WAIT_EOF;
                } else {
                    if(rxBytePos[port]) {
                        RTUBuf[port][rxBufferPos[port]++] |= char2Binary(newByte);
                        rxBytePos[port] = 0;
                    } else {
                        RTUBuf[port][rxBufferPos[port]] = ((char2Binary(newByte)<<4) & 0xF0);
                        rxBytePos[port] = 1;
                    }
                }
            } else {
#endif
                RTUBuf[port][rxBufferPos[port]++] = newByte;
#if MODBUS_ASCII_ENABLED > 0
            }
#endif
        } else {
            rxState[port] = STATE_RX_ERROR;
        }
        break;
#if MODBUS_ASCII_ENABLED > 0
    case STATE_RX_WAIT_EOF:
        if(newByte == MODBUS_ASCII_DEFAULT_LF) {
            //Disable timer and mark complete
            modbus_port_timer_enable(port, false);
            rxState[port] = STATE_RX_COMPLETE;
        } else if(newByte == ':') {
            //Start character so reset
            rxBytePos[port] = 0;
            rxBufferPos[port] = 0;
            rxState[port] = STATE_RX_RCV;
        } else {
            rxState[port] = STATE_RX_IDLE;
        }
        break;
#endif
    default:
        rxState[port] = STATE_RX_ERROR;
        break;
    }

    //Critical Section exit
    modbus_port_enable_interrupts(port, true);
}

/*!
* @brief The transmit FSM for transmitting the Frame bytes one-by-one
*
* This should be called from the transmit ready interrupt to place received bytes
* into the buffer and handle running the timer.
*
* @param port The port to use for function
*/
static void modbus_rtu_transmit_fsm(uint8_t port)
{
    switch(txState[port]) {
    // We should not get a transmitter event if the transmitter is in idle state.
    case STATE_TX_IDLE:
        // enable receiver/disable transmitter because we are idle and shouldn't be here
        modbus_port_serial_enable(port, true, false);
        break;
    case STATE_TX_XMIT:
#if MODBUS_ASCII_ENABLED
        if(portMode[port] == MODBUS_ASCII) {
            modbus_port_put_byte(port, ':');
            txState[port] = STATE_TX_ASCII_H;
        } else {
#endif
            // check if we are finished.
            if(txBufferCount[port] != 0) {
                modbus_port_put_byte(port, (uint8_t)*txBufferCur[port]);
                txBufferCur[port]++;  // next byte in sendbuffer.
                txBufferCount[port]--;
            } else {
                //Change TX interrupt to wait for empty
                modbus_port_notify_of_tx_completion(port);
                txState[port] = STATE_TX_WAIT;
            }
#if MODBUS_ASCII_ENABLED > 0
        }
#endif
        break;
#if MODBUS_ASCII_ENABLED > 0
    case STATE_TX_ASCII_H:
        if(txBufferCount[port] != 0) {
            modbus_port_put_byte(port,binary2Char((((uint8_t)*txBufferCur[port] >> 4) & 0x0F)));
            txState[port] = STATE_TX_ASCII_L;
        } else {
            modbus_port_put_byte(port,MODBUS_ASCII_DEFAULT_CR);
            txBufferCount[port] = 1;
            txState[port] = STATE_TX_ASCII_END;
        }
        break;
    case STATE_TX_ASCII_L:
        modbus_port_put_byte(port,binary2Char(((uint8_t)*txBufferCur[port] & 0x0F)));
        txBufferCur[port]++;
        txBufferCount[port]--;
        txState[port] = STATE_TX_ASCII_H;
        break;
    case STATE_TX_ASCII_END:
        if(txBufferCount[port] != 0) {
            modbus_port_put_byte(port,MODBUS_ASCII_DEFAULT_LF);
            txBufferCount[port] = 0;
        } else {
            //Change TX interrupt to wait for empty
            modbus_port_notify_of_tx_completion(port);
            txState[port] = STATE_TX_WAIT;
        }
        break;
#endif
    case STATE_TX_WAIT:
        // Flag our send as complete
        frameTxComplete[port] = true;

        //Change our receiver back to idle mode
        if((rxState[port] == STATE_RX_COMPLETE) || (rxState[port] == STATE_RX_OFF)) {
            rxState[port] = STATE_RX_IDLE;
        }

        // Disable transmitter. This prevents another transmit buffer empty interrupt.
        modbus_port_serial_enable(port, true, false);
        txState[port] = STATE_TX_IDLE;
        break;
    default:
        break;
    }
}

/*!
* @brief The timer expired function for determining timed actions based on state
*
* This should be called from the timer interrupt that indicates 3.5 character
* times have passed.  Based on the current receive state this will indicate
* what the timeout implies.
*
* @param port The port to use for function
*/
static void modbus_rtu_timer_expired(uint8_t port)
{
    //Critical Section enter
    modbus_port_enable_interrupts(port, false);

#if MODBUS_ASCII_ENABLED > 0
    if(portMode[port] == MODBUS_ASCII) {
        if((rxState[port] != STATE_RX_COMPLETE) && (rxState[port] != STATE_RX_OFF)) {
            rxState[port] = STATE_RX_IDLE;
        }
    } else {
#endif
        switch (rxState[port]) {
        // A frame was received and t35 expired. Change to the complete state
        case STATE_RX_RCV:
            rxState[port] = STATE_RX_COMPLETE;
            break;
        //In the off or complete state the timer shouldn't be running
        case STATE_RX_COMPLETE:
        case STATE_RX_OFF:
            break;
        // Timer t35 expired. Startup phase is finished.
        case STATE_RX_INIT:
        // An error occured while receiving the frame.
        case STATE_RX_ERROR:
        // Function called in an illegal state (idle).
        default:
            rxState[port] = STATE_RX_IDLE;
            break;
        }
#if MODBUS_ASCII_ENABLED > 0
    }
#endif

    modbus_port_timer_enable(port, false);

    //Critical Section exit
    modbus_port_enable_interrupts(port, true);
}

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
modbus_status modbus_rtu_init(modbus_modes mode, uint8_t port, uint32_t baudRate, modbus_parity parity, uint8_t stopBits)
{
    uint16_t timerFrameDelayBits;
    uint8_t charSize = 8;

    // If baudrate > 19200 then we should use the fixed timer value of 1750us. Otherwise delay must be 3.5 times the character time.
#if MODBUS_ASCII_ENABLED > 0
    if(mode == MODBUS_ASCII) {
        charSize = 7;
        //Use ASCII delay time
        timerFrameDelayBits = (((MODBUS_ASCII_TIMEOUT_SEC*10*baudRate) + 9)/10);
        portMode[port] = MODBUS_ASCII;
    } else {
#endif
        portMode[port] = MODBUS_RTU;
        if(baudRate > 37000000) {
            //I don't know who would ever try this baud rate (37MHz) but don't allow it
            return MODBUS_STAT_INVALID_VALUE;
        } else if(baudRate > 19200) {
            //Convert 1750us to bits => 1750/bitTime => 1750/(1000000/baudRate) => ((1750*baudRate)/1000000)
            timerFrameDelayBits = (((1750*baudRate) + 999999)/1000000);
        } else {
            //Char per sec = baudRate/11 => 1/(baudRate/11) secs per char => 1000000/(baudRate/11) uSec per char
            // = > 11000000/baudRate uSec per char, so 3.5 chars = 3.5 * (11000000/baudRate) => 38500000/baudRate
            //timerFrameDelayus = ((38500000)/baudRate);

            //We are doing this in char bit time though, so 11 bit per char * 3.5 = 38.5 or 39 bits
            timerFrameDelayBits = 39;
        }
#if MODBUS_ASCII_ENABLED > 0
    }
#endif

    //Init our states
    frameTxComplete[port] = false;
    txState[port] = STATE_TX_IDLE;
    rxState[port] = STATE_RX_IDLE;

    //Init the timer
    if(!modbus_port_init(port, baudRate, charSize, parity, stopBits, timerFrameDelayBits, modbus_rtu_receive_fsm, modbus_rtu_transmit_fsm, modbus_rtu_timer_expired)) {
        return MODBUS_STAT_PORT_ERROR;
    }

    return MODBUS_STAT_SUCCESS;
}

/*!
* @brief Enables/Disables the RTU communications
*
* This will enable and disable RTU communications by starting up and
* shutting down ports and timers and setting settings as necessary.
*
* @param port The port to use for function
* @param enable True implies enable, false implies disable
*/
void modbus_rtu_enable(uint8_t port, bool enable)
{
    //Critical Section enter
    modbus_port_enable_interrupts(port, false);

    if(enable) {
        /* Initially the receiver is in the state STATE_RX_INIT. we start
         * the timer and if no character is received within t3.5 we change
         * to STATE_RX_IDLE. This makes sure that we delay startup of the
         * modbus protocol stack until the bus is free.
         */
        rxState[port] = STATE_RX_INIT;
        modbus_port_serial_enable(port, true, false);
        modbus_port_timer_enable(port, true);
    } else {
        modbus_port_serial_enable(port, false, false);
        modbus_port_timer_enable(port, false);
    }

    //Critical Section exit
    modbus_port_enable_interrupts(port, true);
}

/*!
* @brief Closes out the port when shut off
*
* This will close out the port and shut everything off when we decide to
* turn MODBUS off.
*
* @param port The port to use for function
*/
void modbus_rtu_close(uint8_t port)
{
    //Simply close out the serial port
    modbus_port_serial_close(port);
    frameTxComplete[port] = false;
    txState[port] = STATE_TX_IDLE;
    rxState[port] = STATE_RX_IDLE;
}

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
modbus_status modbus_rtu_receive(uint8_t port, uint8_t *slaveAddress, uint8_t **frame, uint16_t *length)
{
    //See if we have a packet
    if(rxState[port] != STATE_RX_COMPLETE) {
        return MODBUS_STAT_NO_DATA;
    }

    //Ensure we didn't overflow
    if(rxBufferPos[port] >= MODBUS_SER_PDU_SIZE_MAX) {
        rxState[port] = STATE_RX_IDLE;
        return MODBUS_STAT_BUFFER_OVERFLOW;
    } else if((rxBufferPos[port] >= (
#if MODBUS_ASCII_ENABLED > 0
                   (portMode[port] == MODBUS_ASCII)?(MODBUS_SER_PDU_SIZE_MIN-1):
#endif
                   MODBUS_SER_PDU_SIZE_MIN)) && ((
#if MODBUS_ASCII_ENABLED > 0
                               (portMode[port] == MODBUS_ASCII) && (chksum_calculate(0, (uint8_t *)RTUBuf[port], rxBufferPos[port]) == 0)) || ((portMode[port] == MODBUS_RTU) &&
#endif
                                       (crc16_calculate(0xFFFF, (uint8_t *)RTUBuf[port], rxBufferPos[port]) == 0)))) {
        // Save the address field. All frames are passed to the upper layed and the decision if a frame is used is done there.
        *slaveAddress = RTUBuf[port][MODBUS_SER_PDU_ADDR_OFF];

        // Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus size of address field and CRC checksum.
#if MODBUS_ASCII_ENABLED > 0
        if(portMode[port] == MODBUS_ASCII) {
            *length = (uint16_t)(rxBufferPos[port] - MODBUS_SER_PDU_PDU_OFF - MODBUS_SER_PDU_SIZE_LRC);
        } else {
#endif
            *length = (uint16_t)(rxBufferPos[port] - MODBUS_SER_PDU_PDU_OFF - MODBUS_SER_PDU_SIZE_CRC);
#if MODBUS_ASCII_ENABLED > 0
        }
#endif

        // Return the start of the Modbus PDU to the caller.
        *frame = (uint8_t *)(&RTUBuf[port][MODBUS_SER_PDU_PDU_OFF]);
        rxState[port] = STATE_RX_IDLE;
    } else {
        //Bad CRC or length so back to idle and report error
        rxState[port] = STATE_RX_IDLE;
        return MODBUS_STAT_IO_ERROR;
    }

    return MODBUS_STAT_SUCCESS;
}

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
modbus_status modbus_rtu_transmit(uint8_t port, uint8_t slaveAddress, const uint8_t *frame, uint16_t length)
{
    uint16_t CRC16;

    //Disable our interrupts so we can get an accurate reading
    modbus_port_enable_interrupts(port, false);

    //Put rx into off state if in an appropriate place so no messages are received
    if((rxState[port] == STATE_RX_IDLE) || (rxState[port] == STATE_RX_COMPLETE)) {
        rxState[port] = STATE_RX_OFF;
    }

    //Reenable the interrupts
    modbus_port_enable_interrupts(port, true);

    // Receiver needs to be idle or complete to send
    if((rxState[port] == STATE_RX_OFF) && (txState[port] == STATE_TX_IDLE)) {
        // First byte before the Modbus-PDU is the slave address.
        txBufferCur[port] = (uint8_t *)frame - 1;
        txBufferCount[port] = 1;

        // Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU.
        txBufferCur[port][MODBUS_SER_PDU_ADDR_OFF] = slaveAddress;
        txBufferCount[port] += length;

#if MODBUS_ASCII_ENABLED > 0
        if(portMode[port] == MODBUS_ASCII) {
            // Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
            CRC16 = chksum_calculate(0, (uint8_t *)txBufferCur[port], txBufferCount[port]);
            RTUBuf[port][txBufferCount[port]++] = (uint8_t)(CRC16 & 0xFF);
        } else {
#endif
            // Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
            CRC16 = crc16_calculate(0xFFFF, (uint8_t *)txBufferCur[port], txBufferCount[port]);
            RTUBuf[port][txBufferCount[port]++] = (uint8_t)(CRC16 & 0xFF);
            RTUBuf[port][txBufferCount[port]++] = (uint8_t)(CRC16 >> 8);
#if MODBUS_ASCII_ENABLED > 0
        }
#endif

        // Activate the transmitter.
        txState[port] = STATE_TX_XMIT;
        frameTxComplete[port] = false;
        modbus_port_serial_enable(port, false, true);
    } else {
        //We were too late, back to receive
        return MODBUS_STAT_IO_ERROR;
    }


    return MODBUS_STAT_SUCCESS;
}

/*!
* @brief Lets the RTU system know there is no response to the last receive
*
* This simply flags that there will be no response to the last receive which
* will allow the receive to go back into idle mode and get a new packet.
*
* @param port The port to use for function
*/
void modbus_rtu_no_response(uint8_t port)
{
    // Flag our send as complete
    frameTxComplete[port] = true;

    //Release the buffer back to the receiver
    modbus_rtu_release_tx_buffer(port);
}

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
bool modbus_rtu_get_tx_buffer(uint8_t port, uint8_t **buf)
{
    bool getResult;

    //Disable our interrupts so we can get an accurate reading
    modbus_port_enable_interrupts(port, false);

    getResult = (((rxState[port] == STATE_RX_IDLE) || (rxState[port] == STATE_RX_COMPLETE) || (rxState[port] == STATE_RX_OFF)) && (txState[port] == STATE_TX_IDLE));

    //Put rx into off state if in an appropriate place so no messages are received
    if(getResult) {
        rxState[port] = STATE_RX_OFF;
        *buf = (uint8_t *)&RTUBuf[port][MODBUS_SER_PDU_PDU_OFF];
    } else {
        *buf = NULL;
    }

    //Reenable the interrupts
    modbus_port_enable_interrupts(port, true);

    return getResult;
}

/*!
* @brief Releases a buffer from the transmit state
*
* Used when in master mode to release the buffer after a response is
* received and handled.  It does so without flagging the frame complete
* since we do not care in master mode.
*
* @param port The port to free the buffer for
*/
void modbus_rtu_release_tx_buffer(uint8_t port)
{
    //Change our receiver back to idle mode
    if((rxState[port] == STATE_RX_COMPLETE) || (rxState[port] == STATE_RX_OFF)) {
        rxState[port] = STATE_RX_IDLE;
    }

    // Disable transmitter. This prevents another transmit buffer empty interrupt.
    modbus_port_serial_enable(port, true, false);
    txState[port] = STATE_TX_IDLE;
}

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
bool modbus_rtu_transmit_complete(uint8_t port)
{
    if(frameTxComplete[port]) {
        frameTxComplete[port] = false;
        return true;
    }

    return false;
}

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
bool modbus_rtu_has_frames(uint8_t port)
{
    return ((rxState[port] == STATE_RX_COMPLETE) | frameTxComplete[port]);
}
