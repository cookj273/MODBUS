#include "modbus_port.h"
#include "modbus_tcp.h"

//Only implement Modbus TCP if enabled
#if MODBUS_TCP_ENABLED > 0

/* ----------------------- MBAP Header --------------------------------------*/
/*
 *
 * <------------------------ MODBUS TCP/IP ADU(1) ------------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+------------------------------------------+
 *  | TID | PID | Length | UID  |Code | Data                               |
 *  +-----------+---------------+------------------------------------------+
 *  |     |     |        |      |
 * (2)   (3)   (4)      (5)    (6)
 *
 * (2)  ... MB_TCP_TID          = 0 (Transaction Identifier - 2 Byte)
 * (3)  ... MB_TCP_PID          = 2 (Protocol Identifier - 2 Byte)
 * (4)  ... MB_TCP_LEN          = 4 (Number of bytes - 2 Byte)
 * (5)  ... MB_TCP_UID          = 6 (Unit Identifier - 1 Byte)
 * (6)  ... MB_TCP_FUNC         = 7 (Modbus Function Code)
 *
 * (1)  ... Modbus TCP/IP Application Data Unit
 * (1') ... Modbus Protocol Data Unit
 */

//*****LOCAL DEFINES
#define MB_TCP_TID          0   //Header Byte Position of the 2-Byte Transaction Identifier
#define MB_TCP_PID          2   //Header Byte Position of the 2-Byte Protocol Identifier
#define MB_TCP_LEN          4   //Header Byte Position of the 2-Byte Length Specifier
#define MB_TCP_UID          6   //Header Byte Position of the 1-Byte Unit Identifier
#define MB_TCP_FUNC         7   //Start of the actual Modbus PDU data (starting with function code)

#define MB_TCP_PROTOCOL_ID  0   // Modbus Protocol Identifier => 0 = Modbus Protocol
#define MB_TCP_TIMEOUT_SEC  1

//******LOCAL VARIABLES
static unsigned short tcpPackLength[NUM_MODBUS_PORTS] = {0};        //Length of packet being received
static unsigned char isMaster[NUM_MODBUS_PORTS] = {false};          //Indicates if this is a master or slave (for addressing)

/*!
* @brief Closes out the port when shut off
*
* This will close out the port and shut everything off when we decide to
* turn MODBUS off.
*
* @param port The port to use for function
*/
void modbus_tcp_close(uint8_t port)
{
    /* Make sure that no more clients are connected. */
    modbus_port_serial_close(port);
}

/*********************************************************************
* Function: MB_ERROR tcpReceive(BYTE* rcvAddress, BYTE** inFrame, unsigned short* inLength)
* PreCondition: TCP Modbus communications must have been initialized
* Input: rcvAddress - Pointer to place the slave address into
*        inFrame - Pointer to place the received data frame into
*        inLength - Pointer to place the length of the data frame into
* Output: MB_ERROR - Indicates whether or not a valid Modbus packet has been received over the port.
* Description: Called when a complete modbus packet has been received over TCP.  Checks validation of
*               the packet and places it into the given buffers for processing.
********************************************************************/
modbus_status modbus_tcp_receive(uint8_t port, uint8_t *rcvAddress, uint8_t **inFrame, uint16_t *inLength)
{
    modbus_status status = MODBUS_STAT_IO_ERROR;
    unsigned short tcpPID;

    if(rxState[port] != STATE_RX_COMPLETE) {
        return MODBUS_STAT_NO_DATA;
    }

    tcpPID = mbDataBuf[port][MB_TCP_PID] << 8U;
    tcpPID |= mbDataBuf[port][MB_TCP_PID + 1];

    if((portMode[port] == MODBUS_TCP) && (tcpPID == MB_TCP_PROTOCOL_ID)) {
        *inFrame = (uint8_t *)&mbDataBuf[port][MB_TCP_FUNC];
        *inLength = rxBufferPos[port] - MB_TCP_FUNC;
        status = MODBUS_STAT_SUCCESS;

        /* Modbus TCP uses IP Address already so just hard-code UID. Fake the source address such
         * that the processing part deals with this frame in slave mode.
         */
        if(isMaster[port]) {
            *rcvAddress = mbDataBuf[port][MB_TCP_UID];
        } else {
            *rcvAddress = MB_TCP_PSEUDO_ADDRESS;
        }
    }

    return status;
}

/*********************************************************************
* Function: MB_ERROR tcpSend(BYTE _unused, const BYTE* inFrame, unsigned short inLength)
* PreCondition: TCP Modbus communications must have been initialized
* Input: _unused - Place holder for slave address which is unused by TCP to mesh up with the other protocols
*        inFrame - The frame of data to be sent out over TCP
*        inLength - The length of the data to be sent out.
* Output: MB_ERROR - Indicates whether or not the packet was successfully set to send
* Description: This function packs a response packet with the header and sends it out over the TCP port.
********************************************************************/
modbus_status modbus_tcp_transmit(uint8_t port, uint8_t slaveAddress, const uint8_t *inFrame, uint16_t inLength)
{
    uint8_t* tcpFrame = (uint8_t*)(inFrame - MB_TCP_FUNC);
    uint16_t tcpLength = (inLength + MB_TCP_FUNC);
    uint16_t i;

    //Put rx into off state if in an appropriate place so no messages are received
    if((rxState[port] == STATE_RX_IDLE) || (rxState[port] == STATE_RX_COMPLETE)) {
        rxState[port] = STATE_RX_OFF;
    }

    /* The MBAP header is already initialized because the caller calls this
     * function with the buffer returned by the previous call. Therefore we
     * only have to update the length in the header. Note that the length
     * header includes the size of the Modbus PDU and the UID Byte. Therefore
     * the length is inLength plus one.
     */
    tcpFrame[MB_TCP_LEN] = (inLength + 1) >> 8U;
    tcpFrame[MB_TCP_LEN + 1] = (inLength + 1) & 0xFF;
    if(isMaster[port]) {
        tcpFrame[MB_TCP_UID] = slaveAddress;
        if(tcpFrame[MB_TCP_TID + 1] == 0xFF) {
            tcpFrame[MB_TCP_TID]++;
        }
        tcpFrame[MB_TCP_TID + 1]++;
    }
    if((portMode[port] == MODBUS_TCP) && ((inLength+7) < MODBUS_TCP_PDU_SIZE_MAX)) {
        for(i=0; i < tcpLength; i++) {
            mbDataBuf[port][i] = tcpFrame[i];
        }
        txBufferCount[port] = tcpLength;
        txBufferCur[port] = mbDataBuf[port];
        txState[port] = STATE_TX_XMIT;
        modbus_tcp_set_tx_slave(port, slaveAddress);
        frameTxComplete[port] = false;
    } else {
        return MODBUS_STAT_IO_ERROR;
    }

    return MODBUS_STAT_SUCCESS;
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
void modbus_tcp_release_tx_buffer(uint8_t port)
{
    //Change our receiver back to idle mode
    if((rxState[port] == STATE_RX_COMPLETE) || (rxState[port] == STATE_RX_OFF)) {
        rxState[port] = STATE_RX_IDLE;
    }

    // Disable transmitter. This prevents another transmit buffer empty interrupt.
    txState[port] = STATE_TX_IDLE;
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
bool modbus_tcp_get_tx_buffer(uint8_t port, uint8_t **buf)
{
    bool getResult;

    getResult = (((rxState[port] == STATE_RX_IDLE) || (rxState[port] == STATE_RX_COMPLETE) || (rxState[port] == STATE_RX_OFF)) && (txState[port] == STATE_TX_IDLE));

    //Put rx into off state if in an appropriate place so no messages are received
    if(getResult) {
        rxState[port] = STATE_RX_OFF;
        *buf = (uint8_t *)&mbDataBuf[port][MB_TCP_FUNC];
    } else {
        *buf = NULL;
    }

    return getResult;
}

/*!
* @brief Lets the RTU system know there is no response to the last receive
*
* This simply flags that there will be no response to the last receive which
* will allow the receive to go back into idle mode and get a new packet.
*
* @param port The port to use for function
*/
void modbus_tcp_no_response(uint8_t port)
{
    // Flag our send as complete
    frameTxComplete[port] = true;

    //Release the buffer back to the receiver
    modbus_tcp_release_tx_buffer(port);
}

/*********************************************************************
* Function: BOOL tcpReceiveFSM(void)
* PreCondition: Modbus TCP Communications must have been initialized
* Input: none
* Output: BOOL - indicates a new frame received
* Description: This is simply the FSM for receiving in the bytes of a frame and is called by the
*               TCP Receive routine.
********************************************************************/
void modbus_tcp_receive_fsm(uint8_t port)
{
    uint8_t inByte;

    /* Always read the character. */
    if(!modbus_port_get_byte(port, (uint8_t *) &inByte) && (rxState[port] != STATE_RX_COMPLETE) && (rxState[port] != STATE_RX_OFF)) {
        //We had some sort of error
        rxState[port] = STATE_RX_ERROR;
    }

    switch(rxState[port]) {
    case STATE_RX_IDLE:
        rxBufferPos[port] = 0;
        mbDataBuf[port][rxBufferPos[port]++] = inByte;
        rxState[port] = STATE_RX_RCV;

        modbus_port_timer_enable(port, true);
        break;
    case STATE_RX_RCV:
        mbDataBuf[port][rxBufferPos[port]++] = inByte;
        if(rxBufferPos[port] == 5) {
            if(((((uint16_t)mbDataBuf[port][rxBufferPos[port]-2]<<8)&0xFF00) | ((uint16_t)mbDataBuf[port][rxBufferPos[port]-1]&0x00FF)) != MB_TCP_PROTOCOL_ID) {
                rxState[port] = STATE_RX_IDLE;
                modbus_port_timer_enable(port, false);
            }
        } else if(rxBufferPos[port] == 6) {
            tcpPackLength[port] = ((((uint16_t)mbDataBuf[port][rxBufferPos[port]-2]<<8) & 0xFF00) | ((uint16_t)mbDataBuf[port][rxBufferPos[port]-1] & 0x00FF))+6;
        } else if((rxBufferPos[port] > 6) && (rxBufferPos[port] >= tcpPackLength[port])) {
            modbus_port_timer_enable(port, false);
            rxState[port] = STATE_RX_COMPLETE;
        }
        break;
    case STATE_RX_COMPLETE:
    case STATE_RX_OFF:
        break;
    default:
        rxState[port] = STATE_RX_ERROR;
        break;
    }
}

/*********************************************************************
* Function: BOOL tcpTransmitFSM(void)
* PreCondition: Modbus TCP Communications must have been initialized
* Input: none
* Output: BOOL - indicates a complete response packet has been sent.
* Description: This is simply the FSM for transmitting the bytes of a response packet and
*               is called by the TCP check for a free buffer.
********************************************************************/
void modbus_tcp_transmit_fsm(uint8_t port)
{
    switch(txState[port]) {
    case STATE_TX_XMIT:
        // check if we are finished.
        if(txBufferCount[port] != 0) {
            modbus_port_put_byte(port, (uint8_t)*txBufferCur[port]);
            txBufferCur[port]++;  // next byte in sendbuffer.
            txBufferCount[port]--;
        } else {
            modbus_port_notify_of_tx_completion(port);
            txState[port] = STATE_TX_WAIT;
        }
        break;
    case STATE_TX_WAIT:
        // Flag our send as complete
        frameTxComplete[port] = true;

        //Change our receiver back to idle mode
        if((rxState[port] == STATE_RX_COMPLETE) || (rxState[port] == STATE_RX_OFF)) {
            rxState[port] = STATE_RX_IDLE;
        }

        // Disable transmitter. This prevents another transmit buffer empty interrupt.
        txState[port] = STATE_TX_IDLE;
        break;
    default:
        break;
    }
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
bool modbus_tcp_transmit_complete(uint8_t port)
{
    if(frameTxComplete[port]) {
        frameTxComplete[port] = false;
        return true;
    }

    return false;
}

void modbus_tcp_timer_expired(uint8_t port)
{
    if((rxState[port] != STATE_RX_COMPLETE) && (rxState[port] != STATE_RX_OFF)) {
        rxState[port] = STATE_RX_IDLE;
    }

    modbus_port_timer_enable(port, false);
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
bool modbus_tcp_has_frames(uint8_t port)
{
    return ((rxState[port] == STATE_RX_COMPLETE) || frameTxComplete[port]);
}

modbus_status modbus_tcp_init(uint8_t mbPort, uint16_t tcpPort, modbus_device_types deviceType)
{
    frameTxComplete[mbPort] = false;
    if(!modbus_port_tcp_init(mbPort, tcpPort, MB_TCP_TIMEOUT_SEC, deviceType, modbus_tcp_receive_fsm, modbus_tcp_transmit_fsm, modbus_tcp_timer_expired)) {
        return MODBUS_STAT_PORT_ERROR;
    }
    portMode[mbPort] = MODBUS_TCP;
    isMaster[mbPort] = (deviceType == MODBUS_MASTER_DEVICE);

    return MODBUS_STAT_SUCCESS;
}

#endif
