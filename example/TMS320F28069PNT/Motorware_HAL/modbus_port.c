/*!
* @file modbus_port.c
* @author Jarrod Cook
*
* @brief Implementation of the board specific functionality for MODBUS
*
* This file is the implementation of routines to handle the UART output
* and timer implementation as well as function handlers for MODBUS.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includesl
#include "modbus_port.h"
#include "../project.h"


extern HAL_Handle      handle;       //!< the handle for the hardware abstraction layer (HAL)

//Local Variables
//avr32_usart_ier_t usartIntState;
//static bool curRxEnable, curTxEnable;       //!< These track the current state of the TX and RX enable lines
static volatile bool inTimInterrupt = false;    //!< Tracks when we are in the timer interrupt to avoid disables
static volatile bool inTxSciInterrupt = false;  //!< Tracks when we are in the TXSci interrupt to avoid disables
static volatile bool inRxSciInterrupt = false;  //!< Tracks when we are in the RXSci interrupt to avoid disables
static volatile bool inSCIInterrupt = false;    //!< Tracks when we are in the SCI interrupt to avoid disables
static bool waiting_for_complete = false;
static bool Enabled;
void (*rxIntFunc)(uint16_t port);            //!< The Receive Callback function
void (*txIntFunc)(uint16_t port);            //!< The Transmit Callback function
void (*timeoutIntFunc)(uint16_t port);       //!< The Timeout Callback function

/*
static void port_loopback_test(void) {
    if(SCI_rxDataReady(handle->sciAHandle))
    {
    while(SCI_rxDataReady(handle->sciAHandle) == 0);
    dataRx = SCI_getDataNonBlocking(handle->sciAHandle, &success);
    success = SCI_putDataNonBlocking(handle->sciAHandle, dataRx);
    }
}
*/

//! \brief the ISR for SCI-A receive interrupt
__interrupt void sciARxISR(void)
{
    inRxSciInterrupt = true;

    HAL_Obj *obj = (HAL_Obj *)handle;

    rxIntFunc(MB_PORT_NUM);

    //success = SCI_putDataNonBlocking(handle->sciAHandle, dataRx);
    SCI_clearRxFifoOvf(obj->sciAHandle);
    SCI_clearRxFifoInt(obj->sciAHandle);

    // acknowledge interrupt from SCI group so that SCI interrupt
    // is not received twice
    PIE_clearInt(obj->pieHandle,PIE_GroupNumber_9);
    inRxSciInterrupt = false;
} // end of sciARxISR() function


//! \brief the ISR for SCI-A receive interrupt
__interrupt void sciATxISR(void)
{
    inTxSciInterrupt = true;

    HAL_Obj *obj = (HAL_Obj *)handle;

    txIntFunc(MB_PORT_NUM);

    SCI_clearTxFifoInt(obj->sciAHandle);
    // acknowledge interrupt from SCI group so that SCI interrupt
    // is not received twice

    PIE_clearInt(obj->pieHandle,PIE_GroupNumber_9);

    inTxSciInterrupt = false;
} // end of sciARxISR() function


__interrupt void cpu_timer0_isr(void)
{
    inTimInterrupt = true;
    HAL_Obj *obj = (HAL_Obj *)handle;

    if (waiting_for_complete) {
        txIntFunc(MB_PORT_NUM);
        waiting_for_complete = false;
    }
    else{
        timeoutIntFunc(MB_PORT_NUM);
    }

    HAL_acqTimerInt(handle, 0);
    // Acknowledge interrupt from PIE group 1
    PIE_clearInt(obj->pieHandle,PIE_GroupNumber_1);

    inTimInterrupt = false;
}


/*!
* @brief This function initializes the appropriate hardware to perform communications
*
* This function will initialize a hardware timer to determine packet timeout and ends
* as well as initializing the hardware modules and setup the medium on which to perform
* communications.  After initialization all ports and modules will be left in the disabled
* states.
*
* @param port The port to initialize
* @param baudRate The baud rate to use for communication
* @param charSize The character size to use in communication
* @param parity The parity to use for communication
* @param stopBits The number of stop bits to use in communication
* @param timeoutBits The number of bits that indicate the end of the frame
* @param rxCallback Function to call when a byte is received
* @param txCallback Function to call when a new transmit byte is needed
* @param timeoutCallback Function to call if a timeout occurs
*
* @return Boolean indicating success of port initialization
*/
bool modbus_port_init(uint16_t port, uint32_t baudRate, uint16_t charSize, modbus_parity parity, uint16_t stopBits, uint16_t timeoutBits,
                      void (*rxCallback)(uint16_t port), void (*txCallback)(uint16_t port), void (*timeoutCallback)(uint16_t port))
{
    int32_t test, psc, timeArr, mbTimeout;
    SCI_CharLength_e i;
    HAL_Obj *obj = (HAL_Obj *)handle;

    //We only have one port!
    if(port != MB_PORT_NUM) {
        return false;
    }

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;

    //Register our interrupt handler
    HAL_disableGlobalInts(handle);
    Enabled = false;
    inTxSciInterrupt = false;
    inRxSciInterrupt = false;
    inTimInterrupt = false;

    // Set Control Line to Output


    //Turn the bits into timer clocks (time clock = Sysclk/AHB * 2 /APB1)
    //mbTimeout = ((int)timeoutBits * ((int)(((SystemCoreClock/CLK_AHB_DIVIDER_VAL)*2)/CLK_APB1_DIVIDER_VAL)/(int)baudRate));
    mbTimeout = (int)timeoutBits * (90000000/(baudRate));

    //Now lets find a prescaler and ARR that works
    test = -1;
    psc = 0;
    do {
        test++;
        if(mbTimeout%(test+1) == 0) {
            psc = test;                  // subtract 1 per TI datasheet
        }
    } while(((mbTimeout/(test+1)) > 1000) && (psc < 65536));
    timeArr = (mbTimeout/(psc+1));

    TIMER_setDecimationFactor(obj->timerHandle[0],0);
    TIMER_setPeriod(obj->timerHandle[0],timeArr);
    TIMER_setPreScaler(obj->timerHandle[0],psc);

    SCI_reset(obj->sciAHandle);

    if (parity == MODBUS_PARITY_NONE){
        SCI_disableParity(obj->sciAHandle);
    } else if (parity == MODBUS_PARITY_ODD) {
        SCI_setParity(obj->sciAHandle, SCI_Parity_Odd);
    } else {
        SCI_setParity(obj->sciAHandle, SCI_Parity_Even);
    }

    // Set number of stop bits
    if (!stopBits){
        SCI_setNumStopBits(obj->sciAHandle,SCI_NumStopBits_One);
    } else {
        SCI_setNumStopBits(obj->sciAHandle,SCI_NumStopBits_Two);
    }

    // Set character size
    for(i=SCI_CharLength_1_Bit; i<=SCI_CharLength_8_Bits; i++) {
        if ((charSize - 1) == i){
            SCI_setCharLength(obj->sciAHandle,i);
            break;
        }
    }

    /*
    // Baudrate setup
    switch(baudRate){
        case 9600:
            SCI_setBaudRate(obj->sciAHandle, SCI_BaudRate_9_6_kBaud);
        case 19200:
            SCI_setBaudRate(obj->sciAHandle, SCI_BaudRate_19_2_kBaud);
        case 57600:
            SCI_setBaudRate(obj->sciAHandle, SCI_BaudRate_57_6_kBaud);
        case 115200:
            SCI_setBaudRate(obj->sciAHandle, SCI_BaudRate_115_2_kBaud);
            break;
        default:
            error();
    }
    */
    int32_t temp_baud = (((90000000)/((baudRate*8)))-1);
    obj->sciAHandle->SCIHBAUD = ((uint16_t)temp_baud >> 8);
    obj->sciAHandle->SCILBAUD = temp_baud & 0xFF;

    //SCI_setPriority(obj->sciAHandle,SCI_Priority_FreeRun);


    // Setup UART Interrupts
    SCI_disableTxInt(obj->sciAHandle);
    SCI_disableRxInt(obj->sciAHandle);
    SCI_enableChannels(obj->sciAHandle);
    SCI_enableTxFifoEnh(obj->sciAHandle);
    SCI_setTxFifoIntLevel(obj->sciAHandle, SCI_FifoLevel_Empty);
    SCI_setRxFifoIntLevel(obj->sciAHandle, SCI_FifoLevel_1_Word);
    SCI_setTxDelay(obj->sciAHandle, 0x00);
    HAL_enableGlobalInts(handle);
    Enabled = true;

    //Enable The UART
    SCI_enable(obj->sciAHandle);
    SCI_enableRx(obj->sciAHandle);
    SCI_enableTx(obj->sciAHandle);
    SCI_resetTxFifo(obj->sciAHandle);
    SCI_resetRxFifo(obj->sciAHandle);

    modbus_port_serial_enable(port,false,false);
    return true;
}

/*!
* @brief This function enables the serial port as desired for proper communication
*
* This function will set the enable/disable status of the rx and tx lines
* to their appropriate values for communicating.  They are present as 2
* values mostly for the ability to disable both and gain the power savings
* of the chip progressing into low power mode.
*
* @param port The port to perform enables on
* @param rxEnable True will enable receiving on the port
* @param txEnable True will enable transmitting on the port
*
* @return Boolean indicating success of port initialization
*/
void modbus_port_serial_enable(uint16_t port, bool rxEnable, bool txEnable)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

    //We only have one port, if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //If we interrupt while we have our flags off all hell will break loose
    if(Enabled) {
        HAL_disableGlobalInts(handle);
        Enabled = false;
    }

    //Set the RX up
    if(rxEnable) {
        //Configure the USART rx interrupt
        //usartIntState.usart_mode.rxrdy = 1;
        GPIO_setLow(obj->gpioHandle, GPIO_Number_29); // RE enabled
        SCI_enableRx(obj->sciAHandle);
        SCI_enableRxFifoInt(obj->sciAHandle);
        HAL_acqTimerInt(handle, 0);
        HAL_startTimer(handle, 0);
    }
    else {
        SCI_disableRxFifoInt(obj->sciAHandle);
        SCI_disableRx(obj->sciAHandle);
        HAL_stopTimer(handle, 0);
    }

    //Set the TX up
    if(txEnable && !rxEnable) {
        GPIO_setHigh(obj->gpioHandle, GPIO_Number_29); // DE enabled
        SCI_enableTx(obj->sciAHandle);
        SCI_enableTxFifoInt(obj->sciAHandle);
    }
    else {
        SCI_disableTxFifoInt(obj->sciAHandle);
        SCI_disableTx(obj->sciAHandle);
    }

    //Setup the interrupts
    //usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(!Enabled) {
        HAL_enableGlobalInts(handle);
        Enabled = true;
    }
}

/*!
* @brief This function completely closes out the port
*
* This function is used in case we decide to stop using MODBUS.  It will
* completely deinitialize the module and shut it off to save power.
*
* @param port The port to close down
*/
void modbus_port_serial_close(uint16_t port)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Disable all the interrupts
    if(Enabled) {
        HAL_disableGlobalInts(handle);
        Enabled = false;
    }

    obj->sciAHandle->SCICTL2 &= (~SCI_SCICTL2_TXRDY_BITS);
    obj->sciAHandle->SCICTL2 &= (~SCI_SCICTL2_TXEMPTY_BITS);

    //usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(!Enabled) {
        HAL_enableGlobalInts(handle);
        Enabled = true;
    }

    //Turn off RX and TX
    SCI_disableTx(obj->sciAHandle);
    SCI_disableRx(obj->sciAHandle);


    //Turn all our pins to input to go to power save
    GPIO_setHigh(obj->gpioHandle, GPIO_Number_29); // DE enabled
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_28, GPIO_Direction_Input);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_12, GPIO_Direction_Input);

    SCI_resetRxFifo(obj->sciAHandle);
    SCI_resetTxFifo(obj->sciAHandle);

    //Turn the Clock off
    CLK_disableSciaClock(obj->clkHandle);
}

/*!
* @brief This function changes the TX interrupt to fire when transfer is complete
*
* After queueing up the last byte to transfer this function should be called
* so that the next interrupt only occurs when the transfer is completely
* finished.  That way we don't disable transmitting too early.
*
* @param port The port to change the state of
*/
void modbus_port_notify_of_tx_completion(uint16_t port)
{
    waiting_for_complete = true;

    HAL_Obj *obj = (HAL_Obj *)handle;
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    SCI_disableTxFifoInt(obj->sciAHandle);
    HAL_reloadTimer(handle, 0);
    HAL_startTimer(handle, 0);

}

/*!
* @brief This function will disable/enable the modbus interrupts
*
* This function will both enable and disable the interrupts for modbus.
* this allows interrupts to be disabled for critical sections to prevent
* race conditions.
*
* @param port The port to set enable status of interrupts for
* @param enable True will cause interrupts to be enabled and false disabled
*/
void modbus_port_enable_interrupts(uint16_t port, bool enable)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Since all functionality is within the UART interrupt then don't bother
    //disabling if we are in the interrupt routine.
    if((!inTimInterrupt | !inTxSciInterrupt | !inRxSciInterrupt) && enable) {
        PIE_enableInt(obj->pieHandle,PIE_GroupNumber_1,PIE_InterruptSource_TIMER_0);
        PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIARX);
        PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIATX);
        Enabled = true;
    } else if(!inTimInterrupt | !inTxSciInterrupt | !inRxSciInterrupt) {
        PIE_disableInt(obj->pieHandle,PIE_GroupNumber_1,PIE_InterruptSource_TIMER_0);
        PIE_disableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIARX);
        PIE_disableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIATX);
        Enabled = false;
    }
}

/*!
* @brief This writes a byte to the port
*
* This function will write the given byte to the port to be sent out and
* immediately return.  Transmission complete will be denoted by an interrupt.
*
* @param port The port to send the byte over
* @param sndByte The byte to send
*
* @return Boolean indicating if the write was successful
*/
bool modbus_port_put_byte(uint16_t port, uint16_t sndByte)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return false;
    }

    return (SCI_putDataNonBlocking(obj->sciAHandle, sndByte) == 1);

    return 0;
}

/*!
* @brief This reads a byte from the port
*
* This function will read a byte from the port if one exists.  If none exists
* or an error has occurred a false will be returned.
*
* @param port The port to get the byte on
* @param getByte Pointer to place the retrieved byte in
*
* @return Boolean indicating if a byte was successfully received and read.
*/
bool modbus_port_get_byte(uint16_t port, uint16_t *getByte)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t inWord;
    uint16_t res;
    //We only have one port, so if not it abort
    if(port != MB_PORT_NUM) {
        return false;
    }

    if(!(obj->sciAHandle->SCIRXST == (uint16_t)(1 << 6)) || !(obj->sciAHandle->SCIRXST == (uint16_t)(1 << 5))) {
        //Read the character
        res = SCI_getDataNonBlocking(obj->sciAHandle, &inWord);
        //res = sciaRxFifo(&inWord);
        if(inWord == true) {
            *getByte = (res & 0x00FF);
            return true;
        }
    }
    else{
        SCI_reset(obj->sciAHandle);
        DELAY_US(10);
        SCI_enable(obj->sciAHandle);
    }
    return false;
}

/*!
* @brief This enables/disables the timer to detect frame ends
*
* This function will both enable and disable the frame timer.  Sending the
* enable command will reset the timer to 0 and start it.  Upon reaching the
* set value it will fire an interrupt.  Calling disable will immediately
* disable the timer and clear any pending interrupts.
*
* @param port the port to enable the timer on
* @param enable True will enable, reset, and start timer and false will disable
*/
void modbus_port_timer_enable(uint16_t port, bool enable)
{
    //We only have one port if not it abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Enable or disable as requested
    if(enable) {
        HAL_reloadTimer(handle, 0);
        HAL_startTimer(handle, 0);
    }
        //THIS TIMER auto resets on char received so no need for an ELSE to reload it here
    else {
        HAL_stopTimer(handle, 0);
    }
}
