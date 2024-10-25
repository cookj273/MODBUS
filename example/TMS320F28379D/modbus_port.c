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
#include "../device/device.h"
#include "../device/driverlib.h"

#define USE_ECAP
//#define USE_TIMER

#define MODBUS_SCI_BASE SCIB_BASE

#if (MODBUS_SCI_BASE == SCIA_BASE)
#define MODBUS_SCI_RX_INT INT_SCIA_RX
#define MODBUS_SCI_TX_INT INT_SCIA_TX
#define MODBUS_SCI_ACK_GROUP INTERRUPT_ACK_GROUP9
#elif (MODBUS_SCI_BASE == SCIB_BASE)
#define MODBUS_SCI_RX_INT INT_SCIB_RX
#define MODBUS_SCI_TX_INT INT_SCIB_TX
#define MODBUS_SCI_ACK_GROUP INTERRUPT_ACK_GROUP9
#elif (MODBUS_SCI_BASE == SCIC_BASE)
#define MODBUS_SCI_RX_INT INT_SCIC_RX
#define MODBUS_SCI_TX_INT INT_SCIC_TX
#define MODBUS_SCI_ACK_GROUP INTERRUPT_ACK_GROUP8
#elif (MODBUS_SCI_BASE == SCID_BASE)
#define MODBUS_SCI_RX_INT INT_SCID_RX
#define MODBUS_SCI_TX_INT INT_SCID_TX
#define MODBUS_SCI_ACK_GROUP INTERRUPT_ACK_GROUP8
#endif

//Local Variables
static volatile bool inTimInterrupt = false;    //!< Tracks when we are in the timer interrupt to avoid disables
static volatile bool inTxSciInterrupt = false;  //!< Tracks when we are in the TXSci interrupt to avoid disables
static volatile bool inRxSciInterrupt = false;  //!< Tracks when we are in the RXSci interrupt to avoid disables
static bool waiting_for_complete = false;
static bool Enabled;
void (*rxIntFunc)(uint16_t port);            //!< The Receive Callback function
void (*txIntFunc)(uint16_t port);            //!< The Transmit Callback function
void (*timeoutIntFunc)(uint16_t port);       //!< The Timeout Callback function

__interrupt void modbusRxISR(void);
__interrupt void modbusTxISR(void);
#ifdef USE_TIMER
__interrupt void cpuTimer0ISR(void);
#endif
#ifdef USE_ECAP
__interrupt void ecapISR(void);
#endif


//static void port_loopback_test(void) {
//    uint16_t dataRx = 0;
//    if(SCI_getRxStatus(MODBUS_SCI_BASE) && (SCI_RXST_RXRDY)) {
//        while((SCI_getRxStatus(MODBUS_SCI_BASE) && SCI_RXST_RXRDY) == 0);
//        dataRx = SCI_readCharNonBlocking(MODBUS_SCI_BASE);
//        SCI_writeCharNonBlocking(MODBUS_SCI_BASE, dataRx);
//    }
//}

/*!
* @name sciACheck
* @brief Checks for an error or break condition and resets the receiver
*
* SCIAInit must be called before this function can be used.
* This function checks the break and error flags and if either is set
* it resets the module.
*/

//void sciACheck(void)
//{
//    if(SCI_getRxStatus(MODBUS_SCI_BASE) && (SCI_RXST_RXERROR | SCI_RXST_BRKDT)) {
//        SCI_performSoftwareReset(MODBUS_SCI_BASE);
//        DEVICE_DELAY_US(10);
//        SCI_enableModule(MODBUS_SCI_BASE);
//    }
//}


//! \brief the ISR for MODBUS_SCI receive interrupt
__interrupt void modbusRxISR(void)
{
    inRxSciInterrupt = true;

    rxIntFunc(MB_PORT_NUM);

    //success = SCI_putDataNonBlocking(MODBUS_SCI_BASE, dataRx);
    SCI_clearOverflowStatus(MODBUS_SCI_BASE);

    SCI_clearInterruptStatus(MODBUS_SCI_BASE, SCI_INT_RXFF);

    //
    // Issue PIE ack
    //
    Interrupt_clearACKGroup(MODBUS_SCI_ACK_GROUP);

    inRxSciInterrupt = false;
} // end of sciARxISR() function


//! \brief the ISR for MODBUS_SCI transmit interrupt
__interrupt void modbusTxISR(void)
{
    inTxSciInterrupt = true;

    txIntFunc(MB_PORT_NUM);

    SCI_clearInterruptStatus(MODBUS_SCI_BASE, SCI_INT_TXFF);

    //
    // Issue PIE ACK
    //
    Interrupt_clearACKGroup(MODBUS_SCI_ACK_GROUP);

    inTxSciInterrupt = false;
} // end of sciARxISR() function

#ifdef USE_TIMER
__interrupt void cpuTimer0ISR(void)
{
    inTimInterrupt = true;

    if (waiting_for_complete) {
        txIntFunc(MB_PORT_NUM);
        waiting_for_complete = false;
    }
    else{
        timeoutIntFunc(MB_PORT_NUM);
    }

    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    // Acknowledge interrupt from PIE group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    inTimInterrupt = false;
}
#endif

#ifdef USE_ECAP
__interrupt void ecapISR(void)
{
    inTimInterrupt = true;

    if (waiting_for_complete) {
        txIntFunc(MB_PORT_NUM);
        waiting_for_complete = false;
    }
    else{
        timeoutIntFunc(MB_PORT_NUM);
    }

    ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_COUNTER_PERIOD);
    ECAP_clearGlobalInterrupt(ECAP1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

    inTimInterrupt = false;
}
#endif


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
    uint16_t i = 0;
    int32_t test = 0;
    uint32_t timeArr, psc, mbTimeout = 0;
    //uint32_t tempBaud;
    uint16_t tempStopBits, tempCharSize, tempParity = 0;

    //We only have one port!
    if(port != MB_PORT_NUM) {
        return false;
    }

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;

    Interrupt_register(MODBUS_SCI_RX_INT, modbusRxISR);
    Interrupt_register(MODBUS_SCI_TX_INT, modbusTxISR);
#ifdef USE_TIMER
    Interrupt_register(INT_TIMER0, cpuTimer0ISR);
#endif

#ifdef USE_ECAP
    Interrupt_register(INT_ECAP1, ecapISR);
#endif


    //Register our interrupt handler
    DINT;
    Enabled = false;
    inTxSciInterrupt = false;
    inRxSciInterrupt = false;
    inTimInterrupt = false;

    // Use first time to get timeout value for timer. Should 1.75 ms for timeout value.
    //Turn the bits into timer clocks (time clock = Sysclk/AHB * 2 /APB1)
    //mbTimeout = ((int)timeoutBits * ((int)(((SystemCoreClock/CLK_AHB_DIVIDER_VAL)*2)/CLK_APB1_DIVIDER_VAL)/(int)baudRate));
    mbTimeout = (int)timeoutBits * (DEVICE_SYSCLK_FREQ/(baudRate));

    //Now lets find a prescaler and ARR that works
    test = -1;
    psc = 0;
    do {
        test++;
        if(mbTimeout%(test+1) == 0) {
            psc = test;                  // subtract 1 per TI datasheet
        }
    // LINE WHILE LOOP UP WITH BITS OF TIMER. THIS TIMER IS 32 Bits
    } while(((mbTimeout/(test+1)) > 1000000) && (psc < 4294967296));
    timeArr = (mbTimeout/(psc+1));

#ifdef USE_TIMER
    CPUTimer_setPeriod(CPUTIMER0_BASE, timeArr);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, psc);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPATZERO);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
#endif

#ifdef USE_ECAP
    XBAR_setOutputMuxConfig(XBAR_OUTPUT3, XBAR_OUT_MUX00_ECAP1_OUT);
    XBAR_enableOutputMux(XBAR_OUTPUT3, XBAR_MUX00);
    GPIO_setPadConfig(26U, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_26_OUTPUTXBAR3);

   ECAP_stopCounter(ECAP1_BASE);
   ECAP_enableAPWMMode(ECAP1_BASE);
   ECAP_setAPWMPeriod(ECAP1_BASE, timeArr);   // setup
   ECAP_setEventPrescaler(ECAP1_BASE, psc);
   ECAP_setAPWMCompare(ECAP1_BASE, timeArr/2);   // 1KHz sq. wave on GPIO 26
   ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_COUNTER_PERIOD);
#endif

    SCI_performSoftwareReset(MODBUS_SCI_BASE);

    if (parity == MODBUS_PARITY_NONE){
        tempParity = SCI_CONFIG_PAR_NONE;
    } else if (parity == MODBUS_PARITY_ODD) {
        tempParity = SCI_CONFIG_PAR_ODD;
    } else {
        tempParity = SCI_CONFIG_PAR_EVEN;
    }

    // Set number of stop bits
    if (!stopBits){
        tempStopBits = SCI_CONFIG_STOP_ONE;
    } else {
        tempStopBits = SCI_CONFIG_STOP_TWO;
    }

    // Set character size
    for(i=SCI_CONFIG_WLEN_1; i<=SCI_CONFIG_WLEN_8; i++) {
        if ((charSize - 1) == i){
            tempCharSize = i;
            break;
        }
    }


//    tempBaud = ((DEVICE_LSPCLK_FREQ/(baudRate*8))-1);
//    HWREGH(base + SCI_O_HBAUD) = (tempBaud & 0xFF00U) >> 8U;
//    HWREGH(base + SCI_O_LBAUD) = tempBaud & 0x00FFU;

    SCI_setConfig(MODBUS_SCI_BASE, DEVICE_LSPCLK_FREQ, baudRate, (tempCharSize | tempStopBits | tempParity));

    // Setup UART Interrupts
    SCI_disableInterrupt(MODBUS_SCI_BASE, (SCI_INT_RXERR | SCI_INT_TXFF | SCI_INT_RXFF | SCI_INT_RXRDY_BRKDT));
    SCI_resetChannels(MODBUS_SCI_BASE);
    SCI_setFIFOInterruptLevel(MODBUS_SCI_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1); // Empty for TX, One byte for RX
    EINT;
    Enabled = true;

    //Enable The UART
    SCI_enableModule(MODBUS_SCI_BASE);
    SCI_enableFIFO(MODBUS_SCI_BASE);
    SCI_resetTxFIFO(MODBUS_SCI_BASE);
    SCI_resetRxFIFO(MODBUS_SCI_BASE);

    modbus_port_serial_enable(port,false,false);

    // Sanity Check for Modbus Connection

    /*
    GPIO_writePin(45, 0); // DE enabled
    SCI_enableTxModule(MODBUS_SCI_BASE);
    SCI_enableInterrupt(MODBUS_SCI_BASE, SCI_INT_TXFF);
    EINT;
    ERTM;
    while(true){
        SCI_writeCharNonBlocking(MODBUS_SCI_BASE, 'A');
       // toggleHeartBeatLed(handle);
        DEVICE_DELAY_US(20000);
    }
    */

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

    //We only have one port, if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //If we interrupt while we have our flags off all hell will break loose
    if(Enabled) {
        DINT;
        Enabled = false;
    }

    //Set the RX up
    if(rxEnable) {
        //Configure the USART rx interrupt
        GPIO_writePin(45, 1); // RE enabled
        SCI_enableRxModule(MODBUS_SCI_BASE);
        SCI_enableInterrupt(MODBUS_SCI_BASE, SCI_INT_RXFF);
#ifdef USE_TIMER
        CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
        CPUTimer_startTimer(CPUTIMER0_BASE);
#endif
#ifdef USE_ECAP
        ECAP_reArm(ECAP1_BASE);
        ECAP_startCounter(ECAP1_BASE);
#endif
    }
    else {
        SCI_disableRxModule(MODBUS_SCI_BASE);
        SCI_disableInterrupt(MODBUS_SCI_BASE, SCI_INT_RXFF);
#ifdef USE_TIMER
        CPUTimer_stopTimer(CPUTIMER0_BASE);
#endif
#ifdef USE_ECAP
        ECAP_stopCounter(ECAP1_BASE);
#endif
    }

    //Set the TX up
    if(txEnable && !rxEnable) {
        GPIO_writePin(45, 0); // DE enabled
        SCI_enableTxModule(MODBUS_SCI_BASE);
        SCI_enableInterrupt(MODBUS_SCI_BASE, SCI_INT_TXFF);
    }
    else {
        SCI_disableTxModule(MODBUS_SCI_BASE);
        SCI_disableInterrupt(MODBUS_SCI_BASE, SCI_INT_TXFF);
    }

    //Setup the interrupts
    if(!Enabled) {
        EINT;
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
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Disable all the interrupts
    if(Enabled) {
        DINT;
        Enabled = false;
    }

    HWREGH(MODBUS_SCI_BASE + SCI_O_CTL1) &= ~(SCI_CTL2_TXEMPTY | SCI_CTL2_TXRDY);

    if(!Enabled) {
        EINT;
        Enabled = true;
    }

    //Turn off RX and TX
    SCI_disableTxModule(MODBUS_SCI_BASE);
    SCI_disableRxModule(MODBUS_SCI_BASE);

    //Turn all our pins to input to go to power save
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_IN);

    SCI_resetRxFIFO(MODBUS_SCI_BASE);
    SCI_resetTxFIFO(MODBUS_SCI_BASE);

    //Turn the Clock off
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
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

    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    SCI_disableInterrupt(MODBUS_SCI_BASE, SCI_INT_TXFF);
#ifdef USE_TIMER
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);
#endif
#ifdef USE_ECAP
    ECAP_reArm(ECAP1_BASE);
    ECAP_startCounter(ECAP1_BASE);
#endif
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
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Since all functionality is within the UART interrupt then don't bother
    //disabling if we are in the interrupt routine.
    if((!inTimInterrupt | !inTxSciInterrupt | !inRxSciInterrupt) && enable) {
#ifdef USE_TIMER
        Interrupt_enable(INT_TIMER0);
#endif
#ifdef USE_ECAP
        Interrupt_enable(INT_ECAP1);
#endif
        Interrupt_enable(MODBUS_SCI_RX_INT);
        Interrupt_enable(MODBUS_SCI_TX_INT);
        Enabled = true;
    } else if(!inTimInterrupt | !inTxSciInterrupt | !inRxSciInterrupt) {
#ifdef USE_TIMER
        Interrupt_disable(INT_TIMER0);
#endif
#ifdef USE_ECAP
      Interrupt_disable(INT_ECAP1);
#endif
        Interrupt_disable(MODBUS_SCI_RX_INT);
        Interrupt_disable(MODBUS_SCI_TX_INT);
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

    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return false;
    }

//    sciACheck();

    SCI_writeCharNonBlocking(MODBUS_SCI_BASE, sndByte);

    return 1;
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
//    uint16_t inWord;
//    uint16_t res;
//    uint16_t status;
    //We only have one port, so if not it abort
    if(port != MB_PORT_NUM) {
        return false;
    }

//    sciACheck();
//    status = SCI_getRxStatus(MODBUS_SCI_BASE);

    if(SCI_getRxStatus(MODBUS_SCI_BASE) != (SCI_RXST_RXRDY | SCI_RXST_BRKDT)) {
        //Read the character
//        res = SCI_readCharNonBlocking(MODBUS_SCI_BASE);
        *getByte = (SCI_readCharNonBlocking(MODBUS_SCI_BASE) & 0x00FF);
        return true;
        //}
    }
    else{
        SCI_performSoftwareReset(MODBUS_SCI_BASE);
        DEVICE_DELAY_US(10);
        SCI_enableModule(MODBUS_SCI_BASE);
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
#ifdef USE_TIMER
        CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
        CPUTimer_startTimer(CPUTIMER0_BASE);
#endif
#ifdef USE_ECAP
        ECAP_reArm(ECAP1_BASE);
        ECAP_startCounter(ECAP1_BASE);
#endif
    }
    //Timer auto resets on char received so no need for an ELSE to reload it here
    else {
#ifdef USE_TIMER
        CPUTimer_stopTimer(CPUTIMER0_BASE);
#endif
#ifdef USE_ECAP
        ECAP_stopCounter(ECAP1_BASE);
#endif
    }
}
