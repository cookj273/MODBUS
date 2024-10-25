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

//Includes
#include "modbus_port.h"
//#include "../project.h"


//extern HAL_Handle      handle;       //!< the handle for the hardware abstraction
                                 //!< layer (HAL)

//Local Variables
bool waiting_for_complete = false;
static bool curRxEnable, curTxEnable;       //!< These track the current state of the TX and RX enable lines
extern bool inTimInterrupt = false;    //!< Tracks when we are in the timer interrupt to avoid disables
extern bool inTxSciInterrupt = false;  //!< Tracks when we are in the TXSci interrupt to avoid disables
extern bool inRxSciInterrupt = false;  //!< Tracks when we are in the RXSci interrupt to avoid disables
static bool Enabled;

extern __interrupt void sciaTxFifoIsr(void);
extern __interrupt void sciaRxFifoIsr(void);
extern __interrupt void cpu_timer0_isr(void);

void (*rxIntFunc)(uint16_t port);            //!< The Receive Callback function
void (*txIntFunc)(uint16_t port);            //!< The Transmit Callback function
void (*timeoutIntFunc)(uint16_t port);       //!< The Timeout Callback function

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

    //We only have one port!
    if(port != MB_PORT_NUM) {
        return false;
    }

    // Enable the Uart clock and the timer clock
    //InitPeripheralClocks();

    curTxEnable = false;
    curRxEnable = false;

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;

    //Register our interrupt handler
    Enabled = Disable_global_interrupt(Enabled);
    inTxSciInterrupt = false;
    inRxSciInterrupt = false;
    inTimInterrupt = false;


    //Turn the bits into timer clocks (time clock = Sysclk/AHB * 2 /APB1)
    mbTimeout = (int)timeoutBits * (90000000/4/(baudRate));

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

    CpuTimer0Regs.TPRH.all = psc >> 8;       // Set prescaler
    CpuTimer0Regs.TPR.all = psc;             // prescaler lower register
    CpuTimer0Regs.PRD.all = timeArr;         // set the timeout period.
    CpuTimer0Regs.TCR.bit.TRB = 1;           // Auto reload value

    //hold UART in reset
    SciaRegs.SCICTL1.all = 0x0003;


    //modbus UART options
    //baud rate
    int32_t temp_baud = (((90000000/4)/((baudRate*8)))-1);
    SciaRegs.SCIHBAUD = temp_baud >> 8;
    SciaRegs.SCILBAUD = temp_baud;

    //char length

    if (charSize <= 1) {
        SciaRegs.SCICCR.bit.SCICHAR = charSize - 1;
    }
    else {
        SciaRegs.SCICCR.bit.SCICHAR = 7;
    }

    //parity
    if (parity == MODBUS_PARITY_EVEN) {
        SciaRegs.SCICCR.bit.PARITY = 1;
        SciaRegs.SCICCR.bit.PARITYENA = 1;
    }
    else {
        SciaRegs.SCICCR.bit.PARITY = 0;
        SciaRegs.SCICCR.bit.PARITYENA = 0;
    }

    //stop bits
    SciaRegs.SCICCR.bit.STOPBITS = stopBits;

    //SciaRegs.SCICCR.bit.LOOPBKENA =1;   // Enable loop back

    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;
    SciaRegs.SCIFFTX.all=0xC000;
    SciaRegs.SCIFFRX.all=0x0001;
    SciaRegs.SCIFFCT.all=0x00;
    Enabled = Enable_global_interrupt(Enabled);


    SciaRegs.SCICTL1.all =0x0023;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

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
    //We only have one port, if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }


    //If we interrupt while we have our flags off all hell will break loose
    if(Enabled) {
        Enabled = Disable_global_interrupt(Enabled);
    }

    //Set the RX up
    if(rxEnable) {
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;   // RE enabled
        SciaRegs.SCICTL1.bit.RXENA = 1;
        //SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
        SciaRegs.SCIFFRX.bit.RXFFIENA = 1;

        //Start the timer
        StartCpuTimer0();
    }
    else{
        SciaRegs.SCIFFRX.bit.RXFFIENA = 0;
        //SciaRegs.SCICTL2.bit.RXBKINTENA = 0;
        SciaRegs.SCICTL1.bit.RXENA = 0;
        StopCpuTimer0();
    }

    //Set the TX up
    if(txEnable && !rxEnable) {
        GpioDataRegs.GPASET.bit.GPIO29 = 1;   // DE enabled
        SciaRegs.SCICTL1.bit.TXENA = 1;
        //SciaRegs.SCICTL2.bit.TXINTENA = 1;
        SciaRegs.SCIFFTX.bit.TXFFIENA = 1;
    }
    else{
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
        //SciaRegs.SCICTL2.bit.TXINTENA = 0;
        SciaRegs.SCICTL1.bit.TXENA = 0;
    }

    //Setup the interrupts
    //usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(!Enabled) {
        Enabled = Enable_global_interrupt(Enabled);
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
        Enabled = Disable_global_interrupt(Enabled);
    }
    SciaRegs.SCICTL2.bit.TXRDY = 0;
    SciaRegs.SCICTL2.bit.TXEMPTY = 0;

    //usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(!Enabled) {
        Enabled = Enable_global_interrupt(Enabled);
    }

    //Turn off RX and TX
    //SciaRegs.SCICTL1.bit.TXENA = 0;
    //SciaRegs.SCICTL1.bit.RXENA = 0;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 0;
    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;

    //Turn all our pins to input to go to power save
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;

    //Reset the UART
    //usart_reset(BOARD_MODBUS_USART);
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

    //Turn the Clock off
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;
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

    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    ReloadCpuTimer0();
    StartCpuTimer0();
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
        Enabled = Enable_global_interrupt(Enabled);
    } else if(!inTimInterrupt | !inTxSciInterrupt | !inRxSciInterrupt) {
        Enabled = Disable_global_interrupt(Enabled);
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

    bool tx_ready;

    tx_ready = (SciaRegs.SCICTL2.all & (1 << 7)) >> 7;

    if (tx_ready){
        SciaRegs.SCITXBUF = sndByte;

        return(true);
    }

    return (false);
}


/*!
* @name sciaCheck
* @brief Checks for an error or break condition and resets the receiver
*
* sciaInit must be called before this function can be used.
* This function checks the break and error flags and if either is set
* it resets the module.
*/
void sciaCheck(void)
{
    if(SciaRegs.SCIRXST.bit.RXERROR || SciaRegs.SCIRXST.bit.BRKDT) {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        DELAY_US(10);
        SciaRegs.SCICTL1.bit.SWRESET = 1;
    }
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
    uint16_t inWord;
    uint16_t res;
    bool rx_ready;

    //We only have one port, so if not it abort
    if(port != MB_PORT_NUM) {
        return false;
    }

    rx_ready = (!(SciaRegs.SCIRXST.bit.RXRDY));

    if(rx_ready){
        inWord = SciaRegs.SCIRXBUF.all;
        res = true;
    }
    if(res == true){
        *getByte = (inWord & 0x00FF);
        return true;
    }
    else{
        SciaRegs.SCICTL1.bit.SWRESET = 0;
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
        ReloadCpuTimer0();
        StartCpuTimer0(); // Enable timer
    }
        //THIS TIMER auto resets on char received so no need for an ELSE to reload it here
    else {
        StopCpuTimer0();
    }
}
