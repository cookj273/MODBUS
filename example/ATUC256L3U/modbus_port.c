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
#include <asf.h>
#include "modbus_port.h"

//Local Variables
avr32_usart_ier_t usartIntState;
static bool curRxEnable, curTxEnable;       //!< These track the current state of the TX and RX enable lines
static volatile bool inInterrupt = false;   //!< Tracks when we are in the interrupt to avoid disables
void (*rxIntFunc)(uint8_t port);            //!< The Receive Callback function
void (*txIntFunc)(uint8_t port);            //!< The Transmit Callback function
void (*timeoutIntFunc)(uint8_t port);       //!< The Timeout Callback function

/*
static void port_loopback_test(void) {
    const gpio_map_t MODBUS_GPIO_MAP = {
        {BOARD_MODBUS_RX, BOARD_MODBUS_RX_FUNC},    //!< SPI Clock.
        {BOARD_MODBUS_TX, BOARD_MODBUS_TX_FUNC},    //!< MISO.
    };
    gpio_enable_module(MODBUS_GPIO_MAP, sizeof(MODBUS_GPIO_MAP) / sizeof(MODBUS_GPIO_MAP[0]));

    usart_enable_rx(BOARD_MODBUS_USART, true);
    usart_enable_tx(BOARD_MODBUS_USART, false);
    gpio_configure_pin(BOARD_MODBUS_RX_ENABLE, (GPIO_DIR_OUTPUT | GPIO_INIT_LOW));
    int inChar = 0;
    while(1) {

        //Send character
        //If we have a character send it back
        while(usart_read_char(BOARD_MODBUS_USART, &inChar) == USART_SUCCESS) {
            //Disable Receive and enable transmit
            usart_enable_rx(BOARD_MODBUS_USART, false);
            usart_enable_tx(BOARD_MODBUS_USART, true);
            gpio_configure_pin(BOARD_MODBUS_RX_ENABLE,(GPIO_DIR_INPUT));
            gpio_configure_pin(BOARD_MODBUS_TX_ENABLE, (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH));

            //Send character
            if(usart_putchar(BOARD_MODBUS_USART, inChar) == USART_SUCCESS) {
                while(!usart_tx_empty(BOARD_MODBUS_USART));
            }

            //Disable Transmit and enable receive
            usart_enable_tx(BOARD_MODBUS_USART, false);
            usart_enable_rx(BOARD_MODBUS_USART, true);
            gpio_configure_pin(BOARD_MODBUS_TX_ENABLE,(GPIO_DIR_INPUT));
            gpio_configure_pin(BOARD_MODBUS_RX_ENABLE, (GPIO_DIR_OUTPUT | GPIO_INIT_LOW));
        }
    }
}
*/

/*!
* @brief Interrupt handler for modbus timer
*
* Used to determine timeout for packets which will indicate the end of a
* received packet.
*/
__attribute__((__interrupt__))
static void modbus_irq_handler(void)
{
    //Get our status
    avr32_usart_csr_t usartCSR = BOARD_MODBUS_USART->CSR;
    inInterrupt = true;

    //Find the flags and handle them
    //Received Byte
    if(usartCSR.usart_mode.rxrdy && usartIntState.usart_mode.rxrdy) {
        if(rxIntFunc) {
            rxIntFunc(MB_PORT_NUM);
        } else {
            int dummy;
            usart_read_char(BOARD_MODBUS_USART, &dummy);
        }
    } else if(usartCSR.usart_mode.timeout && usartIntState.usart_mode.timeout) {        //RX Timeout
        if(timeoutIntFunc) {
            timeoutIntFunc(MB_PORT_NUM);
        } else {
            //No clear function was defined so disable the timer here
            usartIntState.usart_mode.timeout = 0;
            usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
        }
    } else if((usartCSR.usart_mode.txrdy && usartIntState.usart_mode.txrdy) || (usartCSR.usart_mode.txempty && usartIntState.usart_mode.txempty)) {     //TX Timeout
        //Call our tx int callback function if registered
        if(txIntFunc) {
            txIntFunc(MB_PORT_NUM);
        } else {
            //No reload function was defined so disable the int here
            usartIntState.usart_mode.txempty = 0;
            usartIntState.usart_mode.txrdy = 0;
            usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
        }
    }

    inInterrupt = false;
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
bool modbus_port_init(uint8_t port, uint32_t baudRate, uint8_t charSize, modbus_parity parity, uint8_t stopBits, uint16_t timeoutBits,
                      void (*rxCallback)(uint8_t port), void (*txCallback)(uint8_t port), void (*timeoutCallback)(uint8_t port))
{
    //We only have one port!
    if(port != MB_PORT_NUM) {
        return false;
    }

    // Setup the modbus UART options
    usart_options_t modbusOptions = {
        .baudrate = baudRate,
        .charlength = charSize,
        .paritytype = ((parity == MODBUS_PARITY_EVEN)?USART_EVEN_PARITY:((parity == MODBUS_PARITY_ODD)?USART_ODD_PARITY:USART_NO_PARITY)),
        .stopbits = ((stopBits == 2)?USART_2_STOPBITS:USART_1_STOPBIT),
        .channelmode = USART_NORMAL_CHMODE
    } ;

    // Enable the Uart clock and the timer clock
    sysclk_enable_peripheral_clock(BOARD_MODBUS_USART);

    //Make all pins input for now so they get pulled externally to disabled states
    gpio_configure_pin(BOARD_MODBUS_RX_ENABLE,GPIO_DIR_INPUT);
    gpio_configure_pin(BOARD_MODBUS_TX_ENABLE,GPIO_DIR_INPUT);
    gpio_configure_pin(BOARD_MODBUS_TX,GPIO_DIR_INPUT);
    gpio_configure_pin(BOARD_MODBUS_RX,GPIO_DIR_INPUT);
    curTxEnable = false;
    curRxEnable = false;

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;

    //Register our interrupt handler
    Disable_global_interrupt();
    inInterrupt = false;
    INTC_register_interrupt(&modbus_irq_handler, BOARD_MODBUS_IRQ, MODBUS_INT_PRIORITY);
    Enable_global_interrupt();

    //Initialize the USART
    if(usart_init_rs232(BOARD_MODBUS_USART, &modbusOptions, sysclk_get_pba_hz()) == USART_SUCCESS) {
        //Init the port states
        uint32_t initVal = 0;
        usartIntState = *((avr32_usart_ier_t *)&initVal);
        modbus_port_serial_enable(port,false,false);
        usart_set_rx_timeout(BOARD_MODBUS_USART, timeoutBits);

        return true;
    }

    //We failed so disable the peripherial clocks if not used elsewhere
    sysclk_disable_peripheral_clock(BOARD_MODBUS_USART);

    return false;
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
void modbus_port_serial_enable(uint8_t port, bool rxEnable, bool txEnable)
{
    const gpio_map_t MODBUS_GPIO_MAP = {
        {BOARD_MODBUS_RX, BOARD_MODBUS_RX_FUNC},    //!< SPI Clock.
        {BOARD_MODBUS_TX, BOARD_MODBUS_TX_FUNC},    //!< MISO.
    };

    bool globIntEnabled = Is_global_interrupt_enabled();

    //We only have one port, if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Get out of power save
    if((!curRxEnable && !curTxEnable) && (rxEnable || txEnable)) {
        //We are moving from power save to on, redo the pins
        gpio_enable_module(MODBUS_GPIO_MAP, sizeof(MODBUS_GPIO_MAP) / sizeof(MODBUS_GPIO_MAP[0]));
    }

    //If we interrupt while we have our flags off all hell will break loose
    if(globIntEnabled) {
        Disable_global_interrupt();
    }

    //Reset all our int flags
    usartIntState.usart_mode.rxrdy = 0;
    usartIntState.usart_mode.txrdy = 0;
    usartIntState.usart_mode.txempty = 0;

    if(!txEnable || rxEnable) {
        usart_enable_tx(BOARD_MODBUS_USART, false);
        gpio_configure_pin(BOARD_MODBUS_TX_ENABLE,GPIO_DIR_INPUT);
    }

    if(!rxEnable) {
        usart_enable_rx(BOARD_MODBUS_USART, false);
        gpio_configure_pin(BOARD_MODBUS_RX_ENABLE,GPIO_DIR_INPUT);
    }

    //Set the RX up
    if(rxEnable) {
        //Configure the USART rx interrupt
        usartIntState.usart_mode.rxrdy = 1;
        usart_rx_timeout_restart(BOARD_MODBUS_USART, false);
        usart_enable_rx(BOARD_MODBUS_USART, true);
        gpio_configure_pin(BOARD_MODBUS_RX_ENABLE, (GPIO_DIR_OUTPUT | GPIO_INIT_LOW));
    }

    //Set the TX up
    if(txEnable && !rxEnable) {
        //Configure the USART tx interrupt
        usartIntState.usart_mode.txrdy = 1;
        usart_enable_tx(BOARD_MODBUS_USART, true);
        gpio_configure_pin(BOARD_MODBUS_TX_ENABLE, (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH));
    }

    //Setup the interrupts
    usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(globIntEnabled) {
        Enable_global_interrupt();
    }

    //Try to go to power save
    if((curRxEnable || curTxEnable) && (!rxEnable && !txEnable)) {
        //Not transmitting or receiving so disable rx and tx
        gpio_configure_pin(BOARD_MODBUS_TX,GPIO_DIR_INPUT);
        gpio_configure_pin(BOARD_MODBUS_RX,GPIO_DIR_INPUT);
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
void modbus_port_serial_close(uint8_t port)
{
    bool globIntEnabled = Is_global_interrupt_enabled();

    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Disable all the interrupts
    if(globIntEnabled) {
        Disable_global_interrupt();
    }
    usartIntState.usart_mode.rxrdy = 0;
    usartIntState.usart_mode.txrdy = 0;
    usartIntState.usart_mode.txempty = 0;
    usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(globIntEnabled) {
        Enable_global_interrupt();
    }

    //Turn off RX and TX
    usart_enable_tx(BOARD_MODBUS_USART, false);
    usart_enable_rx(BOARD_MODBUS_USART, false);

    //Turn all our pins to input to go to power save
    gpio_configure_pin(BOARD_MODBUS_TX,GPIO_DIR_INPUT);
    gpio_configure_pin(BOARD_MODBUS_RX,GPIO_DIR_INPUT);
    gpio_configure_pin(BOARD_MODBUS_TX_ENABLE,GPIO_DIR_INPUT);
    gpio_configure_pin(BOARD_MODBUS_RX_ENABLE,GPIO_DIR_INPUT);

    //Reset the UART
    usart_reset(BOARD_MODBUS_USART);

    //Turn the Clock off
    sysclk_disable_peripheral_clock(BOARD_MODBUS_USART);
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
void modbus_port_notify_of_tx_completion(uint8_t port)
{
    bool globIntEnabled = Is_global_interrupt_enabled();

    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    if(globIntEnabled) {
        Disable_global_interrupt();
    }
    usartIntState.usart_mode.txrdy = 0;
    usartIntState.usart_mode.txempty = 1;
    usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
    if(globIntEnabled) {
        Enable_global_interrupt();
    }
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
void modbus_port_enable_interrupts(uint8_t port, bool enable)
{
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Since all functionality is within the UART interrupt then don't bother
    //disabling if we are in the interrupt routine.
    if(!inInterrupt && enable) {
        Enable_global_interrupt();
    } else if(!inInterrupt) {
        Disable_global_interrupt();
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
bool modbus_port_put_byte(uint8_t port, uint8_t sndByte)
{
    //We only have one port, so if not it then abort
    if(port != MB_PORT_NUM) {
        return false;
    }

    return (usart_putchar(BOARD_MODBUS_USART, sndByte) == USART_SUCCESS);
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
bool modbus_port_get_byte(uint8_t port, uint8_t *getByte)
{
    int inWord;

    //We only have one port, so if not it abort
    if(port != MB_PORT_NUM) {
        return false;
    }

    //Read the character
    int res = usart_read_char(BOARD_MODBUS_USART, &inWord);
    if(res == USART_SUCCESS) {
        *getByte = (inWord & 0x00FF);
        return true;
    } else if(res == USART_RX_ERROR) {
        usart_reset_status(BOARD_MODBUS_USART);
        res = usart_read_char(BOARD_MODBUS_USART, &inWord);
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
void modbus_port_timer_enable(uint8_t port, bool enable)
{
    bool globIntEnabled = Is_global_interrupt_enabled();

    //We only have one port if not it abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Enable or disable as requested
    if(enable) {
        if(!usartIntState.usart_mode.timeout) {
            usart_rx_timeout_restart(BOARD_MODBUS_USART, true);
            usartIntState.usart_mode.timeout = 1;
            usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
        }
        //THIS TIMER auto resets on char received so no need for an ELSE to reload it here
    } else {
        if(globIntEnabled) {
            Disable_global_interrupt();
        }
        usartIntState.usart_mode.timeout = 0;
        usart_configure_interrupts(BOARD_MODBUS_USART, &usartIntState);
        if(globIntEnabled) {
            Enable_global_interrupt();
        }
    }
}