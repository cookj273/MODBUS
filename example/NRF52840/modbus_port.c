/*!
* @file modbus_port.c
* @author Jarrod Cook
*
* @brief Implementation of the board specific functionality for MODBUS
*
* This file is the implementation of routines to handle the UART output
* and timer implementation as well as function handlers for MODBUS. It is using an nrf52840, using both UART
* for RTU mode and using a WIZ610IO (https://www.wiznet.io/product-item/wiz610io/) to provide ethernet functionality!
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includes
#include "modbus_port.h"
#include "global.h"
#include "socket.h"
#include "timer.h"
#include "prs/nrfx_prs.h"
#include "nrf_drv_uart.h"
#include <nrfx_uart.h>
#include "app_timer.h"
#include "nrf_drv_clock.h"

//Defines
#define TCP_PORT_IDX(a) (a - MB_TCP_REDUND_PORT_NUM)

//Typedefs and Enums
typedef struct nrf_port_info_type {
    uint8_t port;
    nrfx_uart_t *uartReg;
    uint32_t appTimerTicks;
    bool waitOnLast;
    bool disableInt;
} nrf_port_info;

//Local Variables
static volatile bool inTimInterrupt = false;    //!< Tracks when we are in the timer interrupt to avoid disables
static volatile bool inUsartInterrupt = false;  //!< Tracks when we are in the Usart interrupt to avoid disables
uint16_t timeArr = 0;                           //!< Auto-reload value for the timer
void (*rxIntFunc)(uint8_t port);                //!< The Receive Callback function
void (*txIntFunc)(uint8_t port);                //!< The Transmit Callback function
void (*timeoutIntFunc)(uint8_t port);           //!< The Timeout Callback function
static bool sevenBit;
static nrf_port_info nrfPortInfo[NUM_MODBUS_PORTS] = {{0, NULL, 0, false, false}};

static int mbSocket[NUM_MB_TCP_PORTS] = {-1,-1};
static int mbPort[NUM_MB_TCP_PORTS] = {-1,-1};
static int mbSlave[NUM_MB_TCP_PORTS] = {-1,-1};
static int curSlave[NUM_MB_TCP_PORTS] = {-1,-1};
static bool tcpSend[NUM_MB_TCP_PORTS] = {false};
static uint32_t tcpTimerTicks[NUM_MB_TCP_PORTS] = {0};
static bool tcpTimerActive[NUM_MB_TCP_PORTS] = {false};
static modbus_device_types tcpDevType[NUM_MB_TCP_PORTS];
static uint16_t tcpTxStop[NUM_MB_TCP_PORTS] = {0}, tcpTxStart[NUM_MB_TCP_PORTS] = {0};
static uint8_t tcpTxBuf[NUM_MB_TCP_PORTS][MB_TCP_BUF_SIZE] = {0};
static uint16_t tcpRxStop[NUM_MB_TCP_PORTS] = {0}, tcpRxStart[NUM_MB_TCP_PORTS] = {0};
static uint8_t tcpRxBuf[NUM_MB_TCP_PORTS][MB_TCP_BUF_SIZE] = {0};
static uint8_t mbIntPort = 0;

void (*rxTCPIntFunc[NUM_MB_TCP_PORTS])(uint8_t port);                //!< The Receive Callback function
void (*txTCPIntFunc[NUM_MB_TCP_PORTS])(uint8_t port);                //!< The Transmit Callback function
void (*timeoutTCPIntFunc[NUM_MB_TCP_PORTS])(uint8_t port);           //!< The Timeout Callback function

nrf_drv_uart_t uart_driver_instance = NRF_DRV_UART_INSTANCE(0);
APP_TIMER_DEF(m_modbus_timer);

static void uart_interrupts_enable(nrfx_uart_t const * p_instance) {
    NVIC_EnableIRQ(nrfx_get_irq_number((void *)p_instance->p_reg));
}

static void uart_interrupts_disable(nrfx_uart_t const * p_instance) {
    NVIC_DisableIRQ(nrfx_get_irq_number((void *)p_instance->p_reg));
}

/*
static void port_loopback_test(void) {
    LL_USART_Enable(MB_USART);
    LL_GPIO_ResetOutputPin(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM);
    LL_USART_SetTransferDirection(MB_USART, LL_USART_DIRECTION_RX);

    char inChar = 0;
    while(1) {

        //Send character
        //If we have a character send it back
        while(LL_USART_IsActiveFlag_RXNE(MB_USART)) {
            //Get character
            inChar = LL_USART_ReceiveData8(MB_USART);

            //Disable Receive and enable transmit
            LL_GPIO_SetOutputPin(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM);
            LL_USART_SetTransferDirection(MB_USART, LL_USART_DIRECTION_TX);

            //Send character
            LL_USART_TransmitData8(MB_USART, inChar);
            while(!LL_USART_IsActiveFlag_TC(MB_USART)) {

            }

            //Disable Transmit and enable receive
            LL_GPIO_ResetOutputPin(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM);
            LL_USART_SetTransferDirection(MB_USART, LL_USART_DIRECTION_RX);
        }
    }
}
*/

void nrfx_uart_0_irq_handler(void) {
    inUsartInterrupt = true;
    if (nrf_uart_int_enable_check(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_INT_MASK_ERROR) &&
        nrf_uart_event_check(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_EVENT_ERROR)) {
        //Just clear, let more bytes come in and let timeout trigger end of bad packet to fresh start
        nrf_uart_event_clear(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_EVENT_ERROR);
    }

    if (nrf_uart_int_enable_check(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_INT_MASK_RXDRDY) &&
        nrf_uart_event_check(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_EVENT_RXDRDY)) {
        if(rxIntFunc) {
            rxIntFunc(mbIntPort);
        } else {
            nrf_uart_event_clear(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_EVENT_RXDRDY);
            (void) nrf_uart_rxd_get(nrfPortInfo[mbIntPort].uartReg->p_reg);
        }
    }

    if (nrf_uart_int_enable_check(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_INT_MASK_TXDRDY) &&
        nrf_uart_event_check(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_EVENT_TXDRDY)) {
        if(txIntFunc && !nrfPortInfo[mbIntPort].waitOnLast) {
            txIntFunc(mbIntPort);
        } else {
            nrf_uart_event_clear(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_EVENT_TXDRDY);
            nrf_uart_int_disable(nrfPortInfo[mbIntPort].uartReg->p_reg, NRF_UART_INT_MASK_TXDRDY);
            if(nrfPortInfo[mbIntPort].waitOnLast) {
                if(nrfPortInfo[mbIntPort].appTimerTicks > 15) {
                    app_timer_start(m_modbus_timer, nrfPortInfo[mbIntPort].appTimerTicks/3, &nrfPortInfo[mbIntPort]);
                } else {
                    app_timer_start(m_modbus_timer, 5, &nrfPortInfo[mbIntPort]);
                }
            }
        }
    }

    inUsartInterrupt = false;
}

/*!
* @brief Interrupt handler for modbus timer
*
* Used to determine timeout for packets which will indicate the end of a
* received packet.
*/
void timeout_handler(void * p_context)
{
    nrf_port_info *portInfo = (nrf_port_info *)p_context;

    if(nrfPortInfo[portInfo->port].disableInt) {
        //We are ignoring ints for now so call again soon
        app_timer_start(m_modbus_timer, 5, &nrfPortInfo[portInfo->port]);
    }

    inTimInterrupt = true;

    //Call our callback
    if(nrfPortInfo[portInfo->port].waitOnLast) {
        //We double-use this timer for our last TX byte as well
        nrfPortInfo[portInfo->port].waitOnLast = false;
        if(txIntFunc) {
            txIntFunc(portInfo->port);
        }
    } else if(timeoutIntFunc) {
        timeoutIntFunc(portInfo->port);
    }

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
bool modbus_port_init(uint8_t port, uint32_t baudRate, uint8_t charSize, modbus_parity parity, uint8_t stopBits, uint16_t timeoutBits,
                      void (*rxCallback)(uint8_t port), void (*txCallback)(uint8_t port), void (*timeoutCallback)(uint8_t port))
{
    static bool isInited = false;
    int test, psc, mbTimeout;
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;

    //We only have one port!
    if((port != MB_RTU_SLAVE_PORT_NUM) && (port != MB_RTU_MASTER_PORT_NUM)) {
        return false;
    }

    //Char size > 8 we can't support if parity is on (9th bit needed for parity)
    sevenBit = false;
    if((charSize < 7) || ((charSize > 8) && (parity != MODBUS_PARITY_NONE)) || (charSize > 9)) {
        return false;
    } else if(stopBits != 1) {
        //This CPU only supports 1 stop bit
        return false;
    } else if(charSize == 7) {
        //The CPU can't support 7-bit mode so we have to use 2 stop bits and make our 8th bit a stop
        if(parity != MODBUS_PARITY_NONE) {
            return false;
        }
        stopBits = 2;
        sevenBit = true;
    }

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;

    //Init directional pin for half-duplex
    nrf_gpio_cfg_output(MB_RTU_DIR_PIN);
    nrf_gpio_pin_clear(MB_RTU_DIR_PIN);

    //Init Timer
    if(!isInited) {
      if(app_timer_create(&m_modbus_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout_handler) != NRF_SUCCESS) {
          return false;
      }
    }

    //Setup config and init uart
    mbIntPort = port;         //ONLY 1 INSTANCE PER PORT OBVIOUSLY SO LAST IN WINS
    nrfPortInfo[port].port = port;
    nrfPortInfo[port].uartReg = &(uart_driver_instance.uart);
    nrfPortInfo[port].waitOnLast = false;
    nrfPortInfo[port].appTimerTicks = (((int)timeoutBits * (((int)APP_TIMER_CLOCK_FREQ*100)/(int)baudRate))+90)/100; //Round up mostly (.1 or more)
    if(nrfPortInfo[port].appTimerTicks < 5) {
        nrfPortInfo[port].appTimerTicks = 5;
    }
    config.p_context = &nrfPortInfo[port];
    config.use_easy_dma = false;
    config.pseltxd  = MB_RTU_TX_PIN;
    config.pselrxd  = MB_RTU_RX_PIN;
    if(baudRate <= 1200) {
        config.baudrate = NRF_UART_BAUDRATE_1200;
    } else if(baudRate <= 2400) {
        config.baudrate = NRF_UART_BAUDRATE_2400;
    } else if(baudRate <= 4800) {
        config.baudrate = NRF_UART_BAUDRATE_4800;
    } else if(baudRate <= 9600) {
        config.baudrate = NRF_UART_BAUDRATE_9600;
    } else if(baudRate <= 14400) {
        config.baudrate = NRF_UART_BAUDRATE_14400;
    } else if(baudRate <= 19200) {
        config.baudrate = NRF_UART_BAUDRATE_19200;
    } else if(baudRate <= 28800) {
        config.baudrate = NRF_UART_BAUDRATE_28800;
    } else if(baudRate <= 31250) {
        config.baudrate = NRF_UART_BAUDRATE_31250;
    } else if(baudRate <= 38400) {
        config.baudrate = NRF_UART_BAUDRATE_38400;
    } else if(baudRate <= 56000) {
        config.baudrate = NRF_UART_BAUDRATE_56000;
    } else if(baudRate <= 57600) {
        config.baudrate = NRF_UART_BAUDRATE_57600;
    } else if(baudRate <= 76800) {
        config.baudrate = NRF_UART_BAUDRATE_76800;
    } else if(baudRate <= 115200) {
        config.baudrate = NRF_UART_BAUDRATE_115200;
    } else if(baudRate <= 230400) {
        config.baudrate = NRF_UART_BAUDRATE_230400;
    } else if(baudRate <= 250000) {
        config.baudrate = NRF_UART_BAUDRATE_250000;
    } else if(baudRate <= 460800) {
        config.baudrate = NRF_UART_BAUDRATE_460800;
    } else if(baudRate <= 921600) {
        config.baudrate = NRF_UART_BAUDRATE_921600;
    } else {
        config.baudrate = NRF_UART_BAUDRATE_1000000;
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    static nrfx_irq_handler_t const irq_handlers[NRFX_UART_ENABLED_COUNT] = {
        #if NRFX_CHECK(NRFX_UART0_ENABLED)
        nrfx_uart_0_irq_handler,
        #endif
    };

    if(!isInited) {
      if (nrfx_prs_acquire(nrfPortInfo[port].uartReg->p_reg, irq_handlers[nrfPortInfo[port].uartReg->drv_inst_idx]) != NRFX_SUCCESS) {
          return false;
      }
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    //Setup and Initialize
    nrf_gpio_pin_set(config.pseltxd);
    nrf_gpio_cfg_output(config.pseltxd);
    nrf_gpio_cfg_input(config.pselrxd, NRF_GPIO_PIN_NOPULL);
    nrf_uart_baudrate_set(nrfPortInfo[port].uartReg->p_reg, config.baudrate);
    nrf_uart_configure(nrfPortInfo[port].uartReg->p_reg, config.parity, config.hwfc);
    nrf_uart_txrx_pins_set(nrfPortInfo[port].uartReg->p_reg, config.pseltxd, config.pselrxd);

    //Turn interttupts on but disable
    nrf_uart_int_disable(nrfPortInfo[port].uartReg->p_reg, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_ERROR | NRF_UART_INT_MASK_TXDRDY | NRF_UART_INT_MASK_RXTO);
    NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number((void *)nrfPortInfo[port].uartReg->p_reg), config.interrupt_priority);
    NRFX_IRQ_ENABLE(nrfx_get_irq_number((void *)nrfPortInfo[port].uartReg->p_reg));

    //Enable the UART
    nrf_uart_enable(nrfPortInfo[port].uartReg->p_reg);

    //Start disabled
    modbus_port_serial_enable(port, false, false);

    isInited = true;
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
void modbus_port_serial_enable(uint8_t port, bool rxEnable, bool txEnable)
{
    //We only have one port, if not it then abort
    if((port != MB_RTU_SLAVE_PORT_NUM) && (port != MB_RTU_MASTER_PORT_NUM)) {
        return;
    }

    //Stop our timer and disable interrupts
    app_timer_stop(m_modbus_timer);
    uart_interrupts_disable(nrfPortInfo[port].uartReg);
    nrfPortInfo[port].disableInt = true;

    //Enable or disable receiving
    if(rxEnable) {
        //Disable driver out
        nrf_gpio_pin_clear(MB_RTU_DIR_PIN);

        app_timer_start(m_modbus_timer, nrfPortInfo[port].appTimerTicks, &nrfPortInfo[port]);

        nrf_uart_event_clear(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_ERROR);
        nrf_uart_event_clear(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_RXDRDY);
        nrf_uart_int_enable(nrfPortInfo[port].uartReg->p_reg, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_ERROR);
        nrf_uart_task_trigger(nrfPortInfo[port].uartReg->p_reg, NRF_UART_TASK_STARTRX);

    } else {
        nrf_uart_int_disable(nrfPortInfo[port].uartReg->p_reg, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_ERROR);
        nrf_uart_task_trigger(nrfPortInfo[port].uartReg->p_reg, NRF_UART_TASK_STOPRX);
    }

    //Enable or disable transmitting
    if(txEnable && !rxEnable) {
        //Enable driver out
        nrf_gpio_pin_set(MB_RTU_DIR_PIN);

        nrf_uart_event_clear(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_TXDRDY);
        nrf_uart_task_trigger(nrfPortInfo[port].uartReg->p_reg, NRF_UART_TASK_STARTTX);
        nrf_uart_int_enable(nrfPortInfo[port].uartReg->p_reg, NRF_UART_INT_MASK_TXDRDY);

        //Prime the first byte
        if(txIntFunc) {
            txIntFunc(port);
        }
    } else {
        nrfPortInfo[port].waitOnLast = false;
        nrf_uart_int_disable(nrfPortInfo[port].uartReg->p_reg, NRF_UART_INT_MASK_TXDRDY);
        nrf_uart_task_trigger(nrfPortInfo[port].uartReg->p_reg, NRF_UART_TASK_STOPTX);
    }

    nrfPortInfo[port].disableInt = false;
    uart_interrupts_enable(nrfPortInfo[port].uartReg);
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
    //We only have one port, so if not it then abort
    if(port >= NUM_MODBUS_PORTS) {
        return;
    }

    if((port == MB_TCP_REDUND_PORT_NUM) || (port == MB_TCP_COMMS_PORT_NUM)) {
        disconnect(mbSocket[TCP_PORT_IDX(port)]);
    } else {
        app_timer_stop(m_modbus_timer);
        nrf_uart_int_disable(nrfPortInfo[port].uartReg->p_reg, NRF_UART_INT_MASK_RXDRDY | NRF_UART_INT_MASK_TXDRDY | NRF_UART_INT_MASK_ERROR  | NRF_UART_INT_MASK_RXTO);
        NRFX_IRQ_DISABLE(nrfx_get_irq_number((void *)nrfPortInfo[port].uartReg->p_reg));
        nrf_uart_disable(nrfPortInfo[port].uartReg->p_reg);
        nrf_gpio_cfg_default(MB_RTU_TX_PIN);
        nrf_gpio_cfg_default(MB_RTU_RX_PIN);  
#if NRFX_CHECK(NRFX_PRS_ENABLED)
        nrfx_prs_release(nrfPortInfo[port].uartReg->p_reg);
#endif
    }
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
    //We only have one port, so if not it then abort
    if(port >= NUM_MODBUS_PORTS) {
        return;
    }

    if((port == MB_TCP_REDUND_PORT_NUM) || (port == MB_TCP_COMMS_PORT_NUM)) {
        tcpSend[TCP_PORT_IDX(port)] = true;
    } else {
        //Set a flag so the interrupt knows to start a timer for last byte
        nrfPortInfo[port].waitOnLast = true;
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
    if((port != MB_RTU_SLAVE_PORT_NUM) && (port != MB_RTU_MASTER_PORT_NUM)) {
        return;
    }

    //Since all functionality is within the UART interrupt then don't bother
    //disabling if we are in the interrupt routine.
    if(!inUsartInterrupt && !inTimInterrupt && enable) {
        uart_interrupts_enable(nrfPortInfo[port].uartReg);
        nrfPortInfo[port].disableInt = false;
    } else if(!inUsartInterrupt && !inTimInterrupt) {
        uart_interrupts_disable(nrfPortInfo[port].uartReg);
        nrfPortInfo[port].disableInt = true;
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
    if(port >= NUM_MODBUS_PORTS) {
        return false;
    }

    if((port == MB_TCP_REDUND_PORT_NUM) || (port == MB_TCP_COMMS_PORT_NUM)) {
        if((tcpTxStop[TCP_PORT_IDX(port)] != (tcpTxStart[TCP_PORT_IDX(port)]-1)) && ((tcpTxStop[TCP_PORT_IDX(port)] != (sizeof(tcpTxBuf[TCP_PORT_IDX(port)])-1)) || (tcpTxStart[TCP_PORT_IDX(port)] != 0))) {
            tcpTxBuf[TCP_PORT_IDX(port)][tcpTxStop[TCP_PORT_IDX(port)]++] = sndByte;
            if(tcpTxStop[TCP_PORT_IDX(port)] >= sizeof(tcpTxBuf[TCP_PORT_IDX(port)])) {
                tcpTxStop[TCP_PORT_IDX(port)] = 0;
            }
            if((tcpTxStop[TCP_PORT_IDX(port)] == (tcpTxStart[TCP_PORT_IDX(port)]-1)) || ((tcpTxStop[TCP_PORT_IDX(port)] == (sizeof(tcpTxBuf[TCP_PORT_IDX(port)])-1)) && (tcpTxStart[TCP_PORT_IDX(port)] == 0))) {
                tcpSend[TCP_PORT_IDX(port)] = true;
            }
        } else {
            
        }
    } else {
        nrf_uart_event_clear(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_TXDRDY);
        nrf_uart_txd_set(nrfPortInfo[port].uartReg->p_reg, (sevenBit?(0x80 | sndByte):sndByte));
        return true;
    }

    return false;
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
    //We only have one port, so if not it abort
    if(port >= NUM_MODBUS_PORTS) {
        return false;
    }

    if((port == MB_TCP_REDUND_PORT_NUM) || (port == MB_TCP_COMMS_PORT_NUM)) {
        if(tcpRxStart[TCP_PORT_IDX(port)] != tcpRxStop[TCP_PORT_IDX(port)]) {
            *getByte = tcpRxBuf[TCP_PORT_IDX(port)][tcpRxStart[TCP_PORT_IDX(port)]++];
            if(tcpRxStart[TCP_PORT_IDX(port)] >= sizeof(tcpRxBuf[TCP_PORT_IDX(port)])) {
                tcpRxStart[TCP_PORT_IDX(port)] = 0;
            }
            return true;
        }
    } else {
        if(nrf_uart_event_check(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_ERROR)) {
            //Just clear, let more bytes come in and let timeout trigger end of bad packet to fresh start
            nrf_uart_event_clear(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_ERROR);
        }
        if(nrf_uart_event_check(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_RXDRDY)) {
            nrf_uart_event_clear(nrfPortInfo[port].uartReg->p_reg, NRF_UART_EVENT_RXDRDY);
            *getByte = nrf_uart_rxd_get(nrfPortInfo[port].uartReg->p_reg);
            //If seven-bit it means the last bit is our stop bit we need to mask off
            if(sevenBit) {
                *getByte &= 0x7F;
            }
            return true;
        }
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
void modbus_port_timer_enable(uint8_t port, bool enable) {
    //We only have one port if not it abort
    if(port >= NUM_MODBUS_PORTS) {
        return;
    }
    
    if((port == MB_TCP_REDUND_PORT_NUM) || (port == MB_TCP_COMMS_PORT_NUM)) {
        if(enable) {
            tcpTimerTicks[TCP_PORT_IDX(port)] = get_sys_ticks();
            tcpTimerActive[TCP_PORT_IDX(port)] = true;
        } else {
            tcpTimerTicks[TCP_PORT_IDX(port)] = 0;
            tcpTimerActive[TCP_PORT_IDX(port)] = false;
        }
    } else {
        //Enable or disable as requested
        if(enable) {
            //Even if already enabled reset the count here
            app_timer_stop(m_modbus_timer);
            app_timer_start(m_modbus_timer, nrfPortInfo[port].appTimerTicks, &nrfPortInfo[port]);
        } else {
            app_timer_stop(m_modbus_timer);
        }
    }
}

bool modbus_port_tcp_init(uint8_t modPort, uint16_t tcpPort, uint16_t timeoutSec, modbus_device_types deviceType,
                      void (*rxCallback)(uint8_t port), void (*txCallback)(uint8_t port), void (*timeoutCallback)(uint8_t port)) {
    if((modPort != MB_TCP_REDUND_PORT_NUM) && (modPort != MB_TCP_COMMS_PORT_NUM)) {
        return false;
    }

    //Bind our callbacks
    rxTCPIntFunc[TCP_PORT_IDX(modPort)] = rxCallback;
    txTCPIntFunc[TCP_PORT_IDX(modPort)] = txCallback;
    timeoutTCPIntFunc[TCP_PORT_IDX(modPort)] = timeoutCallback;

    if(tcpPort != 0) {
        mbPort[TCP_PORT_IDX(modPort)] = (unsigned short)tcpPort;
    } else {
        mbPort[TCP_PORT_IDX(modPort)] = MB_TCP_DEFAULT_PORT;
    }

    tcpSend[TCP_PORT_IDX(modPort)] = false;
    mbSocket[TCP_PORT_IDX(modPort)] = (modPort == MB_TCP_REDUND_PORT_NUM)?SOCK_MODBUS_REDUND:SOCK_MODBUS_COMMS;
    tcpDevType[TCP_PORT_IDX(modPort)] = deviceType;

    return true;
}

uint16_t tcp_buf_space_left(uint16_t bufSize, uint16_t bufStart, uint16_t bufStop) {
    if((bufStop == (bufStart-1)) || ((bufStop == (bufSize -1)) && (bufStart == 0))) {
        return 0;
    } else if(bufStop < bufStart) {
        return (bufStart-bufStop-1);
    } else {
        return (bufSize - bufStop) + (bufStart)-1;
    }
}

void modbus_tcp_connection_FSM(unit_settings *settings, unit_state *state) {
    int16_t len, numRec;
    uint32_t gettime = 0;
    uint16_t idx;
    bool connecting[NUM_MB_TCP_PORTS] = {false};
    uint8_t curState;

    for(idx = MB_TCP_REDUND_PORT_NUM; idx <= MB_TCP_COMMS_PORT_NUM; idx++) {
        if(mbPort[TCP_PORT_IDX(idx)] >= 0) {
            if((curState = getSn_SR(mbSocket[TCP_PORT_IDX(idx)])) != SOCK_INIT) {
                connecting[TCP_PORT_IDX(idx)] = false;
            }
            switch(curState) {
                case SOCK_ESTABLISHED:
                    // Interrupt clear
                    if(getSn_IR(mbSocket[TCP_PORT_IDX(idx)]) & Sn_IR_CON) {
                        setSn_IR(mbSocket[TCP_PORT_IDX(idx)], Sn_IR_CON);
                    }

                    //Handle any received data
                    if ((len = getSn_RX_RSR(mbSocket[TCP_PORT_IDX(idx)])) > 0) {
                        if (len > tcp_buf_space_left(sizeof(tcpRxBuf[TCP_PORT_IDX(idx)]), tcpRxStart[TCP_PORT_IDX(idx)], tcpRxStop[TCP_PORT_IDX(idx)])) {
                            len = tcp_buf_space_left(sizeof(tcpRxBuf[TCP_PORT_IDX(idx)]), tcpRxStart[TCP_PORT_IDX(idx)], tcpRxStop[TCP_PORT_IDX(idx)]);
                        }
                    
                        do {
                            if(len > (sizeof(tcpRxBuf[TCP_PORT_IDX(idx)]) - tcpRxStop[TCP_PORT_IDX(idx)])) {
                                numRec = recv(mbSocket[TCP_PORT_IDX(idx)], (uint8_t *)&(tcpRxBuf[TCP_PORT_IDX(idx)][tcpRxStop[TCP_PORT_IDX(idx)]]), (sizeof(tcpRxBuf[TCP_PORT_IDX(idx)]) - tcpRxStop[TCP_PORT_IDX(idx)]));
                            } else {
                                numRec = recv(mbSocket[TCP_PORT_IDX(idx)], (uint8_t *)&(tcpRxBuf[TCP_PORT_IDX(idx)][tcpRxStop[TCP_PORT_IDX(idx)]]), len);
                            }
                            if(numRec >= 0) {
                              len -= numRec;

                              tcpRxStop[TCP_PORT_IDX(idx)] += numRec;
                              if(tcpRxStop[TCP_PORT_IDX(idx)] >= sizeof(tcpRxBuf[TCP_PORT_IDX(idx)])) {
                                  tcpRxStop[TCP_PORT_IDX(idx)] -= sizeof(tcpRxBuf[TCP_PORT_IDX(idx)]);
                              }
                            } else {
                              break;
                            }
                        } while(len > 0);
                    }

                    //Handle sending any data
                    if(tcpSend[TCP_PORT_IDX(idx)]) {
                        if((tcpDevType[TCP_PORT_IDX(idx)] == MODBUS_MASTER_DEVICE) && (mbSlave[TCP_PORT_IDX(idx)] != curSlave[TCP_PORT_IDX(idx)])) {
                            disconnect(mbSocket[TCP_PORT_IDX(idx)]);
                        } else {
                            if(tcpTxStart[TCP_PORT_IDX(idx)] < tcpTxStop[TCP_PORT_IDX(idx)]) {
                                if(tcpTxStop[TCP_PORT_IDX(idx)] - tcpTxStart[TCP_PORT_IDX(idx)] < 256) {
                                    send(mbSocket[TCP_PORT_IDX(idx)], (uint8_t *)&(tcpTxBuf[TCP_PORT_IDX(idx)][tcpTxStart[TCP_PORT_IDX(idx)]]), (tcpTxStop[TCP_PORT_IDX(idx)] - tcpTxStart[TCP_PORT_IDX(idx)]));
                                    tcpTxStart[TCP_PORT_IDX(idx)] = tcpTxStop[TCP_PORT_IDX(idx)];
                                    tcpSend[TCP_PORT_IDX(idx)] = false;
                                } else {
                                    send(mbSocket[TCP_PORT_IDX(idx)], (uint8_t *)&(tcpTxBuf[TCP_PORT_IDX(idx)][tcpTxStart[TCP_PORT_IDX(idx)]]), 255);
                                    tcpTxStart[TCP_PORT_IDX(idx)] += 255;
                                }
                            } else if(tcpTxStart[TCP_PORT_IDX(idx)] > tcpTxStop[TCP_PORT_IDX(idx)]) {
                                if(sizeof(tcpTxBuf[TCP_PORT_IDX(idx)]) - tcpTxStart[TCP_PORT_IDX(idx)] < 256) {
                                    send(mbSocket[TCP_PORT_IDX(idx)], (uint8_t *)&(tcpTxBuf[TCP_PORT_IDX(idx)][tcpTxStart[TCP_PORT_IDX(idx)]]), (sizeof(tcpTxBuf[TCP_PORT_IDX(idx)]) - tcpTxStart[TCP_PORT_IDX(idx)]));
                                    tcpTxStart[TCP_PORT_IDX(idx)] = 0;
                                } else {
                                    send(mbSocket[TCP_PORT_IDX(idx)], (uint8_t *)&(tcpTxBuf[TCP_PORT_IDX(idx)][tcpTxStart[TCP_PORT_IDX(idx)]]), 255);
                                    tcpTxStart[TCP_PORT_IDX(idx)] += 255;
                                }
                            } else {
                                tcpSend[TCP_PORT_IDX(idx)] = false;
                            }
                        }
                    }
                    break;
                case SOCK_CLOSE_WAIT:
                    disconnect(mbSocket[TCP_PORT_IDX(idx)]);
                    break;
                case SOCK_CLOSED:
                    socket(mbSocket[TCP_PORT_IDX(idx)],((settings->ethSettings[ETHERNT_SET_FLAGS2] & ETHERNET_FLAGS_REDUND_ISV6)?Sn_MR_TCP6:Sn_MR_TCP4), mbPort[TCP_PORT_IDX(idx)], SF_IO_NONBLOCK);
                    break;
                case SOCK_INIT:
                    //Master actively connects and slave listens
                    if(connecting[TCP_PORT_IDX(idx)]) {
                        //Check for timeout
                        if (getSn_IR(mbSocket[TCP_PORT_IDX(idx)]) & Sn_IR_TIMEOUT) {
                             setSn_IRCLR(TCP_PORT_IDX(idx), Sn_IR_TIMEOUT);
                             connecting[TCP_PORT_IDX(idx)] = false;
                          }
                    } else if(tcpDevType[TCP_PORT_IDX(idx)] == MODBUS_MASTER_DEVICE) {
                        if(tcpSend[TCP_PORT_IDX(idx)]) {
                            curSlave[TCP_PORT_IDX(idx)] = mbSlave[TCP_PORT_IDX(idx)];
                            if(curSlave[TCP_PORT_IDX(idx)] == 1) {
                                if(connect(mbSocket[TCP_PORT_IDX(idx)], (((settings->ethSettings[ETHERNT_SET_FLAGS2] & ETHERNET_FLAGS_REDUND_ISV6)?(uint8_t *)&(settings->ipv6Vals[64]):(uint8_t *)&(settings->ethSettings[ETHERNET_SET_PARTNER_MMSB]))), mbPort[TCP_PORT_IDX(idx)], ((settings->ethSettings[ETHERNT_SET_FLAGS2] & ETHERNET_FLAGS_REDUND_ISV6)?16:4)) == SOCK_BUSY) {
                                    connecting[TCP_PORT_IDX(idx)] = true;
                                }
                            }
                        }
                    } else {
                        listen(mbSocket[TCP_PORT_IDX(idx)]);
                    }
                    break;
                case SOCK_LISTEN:
                    break;
                default :
                    break;
            }

            //Flag receival
            if((tcpRxStart[TCP_PORT_IDX(idx)] != tcpRxStop[TCP_PORT_IDX(idx)]) && (rxTCPIntFunc[TCP_PORT_IDX(idx)] != NULL)) {
                rxTCPIntFunc[TCP_PORT_IDX(idx)](idx);
            }

            //Prime transmit
            if(!tcpSend[TCP_PORT_IDX(idx)] && (tcpTxStop[TCP_PORT_IDX(idx)] != (tcpTxStart[TCP_PORT_IDX(idx)] - 1)) && ((tcpTxStop[TCP_PORT_IDX(idx)] != (sizeof(tcpTxBuf[TCP_PORT_IDX(idx)]) - 1)) || (tcpTxStart[TCP_PORT_IDX(idx)] != 0)) && (txTCPIntFunc[TCP_PORT_IDX(idx)] != NULL)) {
                txTCPIntFunc[TCP_PORT_IDX(idx)](idx);
            }

            //Check for timeouts
            if(tcpTimerActive[TCP_PORT_IDX(idx)] && (get_msec_elapsed(get_sys_ticks(), tcpTimerTicks[TCP_PORT_IDX(idx)]) > 1000)) {
                if(timeoutTCPIntFunc[TCP_PORT_IDX(idx)] != NULL) {
                    timeoutTCPIntFunc[TCP_PORT_IDX(idx)](idx);
                    tcpTimerTicks[TCP_PORT_IDX(idx)] = 0;
                    tcpTimerActive[TCP_PORT_IDX(idx)] = false;
                }
            }
        }
    }
}

void modbus_tcp_set_tx_slave(uint8_t port, uint8_t slaveAddress) {
    if((port != MB_TCP_REDUND_PORT_NUM) && (port != MB_TCP_COMMS_PORT_NUM)) {
        return;
    }

    mbSlave[TCP_PORT_IDX(port)] = slaveAddress;
}
