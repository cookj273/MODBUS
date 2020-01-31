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
//#include <asf.h>
#include "modbus_port.h"
#include "project.h"

//Local Variables
static volatile bool inTimInterrupt = false;   	//!< Tracks when we are in the timer interrupt to avoid disables
static volatile bool inUsartInterrupt = false;	//!< Tracks when we are in the Usart interrupt to avoid disables
uint16_t timeArr = 0;							//!< Auto-reload value for the timer
void (*rxIntFunc)(uint8_t port);            	//!< The Receive Callback function
void (*txIntFunc)(uint8_t port);            	//!< The Transmit Callback function
void (*timeoutIntFunc)(uint8_t port);       	//!< The Timeout Callback function

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

/*!
* @brief Interrupt handler for modbus USART
*
* Used to receive and send bytes for modbus
* communications
*/
void USART1_IRQHandler(void) {
	inUsartInterrupt = true;

	if(LL_USART_IsEnabledIT_RXNE(MB_USART) && LL_USART_IsActiveFlag_RXNE(MB_USART)) {
        if(rxIntFunc) {
            rxIntFunc(MB_PORT_NUM);
        } else {
            LL_USART_ReceiveData8(MB_USART);
        }
	} else if((LL_USART_IsEnabledIT_TC(MB_USART) && LL_USART_IsActiveFlag_TC(MB_USART)) || (LL_USART_IsEnabledIT_TXE(MB_USART) && LL_USART_IsActiveFlag_TXE(MB_USART))) {
        //Call our tx int callback function if registered
        if(txIntFunc) {
            txIntFunc(MB_PORT_NUM);
        } else {
            //No reload function was defined so disable the int here
        	LL_USART_DisableIT_TC(MB_USART);
        	LL_USART_DisableIT_TXE(MB_USART);
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
void TIM3_IRQHandler(void) {
	inTimInterrupt = true;

	//Make sure it was an update event
	if(MB_TIMER->SR & TIM_SR_UIF) {
		//Call our callback
		if(timeoutIntFunc) {
			timeoutIntFunc(MB_PORT_NUM);
		} else {
			//No clear function was defined so disable the timer here
			modbus_port_timer_enable(MB_PORT_NUM, false);
		}

		//Clear all flags
		MB_TIMER->SR = 0;
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
	int test, psc, mbTimeout;

	 //We only have one port!
	if(port != MB_PORT_NUM) {
		return false;
	}

	//Char size > 8 we can't support if parity is on (9th bit needed for parity)
	if(((charSize > 8) && (parity != MODBUS_PARITY_NONE)) || (parity > 9)) {
		return false;
	}

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;

	// Enable the peripheral clock of GPIOB
	LL_AHB1_GRP1_EnableClock(MB_GPIO_PERIPH_CLK);

	// Configure TX Pin as : Alternate function, High Speed, PushPull, Pull up
	LL_GPIO_SetPinMode(MB_TX_PIN_PORT, MB_TX_PIN_NUM, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(MB_TX_PIN_PORT, MB_TX_PIN_NUM, MB_TX_PIN_MODE);
	LL_GPIO_SetPinSpeed(MB_TX_PIN_PORT, MB_TX_PIN_NUM, LL_GPIO_SPEED_FREQ_HIGH);
	//LL_GPIO_SetPinOutputType(MB_TX_PIN_PORT, MB_TX_PIN_NUM, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(MB_TX_PIN_PORT, MB_TX_PIN_NUM, LL_GPIO_PULL_UP);

	// Configure RX Pin as : Alternate function, High Speed, PushPull, Pull up
	LL_GPIO_SetPinMode(MB_RX_PIN_PORT, MB_RX_PIN_NUM, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(MB_RX_PIN_PORT, MB_RX_PIN_NUM, MB_RX_PIN_MODE);
	LL_GPIO_SetPinSpeed(MB_RX_PIN_PORT, MB_RX_PIN_NUM, LL_GPIO_SPEED_FREQ_HIGH);
	//LL_GPIO_SetPinOutputType(MB_RX_PIN_PORT, MB_RX_PIN_NUM, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(MB_RX_PIN_PORT, MB_RX_PIN_NUM, LL_GPIO_PULL_UP);

	//Configure RX Enable pin
	LL_GPIO_SetPinMode(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM, LL_GPIO_PULL_DOWN);

	//Configure TX Enable pin
#if MB_FULL_DUPLEX
	LL_GPIO_SetPinMode(MB_TXEN_PIN_PORT, MB_TXEN_PIN_NUM, LL_GPIO_MODE_OUTPUT);
#else
	LL_GPIO_SetPinMode(MB_TXEN_PIN_PORT, MB_TXEN_PIN_NUM, LL_GPIO_MODE_INPUT);
#endif
	LL_GPIO_SetPinSpeed(MB_TXEN_PIN_PORT, MB_TXEN_PIN_NUM, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(MB_TXEN_PIN_PORT, MB_TXEN_PIN_NUM, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(MB_TXEN_PIN_PORT, MB_TXEN_PIN_NUM, LL_GPIO_PULL_DOWN);

	//Configure RS485 Enable pin and turn ON
	LL_GPIO_SetPinMode(MB_485EN_PIN_PORT, MB_485EN_PIN_NUM, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(MB_485EN_PIN_PORT, MB_485EN_PIN_NUM, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(MB_485EN_PIN_PORT, MB_485EN_PIN_NUM, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(MB_485EN_PIN_PORT, MB_485EN_PIN_NUM, LL_GPIO_PULL_UP);
	LL_GPIO_ResetOutputPin(MB_485EN_PIN_PORT, MB_485EN_PIN_NUM);

	// Set priority for USART IRQn
	NVIC_SetPriority(MB_USART_IRQ, 0);

	// Enable the USART Clock
	LL_APB2_GRP1_EnableClock(MB_USART_PERIPH_CLK);

	// Disable USART1 prior modifying configuration registers and by default
	LL_USART_Disable(MB_USART);

	// TX/RX direction - both disable to start
	LL_USART_SetTransferDirection(MB_USART, LL_USART_DIRECTION_NONE);

	// Configure according to parameters
	LL_USART_ConfigCharacter(MB_USART, (((charSize == 9) || ((charSize == 8) && (parity != MODBUS_PARITY_NONE)))?LL_USART_DATAWIDTH_9B:LL_USART_DATAWIDTH_8B), ((parity == MODBUS_PARITY_EVEN)?LL_USART_PARITY_EVEN:((parity == MODBUS_PARITY_ODD)?LL_USART_PARITY_ODD:LL_USART_PARITY_NONE)), ((stopBits == 2)?LL_USART_STOPBITS_2:LL_USART_STOPBITS_1));

	// No Hardware Flow Control we will move the pin manually
	LL_USART_SetHWFlowCtrl(MB_USART, LL_USART_HWCONTROL_NONE);

	// Oversampling by 16
	LL_USART_SetOverSampling(MB_USART, LL_USART_OVERSAMPLING_16);

	// Set Baudrate to correct value using APB frequency set to 100000000 Hz
	// Frequency available for USART peripheral can also be calculated through LL RCC macro
	/* Ex :
	  Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
	  In this example, Peripheral Clock is expected to be equal to 100000000 Hz => equal to SystemCoreClock
	*/
	LL_USART_SetBaudRate(MB_USART, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudRate);

	//TEST FUNCTION COMMENT OUT NORMALLY!!!
	//port_loopback_test();

	//Setup our timer
	LL_APB1_GRP1_EnableClock(MB_TIM_PERIPH_CLK);
	MB_TIMER->CR1 = 0;   // Disable timer
	NVIC_DisableIRQ(MB_TIMER_IRQ); // Disable IRQ

	//Turn the bits into timer clocks (time clock = Sysclk/AHB * 2 /APB1)
	mbTimeout = ((int)timeoutBits * ((int)(((SystemCoreClock/CLK_AHB_DIVIDER_VAL)*2)/CLK_APB1_DIVIDER_VAL)/(int)baudRate));

	//Now lets find a prescaler and ARR that works
	test = -1;
	psc = 0;
	do {
		test++;
		if(mbTimeout%(test+1) == 0) {
			psc = test;
		}
	} while(((mbTimeout/(test+1)) > 1000) && (psc < 65536));
	timeArr = (mbTimeout/(psc+1));
	MB_TIMER->PSC = psc;	        	// Set prescaler
	MB_TIMER->ARR = timeArr;			// Auto reload value
	MB_TIMER->DIER = TIM_DIER_UIE; 		// Enable update interrupt (timer level)

	//TEST TIMER
	//modbus_port_timer_enable(port, true);
	//NVIC_EnableIRQ(MB_TIMER_IRQ);
	//while(1);

	//Enable the USART
	LL_USART_Enable(MB_USART);

	// Setup the correct mode
	modbus_port_serial_enable(port, false, false);

	LL_USART_EnableIT_RXNE(MB_USART);
	LL_USART_EnableIT_ERROR(MB_USART);

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
	if(port != MB_PORT_NUM) {
		return;
	}

	//Disable Interrupts
	NVIC_DisableIRQ(MB_USART_IRQ);
	NVIC_DisableIRQ(MB_TIMER_IRQ);

	//Disable tx complete int
	LL_USART_DisableIT_TC(MB_USART);

	//Set the RX up
	if(rxEnable) {
		//Configure the USART rx interrupt
		LL_USART_EnableIT_RXNE(MB_USART);

		//Start the timer
		MB_TIMER->CNT = 0;
		MB_TIMER->CR1 = TIM_CR1_CEN;   // Enable timer

		//RX Pin mode
		LL_GPIO_ResetOutputPin(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM);
	} else {
		LL_USART_DisableIT_RXNE(MB_USART);
		MB_TIMER->CR1 = 0;   // Disable timer

		//RX Pin mode
		LL_GPIO_SetOutputPin(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM);
	}

	//Set the TX up
	if(txEnable && !rxEnable) {
		//Configure the USART tx interrupt
		LL_USART_EnableIT_TXE(MB_USART);

		//TX Pin Mode if full duplex
#if MB_FULL_DUPLEX
		LL_GPIO_ResetOutputPin(MB_TXEN_GPIO_PORT, MB_TXEN_GPIO_PIN);
#endif
	} else {
		LL_USART_DisableIT_TXE(MB_USART);

		//TX Pin Mode if full duplex
#if MB_FULL_DUPLEX
		LL_GPIO_ResetOutputPin(MB_TXEN_GPIO_PORT, MB_TXEN_GPIO_PIN);
#endif
	}

	//Set the appropriate direction
	LL_USART_SetTransferDirection(MB_USART, ((rxEnable?LL_USART_DIRECTION_RX:LL_USART_DIRECTION_NONE) | (txEnable?LL_USART_DIRECTION_TX:LL_USART_DIRECTION_NONE)));

	//Reenable the interrupts
	NVIC_EnableIRQ(MB_USART_IRQ);
	NVIC_EnableIRQ(MB_TIMER_IRQ);
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
	if(port != MB_PORT_NUM) {
		return;
	}

	//Disable all the interrupts
	NVIC_DisableIRQ(MB_TIMER_IRQ);
	NVIC_DisableIRQ(MB_USART_IRQ);
	LL_USART_DisableIT_TC(MB_USART);
	LL_USART_DisableIT_TXE(MB_USART);
	LL_USART_DisableIT_RXNE(MB_USART);

	//Turn off RX and TX
	LL_USART_SetTransferDirection(MB_USART, LL_USART_DIRECTION_NONE);

	//Turn all our pins to input to go to power save
	LL_GPIO_SetPinMode(MB_TX_PIN_PORT, MB_TX_PIN_NUM, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(MB_RX_PIN_PORT, MB_RX_PIN_NUM, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(MB_RXEN_PIN_PORT, MB_RXEN_PIN_NUM, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(MB_TXEN_PIN_PORT, MB_TXEN_PIN_NUM, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetOutputPin(MB_485EN_PIN_PORT, MB_485EN_PIN_NUM);

	//Disable the UART
	LL_USART_Disable(MB_USART);

	//Disable the timer
	MB_TIMER->CR1 = 0;

	//Turn the Clock off
	LL_APB2_GRP1_DisableClock(MB_USART_PERIPH_CLK);
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
	if(port != MB_PORT_NUM) {
		return;
	}

	LL_USART_DisableIT_TXE(MB_USART);
	LL_USART_EnableIT_TC(MB_USART);
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
	if(!inUsartInterrupt && !inTimInterrupt && enable) {
		NVIC_EnableIRQ(MB_USART_IRQ);
		NVIC_EnableIRQ(MB_TIMER_IRQ);
	} else if(!inUsartInterrupt && !inTimInterrupt) {
		NVIC_DisableIRQ(MB_USART_IRQ);
		NVIC_DisableIRQ(MB_TIMER_IRQ);
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

    if(LL_USART_IsActiveFlag_TXE(MB_USART)) {
    	LL_USART_TransmitData8(MB_USART, sndByte);
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
    if(port != MB_PORT_NUM) {
        return false;
    }

    //Read the character if no errors
    if(!LL_USART_IsActiveFlag_PE(MB_USART) && !LL_USART_IsActiveFlag_FE(MB_USART)
		&& !LL_USART_IsActiveFlag_NE(MB_USART) && !LL_USART_IsActiveFlag_ORE(MB_USART)
		&& LL_USART_IsActiveFlag_RXNE(MB_USART)) {
    	*getByte = LL_USART_ReceiveData8(MB_USART);
    	return true;
	} else {
		//Clear errors and start fresh
		LL_USART_ClearFlag_PE(MB_USART);
		LL_USART_ClearFlag_FE(MB_USART);
		LL_USART_ClearFlag_NE(MB_USART);
		LL_USART_ClearFlag_ORE(MB_USART);
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
    //We only have one port if not it abort
    if(port != MB_PORT_NUM) {
        return;
    }

    //Enable or disable as requested
    if(enable) {
    	//Even if already enabled reset the count here
    	MB_TIMER->CNT = 0;
    	MB_TIMER->CR1 = TIM_CR1_CEN;   // Enable timer
    } else {
    	MB_TIMER->CR1 = 0;   // Disable timer
    }
}
