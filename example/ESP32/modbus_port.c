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
#include "modbus_module.h"

static const char *TAG = "modbus port";

//Local Variables
static uint8_t rx_data = 0;
static bool	rx_received = false;
static bool waiting_for_complete = false;
void (*rxIntFunc)(uint8_t port);            //!< The Receive Callback function
void (*txIntFunc)(uint8_t port);            //!< The Transmit Callback function
void (*timeoutIntFunc)(uint8_t port);       //!< The Timeout Callback function


static int mbSocket = -1;
static int mbPort = -1;
static int mbSlave = -1;
static bool tcpSend = false;
static uint16_t tcpTimerTicks = 0;
static bool tcpTimerActive = false;
static modbus_device_types tcpDevType;
static uint16_t tcpTxStop = 0, tcpTxStart = 0;
static uint8_t tcpTxBuf[MB_TCP_BUF_SIZE] = {0};
static uint16_t tcpRxStop = 0, tcpRxStart = 0;
static uint8_t tcpRxBuf[MB_TCP_BUF_SIZE] = {0};

uint16_t *mb_ticks;

void (*rxTCPIntFunc)(uint8_t port);                //!< The Receive Callback function
void (*txTCPIntFunc)(uint8_t port);                //!< The Transmit Callback function
void (*timeoutTCPIntFunc)(uint8_t port);           //!< The Timeout Callback function

//static const char *TAG = "modbus_events";


//extern uart_dev_t UART0;
//static QueueHandle_t modbus_queue;


void modbus_service_task(void *arg) {
	while(1) {
		modbus_module_service();
		if(waiting_for_complete) {
			waiting_for_complete = false;
			txIntFunc(MB_RTU_PORT_NUM);
		}
		vTaskDelay(2 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


void modbus_task(void *arg)
{
    // Allocate buffers for UART
    uint8_t* data = (uint8_t*) malloc(MODBUS_BUF_SIZE);

    while(1) {
        //Read data from UART
        int len = uart_read_bytes(MODBUS_UART_NUM, data, MODBUS_BUF_SIZE, PACKET_READ_TICS);

        //Write data back to UART
        if (len > 0) {
            for (int i = 0; i < len; i++) {
            	rx_data = data[i];
            	rxIntFunc(MB_RTU_PORT_NUM);
            }
            rx_received = true;
        } else {

        	if(rx_received) {
        		timeoutIntFunc(MB_RTU_PORT_NUM);
        		rx_received = false;
        	}
        }
    }
    free(data);
    data = NULL;
    vTaskDelete(NULL);
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
    //int32_t test, psc, timeArr, mbTimeout;
    //uint32_t tempBaud;
    uint16_t tempStopBits, tempCharSize;
    uart_parity_t tempParity;

    //We only have one port!
    if(port != MB_RTU_PORT_NUM) {
        return false;
    }

    //Bind our callbacks
    rxIntFunc = rxCallback;
    txIntFunc = txCallback;
    timeoutIntFunc = timeoutCallback;


    //Register our interrupt handler
//    Enabled = false;
//    inInterrupt = false;

    if (parity == MODBUS_PARITY_NONE){
        tempParity = UART_PARITY_DISABLE;
    } else if (parity == MODBUS_PARITY_ODD) {
        tempParity = UART_PARITY_ODD;
    } else {
        tempParity = UART_PARITY_EVEN;
    }

    // Set number of stop bits
    if (stopBits == 1){
        tempStopBits = UART_STOP_BITS_1;
    } else {
        tempStopBits = UART_STOP_BITS_2;
    }

    // Set character size
    switch(charSize) {
        case 5:
            tempCharSize = UART_DATA_5_BITS;
        break;
        case 6:
            tempCharSize = UART_DATA_6_BITS;
        break;
        case 7:
            tempCharSize = UART_DATA_7_BITS;
        break;
        case 8:
        	tempCharSize = UART_DATA_8_BITS;
        break;
        default:
            tempCharSize = UART_DATA_8_BITS;
        break;
    }

    uart_config_t uart_config = {
            .baud_rate = baudRate,
            .data_bits = tempCharSize,
            .parity = tempParity,
            .stop_bits = tempStopBits,
			.rx_flow_ctrl_thresh = 122,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
	};

    ESP_ERROR_CHECK(uart_driver_install(MODBUS_UART_NUM, MODBUS_BUF_SIZE * 2, MODBUS_BUF_SIZE * 2, 20, NULL, 0));

	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(MODBUS_UART_NUM, &uart_config));

	// Set UART pins as per KConfig settings
	ESP_ERROR_CHECK(uart_set_pin(MODBUS_UART_NUM, MODBUS_TRANSMIT, MODBUS_RECEIVE, MODBUS_RTS, MODBUS_CTS));

	// Set RS485 half duplex mode
	ESP_ERROR_CHECK(uart_set_mode(MODBUS_UART_NUM, UART_MODE_UART));

	ESP_ERROR_CHECK(uart_set_rx_timeout(MODBUS_UART_NUM, MODBUS_TOUT));

//    Enabled = true;

    //Enable The UART
    modbus_port_serial_enable(port,false,false);

    // Sanity Check for Modbus Connection
//    while(true){
//    	uint16_t buf[1] = {'U'};
//   	    uart_write_bytes(MODBUS_UART_NUM, buf, 1);
//        vTaskDelay(100 / portTICK_PERIOD_MS);
//    }

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
    if(port != MB_RTU_PORT_NUM) {
        return;
    }

    if(txEnable) {
    	waiting_for_complete = true;
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
    //We only have one port, so if not it then abort
    if(port >= NUM_MODBUS_PORTS) {
        return;
    }

    if(port == MB_TCP_PORT_NUM){
    	// TODO: Implement close socket here. Example is close(socket);
//		disconnect(mbSocket);
    } else {
    	//Turn off RX and TX
    	uart_driver_delete(MODBUS_UART_NUM);
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
	 if(port == MB_TCP_PORT_NUM){
		tcpSend = true;
	} else {
		waiting_for_complete = true;
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
    if(port != MB_RTU_PORT_NUM) {
        return;
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
	uint16_t buf[1] = {sndByte};

    //We only have one port, so if not it then abort
	if(port >= NUM_MODBUS_PORTS) {
		return false;
	}
	 if(port == MB_TCP_PORT_NUM) {
		if((tcpTxStop != (tcpTxStart-1)) && ((tcpTxStop != (sizeof(tcpTxBuf)-1)) || (tcpTxStart != 0))) {
			tcpTxBuf[tcpTxStop++] = sndByte;
			if(tcpTxStop >= sizeof(tcpTxBuf)) {
				tcpTxStop = 0;
			}
			if((tcpTxStop == (tcpTxStart-1)) || ((tcpTxStop == (sizeof(tcpTxBuf)-1)) && (tcpTxStart == 0))) {
				tcpSend = true;
			}
			return true;
		}
	} else {
		uart_write_bytes(MODBUS_UART_NUM, buf, 1);
		waiting_for_complete = true;
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
	if(port == MB_TCP_PORT_NUM){
		 if(tcpRxStart != tcpRxStop) {
			*getByte = tcpRxBuf[tcpRxStart++];
			if(tcpRxStart >= sizeof(tcpRxBuf)) {
				tcpRxStart = 0;
			}
			return true;
		}
	} else {
		*getByte = rx_data;
		return true;
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

	if(port == MB_TCP_PORT_NUM){
		if(enable) {
			tcpTimerTicks = *mb_ticks;
			tcpTimerActive = true;
		} else {
			tcpTimerTicks = 0;
			tcpTimerActive = false;
		}
	} else {
		// UART Timer Timeout handled by library for ESP32
	}
}

bool modbus_port_tcp_init(uint8_t modPort, uint16_t tcpPort, uint16_t timeoutSec, modbus_device_types deviceType,
                      void (*rxCallback)(uint8_t port), void (*txCallback)(uint8_t port), void (*timeoutCallback)(uint8_t port)) {
    if(modPort != MB_TCP_PORT_NUM) {
        return false;
    }

    //Bind our callbacks
    rxTCPIntFunc = rxCallback;
    txTCPIntFunc = txCallback;
    timeoutTCPIntFunc = timeoutCallback;

    if(tcpPort != 0) {
        mbPort = (unsigned short)tcpPort;
    } else {
        mbPort = MB_TCP_DEFAULT_PORT;
    }

    tcpSend = false;
    mbSocket = modPort;
    tcpDevType = deviceType;

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

void modbus_tcp_task(void *pvParameters)
{
	mb_ticks = (uint16_t *)pvParameters;
    char addr_str[128];
#if(IP_TYPE == IPV4)
   int addr_family = AF_INET;
#elif(IP_TYPE == IPV6)
    addr_family = AF_INET6;
#endif

    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

#if(IP_TYPE == IPV4)
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(MB_TCP_DEFAULT_PORT);
        ip_protocol = IPPROTO_IP;
    }

#elif (IP_TYPE == IPV6)
    if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if (IP_TYPE == IPV4) && (IP_TYPE == IPV6)
    // Note that by default IPV6 binds to both protocols, it must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

//    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
//    ESP_LOGI(TAG, "Socket bound, port %d", MB_TCP_DEFAULT_PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    if (modbus_module_init(MB_TCP_PORT_NUM, MODBUS_TCP, MODBUS_SLAVE_DEVICE, 1, 0, MODBUS_PARITY_NONE, 1, 502) != MODBUS_STAT_SUCCESS) {
    	ESP_LOGE(TAG, "Modbus Init Failed");
    }

    modbus_module_enable(MB_TCP_PORT_NUM, true);


    ESP_LOGI(TAG, "Socket listening");

	struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
	socklen_t addr_len = sizeof(source_addr);
	mbSocket = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
	if (mbSocket < 0) {
		ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
		goto CLEAN_UP;
	}

	// Set tcp keepalive option
	setsockopt(mbSocket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
	setsockopt(mbSocket, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
	setsockopt(mbSocket, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
	setsockopt(mbSocket, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
	// Convert ip address to string
#if (IP_TYPE == IPV4)
	if (source_addr.ss_family == PF_INET) {
		inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
	}
#elif (IP_TYPE == IPV6)
	if (source_addr.ss_family == PF_INET6) {
		inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
	}
#endif
//	ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

    while (1) {
        int len;
        int bufRemaining = tcp_buf_space_left(sizeof(tcpRxBuf), tcpRxStart, tcpRxStop);
	    do {
	    	if(bufRemaining > (sizeof(tcpRxBuf) - tcpRxStop)) {
				len = recv(mbSocket, (uint8_t *)&(tcpRxBuf[tcpRxStop]), (sizeof(tcpRxBuf) - tcpRxStop), MSG_DONTWAIT);
			} else {
				len = recv(mbSocket, (uint8_t *)&(tcpRxBuf[tcpRxStop]), bufRemaining, MSG_DONTWAIT);
			}
			if(len >= 0) {
				bufRemaining -= len;

				tcpRxStop += len;
				if(tcpRxStop >= sizeof(tcpRxBuf)) {
					tcpRxStop -= sizeof(tcpRxBuf);
				}
//				ESP_LOGI(TAG, "Received %d bytes", len);
			} else {
				break;
			}
	    } while (len > 0);

	    // Send Any data out
	    if(tcpSend) {
//	    	int to_send = 0;
			if(tcpTxStart < tcpTxStop) {
				if(tcpTxStop - tcpTxStart < 256) {
					send(mbSocket, (uint8_t *)&(tcpTxBuf[tcpTxStart]), (tcpTxStop - tcpTxStart), 0);
					tcpTxStart = tcpTxStop;
					tcpSend = false;
				} else {
					send(mbSocket, (uint8_t *)&(tcpTxBuf[tcpTxStart]), 255, 0);
					tcpTxStart += 255;
				}
			} else if(tcpTxStart > tcpTxStop) {
				if(sizeof(tcpTxBuf) - tcpTxStart < 256) {
					send(mbSocket, (uint8_t *)&(tcpTxBuf[tcpTxStart]), (sizeof(tcpTxBuf) - tcpTxStart), 0);
					tcpTxStart = 0;
				} else {
					send(mbSocket, (uint8_t *)&(tcpTxBuf[tcpTxStart]), 255, 0);
					tcpTxStart += 255;
				}
			} else {
				tcpSend = false;
			}
//			ESP_LOGI(TAG, "to_send = %d\r\n", to_send);
		}

		//Flag received
		while((tcpRxStart != tcpRxStop) && (rxTCPIntFunc != NULL)) {
			rxTCPIntFunc(MB_TCP_PORT_NUM);
		}

		//Prime transmit
		if(!tcpSend && (tcpTxStop != (tcpTxStart - 1)) && ((tcpTxStop != (sizeof(tcpTxBuf) - 1)) || (tcpTxStart != 0)) && (txTCPIntFunc != NULL)) {
			txTCPIntFunc(MB_TCP_PORT_NUM);
		}

		//Check for timeouts
		if(tcpTimerActive && ((*mb_ticks - tcpTimerTicks) > 250)) {
			if(timeoutTCPIntFunc != NULL) {
				timeoutTCPIntFunc(MB_TCP_PORT_NUM);
			}
		}
    }
    shutdown(mbSocket, 0);
    close(mbSocket);

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void modbus_tcp_set_tx_slave(uint8_t port, uint8_t slaveAddress) {
    if (port != MB_TCP_PORT_NUM) {
        return;
    }

    mbSlave = slaveAddress;
}
