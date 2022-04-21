

#ifndef MODBUS_TCP_H_
#define MODBUS_TCP_H_

//Only implement Modbus TCP if enabled
#if MODBUS_TCP_ENABLED > 0

#include "conf_modbus.h"
#include "modbus_module.h"

//*****GLOBAL DEFINES
#define MB_TCP_PSEUDO_ADDRESS   255         //Placeholder address

//*****FUNCTION DEFINITIONS
modbus_status modbus_tcp_init(uint8_t mbPort, uint16_t tcpPort, modbus_device_types deviceType);

void modbus_tcp_close(uint8_t port);

modbus_status modbus_tcp_receive(uint8_t port, uint8_t *rcvAddress, uint8_t **inFrame, uint16_t *inLength);

modbus_status modbus_tcp_transmit(uint8_t port, uint8_t slaveAddress, const uint8_t *inFrame, uint16_t inLength);

void modbus_tcp_release_tx_buffer(uint8_t port);
bool modbus_tcp_get_tx_buffer(uint8_t port, uint8_t **buf);
void modbus_tcp_no_response(uint8_t port);
bool modbus_tcp_transmit_complete(uint8_t port);
bool modbus_tcp_has_frames(uint8_t port);

#endif
#endif
