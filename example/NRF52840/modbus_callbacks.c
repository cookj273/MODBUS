/*!
* @file modbus_callbacks.h
* @author Jarrod Cook
*
* @brief Callback functions for retrieving program data for MODBUS
*
* This file contains implementation of stubbed functions in modbus_callbacks.h
* the intent is for these functions to retrieve or set requested program
* registers according to MODBUS commands.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

//Includes
#include <time.h>
#include "modbus_callbacks.h"
#include "global.h"
#include "timer.h"
#include "fds_handler.h"
#include "wiz_spi.h"

//Defines
#define COIL_START_ADDRESS              0       //!< Start Address of MODBUS coils
#define DISCRETE_START_ADDRESS          0       //!< Start Address of MODBUS discrete inputs
#define INPUT_START_ADDRESS             0       //!< Start Address of MODBUS input registers
#define HOLDING_START_ADDRESS           0       //!< Start Address of MODBUS holding registers

//Typedefs
/*!
* The following types defines all available coils
*/
typedef enum {
    COIL_MANUAL_COOL = 0,
    COIL_MANUAL_HEAT,
    COIL_HOT_ALARM,
    COIL_COLD_ALARM,
    COIL_COND_ALARM,
    COIL_FILTER_ALARM,
    COIL_ALARMS_ON,
    COIL_UNIT_IS_CELSIUS,
    COIL_REDUND_IS_LEAD,
    COIL_REDUND_TIME_BASIS_IS_DAYS,
    COIL_POWER_ON,
    COIL_POA_ALARM_ON,
    COIL_STEALTH_MODE,
    COIL_DC_MOTOR_ON,
    COIL_ETHERNET_ENABLED,
    COIL_DHCP_ENABLED,
    COIL_RESET_FILTER_DAYS,
    COIL_POA_ALARM,
    COIL_HI_ENC_ALARM,
    COIL_LO_ENC_ALARM,
    COIL_HI_CONDENSE_ALARM,
    COIL_FILTER_ALARM_ACTIVE,
    COIL_SENS1_OPEN_ALARM,
    COIL_SENS2_OPEN_ALARM,
    COIL_SENS1_SHORT_ALARM,
    COIL_SENS2_SHORT_ALARM,
    COIL_NETWORK_ALARM,
    COIL_OW_TEMP_ALARM,
    COIL_FROZEN_ALARM,
    COIL_10_MIN_MANUAL,
    COIL_LAST_ONE               //!< Marker of end of coil range
} board_coils;

/*!
* The following types defines all available discrete inputs
*/
typedef enum {
    DISCRETE_COOL_INSTALLED = 0,
    DISCRETE_HEAT_INSTALLED,
    DISCRETE_LAST_ONE           //!< Marker of end of discrete range
} board_discretes;

/*!
* The following types defines all available input registers
*/
typedef enum {
    INPUT_FILTER_LEFT = 0,                    //!< 
    INPUT_HEAT_SPEED,
    INPUT_COOL_SPEED,
    INPUT_ENC_TEMP_WUNITS,
    INPUT_COND_TEMP_WUNITS,
    INPUT_FREEZE_TEMP_WUNITS,
    INPUT_SYSTEM_STATE,
    INPUT_REDUND_STATE,
    INPUT_LAST_ONE                      //!< Marking of the end of the input registers
} board_inputs;

/*!
* The following types defines all available holding registers
*/
typedef enum {
    HOLDING_TEMP_SET_LO = 0,
    HOLDING_TEMP_SET_HI,
    HOLDING_TEMP_SET_DELAY,
    HOLDING_TEMP_SET_UNIT,
    HOLDING_TEMP_SET_VARIANT_LO,
    HOLDING_TEMP_SET_VARIANT_HI,
    HOLDING_TEMP_ALARM_LO,
    HOLDING_TEMP_ALARM_HI,
    HOLDING_TEMP_ALARM_COND,
    HOLDING_ETH_FLAGS,
    HOLDING_ETH_OFFSETS,
    HOLDING_UNIT_RUNTIME,                  //!< 
    HOLDING_ADV_OPT,
    HOLDING_ADV_ALARMS,
    HOLDING_FILTER_DAYS,
    HOLDING_FILTER_WARN,
    HOLDING_SENS_OFFSET_AIR,
    HOLDING_SENS_OFFSET_CONDENSE,
    HOLDING_TEMP_ALARM_LO_WUNITS,
    HOLDING_TEMP_ALARM_HI_WUNITS,
    HOLDING_TEMP_ALARM_COND_WUNITS,
    HOLDING_AUDIBLE_ALARM,
    HOLDING_REDUND_ON,
    HOLDING_REDUND_SWAP_TIME,
    HOLDING_TEMP_SET_LO_WUNITS,
    HOLDING_TEMP_SET_HI_WUNITS,
    HOLDING_MODBUS_SLAVE_ADDRESS,		//!< The MODBUS address assigned to this slave
	HOLDING_MODBUS_BAUD_DIV100,			//!< MODBUS Baud Rate
	HOLDING_MODBUS_PARITY,				//!< The parity to use for MODBUS comms
	HOLDING_MODBUS_STOP_BITS,			//!< Number of stop bits for MODBUS comms
    HOLDING_UNIT_NAME_C0_1,
    HOLDING_UNIT_NAME_C2_3,
    HOLDING_UNIT_NAME_C4_5,
    HOLDING_UNIT_NAME_C6_7,
    HOLDING_UNIT_NAME_C8_9,
    HOLDING_UNIT_NAME_C10_11,
    HOLDING_UNIT_NAME_C12,
    HOLDING_CABINET_NAME_C0_1,
    HOLDING_CABINET_NAME_C2_3,
    HOLDING_CABINET_NAME_C4_5,
    HOLDING_CABINET_NAME_C6_7,
    HOLDING_CABINET_NAME_C8_9,
    HOLDING_CABINET_NAME_C10_11,
    HOLDING_CABINET_NAME_C12,
    HOLDING_HEAT_VARIANCE,
    HOLDING_COOL_VARIANCE,
    HOLDING_HEAT_DELAY,
    HOLDING_COMP_DELAY,
    HOLDING_FREEZE_SHUTOFF_WUNITS,
    HOLDING_UNFREEZE_ENABLE_WUNITS,
    HOLDING_IPV4_ADD_HW,
    HOLDING_IPV4_ADD_LW,
    HOLDING_IPV4_SUB_HW,
    HOLDING_IPV4_SUB_LW,
    HOLDING_IPV4_GW_HW,
    HOLDING_IPV4_GW_LW,
    HOLDING_SNMP_IP_HW,
    HOLDING_SNMP_IP_LW,
    HOLDING_IPV6_LLA_W1,
    HOLDING_IPV6_LLA_W2,
    HOLDING_IPV6_LLA_W3,
    HOLDING_IPV6_LLA_W4,
    HOLDING_IPV6_LLA_W5,
    HOLDING_IPV6_LLA_W6,
    HOLDING_IPV6_LLA_W7,
    HOLDING_IPV6_LLA_W8,
    HOLDING_IPV6_GUA_W1,
    HOLDING_IPV6_GUA_W2,
    HOLDING_IPV6_GUA_W3,
    HOLDING_IPV6_GUA_W4,
    HOLDING_IPV6_GUA_W5,
    HOLDING_IPV6_GUA_W6,
    HOLDING_IPV6_GUA_W7,
    HOLDING_IPV6_GUA_W8,
    HOLDING_IPV6_SN6_W1,
    HOLDING_IPV6_SN6_W2,
    HOLDING_IPV6_SN6_W3,
    HOLDING_IPV6_SN6_W4,
    HOLDING_IPV6_SN6_W5,
    HOLDING_IPV6_SN6_W6,
    HOLDING_IPV6_SN6_W7,
    HOLDING_IPV6_SN6_W8,
    HOLDING_IPV6_GW6_W1,
    HOLDING_IPV6_GW6_W2,
    HOLDING_IPV6_GW6_W3,
    HOLDING_IPV6_GW6_W4,
    HOLDING_IPV6_GW6_W5,
    HOLDING_IPV6_GW6_W6,
    HOLDING_IPV6_GW6_W7,
    HOLDING_IPV6_GW6_W8,
    HOLDING_REDUND_OFFSET,
    HOLDING_REDUND_IP_HW,
    HOLDING_REDUND_IP_LW,
    HOLDING_IPV6_REDUND_W1,
    HOLDING_IPV6_REDUND_W2,
    HOLDING_IPV6_REDUND_W3,
    HOLDING_IPV6_REDUND_W4,
    HOLDING_IPV6_REDUND_W5,
    HOLDING_IPV6_REDUND_W6,
    HOLDING_IPV6_REDUND_W7,
    HOLDING_IPV6_REDUND_W8,
    HOLDING_REDUND_MASTER_STATE,
    HOLDING_LAST_ONE,                      //!< Marker of the end of the holding registers
} board_holding;

/*!
* The following structure is used for tracking item changes during
* modbus writes.
*/
typedef struct {
    bool changed;       //!< Indicates if the value has changed or not
    uint32_t val;       //!< The current value of the setting
} reg_stat;

//Local Variables
static uint32_t mbRedunActiveSecs = 0;
static uint32_t mbRTUActiveSecs = 0;
static unit_settings *settingPtr = NULL;
static unit_state *statePtr = NULL;
static bool holdChange[NUM_MODBUS_PORTS] = {false};
static bool coilChange[NUM_MODBUS_PORTS] = {false};
static struct {
  uint8_t mbAddress;
  uint16_t mbBaud;
  uint8_t mbParity;
  uint8_t mbStopBits;
} tempHoldRegs[NUM_MODBUS_PORTS];

/*!
* @brief The following function is called when a frame fully completes
*
* This function is intended to handle tasks that need to happen post frame
* completion.  Basically things like resetting the controller and changing
* settings where we need to issue a response before performing the
* changes.
*
* @param port The port the frame was completed on
* @param completedFunc The function that was just completed
*/
void modbus_frame_complete_callback(uint8_t port, modbus_function_codes completedFunc)
{
    uint8_t idx;
    time_t systemTime;
    struct tm *timePtr;
    bool isChange = false;

    //We only have one port so if its not it then we shouldn't be here
    if(port >= NUM_MODBUS_PORTS) {
        return;
    }

    //Save settings if any changed
    if(holdChange[port]) {
        holdChange[port] = false;
        settingPtr->advSettings[ADV_SET_MODBUS_SLAVE_ADDRESS] = tempHoldRegs[port].mbAddress;
        settingPtr->advSettings[ADV_SET_MODBUS_BAUD_MSB] = ((tempHoldRegs[port].mbBaud >> 8) & 0xFF);
        settingPtr->advSettings[ADV_SET_MODBUS_BAUD_LSB] = (tempHoldRegs[port].mbBaud & 0xFF);
        settingPtr->advSettings[ADV_SET_MODBUS_UART_OPTS] = ((tempHoldRegs[port].mbParity << 4) & 0xF0) | (tempHoldRegs[port].mbStopBits & 0x0F);
        settingPtr->cabName[sizeof(settingPtr->cabName) - 1] = '\0';
        settingPtr->name[sizeof(settingPtr->name) - 1] = '\0';
        isChange = true;
    }
    if(coilChange[port]){
      coilChange[port] = false;
      isChange = true;
    }
    if(isChange) {
      fds_update_device_config(settingPtr);
    }
}

/*!
* @brief The following function gets device info for this board
*
* This function handles the get device info MODBUS command.  It simply
* returns the additional device information as defined within the manual.
*
* @param port The port this request came over
* @param devInfo Pointer to the frame to place data into
*
* @return The number of bytes written
*/
uint8_t modbus_device_info_callback(uint8_t port, uint8_t *devInfo)
{
    //We only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return 0;
    }

    devInfo[0] = 0x00; //SERVER ID (can be longer)
    devInfo[1] = 0xFF; //Run Status 0xFF = on, ox00 = off
    devInfo[2] = 0x00; //Additional info (can be longer)

    return 3;
}

/*!
* @brief Gets the start address of coils for a port
*
* This function simply returns the address at which coils start
* for this port.
*
* @param port The port to get the start address for
*
* @return The coil start address
*/
uint16_t modbus_coil_start_address(uint8_t port)
{
    return COIL_START_ADDRESS;
}

/*!
* @brief Gets the total number of available coils
*
* This function simply returns the number of coils available in the
* system.
*
* @param port The port this request came over
*
* @return The number of available coils
*/
uint16_t modbus_number_of_coils(uint8_t port)
{
    return COIL_LAST_ONE;
}

/*!
* @brief Reads and returns the specified coil
*
* This function reads the coil number given and returns
* it's value.
*
* @param port The port this request came over
* @param coil The number of the coil to read
*
* @return True(0xFF) if coil is on, otherwise False(0x00)
*/
uint8_t modbus_coil_read_callback(uint8_t port, uint16_t coil)
{
    int32_t tempVal;
    uint8_t tempLen;

    //We only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return 0x00;
    }

    //Get the coil
    switch((board_coils)coil) {
    case COIL_MANUAL_COOL:
      get_manual_cool((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_MANUAL_HEAT:
      get_manual_heat((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_HOT_ALARM:
      get_hot_alarm((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_COLD_ALARM:
      get_cold_alarm((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_COND_ALARM:
      get_cond_alarm((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_FILTER_ALARM:
      get_filter_alarm((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_ALARMS_ON:
      get_alarm_out((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_UNIT_IS_CELSIUS:
      return (settingPtr->tempUnit == CELSIUS)?0xFF:00;
    case COIL_REDUND_IS_LEAD:
      get_redund_mode((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_REDUND_TIME_BASIS_IS_DAYS:
      get_redund_basis((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_POWER_ON:
      get_power_on((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_POA_ALARM_ON:
      get_poa_alarm((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_STEALTH_MODE:
      get_stealth_mode((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_DC_MOTOR_ON:
      get_dc_enabled((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_ETHERNET_ENABLED:
      get_eth_enabled((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_DHCP_ENABLED:
      get_dhcp_enabled((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_RESET_FILTER_DAYS:
      get_filter_reset((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case COIL_POA_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & POA_ALARM)?0xFF:00;
    case COIL_HI_ENC_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & HI_ENC_ALARM)?0xFF:00;
    case COIL_LO_ENC_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & LO_ENC_ALARM)?0xFF:00;
    case COIL_HI_CONDENSE_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & HI_CONDENSE_ALARM)?0xFF:00;
    case COIL_FILTER_ALARM_ACTIVE:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & FILTER_ALARM)?0xFF:00;
    case COIL_SENS1_OPEN_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & SENS1_OPEN_ALARM)?0xFF:00;
    case COIL_SENS2_OPEN_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & SENS2_OPEN_ALARM)?0xFF:00;
    case COIL_SENS1_SHORT_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & SENS1_SHORT_ALARM)?0xFF:00;
    case COIL_SENS2_SHORT_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & SENS2_SHORT_ALARM)?0xFF:00;
    case COIL_NETWORK_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & NETWORK_ALARM)?0xFF:00;
    case COIL_OW_TEMP_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & OW_TEMP_ALARM)?0xFF:00;
    case COIL_FROZEN_ALARM:
      get_alarm_status((void *)&tempVal, &tempLen);
      return (tempVal & FROZEN_ALARM)?0xFF:00;
    case COIL_10_MIN_MANUAL:
      get_man_timer((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    default:
        break;
    }

    return 0x00;
}

/*!
* @brief Writes the specified coil
*
* This function writes the coil number given.
*
* @param port The port this request came over
* @param coil The number of the coil to write
* @param on Whether to turn the coil on(true/0xFF) or off(false/0x00)
*
* @return Exception code indicating if coil was successfully written.
*/
modbus_exceptions modbus_coil_write_callback(uint8_t port, uint16_t coil, uint8_t on)
{
    //We only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    //Write the coil
    switch((board_coils)coil) {
    case COIL_MANUAL_COOL:
      set_manual_cool(on);
      break;
    case COIL_MANUAL_HEAT:
      set_manual_heat(on);
      break;
    case COIL_HOT_ALARM:
      coilChange[port] |= set_hot_alarm_noup(on);
      break;
    case COIL_COLD_ALARM:
      coilChange[port] |= set_cold_alarm_noup(on);
      break;
    case COIL_COND_ALARM:
      coilChange[port] |= set_cond_alarm_noup(on);
      break;
    case COIL_FILTER_ALARM:
      coilChange[port] |= set_filter_alarm_noup(on);
      break;
    case COIL_ALARMS_ON:
      coilChange[port] |= set_alarm_out_noup(on);
      break;
    case COIL_UNIT_IS_CELSIUS:
      if(settingPtr->tempUnit != (on?CELSIUS:FAHRENHEIT)) {
        settingPtr->tempUnit = (on?CELSIUS:FAHRENHEIT);
        coilChange[port] = true;
      }
      break;
    case COIL_REDUND_IS_LEAD:
      coilChange[port] |= set_redund_mode_noup(on);
      break;
    case COIL_REDUND_TIME_BASIS_IS_DAYS:
      coilChange[port] |= set_redund_basis_noup(on);
      break;
    case COIL_POWER_ON:
      coilChange[port] |= set_power_on_noup(on);
      break;
    case COIL_POA_ALARM_ON:
      coilChange[port] |= set_poa_alarm_noup(on);
      break;
    case COIL_STEALTH_MODE:
      coilChange[port] |= set_stealth_mode_noup(on);
      break;
    case COIL_DC_MOTOR_ON:
      coilChange[port] |= set_dc_enabled_noup(on);
      break;
    case COIL_ETHERNET_ENABLED:
      coilChange[port] |= set_eth_enabled_noup(on);
      break;
    case COIL_DHCP_ENABLED:
      coilChange[port] |= set_dhcp_enabled_noup(on);
      break;
    case COIL_RESET_FILTER_DAYS:
      set_filter_reset(on);
      break;
    case COIL_POA_ALARM:
      set_alarm_status((on?0:POA_ALARM));
      break;
    case COIL_HI_ENC_ALARM:
      set_alarm_status((on?0:HI_ENC_ALARM));
      break;
    case COIL_LO_ENC_ALARM:
      set_alarm_status((on?0:LO_ENC_ALARM));
      break;
    case COIL_HI_CONDENSE_ALARM:
      set_alarm_status((on?0:HI_CONDENSE_ALARM));
      break;
    case COIL_FILTER_ALARM_ACTIVE:
      set_alarm_status((on?0:FILTER_ALARM));
      break;
    case COIL_SENS1_OPEN_ALARM:
      set_alarm_status((on?0:SENS1_OPEN_ALARM));
      break;
    case COIL_SENS2_OPEN_ALARM:
      set_alarm_status((on?0:SENS2_OPEN_ALARM));
      break;
    case COIL_SENS1_SHORT_ALARM:
      set_alarm_status((on?0:SENS1_SHORT_ALARM));
      break;
    case COIL_SENS2_SHORT_ALARM:
      set_alarm_status((on?0:SENS2_SHORT_ALARM));
      break;
    case COIL_NETWORK_ALARM:
      set_alarm_status((on?0:NETWORK_ALARM));
      break;
    case COIL_OW_TEMP_ALARM:
      set_alarm_status((on?0:OW_TEMP_ALARM));
      break;
    case COIL_FROZEN_ALARM:
      set_alarm_status((on?0:FROZEN_ALARM));
      break;
    case COIL_10_MIN_MANUAL:
      coilChange[port] |= set_man_timer_noup(on);
      break;
    default:
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        break;
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief Gets the start address of the discrete inputs
*
* This function simply gets the address the discretes for this port start at
*
* @param port The port this request came over
*
* @return The start address of the discretes
*/
uint16_t modbus_discrete_start_address(uint8_t port)
{
    return DISCRETE_START_ADDRESS;
}

/*!
* @brief Gets the total number of available discrete inputs
*
* This function simply returns the number of discrete inputs available in
* the system.
*
* @param port The port this request came over
*
* @return The number of available discrete inputs
*/
uint16_t modbus_number_of_discretes(uint8_t port)
{
    return DISCRETE_LAST_ONE;
}

/*!
* @brief Reads and returns the specified discrete input
*
* This function reads the discrete input number given and returns
* it's value.
*
* @param port The port this request came over
* @param coil The number of the discrete input to read
*
* @return True(0xFF) if input is on, otherwise False(0x00)
*/
uint8_t modbus_discrete_read_callback(uint8_t port, uint16_t discrete)
{
    int32_t tempVal;
    uint8_t tempLen;

    //We only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return 0x00;
    }

    switch((board_discretes)discrete) {
    case DISCRETE_COOL_INSTALLED:
      get_cool_install((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    case DISCRETE_HEAT_INSTALLED:
      get_heat_install((void *)&tempVal, &tempLen);
      return (tempVal > 0)?0xFF:00;
    default:
        break;
    }

    return 0x00;
}

/*!
* @brief Gets the address that the input registers start at
*
* This function simply returns the address that the input registers
* begin at for this port.
*
* @param port The port this request came over
*
* @return The start address of the input registers
*/
uint16_t modbus_input_start_address(uint8_t port)
{
    return INPUT_START_ADDRESS;
}

/*!
* @brief Gets the total number of available input registers
*
* This function simply returns the number of input registers available
* in the system.
*
* @param port The port this request came over
*
* @return The number of available input registers
*/
uint16_t modbus_number_of_inputs(uint8_t port)
{
    return INPUT_LAST_ONE;
}

/*!
* @brief Reads and returns the specified input register
*
* This function reads the input register given and returns
* it's value.
*
* @param port The port this request came over
* @param reg The number of the input register to read
*
* @return The value of the input register
*/
uint16_t modbus_input_read_callback(uint8_t port, uint16_t reg)
{
    int32_t tempVal;
    uint8_t tempLen;

    //We only have one port so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return 0x0000;
    }

    //Get the input register
    switch((board_inputs)reg) {
    case INPUT_FILTER_LEFT:
        return settingPtr->filterDays[2];
    case INPUT_HEAT_SPEED:
        return statePtr->heatSpeed;
    case INPUT_COOL_SPEED:
        return statePtr->coolSpeed;
    case INPUT_ENC_TEMP_WUNITS:
        return getSnmpOutTemp(statePtr->curTemp[AIR_TEMP], settingPtr->tempUnit);
    case INPUT_COND_TEMP_WUNITS:
        return getSnmpOutTemp(statePtr->curTemp[CONDENSE_TEMP], settingPtr->tempUnit);
    case INPUT_FREEZE_TEMP_WUNITS:
        return getSnmpOutTemp(statePtr->curTemp[NUM_TEMPS], settingPtr->tempUnit);
    case INPUT_SYSTEM_STATE:
        get_cool_running((void *)&tempVal, &tempLen);
        if(tempVal > 0) {
          return tempVal*2;
        } else {
          get_heat_running((void *)&tempVal, &tempLen);
          return (tempVal > 1)?3:tempVal;
        }
    case INPUT_REDUND_STATE:
        get_redund_state(&tempVal, &tempLen);
        return tempVal;
    default:
        break;
    }

    return 0x0000;
}

/*!
* @brief Gets the start address of the holding registers
*
* This function simply returns the starting address for the holding
* registers on this port.
*
* @param port The port this request came over
*
* @return The start address of the port holding registers
*/
uint16_t modbus_holding_start_address(uint8_t port)
{
    return HOLDING_START_ADDRESS;
}

/*!
* @brief Gets the total number of available holding registers
*
* This function simply returns the number of holding registers available
* in the system.
*
* @param port The port this request came over
*
* @return The number of available holding registers
*/
uint16_t modbus_number_of_holding(uint8_t port)
{
    return HOLDING_LAST_ONE;
}

/*!
* @name modbus_number_redund_holding
* @brief Gets the total number of holding registers used for redundancy
*
* @param port Not sure why this is here....
*/
uint16_t modbus_number_redund_holding(uint8_t port) {
    return HOLDING_UNIT_RUNTIME + 1;
}


bool modbusMasterCallback(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t val)
{
    mbRedunActiveSecs = get_sec_ticks();

    return true;
}

/*!
* @name modbus_write_redund_register
* @brief Does a redundancy write because some weirdness is needed in register orders
*
* @param reg The regsiter number to write
*
* @return The next register to write to
*/
uint16_t modbus_write_redund_register(uint16_t reg) {
    uint16_t nextReg;
    modbus_write_holding_register_command(MB_TCP_REDUND_PORT_NUM, MB_REDUND_SLAVE_ID, ((reg == HOLDING_UNIT_RUNTIME)?HOLDING_REDUND_MASTER_STATE:reg), modbus_holding_read_callback(MB_TCP_REDUND_PORT_NUM, ((reg == HOLDING_UNIT_RUNTIME)?HOLDING_REDUND_MASTER_STATE:reg), false), modbusMasterCallback);
    nextReg = reg + 1;
    if(nextReg >= modbus_number_redund_holding(MB_TCP_REDUND_PORT_NUM)) {
        nextReg = 0;
    }

    return nextReg;
}

/*!
* @brief Reads and returns the specified holding register
*
* This function reads the holding register given and returns
* it's value.
*
* @param port The port this request came over
* @param reg The number of the holding register to read
* @param isWriteRead Indicates if this is part of a dual write/read command
*
* @return The value of the holding register
*/
uint16_t modbus_holding_read_callback(uint8_t port, uint16_t reg, bool isWriteRead)
{
    int32_t tempVal;
    uint8_t tempLen;

    //We only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return 0x0000;
    }

    switch((board_holding)reg) {
    case HOLDING_TEMP_SET_LO:
        return settingPtr->tempSettings[LO_TEMP];
    case HOLDING_TEMP_SET_HI:
        return settingPtr->tempSettings[HI_TEMP];
    case HOLDING_TEMP_SET_DELAY:
        return settingPtr->tempSettings[TEMP_DELAY_TIME];
    case HOLDING_ADV_OPT:
        return settingPtr->advSettings[ADV_SET_FLAGS];
    case HOLDING_TEMP_SET_UNIT:
        return settingPtr->tempUnit;
    case HOLDING_TEMP_SET_VARIANT_LO:
        return settingPtr->tempVariant[LO_TEMP];
    case HOLDING_TEMP_SET_VARIANT_HI:
        return settingPtr->tempVariant[HI_TEMP];
    case HOLDING_ADV_ALARMS:
        return settingPtr->advAlarmBits;
    case HOLDING_TEMP_ALARM_LO:
        return settingPtr->tempAlarm[LO_TEMP];
    case HOLDING_TEMP_ALARM_HI:
        return settingPtr->tempAlarm[HI_TEMP];
    case HOLDING_TEMP_ALARM_COND:
        return settingPtr->hiCondAlarm;
    case HOLDING_FILTER_DAYS:
        return settingPtr->filterDays[0];
    case HOLDING_FILTER_WARN:
        return settingPtr->filterDays[1];
    case HOLDING_SENS_OFFSET_AIR:
        return settingPtr->sensOffset[AIR_TEMP];
    case HOLDING_SENS_OFFSET_CONDENSE:
        return settingPtr->sensOffset[CONDENSE_TEMP];
    case HOLDING_UNIT_RUNTIME:
        return settingPtr->runCountHours;
    case HOLDING_ETH_FLAGS:
        return settingPtr->ethSettings[ETHERNET_SET_FLAGS];
    case HOLDING_ETH_OFFSETS:
        return ((((uint16_t)settingPtr->ethSettings[ETHERNET_SET_REDUND_OFFSET])<<8) & 0xFF00) | ((uint16_t)settingPtr->ethSettings[ETHERNET_SET_REDUND_SWAP_DAYS] & 0x00FF);
    case HOLDING_TEMP_ALARM_LO_WUNITS:
      get_cold_limit((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_TEMP_ALARM_HI_WUNITS:
      get_hot_limit((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_TEMP_ALARM_COND_WUNITS:
      get_cond_limit((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_AUDIBLE_ALARM:
      get_audible_alarm((void *)&tempVal, &tempLen);
      return (tempVal & 0x00FF);
    case HOLDING_REDUND_ON:
      get_redund_enable((void *)&tempVal, &tempLen);
      return (tempVal & 0x000F);
    case HOLDING_REDUND_SWAP_TIME:
      get_redund_time((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_TEMP_SET_LO_WUNITS:
      get_heat_temp((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_TEMP_SET_HI_WUNITS:
      get_cool_temp((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_MODBUS_SLAVE_ADDRESS:
    	if(isWriteRead) {
    		return tempHoldRegs[port].mbAddress;
    	} else {
    		return settingPtr->advSettings[ADV_SET_MODBUS_SLAVE_ADDRESS];
    	}
    case HOLDING_MODBUS_BAUD_DIV100:
    	if(isWriteRead) {
    		return tempHoldRegs[port].mbBaud;
    	} else {
    		return (((((uint16_t)settingPtr->advSettings[ADV_SET_MODBUS_BAUD_MSB])<<8) & 0xFF00) | settingPtr->advSettings[ADV_SET_MODBUS_BAUD_LSB]);
    	}
    case HOLDING_MODBUS_PARITY:
    	if(isWriteRead) {
    		return tempHoldRegs[port].mbParity;
    	} else {
    		return ((settingPtr->advSettings[ADV_SET_MODBUS_UART_OPTS] >> 4) & 0x0F);
    	}
    case HOLDING_MODBUS_STOP_BITS:
    	if(isWriteRead) {
    		return tempHoldRegs[port].mbStopBits;
    	} else {
    		return (settingPtr->advSettings[ADV_SET_MODBUS_UART_OPTS] & 0x0F);
    	}
    case HOLDING_UNIT_NAME_C0_1:
    case HOLDING_UNIT_NAME_C2_3:
    case HOLDING_UNIT_NAME_C4_5:
    case HOLDING_UNIT_NAME_C6_7:
    case HOLDING_UNIT_NAME_C8_9:
    case HOLDING_UNIT_NAME_C10_11:
    case HOLDING_UNIT_NAME_C12:
      return (((settingPtr->name[(reg - HOLDING_UNIT_NAME_C0_1)<<1] << 8) & 0xFF00) | (settingPtr->name[((reg - HOLDING_UNIT_NAME_C0_1)<<1) + 1] & 0x00FF));
    case HOLDING_CABINET_NAME_C0_1:
    case HOLDING_CABINET_NAME_C2_3:
    case HOLDING_CABINET_NAME_C4_5:
    case HOLDING_CABINET_NAME_C6_7:
    case HOLDING_CABINET_NAME_C8_9:
    case HOLDING_CABINET_NAME_C10_11:
    case HOLDING_CABINET_NAME_C12:
      return (((settingPtr->cabName[(reg - HOLDING_CABINET_NAME_C0_1)<<1] << 8) & 0xFF00) | (settingPtr->cabName[((reg - HOLDING_CABINET_NAME_C0_1)<<1) + 1] & 0x00FF));
    case HOLDING_HEAT_VARIANCE:
      get_heat_var((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_COOL_VARIANCE:
      get_cool_var((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_HEAT_DELAY:
      get_heat_delay((void *)&tempVal, &tempLen);
      return (tempVal & 0x00FF);
    case HOLDING_COMP_DELAY:
      get_cool_delay((void *)&tempVal, &tempLen);
      return (tempVal & 0x00FF);
    case HOLDING_FREEZE_SHUTOFF_WUNITS:
      get_freeze_off((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_UNFREEZE_ENABLE_WUNITS:
      get_freeze_on((void *)&tempVal, &tempLen);
      return (tempVal & 0xFFFF);
    case HOLDING_IPV4_ADD_HW:
    case HOLDING_IPV4_ADD_LW:
    case HOLDING_IPV4_SUB_HW:
    case HOLDING_IPV4_SUB_LW:
    case HOLDING_IPV4_GW_HW:
    case HOLDING_IPV4_GW_LW:
    case HOLDING_SNMP_IP_HW:
    case HOLDING_SNMP_IP_LW:
      return ((((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_IP_MMSB + ((reg - HOLDING_IPV4_ADD_HW)*2)]))<<8) | ((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_IP_MMSB + (((reg - HOLDING_IPV4_ADD_HW)*2)+1)])));
    case HOLDING_IPV6_LLA_W1:
    case HOLDING_IPV6_LLA_W2:
    case HOLDING_IPV6_LLA_W3:
    case HOLDING_IPV6_LLA_W4:
    case HOLDING_IPV6_LLA_W5:
    case HOLDING_IPV6_LLA_W6:
    case HOLDING_IPV6_LLA_W7:
    case HOLDING_IPV6_LLA_W8:
    case HOLDING_IPV6_GUA_W1:
    case HOLDING_IPV6_GUA_W2:
    case HOLDING_IPV6_GUA_W3:
    case HOLDING_IPV6_GUA_W4:
    case HOLDING_IPV6_GUA_W5:
    case HOLDING_IPV6_GUA_W6:
    case HOLDING_IPV6_GUA_W7:
    case HOLDING_IPV6_GUA_W8:
    case HOLDING_IPV6_SN6_W1:
    case HOLDING_IPV6_SN6_W2:
    case HOLDING_IPV6_SN6_W3:
    case HOLDING_IPV6_SN6_W4:
    case HOLDING_IPV6_SN6_W5:
    case HOLDING_IPV6_SN6_W6:
    case HOLDING_IPV6_SN6_W7:
    case HOLDING_IPV6_SN6_W8:
    case HOLDING_IPV6_GW6_W1:
    case HOLDING_IPV6_GW6_W2:
    case HOLDING_IPV6_GW6_W3:
    case HOLDING_IPV6_GW6_W4:
    case HOLDING_IPV6_GW6_W5:
    case HOLDING_IPV6_GW6_W6:
    case HOLDING_IPV6_GW6_W7:
    case HOLDING_IPV6_GW6_W8:
      return ((((uint16_t)(settingPtr->ipv6Vals[(reg-HOLDING_IPV6_LLA_W1)*2]))<<8) | ((uint16_t)(settingPtr->ipv6Vals[((reg-HOLDING_IPV6_LLA_W1)*2)+1])));
    case HOLDING_REDUND_OFFSET:
      get_redund_offset((void *)&tempVal, &tempLen);
      return (tempVal & 0x00FF);
    case HOLDING_REDUND_IP_HW:
    case HOLDING_REDUND_IP_LW:
      return ((((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_PARTNER_MMSB + ((reg - HOLDING_REDUND_IP_HW)*2)]))<<8) | ((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_PARTNER_MMSB + (((reg - HOLDING_REDUND_IP_HW)*2)+1)])));
    case HOLDING_IPV6_REDUND_W1:
    case HOLDING_IPV6_REDUND_W2:
    case HOLDING_IPV6_REDUND_W3:
    case HOLDING_IPV6_REDUND_W4:
    case HOLDING_IPV6_REDUND_W5:
    case HOLDING_IPV6_REDUND_W6:
    case HOLDING_IPV6_REDUND_W7:
    case HOLDING_IPV6_REDUND_W8:
      return ((((uint16_t)(settingPtr->ipv6Vals[(reg-HOLDING_IPV6_REDUND_W1+32)*2]))<<8) | ((uint16_t)(settingPtr->ipv6Vals[((reg-HOLDING_IPV6_REDUND_W1+32)*2)+1])));
    case HOLDING_REDUND_MASTER_STATE:
      return statePtr->redundMasterState;
    default:
        break;
    }

    return 0x0000;
}

/*!
* @brief Initializes temporary storage for writing registers
*
* Since there are restrictions on many of the registers we allow to be
* written we actually store all writes to a temporary place first, then
* verify the contents before deciding on success and performing the writes.
*
* @param port The port this request came over
*/
void modbus_holding_start_write_callback(uint8_t port)
{
    //We only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return;
    }

    tempHoldRegs[port].mbAddress = settingPtr->advSettings[ADV_SET_MODBUS_SLAVE_ADDRESS];
    tempHoldRegs[port].mbBaud = (((((uint16_t)settingPtr->advSettings[ADV_SET_MODBUS_BAUD_MSB])<<8) & 0xFF00) | settingPtr->advSettings[ADV_SET_MODBUS_BAUD_LSB]);
    tempHoldRegs[port].mbParity = ((settingPtr->advSettings[ADV_SET_MODBUS_UART_OPTS] >> 4) & 0x0F);
    tempHoldRegs[port].mbStopBits = (settingPtr->advSettings[ADV_SET_MODBUS_UART_OPTS] & 0x0F);
}

/*!
* @brief Verifies if the holding register writes just performed were valid
*
* This function is called immediately after all the write values are
* applied using modbus_holding_write_callback.  It looks at all the changed
* values in the temp aread to ensure they are valid.  It will then return
* a status indicating if the write is good or not.
*
* @param port The port this request came over
*
* @return Exception status that indicates if the write values are valid or not
*/
modbus_exceptions modbus_holding_verify_write_callback(uint8_t port)
{
    uint8_t idx;

    //We only have one port so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    if(port == MB_TCP_REDUND_PORT_NUM) {
      mbRedunActiveSecs = get_sec_ticks();
    }

    return MODBUS_EX_NONE;
}

/*!
* @brief Writes the specified holding register
*
* This function writes the holding register number given.
*
* @param port The port this request came over
* @param reg The number of the holding register to write
* @param val The value to write into the register
*
* @return Exception code indicating if the holding register was successfully written.
*/
modbus_exceptions modbus_holding_write_callback(uint8_t port, uint16_t reg, uint16_t val)
{
    //WE only have one port, so if not it we shouldn't be here
    if(port > MB_RTU_SLAVE_PORT_NUM) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    switch((board_holding)reg) {
    case HOLDING_TEMP_SET_LO:
        if(settingPtr->tempSettings[LO_TEMP] != val) {
            settingPtr->tempSettings[LO_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_SET_HI:
        if(settingPtr->tempSettings[HI_TEMP] != val) {
            settingPtr->tempSettings[HI_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_SET_DELAY:
        if(settingPtr->tempSettings[TEMP_DELAY_TIME] != val) {
            settingPtr->tempSettings[TEMP_DELAY_TIME] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_ADV_OPT:
        if(settingPtr->advSettings[ADV_SET_FLAGS] != val) {
            settingPtr->advSettings[ADV_SET_FLAGS] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_SET_UNIT:
        if(settingPtr->tempUnit != val) {
            settingPtr->tempUnit = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_SET_VARIANT_LO:
        if(settingPtr->tempVariant[LO_TEMP] != val) {
            settingPtr->tempVariant[LO_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_SET_VARIANT_HI:
        if(settingPtr->tempVariant[HI_TEMP] != val) {
            settingPtr->tempVariant[HI_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_ADV_ALARMS:
        if(settingPtr->advAlarmBits != val) {
            settingPtr->advAlarmBits = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_ALARM_LO:
        if(settingPtr->tempAlarm[LO_TEMP] != val) {
            settingPtr->tempAlarm[LO_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_ALARM_HI:
        if(settingPtr->tempAlarm[HI_TEMP] != val) {
            settingPtr->tempAlarm[HI_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_ALARM_COND:
        if(settingPtr->hiCondAlarm != val) {
            settingPtr->hiCondAlarm = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_FILTER_DAYS:
        if(settingPtr->filterDays[0] != val) {
            settingPtr->filterDays[0] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_FILTER_WARN:
        if(settingPtr->filterDays[1] != val) {
            settingPtr->filterDays[1] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_SENS_OFFSET_AIR:
        if(settingPtr->sensOffset[AIR_TEMP] != val) {
            settingPtr->sensOffset[AIR_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_SENS_OFFSET_CONDENSE:
        if(settingPtr->sensOffset[CONDENSE_TEMP] != val) {
            settingPtr->sensOffset[CONDENSE_TEMP] = val;
            holdChange[port] = true;
        }
        break;
    case HOLDING_UNIT_RUNTIME:
        if(settingPtr->runCountHours != val) {
            settingPtr->runCountHours = val;
        }
        break;
    case HOLDING_ETH_FLAGS:
        if((settingPtr->ethSettings[ETHERNET_SET_FLAGS] & 0x48) != (val & 0x0048)) {
            settingPtr->ethSettings[ETHERNET_SET_FLAGS] = ((settingPtr->ethSettings[ETHERNET_SET_FLAGS] & ~(0x48)) | (val & 0x0048));
            holdChange[port] = true;
        }
        break;
    case HOLDING_ETH_OFFSETS:
        if((((((uint16_t)settingPtr->ethSettings[ETHERNET_SET_REDUND_OFFSET])<<8) & 0xFF00) | ((uint16_t)settingPtr->ethSettings[ETHERNET_SET_REDUND_SWAP_DAYS] & 0x00FF)) != val) {
            settingPtr->ethSettings[ETHERNET_SET_REDUND_OFFSET] = ((val>>8) & 0x00FF);
            settingPtr->ethSettings[ETHERNET_SET_REDUND_SWAP_DAYS] = (val & 0x00FF);
            holdChange[port] = true;
        }
        break;
    case HOLDING_TEMP_ALARM_LO_WUNITS:
      holdChange[port] |= set_cold_limit_noup(val);
      break;
    case HOLDING_TEMP_ALARM_HI_WUNITS:
      holdChange[port] |= set_hot_limit_noup(val);
      break;
    case HOLDING_TEMP_ALARM_COND_WUNITS:
      holdChange[port] |= set_cond_limit_noup(val);
      break;
    case HOLDING_AUDIBLE_ALARM:
      if(val > 100) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
      } else {
        holdChange[port] |= set_audible_alarm_noup(val);
      }
      break;
    case HOLDING_REDUND_ON:
      if(val > 2) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
      } else {
        holdChange[port] |= set_redund_enable_noup(val);
      }
      break;
    case HOLDING_REDUND_SWAP_TIME:
      if(val > 255) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
      } else {
        holdChange[port] |= set_redund_time_noup(val);
      }
      break;
    case HOLDING_TEMP_SET_LO_WUNITS:
      holdChange[port] |= set_heat_temp_noup(val);
      break;
    case HOLDING_TEMP_SET_HI_WUNITS:
      holdChange[port] |= set_cool_temp_noup(val);
      break;
    case HOLDING_MODBUS_SLAVE_ADDRESS:
        if(val > 247) {
          return MODBUS_EX_ILLEGAL_DATA_VALUE;
        } else if(tempHoldRegs[port].mbAddress != val) {
          tempHoldRegs[port].mbAddress = val;
          holdChange[port] = true;
        }
        break;
    case HOLDING_MODBUS_BAUD_DIV100:
        if(tempHoldRegs[port].mbBaud != val) {
          tempHoldRegs[port].mbBaud = val;
          holdChange[port] = true;
        }
        break;
    case HOLDING_MODBUS_PARITY:
        if(val >= MODBUS_PARITY_LAST_ONE) {
          return MODBUS_EX_ILLEGAL_DATA_VALUE;
        } else if(tempHoldRegs[port].mbParity != val) {
          tempHoldRegs[port].mbParity = val;
          holdChange[port] = true;
        }
        break;
    case HOLDING_MODBUS_STOP_BITS:
        if((val > 2) || (val < 1)) {
          return MODBUS_EX_ILLEGAL_DATA_VALUE;
        } else if(tempHoldRegs[port].mbStopBits != val) {
          tempHoldRegs[port].mbStopBits = val;
          holdChange[port] = true;
        }
        break;
    case HOLDING_UNIT_NAME_C0_1:
    case HOLDING_UNIT_NAME_C2_3:
    case HOLDING_UNIT_NAME_C4_5:
    case HOLDING_UNIT_NAME_C6_7:
    case HOLDING_UNIT_NAME_C8_9:
    case HOLDING_UNIT_NAME_C10_11:
    case HOLDING_UNIT_NAME_C12:
      if(settingPtr->name[(reg - HOLDING_UNIT_NAME_C0_1)<<1] != ((val >> 8) & 0xFF)) {
        settingPtr->name[(reg - HOLDING_UNIT_NAME_C0_1)<<1] = ((val >> 8) & 0xFF);
        holdChange[port] = true;
      }
      if((reg < HOLDING_UNIT_NAME_C12) && (settingPtr->name[((reg - HOLDING_UNIT_NAME_C0_1)<<1)+1] != (val & 0xFF))) {
        settingPtr->name[((reg - HOLDING_UNIT_NAME_C0_1)<<1)+1] = (val & 0xFF);
        holdChange[port] = true;
      }
      break;
    case HOLDING_CABINET_NAME_C0_1:
    case HOLDING_CABINET_NAME_C2_3:
    case HOLDING_CABINET_NAME_C4_5:
    case HOLDING_CABINET_NAME_C6_7:
    case HOLDING_CABINET_NAME_C8_9:
    case HOLDING_CABINET_NAME_C10_11:
    case HOLDING_CABINET_NAME_C12:
      if(settingPtr->cabName[(reg - HOLDING_CABINET_NAME_C0_1)<<1] != ((val >> 8) & 0xFF)) {
        settingPtr->cabName[(reg - HOLDING_CABINET_NAME_C0_1)<<1] = ((val >> 8) & 0xFF);
        holdChange[port] = true;
      }
      if((reg < HOLDING_CABINET_NAME_C12) && (settingPtr->cabName[((reg - HOLDING_CABINET_NAME_C0_1)<<1)+1] != (val & 0xFF))) {
        settingPtr->cabName[((reg - HOLDING_CABINET_NAME_C0_1)<<1)+1] = (val & 0xFF);
        holdChange[port] = true;
      }
      break;
    case HOLDING_HEAT_VARIANCE:
      holdChange[port] |= set_heat_var_noup(val);
      break;
    case HOLDING_COOL_VARIANCE:
      holdChange[port] |= set_cool_var_noup(val);
      break;
    case HOLDING_HEAT_DELAY:
      holdChange[port] |= set_heat_delay_noup(val);
      break;
    case HOLDING_COMP_DELAY:
      holdChange[port] |= set_cool_delay_noup(val);
      break;
    case HOLDING_FREEZE_SHUTOFF_WUNITS:
      holdChange[port] |= set_freeze_off_noup(val);
      break;
    case HOLDING_UNFREEZE_ENABLE_WUNITS:
      holdChange[port] |= set_freeze_on_noup(val);
      break;
    case HOLDING_IPV4_ADD_HW:
    case HOLDING_IPV4_ADD_LW:
    case HOLDING_IPV4_SUB_HW:
    case HOLDING_IPV4_SUB_LW:
    case HOLDING_IPV4_GW_HW:
    case HOLDING_IPV4_GW_LW:
    case HOLDING_SNMP_IP_HW:
    case HOLDING_SNMP_IP_LW:
      if(((((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_IP_MMSB + ((reg - HOLDING_IPV4_ADD_HW)*2)]))<<8) | ((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_IP_MMSB + (((reg - HOLDING_IPV4_ADD_HW)*2)+1)]))) != val) {
        settingPtr->ethSettings[ETHERNET_SET_IP_MMSB + ((reg - HOLDING_IPV4_ADD_HW)*2)] = ((val >> 8) & 0xFF);
        settingPtr->ethSettings[ETHERNET_SET_IP_MMSB + (((reg - HOLDING_IPV4_ADD_HW)*2) + 1)] = (val & 0xFF);
        holdChange[port] = true;
      }
      break;
    case HOLDING_IPV6_LLA_W1:
    case HOLDING_IPV6_LLA_W2:
    case HOLDING_IPV6_LLA_W3:
    case HOLDING_IPV6_LLA_W4:
    case HOLDING_IPV6_LLA_W5:
    case HOLDING_IPV6_LLA_W6:
    case HOLDING_IPV6_LLA_W7:
    case HOLDING_IPV6_LLA_W8:
    case HOLDING_IPV6_GUA_W1:
    case HOLDING_IPV6_GUA_W2:
    case HOLDING_IPV6_GUA_W3:
    case HOLDING_IPV6_GUA_W4:
    case HOLDING_IPV6_GUA_W5:
    case HOLDING_IPV6_GUA_W6:
    case HOLDING_IPV6_GUA_W7:
    case HOLDING_IPV6_GUA_W8:
    case HOLDING_IPV6_SN6_W1:
    case HOLDING_IPV6_SN6_W2:
    case HOLDING_IPV6_SN6_W3:
    case HOLDING_IPV6_SN6_W4:
    case HOLDING_IPV6_SN6_W5:
    case HOLDING_IPV6_SN6_W6:
    case HOLDING_IPV6_SN6_W7:
    case HOLDING_IPV6_SN6_W8:
    case HOLDING_IPV6_GW6_W1:
    case HOLDING_IPV6_GW6_W2:
    case HOLDING_IPV6_GW6_W3:
    case HOLDING_IPV6_GW6_W4:
    case HOLDING_IPV6_GW6_W5:
    case HOLDING_IPV6_GW6_W6:
    case HOLDING_IPV6_GW6_W7:
    case HOLDING_IPV6_GW6_W8:
      if(((((uint16_t)(settingPtr->ipv6Vals[(reg-HOLDING_IPV6_LLA_W1)*2]))<<8) | ((uint16_t)(settingPtr->ipv6Vals[((reg-HOLDING_IPV6_LLA_W1)*2)+1]))) != val) {
        settingPtr->ipv6Vals[(reg-HOLDING_IPV6_LLA_W1)*2] = ((val >> 8) & 0xFF);
        settingPtr->ipv6Vals[((reg-HOLDING_IPV6_LLA_W1)*2) + 1] = (val & 0xFF);
        holdChange[port] = true;
      }
      break;
    case HOLDING_REDUND_OFFSET:
      holdChange[port] |= set_redund_offset_noup(val);
      break;
    case HOLDING_REDUND_IP_HW:
    case HOLDING_REDUND_IP_LW:
      if(((((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_PARTNER_MMSB + ((reg - HOLDING_REDUND_IP_HW)*2)]))<<8) | ((uint16_t)(settingPtr->ethSettings[ETHERNET_SET_PARTNER_MMSB + (((reg - HOLDING_REDUND_IP_HW)*2)+1)]))) != val) {
        settingPtr->ethSettings[ETHERNET_SET_PARTNER_MMSB + ((reg - HOLDING_REDUND_IP_HW)*2)] = ((val >> 8) & 0xFF);
        settingPtr->ethSettings[ETHERNET_SET_PARTNER_MMSB + (((reg - HOLDING_REDUND_IP_HW)*2) + 1)] = (val & 0xFF);
        holdChange[port] = true;
      }
      break;
    case HOLDING_IPV6_REDUND_W1:
    case HOLDING_IPV6_REDUND_W2:
    case HOLDING_IPV6_REDUND_W3:
    case HOLDING_IPV6_REDUND_W4:
    case HOLDING_IPV6_REDUND_W5:
    case HOLDING_IPV6_REDUND_W6:
    case HOLDING_IPV6_REDUND_W7:
    case HOLDING_IPV6_REDUND_W8:
      if(((((uint16_t)(settingPtr->ipv6Vals[(reg-HOLDING_IPV6_REDUND_W1 + 32)*2]))<<8) | ((uint16_t)(settingPtr->ipv6Vals[((reg-HOLDING_IPV6_REDUND_W1 + 32)*2)+1]))) != val) {
        settingPtr->ipv6Vals[(reg-HOLDING_IPV6_REDUND_W1 + 32)*2] = ((val >> 8) & 0xFF);
        settingPtr->ipv6Vals[((reg-HOLDING_IPV6_REDUND_W1 + 32)*2) + 1] = (val & 0xFF);
        holdChange[port] = true;
      }
      break;
    case HOLDING_REDUND_MASTER_STATE:
      if(settingPtr->ethSettings[ETHERNET_SET_FLAGS] & ETHERNET_FLAGS_REDUN_SLAVE) {
        statePtr->redundMasterState = val;
      }
    default:
        break;
    }

    return MODBUS_EX_NONE;
}

bool modbus_callbacks_set_ptrs(unit_settings *settings, unit_state *state) {
    settingPtr = settings;
    statePtr = state;
}

bool modbusRTUCallback(uint8_t port, uint8_t slaveAddress, modbus_function_codes functionCode, uint16_t address, uint16_t val)
{
    mbRTUActiveSecs = get_sec_ticks();

    return true;
}

bool redundacyActive(void) {
    return ((get_sec_ticks() - mbRedunActiveSecs) < MB_REDUND_COMMS_TIMEOUT_SEC);
}
