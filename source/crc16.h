/*!
* @file crc16.h
* @author Jarrod Cook
*
* @brief Interface for the CRC16 module.
*
* This file is the interface for the module to compute CRC16 checksums.
*
* @copyright Copyright 2020 Matric Limited. Licensed under the MIT License. See LICENSE file in the project root for full license information.
*/

#ifndef CRC16_H_
#define CRC16_H_
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"
/**
 * \brief Compute the updated CRC16 value.
 *
 * Compute the updated CRC16 value based on the current CRC16 value and the additional data.
 *
 * \param crc the current CRC16 value
 * \param data the new data
 * \param length the number of bytes of new data
 *
 * \return the updated CRC16 value
 */
uint16_t crc16_calculate(uint16_t crc, uint8_t* data, uint16_t length);

#endif /* CRC16_H_ */
