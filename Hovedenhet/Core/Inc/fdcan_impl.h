/*
 * CAN.h
 *
 *  Created on: Apr 1, 2025
 *      Author: Thomas
 */

#ifndef INC_FDCAN_IMPL_H_
#define INC_FDCAN_IMPL_H_

#include "main.h"

#define FDCAN_FLEXIBLE_DATARATE 0

void FDCAN_parse_message(uint32_t id, void* pData, uint8_t size); // Needs to be implemented
void FDCAN_init(FDCAN_HandleTypeDef* hfdcan);
HAL_StatusTypeDef FDCAN_sendmsg(uint32_t id, void* pData, uint8_t size);


#endif /* INC_FDCAN_IMPL_H_ */
