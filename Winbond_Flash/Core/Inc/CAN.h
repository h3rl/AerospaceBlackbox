/*
 * CAN.h
 *
 *  Created on: Apr 1, 2025
 *      Author: Thomas
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

void CAN_ReceiveMessage(void);
void CAN_SendMessage(uint16_t ID);
#endif /* INC_CAN_H_ */
