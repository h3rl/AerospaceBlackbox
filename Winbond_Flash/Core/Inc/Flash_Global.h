/*
 * Flash_Global.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Thomas
 */

#ifndef INC_FLASH_GLOBAL_H_
#define INC_FLASH_GLOBAL_H_

void Read_Register(void);
void Write_to_page(void);
void Write_Data(uint8_t* data, uint16_t lenght);
void Read_Data(uint16_t page, uint8_t* data);

extern uint8_t SR_1;
extern uint8_t SR_2;
extern uint8_t SR_3;
extern uint8_t Read_data[2048];
#endif /* INC_FLASH_GLOBAL_H_ */
