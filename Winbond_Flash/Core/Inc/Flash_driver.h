/*
 * Flash_driver.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Thomas
 */

#ifndef INC_FLASH_DRIVER_H_
#define INC_FLASH_DRIVER_H_

void Flash_Init(uint8_t BUF);
uint8_t Read_Status_Register(uint8_t SR);
void Write_Status_Register(uint8_t SR, uint8_t REG_DATA);
uint32_t Read_ID(void);
void Block_Erase(uint16_t Page_Addr);
void Write_Data_Buffer(uint16_t Buffer_Addr, uint8_t *Data, uint16_t len);
void Write_Data_Flash(uint16_t Page_Addr);
void Select_Page_Read(uint16_t Page_Addr);
void Read_Data_Buffer(uint8_t *Data, uint16_t len);

extern uint8_t SR_1_Addr;
extern uint8_t SR_2_Addr;
extern uint8_t SR_3_Addr;

#endif /* INC_FLASH_DRIVER_H_ */
