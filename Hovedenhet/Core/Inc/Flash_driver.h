/*
 * Flash_driver.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Thomas
 */

#ifndef INC_FLASH_DRIVER_H_
#define INC_FLASH_DRIVER_H_

#include "main.h"

typedef struct {
  uint8_t Buffer_0[2048]; // Data buffer 0
  uint8_t Buffer_1[2048]; // Data buffer 1
  uint8_t Buffer_Select;  // Buffer Select (0 or 1)
  uint8_t Memory_Full;    // Flag to set when memory is full
  uint16_t Block_Mem;     // Block memory. Used to detect block change
  uint16_t Page_Index;    // Page index. Used to track current page
  uint16_t Buffer_Index;  // BUffer index. Used to control when buffer is full
  uint32_t ID;            // JEDEC ID for flash IC
  uint8_t *Buffer_p; 	// Pointer to data buffer. Used to store data in buffer
  uint8_t SR_1;      	// Status register 1
  uint8_t SR_2;      	// Status register 2
  uint8_t SR_3;      	// Status register 3
} flash_data_t;

extern flash_data_t flash;

// Software
void Read_All_Status_Register(void);
void Write_to_page(void);
void Write_Data(uint8_t *data, uint16_t lenght);
void Read_Data(uint16_t page, uint8_t *data, uint16_t len);
void Automatic_Block_Managment(uint16_t Page_Index);
void Chip_Erase(void);
void Read_Data_Cont(uint16_t len);
void Flash_Init(uint8_t BUF);
uint32_t Read_ID(void);
uint8_t Read_Status_Register(uint8_t SR);

void Write_Data_Flash(uint16_t Page_Addr);
#endif /* INC_FLASH_DRIVER_H_ */
