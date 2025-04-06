/*
 * Flash_driver.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Thomas
 */

#ifndef INC_FLASH_DRIVER_H_
#define INC_FLASH_DRIVER_H_

#define numBLOCK 1024  //Antall blokker
#define numPAGES 65536 //Antall sider totalt. 64 sider per. blokk
#define numBYTES 2048  //Antall bytes per. side

//Software
void Read_Register(void);
void Write_to_page(void);
void Write_Data(uint8_t* data, uint16_t lenght);
void Read_Data(uint16_t page, uint8_t* data);
void Automatic_Block_Managment(uint16_t Page_Index);
void Chip_Erase(void);
void Read_Data_Cont(uint16_t len);
#endif /* INC_FLASH_DRIVER_H_ */
