/*
 * TypeDef.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Thomas
 */

#ifndef INC_TYPEDEF_H_
#define INC_TYPEDEF_H_

//Init TypeDef struct
typedef struct{
	uint8_t SR_1;		//Status register 1
	uint8_t SR_2;		//Status register 2
	uint8_t SR_3;		//Status register 3
	uint8_t SR_1_Addr;	//Status register 1 address
	uint8_t SR_2_Addr;	//Status register 1 address
	uint8_t SR_3_Addr;	//Status register 1 address
}SR_Data;

typedef struct{
	uint8_t Tx_Buffer[5];	//Transmit data buffer SPI
	uint8_t Rx_Buffer[5];	//Receive data buffer SPI
}SPI_Data;

typedef struct{
	uint8_t Tx_Buffer[8];	//Transmit data buffer CAN
	uint8_t Rx_Buffer[8];	//Receive data buffer CAN
}CAN_Data;

typedef struct{
	uint8_t Buffer_0[2048];	//Data buffer 0
	uint8_t Buffer_1[2048];	//Data buffer 1
	uint8_t Buffer_Select;	//Buffer Select (0 or 1)
	uint8_t Memory_Full;	//Flag to set when memory is full
	uint16_t Block_Mem;		//Block memory. Used to detect block change
	uint16_t Page_Index;	//Page index. Used to track current page
	uint16_t Buffer_Index;	//BUffer index. Used to control when buffer is full
	uint32_t ID;			//JEDEC ID for flash IC
	uint8_t* Buffer_p;		//Pointer to data buffer. Used to store data in buffer
}Flash_Data;

typedef struct{
	uint8_t Tx_Buffer[2];		//Transmit data buffer CAM module
	uint8_t Rx_Buffer[2];		//Receive data buffer CAM module
	uint8_t Status[2];			//Status data from CAM module
	UART_HandleTypeDef* huart;	//typedef pointer to UART module used by CAM module
}CAM_Data;
#endif /* INC_TYPEDEF_H_ */
