/*
 * EX_Global_var.h
 *
 *  Created on: Apr 1, 2025
 *      Author: Thomas
 */

#ifndef INC_EX_GLOBAL_VAR_H_
#define INC_EX_GLOBAL_VAR_H_

//Variabler
extern uint8_t Start_Flight_Recording;
extern uint8_t command;
extern uint32_t Local_Time;

//Flag
extern uint8_t GoPro;

//TypeDef
extern CAN_Data CAN;
extern SPI_Data SPI;
extern SR_Data SR;
extern Flash_Data Flash;
extern CAM_Data CAM1;
extern CAM_Data CAM2;
extern CAM_Data CAM3;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;

#endif /* INC_EX_GLOBAL_VAR_H_ */
