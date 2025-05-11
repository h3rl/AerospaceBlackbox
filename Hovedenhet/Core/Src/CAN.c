/*
 * CAN.c
 *
 *  Created on: Apr 1, 2025
 *      Author: Thomas
 */
#include "main.h"
#include "CAN.h"
#include "EX_Global_Var.h"

extern FDCAN_HandleTypeDef hfdcan1;

void CAN_ReceiveMessage(void) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CAN.Rx_Buffer) == HAL_OK) {
    	USART3_Printf("Received CAN Message: ");
        for (int i = 0; i < 8; i++) {
        	USART3_Printf("%02X ", CAN.Rx_Buffer[i]);
        }
        USART3_Printf("\r\n");
    }
}

void CAN_SendMessage(uint16_t ID) {
    // Configure TX Header
    TxHeader.Identifier = ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Send Message
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN.Tx_Buffer) != HAL_OK) {
        Error_Handler();
    }
}

void CAN_SendStatus(uint8_t CAN_Timeout){
	uint32_t Status = 0;

	if(HAL_GPIO_ReadPin(GPIOC, CAM1_PWR_Pin)){
		Status |= (1 << 0);
	}
	if(HAL_GPIO_ReadPin(GPIOE, CAM2_PWR_Pin)){
		Status |= (1 << 1);
	}
	if(HAL_GPIO_ReadPin(GPIOB, CAM3_PWR_Pin)){
		Status |= (1 << 2);
	}
	if(CAM1.Status[0] == 0x42){
		Status |= (1 << 3);
	}
	if(CAM2.Status[0] == 0x42){
		Status |= (1 << 4);
	}
	if(CAM3.Status[0] == 0x42){
		Status |= (1 << 5);
	}
	if(CAN_Timeout){
		Status |= (1 << 6);
	}
	if(Start_Flight_Recording){
		Status |= (1 << 7);
	}

	*(uint32_t*)&CAN.Tx_Buffer[0] = Status;
	*(uint16_t*)&CAN.Tx_Buffer[4] = Flash.Page_Index;
	CAN_SendMessage(400);
}
