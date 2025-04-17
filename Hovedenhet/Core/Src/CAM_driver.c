/*
 * CAM_driver.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Thomas
 */
#include "main.h"
#include "EX_Global_var.h"

void command_cam(CAM_Data CAM, uint8_t CMD){
	CAM.Tx_Buffer[0] = CMD;
	CAM.Tx_Buffer[1] = CMD;
	HAL_UART_Transmit(CAM.huart, CAM.Tx_Buffer, 2, 100);
}
