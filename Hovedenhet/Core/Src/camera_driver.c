/*
 * CAM_driver.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Thomas
 */

#include "camera_driver.h"

void CAM_send_command(camera_data_t* camera, uint8_t command){
	uint8_t buffer[2] = { command, command };
	HAL_UART_Transmit(camera->huart, buffer, 2, 100);
}

void CAM_init(camera_data_t* camera, UART_HandleTypeDef *huart)
{
	camera->huart = huart;
	zerobuf(camera->rx_buffer);
	zerobuf(camera->status);
}
