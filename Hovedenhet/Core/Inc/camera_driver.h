/*
 * camera_driver.h
 *
 *  Created on: Apr 16, 2025
 *      Author: Thomas
 */

#ifndef INC_CAMERA_DRIVER_H_
#define INC_CAMERA_DRIVER_H_

#include "main.h"

// Cammera commands
#define CAMERA_IDLE 0x41
#define CAMERA_RECORDING 0x42
#define CAMERA_FORMAT 0x43
#define CAMERA_REBOOT 0x44
#define CAMERA_DEBUG 0x45

typedef struct {
  uint8_t rx_buffer[2];      // Receive data buffer CAM module
  uint8_t status[2];         // Status data from CAM module
  UART_HandleTypeDef *huart; // typedef pointer to UART module used by CAM module
} camera_data_t;

void CAM_init(camera_data_t* camera, UART_HandleTypeDef *huart);
void CAM_send_command(camera_data_t *camera, uint8_t CMD);

#endif /* INC_CAMERA_DRIVER_H_ */
