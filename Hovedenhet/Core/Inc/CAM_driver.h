/*
 * CAM_driver.h
 *
 *  Created on: Apr 16, 2025
 *      Author: Thomas
 */

#ifndef INC_CAM_DRIVER_H_
#define INC_CAM_DRIVER_H_

void command_cam(CAM_Data CAM, uint8_t CMD);

//Commands to CAM
enum{
	IDLE = 0x41,
	REC = 0x42,
	FORMAT = 0x43,
	REBOOT = 0x44,
	DEB = 0x45
};

#endif /* INC_CAM_DRIVER_H_ */
