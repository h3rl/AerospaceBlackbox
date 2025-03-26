/*
 * Misc.h
 *
 *  Created on: Mar 26, 2025
 *      Author: Thomas
 */

#ifndef INC_MISC_H_
#define INC_MISC_H_

void send_uart(char *string);
void send_byte_as_binary(uint8_t byte);
void delay_ns(uint32_t ns);

#endif /* INC_MISC_H_ */
