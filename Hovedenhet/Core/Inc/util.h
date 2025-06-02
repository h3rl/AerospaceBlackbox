/*
 * util.h
 *
 *  Created on: Jun 2, 2025
 *      Author: halva
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <stdio.h>

void print(const char* format, ...);
void zeromem(void* pdata, size_t size);

#define zerobuf(buffer) zeromem(buffer, sizeof(buffer));

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define clamp(value, minimum, maximum) (((value) < (minimum)) ? (minimum) : (((value) > (maximum)) ? (maximum) : (value)))



#endif /* INC_UTIL_H_ */
