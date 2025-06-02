/*
 * util.c
 *
 *  Created on: Jun 2, 2025
 *      Author: halva
 */

#include "main.h"
#include "util.h"

#include <stdarg.h>
#include <string.h>

void zeromem(void* pdata, size_t size)
{
    memset(pdata, 0, size);
}

void print(const char* format, ...)
{
#ifndef ENABLE_SERIAL_PRINTF
    (void)format;
#else
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}
