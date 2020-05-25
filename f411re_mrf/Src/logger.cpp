#include "logger.h"
#include "stdarg.h"
#include <ostream>
#include "string.h"
#include "stm32f4xx_hal.h"
#include <iostream>

extern UART_HandleTypeDef huart2;

/**
 * C (printf) style logger
 * @param format format string
 * @param ... variable args
 */
void log(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  char buffer[100];
  int length = vsprintf(buffer, format, args);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, length, HAL_TIMEOUT);
  va_end(args);
}
