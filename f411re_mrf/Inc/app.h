#ifndef APP_H_
#define APP_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus

#include <string>

extern "C"
{
#endif

void appInit(SPI_HandleTypeDef* spiHandler);

#ifdef __cplusplus
}

void appHandleIrq();


#endif
#endif /* APP_H_ */
