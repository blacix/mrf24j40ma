#include "app.h"
#include "logger.h"
#include "MRF24J40.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"

const uint16_t MRF_NODE_ADDRESS = 4;
#define PID_SHP     1

typedef struct {
  uint8_t functionActive;
  uint8_t deviceToControl;
  uint8_t firstDevice;
  uint8_t lastDevice;
  uint8_t imuStatus;
  uint8_t magnetoStatus;
  uint8_t gyroStatus;
  uint8_t acceleroStatus;
  float w;
  float x;
  float y;
  float z;
  uint16_t infrared;
  uint16_t battery;
  int temperature;
} radioSHP;

radioSHP headset;

MRF mrf;
void appInit(SPI_HandleTypeDef* spiHandler)
{
  mrf = MRF(spiHandler);
  mrf.begin(MRF_DEFAULT_CHANNEL, MRF_DEFAULT_PAN, MRF_NODE_ADDRESS);
  mrf.clearISR();
  if(mrf.isOnSpi(MRF_DEFAULT_PAN))
  {
    log("mrf started on SPI\n");
  }
  else
  {
    log("MRF INIT ERROR\n");
  }
}

extern "C" void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    log("lqqi: %d \n", mrf.getAverageLinkQuality());
    log("x: %f y: %f z: %f \n", headset.x, headset.y, headset.z);
    osDelay(1000);
  }
}

void handleMrfIrq()
{
  uint8_t last_interrupt = mrf.readShort(MRF_INTSTAT);
  if (last_interrupt & MRF_I_RXIF)
  {
    mrf.enableRX(false);

    std::vector<uint8_t> data;
    mrf.RXcopy(data);

    if(data.size() > 0)
    {
      uint8_t messageType = data[0];
      if(messageType == PID_SHP)
      {
        memcpy(&headset, mrf.read, sizeof(headset));
      }
    }

    mrf.flushRX();
    mrf.enableRX(true);
  }
}

/**
 * declared as __weak in FreeRTOS
 * freertos will call it on ext irq
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //log("HAL_GPIO_EXTI_Callback - gpio: %u\n", GPIO_Pin);
  switch(GPIO_Pin)
  {
  case MRF_INT_Pin:
    handleMrfIrq();
    break;
  default:
    break;
  }
}

