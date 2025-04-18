#include "cmsis_os.h"
#include "freertos.h"
#include "stm32f4xx_hal.h"
#include "defines.h"

/* USER CODE BEGIN Header_BlinkTask_Start */
/**
* @brief Function implementing the BlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlinkTask_Start */
__weak void BlinkTask_Start(void *argument)
{
  /* USER CODE BEGIN BlinkTask_Start */
  /* Infinite loop */
  for(;;)
  {
    for (;;) {
      HAL_GPIO_TogglePin(C13_GPIO_Port, C13_Pin);
      osDelay(2000);
  
      HAL_GPIO_TogglePin(C13_GPIO_Port, C13_Pin);
      osDelay(500);
    } 
  }
  /* USER CODE END BlinkTask_Start */
}