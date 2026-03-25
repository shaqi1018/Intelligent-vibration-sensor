/**
  ******************************************************************************
  * @file   user_app.c
  * @brief  用户应用程序部分的代码
  * 
  ******************************************************************************
  */
#include "user_app.h"

/**
* @brief Function implementing the blueBlinkTask thread.
* @param argument: Not used
* @retval None
*/
void StartBlueBlinkTask(void *argument)
{
	//无限循环
  for(;;)
  {

		//翻转输出指示灯
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
    osDelay(500);
  }
}


/**
* @brief Function implementing the greenBlinkTask thread.
* @param argument: Not used
* @retval None
*/
void StartGreenBlinkTask(void *argument)
{
  //无限循环
  for(;;)
  {

		//翻转输出指示灯
		HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);
    osDelay(250);
  }
}
