/**
  ******************************************************************************
  * @file    Projects/Multi/Examples/FreeFallDetection/Src/main.c
  * @author  CL
  * @version V1.3.0
  * @date    28-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include <math.h>   // trunc

/** @addtogroup X_NUCLEO_IKS01A1_Examples
  * @{
  */

/** @addtogroup FREE_FALL_DETECTION
  * @{
  */

/* Private variables ---------------------------------------------------------*/
static int free_fall_detected = 0;
static volatile int ff_enable = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how X_NUCLEO_IKS01A1 expansion board is able to detect free fall event.
 *         In order to run the example you need to plug into the X_NUCLEO_IKS01A1 expansion board a DIL24
 *         expansion component with a LSM6DS3 sensor.
 *
 *         After application starts:
 *         - the user can try to leave falling the STM32 Nucleo board and when the free fall is detected, the LED is switched on for a while.
 *         - the user can enable/disable the free fall detection feature pushing the user button.
 * @retval Integer
 */
int main(void)
{
  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
   */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize LEDs */
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED2);
  
  /* Initialize Button */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
#endif
  
#if (defined (USE_STM32L1XX_NUCLEO))
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
#endif
  
  /* Initialize the IMU 6-axes */
  BSP_IMU_6AXES_Init();
  /* Enable free fall detection */
  BSP_IMU_6AXES_Enable_Free_Fall_Detection_Ext();
  ff_enable = 1;
  
  while(1)
  {
    /* Check if free fall detection happened */
    if(free_fall_detected != 0)
    {
      BSP_LED_On(LED2);
      
      HAL_Delay(500);
      
      BSP_LED_Off(LED2);
      free_fall_detected = 0;
    }
    
    /* Check if the user has pushed the button */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
    if(BSP_PB_GetState(BUTTON_KEY) == RESET)
#endif
    
#if (defined (USE_STM32L1XX_NUCLEO))
      if(BSP_PB_GetState(BUTTON_USER) == RESET)
#endif
      {
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
        while (BSP_PB_GetState(BUTTON_KEY) == RESET);
#endif
        
#if (defined (USE_STM32L1XX_NUCLEO))
        while (BSP_PB_GetState(BUTTON_USER) == RESET);
#endif
        
        if(!ff_enable)
        {
          BSP_IMU_6AXES_Enable_Free_Fall_Detection_Ext();
          ff_enable = 1;
        }
        else
        {
          BSP_IMU_6AXES_Disable_Free_Fall_Detection_Ext();
          ff_enable = 0;
        }
      }
      
    /* Wait a while */
    HAL_Delay(10);
  }
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  if(GPIO_Pin == MEMS_INT1_PIN)
  {
    uint8_t stat = 0;
    BSP_IMU_6AXES_Get_Status_Free_Fall_Detection_Ext(&stat);
    if(stat)
    {
      free_fall_detected = 1;
    }
  }
}

/**
 * @brief  This function is executed in case of error occurrence
 * @retval None
 */
void Error_Handler(void)
{
  while(1)
  {}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
