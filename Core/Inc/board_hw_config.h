/**
  ******************************************************************************
  * @file    	functions.h
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_HW_CONFIG_H
#define __BOARD_HW_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
	 
#include "main.h"

#define TIM1_STEPPER_AUTORELOAD_VALUE 10000
	 
void SYSCFG_PWR_Init(void);
void SystemClock_Config(void);
void HW_Clock_Config(void);




void USART1_Terminal_Init(void);
void USART3_Dynamixel_Init(void);
void UART4_RS485_Init(void);
void UART5_Serial_Init(void);
void USART6_RS485_Init(void);

void SPI1_Board_HW_Init(void);

void TIM1_Steppers_PWM_Init(void);
void TIM4_RC_Servos_0123_Init(void);
void TIM5_RC_Servos_4567_Init(void);
void TIM8_WS2812B_Init(void);
void TIM14_USARTx_Timeout_Init(void);

// void TIM10_Kinematics_Init(void);
// void TIM11_Timout_100S_Init(void);

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_HAL_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
