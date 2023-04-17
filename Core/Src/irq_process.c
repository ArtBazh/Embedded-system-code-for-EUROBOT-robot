/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         functions.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "irq_process.h" 
#include "usartx_communication.h" 
#include "robot_behavior_routine.h" 
#include "stepper.h"
#include "usartx_timeout.h" 

#include "gpio_board_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#include "ws2812b.h" 

void USART1_IRQHandler()
{
  USARTx_IRQ_Process(USART1);
}

void USART3_IRQHandler()
{
  USARTx_IRQ_Process(USART3);
}

void UART4_IRQHandler()
{
  USARTx_IRQ_Process(UART4);
}

void UART5_IRQHandler()
{
  USARTx_IRQ_Process(UART5);
}

void USART6_IRQHandler()
{
  USARTx_IRQ_Process(USART6);
}

void TIM1_CC_IRQHandler()
{
	TIM1_Steppers_PWM_CC_IRQ_Process();
}

void TIM8_TRG_COM_TIM14_IRQHandler()
{
  USARTx_Timeout_IRQ_Process();
}

void DMA2_Stream1_IRQHandler()
{
  DMA_WS2812B_IRQ_Process();
}

// void EXTI0_IRQHandler()
// {
// 	EXTI_0_31_ButtonProcess();
// }

// void EXTI1_IRQHandler()
// {
// 	EXTI_0_31_ButtonProcess();
// }

// void EXTI9_5_IRQHandler()
// {
// 	EXTI_0_31_ButtonProcess();
// }

// void EXTI15_10_IRQHandler()
// {
// 	EXTI_0_31_ButtonProcess();
// }

// void TIM8_UP_TIM13_IRQHandler()
// {
//   // Multidelay_Timer_IRQ_Process(TIM13);
// 	// TIM13_USART6_Timeout_IRQ_Process();
// }



// void TIM1_TRG_COM_TIM11_IRQHandler()
// {
// 	TIM11_IT_IRQ_100S_Process();
// }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
