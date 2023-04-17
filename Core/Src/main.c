/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "board_hw_config.h"
#include "spi_gpio.h" 
#include "spi_gpio_defines.h"
#include "gpio_board_defines.h"
#include "gpio_board_config.h"
#include "gpio_robot_config.h"
#include "maxon.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "byte2word.h" 

#include "dynamixel_protocol10.h"
#include "dynamixel_ax.h" 
#include "usartx_communication.h" 


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "odometry_task.h" 
#include "hl_old.h"
#include "terminal_task.h"
#include "experimental_task_1.h" 
#include "experimental_task_2.h"
#include "grub_pile_task.h"
#include "release_pile_task.h"
#include "hl_new.h"
#include "button_task.h"
#include "game_task.h"
#include "task_DC.h"

#include "serial_protocol.h"
#include "encoder_board_drvr.h"


#include "task_DC1.h"
#include "task_DC2.h"
#include "task_DC0.h"
#include "task_sort.h"

#include "task_sort_optimize.h" 
#include "task_zero_state.h" 
#include "task_gripper_calibration.h" 
#include "task_vacuum.h"

/**
  * @brief  The application entry point.
  * @retval int
  */

void EXTI_Init(void);
void HB_Init(void);
int main(void)
{
  	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	SYSCFG_PWR_Init();
	SystemClock_Config();
	
	/*	IT IS IMPORTANT FOR FreeRTOS	*/
	NVIC_SetPriorityGrouping(0);
	
	HW_Clock_Config();
	GPIO_Board_Config();
	GPIO_Robot_Config();

	SPI1_Board_HW_Init();
	SPI_GPIO_ResetOutputPin(SPI_OUTPUT_ALL);

	

	USART1_Terminal_Init();
	USART3_Dynamixel_Init();
	UART4_RS485_Init();
  UART5_Serial_Init();
	USART6_RS485_Init();

	
  
	
	TIM1_Steppers_PWM_Init();
	TIM4_RC_Servos_0123_Init();
	TIM5_RC_Servos_4567_Init();
	TIM8_WS2812B_Init();
	TIM14_USARTx_Timeout_Init();

	#if defined (ROBOT_BIG)

		

	#endif

	#if defined (ROBOT_GUIDO)

		

	#endif

	#if defined (ROBOT_BIG)

		

	#endif
	
	
	USARTx_AttachProtocol(USART1,HL_new.Protocol,SLAVE);
	USARTx_AttachProtocol(USART3,Dynamixel_Protocol10.Protocol,MASTER);
	USARTx_AttachProtocol(UART4,Dynamixel_Protocol10.Protocol,MASTER);
  USARTx_AttachProtocol(UART5,Serial.Protocol,MASTER_SLAVE);
	USARTx_AttachProtocol(USART6,Dynamixel_Protocol10.Protocol,MASTER);

  EXTI_Init();
  HB_Init();

	Button_Task.CreateTask(2);
	Odometry_Task.CreateTask(1);
	Terminal_Task.CreateTask(1);
  Game_Task.CreateTask(1);
  Grub_Pile_Task.CreateTask(1);
  Release_Pile_Task.CreateTask(1);
  DC_Task.CreateTask(1);
  DC0_Task.CreateTask(1);
  DC1_Task.CreateTask(1);
  DC2_Task.CreateTask(1);
  Sort_Task.CreateTask(1);
  Zero_State_Task.CreateTask(1);
  Sort_Optimize_Task.CreateTask(1);
  Gripper_Calibration_Task.CreateTask(1);
  Vacuum_Task.CreateTask(1);




	// Experimental_Task_1.CreateTask(1);
	// Experimental_Task_2.CreateTask(1);

  vTaskStartScheduler();
	
  while (1);
}
	



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void EXTI_Init(void){
  GPIOA->MODER &= ~(3<<16);
  GPIOA->PUPDR |= 1 << 17;
  GPIOA->PUPDR &= ~(1 << 16);
  SYSCFG->EXTICR[2] &= ~15;

  EXTI->FTSR |= EXTI_FTSR_TR8;
  EXTI->PR = EXTI_PR_PR8; 
  EXTI->IMR |= EXTI_IMR_MR8;   
  
  NVIC_EnableIRQ(EXTI9_5_IRQn); 

}

void HB_Init(void){
  GPIOC->MODER |= (1<<28); //AB1 output
  GPIOC->MODER &= ~(1<<29);

  GPIOC->MODER |= (1<<30);
  GPIOC->MODER &= ~(1<<31);
  //-----------------------//

  GPIOE->MODER |= (1<<12); //AB2 output
  GPIOE->MODER &= ~(1<<13);

  GPIOC->MODER |= (1<<26);
  GPIOC->MODER &= ~(1<<27);
  //-----------------------//

  GPIOE->MODER |= (1<<10); //AB3 output
  GPIOE->MODER &= ~(1<<11);

  // GPIOE->MODER |= (1<<8);
  // GPIOE->MODER &= ~(1<<9);
  //-----------------------//

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
