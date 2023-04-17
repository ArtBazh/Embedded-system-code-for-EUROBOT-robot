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

#include "task_DC2.h" 
#include "gpio_board_defines.h"

#include "spi_gpio_defines.h"
#include "spi_gpio.h" 

#include "flags.h"
#include "stm32f4xx_ll_gpio.h"

#include "serial_protocol.h"
#include "encoder_board_drvr.h"


#define TASK_STACK_SIZE             512


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
    // uint32_t encoder_counts;
  
	while(1)
	{
        vTaskSuspend(TaskHandle);
        LL_GPIO_TogglePin(GPIOA, SERV7_GPIOA_PIN3);
        // Encoder_Board.GetEncoderCounts(UART5, 2, &encoder_counts);
        while(abs(*flag.DC2_current_pos - *flag.angle_DC2) > 3){
        // Encoder_Board.GetEncoderCounts(UART5, 2, &encoder_counts);
            setPosition2motor(2, *flag.angle_DC2);
            vTaskDelay(15);
        }
        *flag.DC2_state = 1;
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "DC2_task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct DC2_task_s DC2_Task = {.CreateTask = CreateTask,
                              .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
