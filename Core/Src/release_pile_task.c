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

#include "release_pile_task.h" 
#include "gpio_board_defines.h"
#include "dynamixel_ax.h"
#include "flags.h"

#define TASK_STACK_SIZE             512


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
  
	while(1)
	{
    vTaskSuspend(Release_Pile_Task.GetTaskHandle());
    const uint16_t angle1 = 22;
    const uint16_t angle2 = 135;

    uint8_t ID1;
    uint8_t ID2;

    uint8_t side = *flag.side;

    if (side == 1)
      ID2 = 5;
    else if (side == 2)
      ID2 = 6;
    else
      ID2 = 4;
    ID1 = side;

    Dynamixel_AX.TorqueEnable(USART3,ID1);
    Dynamixel_AX.SetAngle(USART3,ID1,angle1);

    Dynamixel_AX.TorqueEnable(USART3,ID2);
    Dynamixel_AX.SetAngle(USART3,ID2,angle2);



	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "Release_Pile_Task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct release_pile_task_s Release_Pile_Task = {.CreateTask = CreateTask,
                        .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
