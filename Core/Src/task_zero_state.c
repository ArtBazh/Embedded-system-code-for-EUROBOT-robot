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

#include "task_zero_state.h" 
#include "gpio_board_defines.h"
#include "flags.h"
#include "task_DC.h" 
#include "release_pile_task.h"
#include "grub_pile_task.h"



#define TASK_STACK_SIZE             512


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;
static int32_t UP_POSITION = 363;

static void Task(void *arg)
{
  
	while(1)
	{
      vTaskSuspend(TaskHandle);

      *flag.ID_DC = 2;
      *flag.angle_DC = UP_POSITION;
      *flag.DC_state = 0;
      vTaskResume(DC_Task.GetTaskHandle());
      while(*flag.DC_state == 0);

      // vTaskDelay(10);
      *flag.ID_DC = 0;
      *flag.angle_DC = 0;
      *flag.DC_state = 0;
      vTaskResume(DC_Task.GetTaskHandle());
      while(*flag.DC_state == 0);

      vTaskDelay(50);
      *flag.ID_DC = 1;
      *flag.angle_DC = 0;
      *flag.DC_state = 0;
      vTaskResume(DC_Task.GetTaskHandle());
      while(*flag.DC_state == 0);

      *flag.side = 1;
      vTaskResume(Grub_Pile_Task.GetTaskHandle());
      *flag.side = 2;
      vTaskResume(Grub_Pile_Task.GetTaskHandle());
      *flag.side = 3;
      vTaskResume(Grub_Pile_Task.GetTaskHandle());

      vTaskDelay(300);
      *flag.ID_DC = 2;
      *flag.angle_DC = 0;
      *flag.DC_state = 0;
      vTaskResume(DC_Task.GetTaskHandle());
      while(*flag.DC_state == 0);

      *flag.side = 1;
      vTaskResume(Release_Pile_Task.GetTaskHandle());
      *flag.side = 2;
      vTaskResume(Release_Pile_Task.GetTaskHandle());
      *flag.side = 3;
      vTaskResume(Release_Pile_Task.GetTaskHandle());
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "Zero_State_Task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct zero_state_task_s Zero_State_Task = {.CreateTask = CreateTask,
                                            .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
