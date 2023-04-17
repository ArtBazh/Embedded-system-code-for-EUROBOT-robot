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

#include "task_gripper_calibration.h" 
#include "gpio_board_defines.h"
#include "dynamixel_ax.h"
#include "stm32f4xx_ll_gpio.h"



#define TASK_STACK_SIZE             512


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
  
	while(1)
	{
      vTaskSuspend(TaskHandle);
      uint16_t angle1 = 129;
      uint16_t angle2 = 25;
      Dynamixel_AX.TorqueEnable(USART3,4);
      Dynamixel_AX.SetAngle(USART3,4,angle1);

      Dynamixel_AX.TorqueEnable(USART3,5);
      Dynamixel_AX.SetAngle(USART3,5,angle1);

      Dynamixel_AX.TorqueEnable(USART3,6);
      Dynamixel_AX.SetAngle(USART3,6,angle1);

      Dynamixel_AX.TorqueEnable(USART3,1);
      Dynamixel_AX.SetAngle(USART3,1,angle2);

      Dynamixel_AX.TorqueEnable(USART3,2);
      Dynamixel_AX.SetAngle(USART3,2,angle2);

      Dynamixel_AX.TorqueEnable(USART3,3);
      Dynamixel_AX.SetAngle(USART3,3,angle2);
    

	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "grip_cal_Task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct gripper_calibration_task_s Gripper_Calibration_Task = {.CreateTask = CreateTask,
                                                              .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
