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

#include "button_task.h"
#include "button.h"
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "gpio_robot_defines.h"
#include "buttons_enum.h"
#include "robot_behavior_routine.h" 


#define TASK_STACK_SIZE             64


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
  TickType_t xLastWakeTime;

  Button.AddButton(STARTING_CORD_BTN, GPIOC, STARTING_CORD_BTN_GPIO14_PC15, BUTTON_POLARITY_HI, ControlButtons_Callback);
  Button.AddButton(PLAYGROUND_SIDE_BTN, GPIOE, PLAYGROUND_SIDE_BTN_GPIO13_PE6, BUTTON_POLARITY_HI, ControlButtons_Callback);
  Button.AddButton(STRATEGY_BTN, GPIOE, STRATEGY_BTN_GPIO12_PE5, BUTTON_POLARITY_HI, ControlButtons_Callback);
  Button.AddButton(MAXON_ENABLE_BTN, GPIOE, MAXON_ENABLE_BTN_GPIO11_PE2, BUTTON_POLARITY_HI, ControlButtons_Callback);
  Button.AddButton(START_GAME, GPIOE, GPIO17_PE4, BUTTON_POLARITY_HI, ControlButtons_Callback);
  Button.AddButton(UP_BTN, GPIOE, GPIO18_PE3, BUTTON_POLARITY_HI, ControlButtons_Callback);
  Button.AddButton(DOWN_BTN, GPIOE, GPIO19_PE0, BUTTON_POLARITY_HI, ControlButtons_Callback);

	while(1)
	{
    xLastWakeTime = xTaskGetTickCount();
    Button.PollButtons();
    xTaskDelayUntil(&xLastWakeTime,1);
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "Button_Task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct button_task_s Button_Task = {.CreateTask = CreateTask,
                                    .GetTaskHandle = GetTaskHandle};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
