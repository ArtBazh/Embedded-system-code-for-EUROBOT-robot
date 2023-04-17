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

#include "terminal_task.h" 
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "terminal_cmd_handler.h" 
#include "terminal.h" 




#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
    struct terminal_request_s terminal_request;
    Terminal_Cmd_Handlers_Init();

    vTaskDelay(100);
    
	while(1)
	{
        while(Terminal.Get_Request(&terminal_request))
        {
            if((Terminal_Cmd_Handlers[terminal_request.cmd]!=NULL)&&(terminal_request.cmd<TERMINAL_CMD_TABLE_SIZE))
                Terminal_Cmd_Handlers[terminal_request.cmd](terminal_request.data,terminal_request.data_length);
        }

        taskYIELD();
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "Terminal_Task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct terminal_task_s Terminal_Task = {.CreateTask = CreateTask,
                                        .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
