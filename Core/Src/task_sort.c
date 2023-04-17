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

#include "task_sort.h" 
#include "gpio_board_defines.h"

#include "spi_gpio_defines.h"
#include "spi_gpio.h" 

#include "flags.h"
#include "stm32f4xx_ll_gpio.h"

#include "serial_protocol.h"
#include "encoder_board_drvr.h"
#include "dynamixel_ax.h" 

#include "grub_pile_task.h"
#include "release_pile_task.h"
#include "rc_servo.h"

#include "serial_protocol.h"
#include "encoder_board_drvr.h"
#include "DC_Motor.h"
#include "task_DC.h" 
#include "task_DC0.h" 
#include "task_DC1.h" 
#include "task_DC2.h" 




#define TASK_STACK_SIZE             512


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;

static int32_t UP_POSITION = 363;
static int32_t ZERO_POSITION = 0;
static int32_t ROTATION = 578;
static int32_t MEAN_POSITION_82 = 82;
static int32_t MEAN_POSITION_110 = 110;
static int32_t MEAN_POSITION_170 = 170;

static uint16_t DYN_angle1 = 0;
static uint16_t DYN_angle2 = 0;
static uint16_t DYN_angle3 = 0;
static uint16_t DYN_angle4 = 0;
static uint16_t DYN_angle5 = 0;
static uint16_t DYN_angle6 = 0;

static uint16_t PRES = 6;




static void Task(void *arg)
{
  
	while(1)
	{
    vTaskSuspend(TaskHandle);
    *flag.sort = 0;
    // Dynamixel_AX.SetMovingSpeed(USART3, 1, 100);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 2, 100);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 3, 100);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 4, 100);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 5, 100);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 6, 100);

    *flag.side = 1;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 2;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 3;
    vTaskResume(Release_Pile_Task.GetTaskHandle());

    vTaskDelay(300);

    // Dynamixel_AX.SetMovingSpeed(USART3, 1, 1023);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 2, 1023);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 3, 1023);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 4, 1023);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 5, 1023);
    // vTaskDelay(10);
    // Dynamixel_AX.SetMovingSpeed(USART3, 6, 1023);

    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_82;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    Dynamixel_AX.TorqueEnable(USART3, 4);
    Dynamixel_AX.SetAngle(USART3, 4, 151);
    Dynamixel_AX.TorqueEnable(USART3, 5);
    Dynamixel_AX.SetAngle(USART3, 5, 151);
    Dynamixel_AX.TorqueEnable(USART3, 6);
    Dynamixel_AX.SetAngle(USART3, 6, 151);

    Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
    Dynamixel_AX.GetAngle(USART3, 5, &DYN_angle5);
    Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);

    while(abs(151 - DYN_angle4) > PRES || abs(151 - DYN_angle5) > PRES || abs(151 - DYN_angle6) > PRES){
        Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
        Dynamixel_AX.GetAngle(USART3, 5, &DYN_angle5);
        Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);
    }
    vTaskDelay(50);

    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_110;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.side = 1;
    vTaskResume(Grub_Pile_Task.GetTaskHandle());
    *flag.side = 2;
    vTaskResume(Grub_Pile_Task.GetTaskHandle());
    *flag.side = 3;
    vTaskResume(Grub_Pile_Task.GetTaskHandle());

    vTaskDelay(300);


    // *flag.ID_DC = 2;
    // *flag.angle_DC = MEAN_POSITION_500;
    // *flag.DC_state = 0;
    // vTaskResume(DC_Task.GetTaskHandle());
    // while(*flag.DC_state == 0);

    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 0;
    *flag.angle_DC = -ROTATION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 1;
    *flag.angle_DC = -ROTATION * 2;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_110;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.side = 1;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 2;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 3;
    vTaskResume(Release_Pile_Task.GetTaskHandle());

    vTaskDelay(300);
    *flag.ID_DC = 2;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    Dynamixel_AX.TorqueEnable(USART3, 2);
    Dynamixel_AX.SetAngle(USART3, 2, 40);
    Dynamixel_AX.TorqueEnable(USART3, 3);
    Dynamixel_AX.SetAngle(USART3, 3, 40);
    Dynamixel_AX.TorqueEnable(USART3, 4);
    Dynamixel_AX.SetAngle(USART3, 4, 151);
    Dynamixel_AX.TorqueEnable(USART3, 6);
    Dynamixel_AX.SetAngle(USART3, 6, 151);

    Dynamixel_AX.GetAngle(USART3, 2, &DYN_angle2);
    Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
    Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
    Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);

    while(abs(40 - DYN_angle2) > PRES || abs(40 - DYN_angle3) > PRES || abs(151 - DYN_angle4) > PRES || abs(151 - DYN_angle6) > PRES){
        Dynamixel_AX.GetAngle(USART3, 2, &DYN_angle2);
        Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
        Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
        Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);
    }

    vTaskDelay(50);
    *flag.sort = 1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    vTaskSuspend(TaskHandle); // 2 cake sorting

    *flag.sort = 0;

    *flag.side = 1;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 2;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 3;
    vTaskResume(Release_Pile_Task.GetTaskHandle());

    vTaskDelay(300);
    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_82;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    // vTaskDelay(500);
    Dynamixel_AX.TorqueEnable(USART3, 4);
    Dynamixel_AX.SetAngle(USART3, 4, 151);
    Dynamixel_AX.TorqueEnable(USART3, 6);
    Dynamixel_AX.SetAngle(USART3, 6, 151);


    Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
    Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);

    while(abs(151 - DYN_angle4) > PRES || abs(151 - DYN_angle6) > PRES){
        Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
        Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);
    }


    vTaskDelay(300);
    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_110;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    Dynamixel_AX.TorqueEnable(USART3, 2);
    Dynamixel_AX.SetAngle(USART3, 2, 40);
    Dynamixel_AX.TorqueEnable(USART3, 3);
    Dynamixel_AX.SetAngle(USART3, 3, 40);

    Dynamixel_AX.GetAngle(USART3, 2, &DYN_angle2);
    Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);

    while(abs(40 - DYN_angle2) > PRES || abs(40 - DYN_angle3) > PRES){
        Dynamixel_AX.GetAngle(USART3, 2, &DYN_angle2);
        Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
    }
    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 0;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 1;
    *flag.angle_DC = -ROTATION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_110;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    Dynamixel_AX.TorqueEnable(USART3, 3);
    Dynamixel_AX.SetAngle(USART3, 3, 22);
    Dynamixel_AX.TorqueEnable(USART3, 6);
    Dynamixel_AX.SetAngle(USART3, 6, 135);

    Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
    Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);

    while(abs(22 - DYN_angle3) > PRES || abs(135 - DYN_angle6) > PRES){
        Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
        Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);
    }

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 0;
    *flag.angle_DC = ROTATION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_170;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    Dynamixel_AX.TorqueEnable(USART3, 1);
    Dynamixel_AX.SetAngle(USART3, 1, 22);
    Dynamixel_AX.TorqueEnable(USART3, 2);
    Dynamixel_AX.SetAngle(USART3, 2, 22);
    Dynamixel_AX.TorqueEnable(USART3, 3);
    Dynamixel_AX.SetAngle(USART3, 3, 22);
    Dynamixel_AX.TorqueEnable(USART3, 5);
    Dynamixel_AX.SetAngle(USART3, 5, 135);
    Dynamixel_AX.TorqueEnable(USART3, 6);
    Dynamixel_AX.SetAngle(USART3, 6, 135);

    Dynamixel_AX.GetAngle(USART3, 1, &DYN_angle1);
    Dynamixel_AX.GetAngle(USART3, 2, &DYN_angle2);
    Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
    Dynamixel_AX.GetAngle(USART3, 5, &DYN_angle5);
    Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);

    while(abs(22 - DYN_angle1) > PRES || abs(22 - DYN_angle2) > PRES || abs(22 - DYN_angle3) > PRES || abs(135 - DYN_angle5) > PRES || abs(135 - DYN_angle6) > PRES){
        Dynamixel_AX.GetAngle(USART3, 1, &DYN_angle1);
        Dynamixel_AX.GetAngle(USART3, 2, &DYN_angle2);
        Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
        Dynamixel_AX.GetAngle(USART3, 5, &DYN_angle5);
        Dynamixel_AX.GetAngle(USART3, 6, &DYN_angle6);
    }

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    Dynamixel_AX.TorqueEnable(USART3, 1);
    Dynamixel_AX.SetAngle(USART3, 1, 40);
    Dynamixel_AX.TorqueEnable(USART3, 3);
    Dynamixel_AX.SetAngle(USART3, 3, 40);
    Dynamixel_AX.TorqueEnable(USART3, 4);
    Dynamixel_AX.SetAngle(USART3, 4, 151);
    Dynamixel_AX.TorqueEnable(USART3, 5);
    Dynamixel_AX.SetAngle(USART3, 5, 151);

    Dynamixel_AX.GetAngle(USART3, 1, &DYN_angle1);
    Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
    Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
    Dynamixel_AX.GetAngle(USART3, 5, &DYN_angle5);

    while(abs(40 - DYN_angle1) > PRES || abs(40 - DYN_angle3) > PRES || abs(151 - DYN_angle4) > PRES || abs(151 - DYN_angle5) > PRES){
        Dynamixel_AX.GetAngle(USART3, 1, &DYN_angle1);
        Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
        Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
        Dynamixel_AX.GetAngle(USART3, 5, &DYN_angle5);
    } 

    vTaskDelay(50);
    *flag.sort = 1; 

    vTaskSuspend(TaskHandle);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    *flag.sort = 0;

    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);


    *flag.ID_DC = 1;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_82;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50); 
    Dynamixel_AX.TorqueEnable(USART3, 4);
    Dynamixel_AX.SetAngle(USART3, 4, 135);

    Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);

    while(abs(135 - DYN_angle4) > 4){
        Dynamixel_AX.GetAngle(USART3, 4, &DYN_angle4);
    }

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    *flag.ID_DC = 0;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_110;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);


    Dynamixel_AX.TorqueEnable(USART3, 3);
    Dynamixel_AX.SetAngle(USART3, 3, 22);

    Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);

    while(abs(22 - DYN_angle3) > 4){
        Dynamixel_AX.GetAngle(USART3, 3, &DYN_angle3);
    }

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    *flag.ID_DC = 0;
    *flag.angle_DC = -ROTATION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = MEAN_POSITION_170;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    Dynamixel_AX.TorqueEnable(USART3, 1);
    Dynamixel_AX.SetAngle(USART3, 1, 22);

    Dynamixel_AX.GetAngle(USART3, 1, &DYN_angle2);

    while(abs(22 - DYN_angle2) > 4){
        Dynamixel_AX.GetAngle(USART3, 1, &DYN_angle2);
    }

    vTaskDelay(50);
    *flag.ID_DC = 2;
    *flag.angle_DC = UP_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);

    vTaskDelay(50);
    *flag.ID_DC = 0;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 1;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.ID_DC = 2;
    *flag.angle_DC = ZERO_POSITION;
    *flag.DC_state = 0;
    vTaskResume(DC_Task.GetTaskHandle());
    while(*flag.DC_state == 0);
    vTaskDelay(50);

    *flag.side = 1;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 2;
    vTaskResume(Release_Pile_Task.GetTaskHandle());
    *flag.side = 3;
    vTaskResume(Release_Pile_Task.GetTaskHandle());

    *flag.sort = 1;
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "sort_Task", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct Sort_task_s Sort_Task = {.CreateTask = CreateTask,
                        .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
