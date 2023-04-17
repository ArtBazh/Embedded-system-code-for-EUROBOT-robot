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

#include "experimental_task_2.h" 
#include "dynamixel_protocol10.h"
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "gpio_robot_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "stm32f4xx_ll_tim.h"

#include "spi_gpio_defines.h"
#include "spi_gpio.h" 

#include "terminal.h"
#include "terminal_cmd_table.h"
#include "ws2812b.h" 

#include "stm32f4xx_ll_rng.h"
#include "stm32f4xx_ll_bus.h"

#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;



static struct led_rgb_s ws_led[20] = {128};


static void Task(void *arg)
{
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Punch;
  terminal_response.data[0] = 0x05;
  terminal_response.data[1] = 0x15;
  terminal_response.data[2] = 0x25;
  terminal_response.data_length = 3;

  
  // ws_led[0].b = 0;
  // ws_led[0].r = 255;
  // ws_led[0].g = 0;

  // ws_led[1].b = 0;
  // ws_led[1].r = 255;
  // ws_led[1].g = 0;

  // ws_led[2].b = 0;
  // ws_led[2].r = 255;
  // ws_led[2].g = 0;

  for(uint8_t ik=0; ik<20; ik++){
    ws_led[ik].b = 255-ik*12;
    ws_led[ik].r = 0+ik*12;
    ws_led[ik].g = 0;
    // ws_led[ik].b = 0x55;
    // ws_led[ik].r = 0x55;
    // ws_led[ik].g = 0x55;
    // ws_led[ik].b = 0;
    // ws_led[ik].r = 0;
    // ws_led[ik].g = 255;
  }

  Set_RGBColor(ws_led,20);
  vTaskDelay(5);
  Set_RGBColor(ws_led,20);

  uint8_t r = 0;
  uint8_t b = 255;

  /* Enable RNG reset state */
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_RNG);

  /* Release RNG from reset state */
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_RNG);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);

  LL_RNG_Enable(RNG);

	while(1)
	{
    SPI_GPIO_SetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_10|SPI_POWER_OUTPUT_11|SPI_POWER_OUTPUT_20);
    vTaskDelay(500);
    SPI_GPIO_ResetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_10|SPI_POWER_OUTPUT_11|SPI_POWER_OUTPUT_20);
    vTaskDelay(500);
    // for(uint8_t ik=0; ik<100; ik++)
    //   Terminal.Send_Response(&terminal_response);

    if(Dynamixel_AX.Ping(USART3,0x02)==0)
      LL_GPIO_TogglePin(GPIOA, SERV7_GPIOA_PIN3);

    // Maxon.SetTPM(USART6,0x25,4096*100);

    uint32_t random = 0;
    // if(LL_RNG_IsActiveFlag_DRDY(RNG))
    //   random = LL_RNG_ReadRandData32(RNG);

    for(uint8_t ik=17; ik<20; ik++){
      // if(LL_RNG_IsActiveFlag_DRDY(RNG))
      //   random = LL_RNG_ReadRandData32(RNG);
      // Word32_To_Byte(&random,(uint8_t*)&ws_led[ik]);
      ws_led[ik] = ws_led[ik+1];
      // ws_led[ik].b = b;
      // ws_led[ik].r = r;
      // ws_led[ik].g = 0;
      // ws_led[ik].b = b;
      // ws_led[ik].r = r;
      // ws_led[ik].g = 0;
    }

    if(LL_RNG_IsActiveFlag_DRDY(RNG))
        random = LL_RNG_ReadRandData32(RNG);

    random = random>>8;
    Word32_To_Byte(&random,(uint8_t*)&ws_led[19]);

    // if(LL_RNG_IsActiveFlag_DRDY(RNG))
    //     random = LL_RNG_ReadRandData32(RNG);

    // ws_led[19].g = random>>24;

    // if(LL_RNG_IsActiveFlag_DRDY(RNG))
    //     random = LL_RNG_ReadRandData32(RNG);

    // ws_led[19].r = random>>24;

    // if(LL_RNG_IsActiveFlag_DRDY(RNG))
    //     random = LL_RNG_ReadRandData32(RNG);

    // ws_led[19].b = random>>24;

    // Word32_To_Byte(&random,(uint8_t*)&ws_led[19]);

    Set_RGBColor(ws_led,20);

    r = ~r;
    b = ~b;

    // vTaskDelay(250);

    // struct terminal_response_s terminal_response;

    // terminal_response.cmd = 0x03;
    // terminal_response.data_length = 0;

    // Terminal.Send_Response(&terminal_response);
    taskYIELD();
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "Experimental_Task_2", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct experimental_task_2_s Experimental_Task_2 = {.CreateTask = CreateTask,
                                                    .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
