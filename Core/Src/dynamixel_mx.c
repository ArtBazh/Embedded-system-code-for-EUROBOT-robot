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

#include "dynamixel_mx.h" 
#include "dynamixel_protocol10.h"
#include "byte2word.h"

#include "gpio_board_defines.h"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

static uint8_t Ping(USART_TypeDef* USART, uint8_t ID)
{
  struct Dynamixel_Protocol10_Response_s response;
  Dynamixel_Protocol10.Ping(USART,ID,&response);
  while(response.data_ready_flag==0) taskYIELD();
  return response.bus_error;
}

static void SetID(USART_TypeDef* USART, uint8_t ID, uint8_t newID)
{
  Dynamixel_Protocol10.Write(USART,ID,MX_ID,&newID,1,NULL); 
}

static void TorqueEnable(USART_TypeDef* USART, uint8_t ID)
{
  uint8_t data = 1;
  Dynamixel_Protocol10.Write(USART,ID,MX_TorqueEnable,&data,1,NULL); 
}

static void TorqueDisable(USART_TypeDef* USART, uint8_t ID)
{
  uint8_t data = 0;
  Dynamixel_Protocol10.Write(USART,ID,MX_TorqueEnable,&data,1,NULL); 
}

static void SetAngle(USART_TypeDef* USART, uint8_t ID, uint16_t angle)
{
  // uint8_t data[2];
  uint16_t temp = ((uint32_t)angle*4095)/360;
  // Word16_To_Byte(&temp,data);
  Dynamixel_Protocol10.Write(USART,ID,MX_GoalPosition,(uint8_t*)&temp,2,NULL); 
}

static uint8_t GetAngle(USART_TypeDef* USART, uint8_t ID, uint16_t* angle)
{
  struct Dynamixel_Protocol10_Response_s response;
  uint32_t temp;
  
  Dynamixel_Protocol10.Read(USART,ID,MX_PresentPosition,2,&response);
  while(response.data_ready_flag==0) taskYIELD();
    
  Byte_To_Word16(response.data,&temp);
  *angle = (temp*360)/4095;

  return response.bus_error;
}

static void SetWheelMode(USART_TypeDef* USART, uint8_t ID)
{
  // uint8_t data[4] = {0x00,0x00,0x00,0x00};
  // Dynamixel_Protocol10.Write(USART,ID,MX_CW_AngleLimit,data,4,NULL);
}

static void SetJointMode(USART_TypeDef* USART, uint8_t ID)
{
  // uint8_t data[4] = {0x00,0x00,0xFF,0x03};
  // Dynamixel_Protocol10.Write(USART,ID,MX_CW_AngleLimit,data,4,NULL); 
}

static void SetMovingSpeed(USART_TypeDef* USART, uint8_t ID, int16_t speed)
{
  // uint8_t data[2];

  // if(speed<0)
  //   speed = 1024 - speed;

  // Word16_To_Byte(&speed,data);
  // Dynamixel_Protocol10.Write(USART,ID,MX_MovingSpeed,data,2,NULL); 
}

static uint8_t GetPresentLoad(USART_TypeDef* USART, uint8_t ID, int16_t* load)
{
  // struct Dynamixel_Protocol10_Response_s response;
  // int16_t temp;
  
  // Dynamixel_Protocol10.Read(USART,ID,MX_PresentLoad,2,&response);
  // while(response.data_ready_flag==0);
    
	
  // Byte_To_Word16(response.data,&temp);

  // if(temp>1023)
  //   temp = 1024 - temp; 

  // *load = temp;

  // return response.bus_error;
  return 0;
}

static void EnableLED(USART_TypeDef* USART, uint8_t ID)
{
  uint8_t data = 1;
  Dynamixel_Protocol10.Write(USART,ID,MX_LED,&data,1,NULL); 
}

static void DisableLED(USART_TypeDef* USART, uint8_t ID)
{
  uint8_t data = 0;
  Dynamixel_Protocol10.Write(USART,ID,MX_LED,&data,1,NULL);
}

struct Dynamixel_MX_s Dynamixel_MX = {.Ping = Ping,
                                      .SetID = SetID,
                                      .TorqueEnable = TorqueEnable,
                                      .TorqueDisable = TorqueDisable,
                                      .SetAngle = SetAngle,
                                      .GetAngle = GetAngle,
                                      .SetWheelMode = SetWheelMode,
                                      .SetJointMode = SetJointMode,
                                      .SetMovingSpeed = SetMovingSpeed,
                                      .GetPresentLoad = GetPresentLoad,
                                      .EnableLED = EnableLED,
                                      .DisableLED = DisableLED};



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
