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
#include "maxon.h"
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
    
static void SetID(USART_TypeDef* USART, uint8_t packetID, uint8_t ID)
{
  Dynamixel_Protocol10.Write(USART,packetID,MAXON_ID,&ID,1,NULL); 
}
    
static void SetRangeTPM(USART_TypeDef* USART, uint8_t ID, uint32_t TPM_bottom, uint32_t TPM_top)
{
  uint8_t data[8];
  Word32_To_Byte(&TPM_bottom,data);
  Word32_To_Byte(&TPM_top,data+4);
  Dynamixel_Protocol10.Write(USART,ID,MAXON_RangeTPM,data,8,NULL);
}

static uint8_t GetRangeTPM(USART_TypeDef* USART, uint8_t ID, uint32_t* TPM_bottom, uint32_t* TPM_top)
{
  struct Dynamixel_Protocol10_Response_s response;
  
  Dynamixel_Protocol10.Read(USART,ID,MAXON_RangeTPM,8,&response);

  while(response.data_ready_flag==0) taskYIELD();
  Byte_To_Word32(response.data,TPM_bottom);
  Byte_To_Word32(response.data+4,TPM_top);

  return response.bus_error;
}

static void SetTPM(USART_TypeDef* USART, uint8_t ID, int32_t TPM)
{
  uint8_t data[4];
  Word32_To_Byte(&TPM,data);
  Dynamixel_Protocol10.Write(USART,ID,MAXON_TPM,data,4,NULL); 
}

static uint8_t GetTPM(USART_TypeDef* USART, uint8_t ID, int32_t* TPM)
{
  struct Dynamixel_Protocol10_Response_s response;
  
  Dynamixel_Protocol10.Read(USART,ID,MAXON_TPM,4,&response);
  while(response.data_ready_flag==0) taskYIELD();
  Byte_To_Word32(response.data,TPM);

  return response.bus_error;
}

static void SetTotalTicks(USART_TypeDef* USART, uint8_t ID, int32_t ticks)
{
  uint8_t data[4];
  Word32_To_Byte(&ticks,data);
  Dynamixel_Protocol10.Write(USART,ID,MAXON_TotalTicks,data,4,NULL); 
}

static uint8_t GetTotalTicks(USART_TypeDef* USART, uint8_t ID, int32_t* ticks)
{
  struct Dynamixel_Protocol10_Response_s response;
  
  Dynamixel_Protocol10.Read(USART,ID,MAXON_TotalTicks,4,&response);
  while(response.data_ready_flag==0) taskYIELD();
  Byte_To_Word32(response.data,ticks);

  return response.bus_error;
}

static void Enable(USART_TypeDef* USART, uint8_t ID)
{
  uint8_t data = 1;
  Dynamixel_Protocol10.Write(USART,ID,MAXON_Enable,&data,1,NULL);
}

static void Disable(USART_TypeDef* USART, uint8_t ID)
{
  uint8_t data = 0;
  Dynamixel_Protocol10.Write(USART,ID,MAXON_Enable,&data,1,NULL);
}

struct Maxon_s Maxon = {.Ping = Ping,
                        .SetID = SetID,
                        .SetRangeTPM = SetRangeTPM,
                        .GetRangeTPM = GetRangeTPM,
                        .SetTPM = SetTPM,
                        .GetTPM = GetTPM,
                        .SetTotalTicks = SetTotalTicks,
                        .GetTotalTicks = GetTotalTicks,
                        .Enable = Enable,
                        .Disable = Disable};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
