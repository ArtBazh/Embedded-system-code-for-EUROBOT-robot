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
#include "encoder_board_drvr.h"
#include "serial_protocol.h"
#include "byte2word.h"

#include "gpio_board_defines.h"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
int32_t POLULU_BIAS = 32000;
int32_t MAXON_BIAS = 2000032000;

static uint8_t GetEncoderCounts(USART_TypeDef* USART, uint8_t encoder_number, int32_t* encoder_counts)
{
    Serial.WriteBytes(USART,&encoder_number,1);

    uint8_t bytes[5];
    uint32_t number_of_bytes = Serial.ReadBytes(USART,bytes,10);

    if(number_of_bytes==0)
        return ENCODER_DRVR_TIMEOUT_ERROR;

    if(bytes[0]!=encoder_number)
        return ENCODER_DRVR_ENCODER_NUMBER_ERROR;
    uint32_t temp_encoder_counts;
    Byte_To_Word32(bytes+1,&temp_encoder_counts);
    *encoder_counts = (int32_t)(temp_encoder_counts - MAXON_BIAS) / 200;
    return ENCODER_DRVR_OK;
}

static uint8_t GetAllEncoderCounts(USART_TypeDef* USART, uint32_t* encoder_counts1, uint32_t* encoder_counts2)
{
    uint8_t code = 22;
    Serial.WriteBytes(USART, &code, 1);

    uint8_t bytes[8];
    uint32_t number_of_bytes = Serial.ReadBytes(USART,bytes,10);

    if(number_of_bytes==0)
        return ENCODER_DRVR_TIMEOUT_ERROR;

    if(bytes[0]!=code)
        return ENCODER_DRVR_ENCODER_NUMBER_ERROR;
    
    Byte_To_Word32(bytes,encoder_counts1);
    Byte_To_Word32(bytes+4,encoder_counts2);

    return ENCODER_DRVR_OK;
}


struct Encoder_Board_s Encoder_Board = {.GetEncoderCounts = GetEncoderCounts,
                                        .GetAllEncoderCounts = GetAllEncoderCounts};
                        

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
