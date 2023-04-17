/**
  ******************************************************************************
  * @file    	functions.h
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_BOARD_DRVR_H
#define __ENCODER_BOARD_DRVR_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "stm32f4xx_ll_usart.h"

#define ENCODER_DRVR_OK                     0
#define ENCODER_DRVR_TIMEOUT_ERROR          1
#define ENCODER_DRVR_ENCODER_NUMBER_ERROR   2 
	 
struct Encoder_Board_s{
    uint8_t (*GetEncoderCounts)(USART_TypeDef* USART, uint8_t encoder_number, int32_t* encoder_counts);
    uint8_t (*GetAllEncoderCounts)(USART_TypeDef* USART, uint32_t* encoder_counts1, uint32_t* encoder_counts2);
};

extern struct Encoder_Board_s Encoder_Board;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
