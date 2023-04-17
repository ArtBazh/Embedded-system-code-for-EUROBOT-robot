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
#ifndef __HL_NEW_H
#define __HL_NEW_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "stm32f4xx_ll_usart.h"
#include "usartx_communication.h"


#define HL_NEW_MAX_DATA_LENGTH     32


struct HL_new_Request_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[HL_NEW_MAX_DATA_LENGTH];
};

struct HL_new_Response_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[HL_NEW_MAX_DATA_LENGTH];
};



struct HL_new_s{
    void (*Send_Response)(USART_TypeDef*, struct HL_new_Response_s*);
    uint8_t (*Get_Request)(USART_TypeDef*, struct HL_new_Request_s*);
    void (*Protocol)(struct LineFunctions_s*, struct LineInfo_s*, uint8_t, uint8_t*, uint32_t, uint8_t*, uint32_t);
};

extern struct HL_new_s HL_new;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
