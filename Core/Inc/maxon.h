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
#ifndef __MAXON_H
#define __MAXON_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "stm32f4xx_ll_usart.h"

	 
/*	MAXON FLASH MEMORY Area registers define ------------------------------------------------------------ */
#define MAXON_ID              		0x01    /* Mark Position (3 bytes) 				*/  
#define MAXON_RangeTPM            0x04

/*	MAXON SRAM MEMORY Area registers define ------------------------------------------------------------ */
#define MAXON_TPM                 0x10  
#define MAXON_TotalTicks          0x14
#define MAXON_Enable              0x18



struct Maxon_s{
    uint8_t (*Ping)(USART_TypeDef* USART, uint8_t ID);
    void (*SetID)(USART_TypeDef* USART, uint8_t packetID, uint8_t ID);
    void (*SetRangeTPM)(USART_TypeDef* USART, uint8_t ID, uint32_t TPM_bottom, uint32_t TPM_top);
    uint8_t (*GetRangeTPM)(USART_TypeDef* USART, uint8_t ID, uint32_t* TPM_bottom, uint32_t* TPM_top);
    void (*SetTPM)(USART_TypeDef* USART, uint8_t ID, int32_t TPM);
    uint8_t (*GetTPM)(USART_TypeDef* USART, uint8_t ID, int32_t* TPM);
    void (*SetTotalTicks)(USART_TypeDef* USART, uint8_t ID, int32_t ticks);
    uint8_t (*GetTotalTicks)(USART_TypeDef* USART, uint8_t ID, int32_t* ticks);
    void (*Enable)(USART_TypeDef* USART, uint8_t ID);
    void (*Disable)(USART_TypeDef* USART, uint8_t ID);
};

extern struct Maxon_s Maxon;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
