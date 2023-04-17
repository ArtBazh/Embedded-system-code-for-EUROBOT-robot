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
#ifndef __BYTE_2_WORD_H
#define __BYTE_2_WORD_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
// #include "stdint.h"
#include "string.h" 


static inline void Byte_To_Word32(uint8_t* byte, void* var)
{
  memcpy(var,byte,4);
}

static inline void Word32_To_Byte(void* var, uint8_t* byte)
{
  memcpy(byte,var,4);
}

static inline void Byte_To_Word16(uint8_t* byte, void* var)
{
  memcpy(var,byte,2);
}

static inline void Word16_To_Byte(void* var, uint8_t* byte)
{
  memcpy(byte,var,2);
}

static inline void Byte_To_Word8(uint8_t* byte, void* var)
{
  memcpy(var,byte,1);
}

static inline void Word8_To_Byte(void* var, uint8_t* byte)
{
  memcpy(byte,var,1);
}


static inline void Byte_To_Word32_s(int8_t* byte, void* var)
{
  memcpy(var,byte,4);
}

static inline void Word32_To_Byte_s(void* var, int8_t* byte)
{
  memcpy(byte,var,4);
}

static inline void Byte_To_Word16_s(int8_t* byte, void* var)
{
  memcpy(var,byte,2);
}

static inline void Word16_To_Byte_s(void* var, int8_t* byte)
{
  memcpy(byte,var,2);
}

static inline void Byte_To_Word8_s(int8_t* byte, void* var)
{
  memcpy(var,byte,1);
}

static inline void Word8_To_Byte_s(void* var, int8_t* byte)
{
  memcpy(byte,var,1);
}
#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
