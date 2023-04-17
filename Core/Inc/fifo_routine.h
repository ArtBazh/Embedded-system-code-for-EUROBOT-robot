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
#ifndef __FIFO_ROUTINE_H
#define __FIFO_ROUTINE_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"

struct FIFO_s{
  void* FIFO_ptr;
  void* head_ptr;
  void* tail_ptr;
  uint32_t FIFO_size;
  uint32_t FIFO_CellSize;
  uint32_t elements_count;
};


void FIFO_Init(struct FIFO_s* fifo, void* fifo_ptr, uint32_t fifo_size, uint32_t fifo_cell_size);
uint8_t FIFO_Add(struct FIFO_s* fifo, void* data_in_ptr);
uint8_t FIFO_Remove(struct FIFO_s* fifo, void* data_out_ptr);
uint32_t FIFO_GetElemtntsCount(struct FIFO_s* fifo);
uint8_t FIFO_IsFull(struct FIFO_s* fifo);
uint8_t FIFO_IsNotFull(struct FIFO_s* fifo);
uint8_t FIFO_IsEmpty(struct FIFO_s* fifo);
uint8_t FIFO_IsNotEmpty(struct FIFO_s* fifo);

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
