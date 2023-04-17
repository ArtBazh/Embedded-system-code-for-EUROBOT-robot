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
#include "fifo_routine.h" 


void FIFO_Init(struct FIFO_s* fifo, void* fifo_ptr, uint32_t fifo_size, uint32_t fifo_cell_size)
{
  fifo->FIFO_ptr = fifo_ptr;
  fifo->head_ptr = fifo_ptr;
  fifo->tail_ptr = fifo_ptr;
  fifo->FIFO_size = fifo_size;
  fifo->FIFO_CellSize = fifo_cell_size;
  fifo->elements_count = 0;
}

void FIFO_MovePtr(void* ptr_to_ptr, void* fifo_ptr, uint32_t fifo_size, uint32_t fifo_cell_size)
{
	if(*((uint32_t*)ptr_to_ptr)<((uint32_t)fifo_ptr+(fifo_size-1)*fifo_cell_size))
		*((uint32_t*)ptr_to_ptr) += fifo_cell_size;
	else
		*((uint32_t*)ptr_to_ptr) = (uint32_t)fifo_ptr;
}

uint8_t FIFO_Add(struct FIFO_s* fifo, void* data_in_ptr)
{
  if((fifo->elements_count+1)==fifo->FIFO_size)
    return 1;

  memcpy(fifo->tail_ptr, data_in_ptr, fifo->FIFO_CellSize);
  FIFO_MovePtr(&(fifo->tail_ptr),fifo->FIFO_ptr,fifo->FIFO_size,fifo->FIFO_CellSize);
  fifo->elements_count++;
  return 0;
}

uint8_t FIFO_Remove(struct FIFO_s* fifo, void* data_out_ptr)
{
  if(fifo->elements_count==0)
    return 1;

  memcpy(data_out_ptr, fifo->head_ptr, fifo->FIFO_CellSize);
  FIFO_MovePtr(&(fifo->head_ptr),fifo->FIFO_ptr,fifo->FIFO_size,fifo->FIFO_CellSize);
  fifo->elements_count--;
  return 0;
}

uint32_t FIFO_GetElemtntsCount(struct FIFO_s* fifo)
{
  return fifo->elements_count;
}

uint8_t FIFO_IsFull(struct FIFO_s* fifo)
{
  return ((fifo->elements_count+1)==fifo->FIFO_size) ? 1 : 0;
}

uint8_t FIFO_IsNotFull(struct FIFO_s* fifo)
{
  return ((fifo->elements_count+1)==fifo->FIFO_size) ? 0 : 1;
}

uint8_t FIFO_IsEmpty(struct FIFO_s* fifo)
{
  return (fifo->elements_count==0) ? 1 : 0;
}

uint8_t FIFO_IsNotEmpty(struct FIFO_s* fifo)
{
  return (fifo->elements_count==0) ? 0 : 1;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
