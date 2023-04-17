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
#ifndef __USARTX_COMMUNICATION_H
#define __USARTX_COMMUNICATION_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "fifo_routine.h" 

enum LineState_e{
    LINE_FREE,
    LINE_BUSY
};

enum Protocol_Role_e{
  MASTER,
  SLAVE,
  MASTER_SLAVE
};

enum USART_Mode_e{
  HALF_DUPLEX,
  FULL_DUPLEX
};

enum Line_Access_e{
  ACCESS_UNLOCK,
  ACCESS_LOCK
};

enum Bus_Error_e{
  NO_BUS_ERROR,
  BUS_ERROR
};



struct LineInfo_s{
  USART_TypeDef* USART;
  DMA_TypeDef* TX_DMA;
  uint32_t TX_DMA_STREAM;
	DMA_TypeDef* RX_DMA;
	uint32_t RX_DMA_STREAM;
  uint8_t* TX_BUFFER;
  uint32_t TX_BUFFER_size;
  uint8_t* RX_BUFFER;
  uint32_t RX_BUFFER_size;
  struct FIFO_s TX_FIFO;
  struct FIFO_s RX_FIFO;
	GPIO_TypeDef* DE_GPIO;
	uint32_t DE_PIN;
  uint32_t lineState;
  uint32_t USART_mode;
  uint32_t lineID;
  void (*RX_Timeout_Callback)(void*);
  void (*TX_Complete_Callback)(struct LineInfo_s*);
  void (*RX_Complete_Callback)(struct LineInfo_s*, uint32_t);
};

struct LineFunctions_s{
  struct LineInfo_s* (*USARTx_GetLine)(USART_TypeDef*);
  void (*StartTX)(struct LineInfo_s*, uint32_t);
  void (*StartRX)(struct LineInfo_s*);
  void (*StopRX)(struct LineInfo_s*);
  void (*StopTX)(struct LineInfo_s*);
  void (*Set_RX_Timeout)(uint32_t, struct LineInfo_s*);
  void (*Reset_RX_Timeout)(struct LineInfo_s*);
};

void USARTx_AttachProtocol(USART_TypeDef* USART,void (*Protocol)(struct LineFunctions_s*,struct LineInfo_s*,uint8_t,uint8_t*,uint32_t,uint8_t*,uint32_t),uint8_t role);
void USARTx_IRQ_Process(USART_TypeDef* USARTx);


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
