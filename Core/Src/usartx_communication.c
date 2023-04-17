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
#include "usartx_communication.h"
#include "gpio_board_defines.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "usartx_timeout.h" 


#define USART1_USED
// #define USART2_USED
#define USART3_USED
#define UART4_USED
#define UART5_USED
#define USART6_USED


/*	USART1 TX/RX FIFO DEFINITION	*/
#if defined (USART1_USED)

  #define USART1_TX_BUFFER_SIZE     128
  #define USART1_RX_BUFFER_SIZE     128

  static uint8_t USART1_TX_BUFFER[USART1_TX_BUFFER_SIZE];
  static uint8_t USART1_RX_BUFFER[USART1_RX_BUFFER_SIZE];

#endif


/*	USART2 TX/RX FIFO DEFINITION	*/
#if defined (USART2_USED)

  #define USART2_TX_BUFFER_SIZE     1
  #define USART2_RX_BUFFER_SIZE     1

  static uint8_t USART2_TX_BUFFER[USART2_TX_BUFFER_SIZE];
  static uint8_t USART2_RX_BUFFER[USART2_RX_BUFFER_SIZE];

#endif


/*	USART3 TX/RX FIFO DEFINITION	*/
#if defined (USART3_USED)

  #define USART3_TX_BUFFER_SIZE     256
  #define USART3_RX_BUFFER_SIZE     128

  static uint8_t USART3_TX_BUFFER[USART3_TX_BUFFER_SIZE];
  static uint8_t USART3_RX_BUFFER[USART3_RX_BUFFER_SIZE];

#endif


/*	UART4 TX/RX FIFO DEFINITION	*/
#if defined (UART4_USED)

  #define UART4_TX_BUFFER_SIZE     512
  #define UART4_RX_BUFFER_SIZE     256

  static uint8_t UART4_TX_BUFFER[UART4_TX_BUFFER_SIZE];
  static uint8_t UART4_RX_BUFFER[UART4_RX_BUFFER_SIZE];

#endif


/*	UART5 TX/RX FIFO DEFINITION	*/
#if defined (UART5_USED)

  #define UART5_TX_BUFFER_SIZE     64
  #define UART5_RX_BUFFER_SIZE     64

  static uint8_t UART5_TX_BUFFER[UART5_TX_BUFFER_SIZE];
  static uint8_t UART5_RX_BUFFER[UART5_RX_BUFFER_SIZE];
#endif


/*	USART6 TX/RX FIFO DEFINITION	*/
#if defined (USART6_USED)

  #define USART6_TX_BUFFER_SIZE     512
  #define USART6_RX_BUFFER_SIZE     256

  static uint8_t USART6_TX_BUFFER[USART6_TX_BUFFER_SIZE];
  static uint8_t USART6_RX_BUFFER[USART6_RX_BUFFER_SIZE];

#endif




enum UsartLines{
  USART1_LINE,
  USART2_LINE,
  USART3_LINE,
  UART4_LINE,
  UART5_LINE,
  USART6_LINE
};




static struct LineInfo_s LINE_INFO[6];

static struct LineInfo_s* USARTx_GetLine(USART_TypeDef* USART)
{
  switch((uint32_t)USART)
	{
		case (uint32_t)USART1: return &LINE_INFO[USART1_LINE];
		case (uint32_t)USART2: return &LINE_INFO[USART2_LINE];
		case (uint32_t)USART3: return &LINE_INFO[USART3_LINE];
		case (uint32_t)UART4: return &LINE_INFO[UART4_LINE];
		case (uint32_t)UART5: return &LINE_INFO[UART5_LINE];
		case (uint32_t)USART6: return &LINE_INFO[USART6_LINE];
    default: return NULL;
	}
}

static void (*LL_DMA_ClearFlag_TCx[8])(DMA_TypeDef*) = {LL_DMA_ClearFlag_TC0,
                                                  LL_DMA_ClearFlag_TC1,
                                                  LL_DMA_ClearFlag_TC2,
                                                  LL_DMA_ClearFlag_TC3,
                                                  LL_DMA_ClearFlag_TC4,
                                                  LL_DMA_ClearFlag_TC5,
                                                  LL_DMA_ClearFlag_TC6,
                                                  LL_DMA_ClearFlag_TC7};

static void LL_DMA_ClearFlag_TC(DMA_TypeDef* DMA, uint32_t LL_DMA_STREAM)
{
  LL_DMA_ClearFlag_TCx[LL_DMA_STREAM](DMA);
}

static void StartDMA(DMA_TypeDef* DMA, uint32_t DMA_STREAM, uint8_t* data, uint32_t length)
{
  LL_DMA_ClearFlag_TC(DMA,DMA_STREAM);
  LL_DMA_SetMemoryAddress(DMA,DMA_STREAM,(uint32_t)data);
	LL_DMA_SetDataLength(DMA,DMA_STREAM,length);
  LL_DMA_EnableStream(DMA,DMA_STREAM);
}

static void StopDMA(DMA_TypeDef* DMA, uint32_t DMA_STREAM)
{
  LL_DMA_DisableStream(DMA,DMA_STREAM);
  LL_DMA_ClearFlag_TC(DMA,DMA_STREAM);
}

static void StartTX(struct LineInfo_s* line, uint32_t data_length)
{
  if(line->USART_mode==HALF_DUPLEX)
    LL_GPIO_SetOutputPin(line->DE_GPIO, line->DE_PIN);

  LL_USART_EnableDirectionTx(line->USART);

  LL_USART_ClearFlag_TC(line->USART);
  StartDMA(line->TX_DMA, line->TX_DMA_STREAM, line->TX_BUFFER, data_length);
  LL_USART_EnableIT_TC(line->USART);
}

static void StopTX(struct LineInfo_s* line)
{
  LL_USART_DisableDirectionTx(line->USART);

  if(line->USART_mode==HALF_DUPLEX)
    LL_GPIO_ResetOutputPin(line->DE_GPIO, line->DE_PIN);

  StopDMA(line->TX_DMA, line->TX_DMA_STREAM);
  LL_USART_DisableIT_TC(line->USART);
}

static void StartRX(struct LineInfo_s* line)
{
  if(line->USART_mode==HALF_DUPLEX)
    LL_GPIO_ResetOutputPin(line->DE_GPIO, line->DE_PIN);
  
  LL_USART_EnableDirectionRx(line->USART);

  LL_USART_ClearFlag_ORE(line->USART);
  LL_USART_ClearFlag_RXNE(line->USART);
  LL_USART_ClearFlag_IDLE(line->USART);
  
  StartDMA(line->RX_DMA, line->RX_DMA_STREAM,line->RX_BUFFER,line->RX_BUFFER_size);
  LL_USART_EnableIT_IDLE(line->USART);
}

static void StopRX(struct LineInfo_s* line)
{
  LL_USART_DisableDirectionRx(line->USART);

  StopDMA(line->RX_DMA, line->RX_DMA_STREAM);
  LL_USART_DisableIT_IDLE(line->USART);
}

static void Set_RX_Timeout(uint32_t delay_us, struct LineInfo_s* line)
{
  Set_Timeout(delay_us, line->lineID, line->RX_Timeout_Callback, line);
}

static void Reset_RX_Timeout(struct LineInfo_s* line)
{
  Reset_Timeout(line->lineID);
}



void USARTx_IRQ_Process(USART_TypeDef* USARTx)
{
  struct LineInfo_s* line = USARTx_GetLine(USARTx);

  /*	LL_USART_IsActiveFlag_TC	*/
	if(LL_USART_IsActiveFlag_TC(USARTx))
	{
		LL_USART_ClearFlag_TC(USARTx);

		if(LL_USART_IsEnabledIT_TC(USARTx)){
      line->TX_Complete_Callback(line);
    }
      
	}

	/*	LL_USART_IsActiveFlag_IDLE	*/
	if(LL_USART_IsActiveFlag_IDLE(USARTx)!=0)
	{
		LL_USART_ClearFlag_IDLE(USARTx);

		if(LL_USART_IsEnabledIT_IDLE(USARTx)){
      uint32_t data_length = line->RX_BUFFER_size - LL_DMA_GetDataLength(line->RX_DMA,line->RX_DMA_STREAM);
      line->RX_Complete_Callback(line,data_length);
		}
	}
}


static struct LineFunctions_s LineFunctions = {.USARTx_GetLine = USARTx_GetLine,
                                                .StartTX = StartTX,
                                                .StartRX = StartRX,
                                                .StopRX = StopRX,
                                                .StopTX = StopTX,
                                                .Set_RX_Timeout = Set_RX_Timeout,
                                                .Reset_RX_Timeout = Reset_RX_Timeout};


void USARTx_AttachProtocol(USART_TypeDef* USART,void (*Protocol)(struct LineFunctions_s*,struct LineInfo_s*,uint8_t,uint8_t*,uint32_t,uint8_t*,uint32_t),uint8_t role)
{
  struct LineInfo_s* line = USARTx_GetLine(USART);

  switch ((uint32_t)USART)
  {
    #if defined (USART1_USED)
      case (uint32_t)USART1:
        line->USART = USART1;
        line->TX_DMA = DMA2;
        line->TX_DMA_STREAM = LL_DMA_STREAM_7;
        line->RX_DMA = DMA2;
        line->RX_DMA_STREAM = LL_DMA_STREAM_5;
        line->lineID = USART1_LINE;
        Protocol(&LineFunctions,line,role,USART1_TX_BUFFER,USART1_TX_BUFFER_SIZE,USART1_RX_BUFFER,USART1_RX_BUFFER_SIZE);
      break;
    #endif

    #if defined (USART2_USED)

    #endif

    #if defined (USART3_USED)
      case (uint32_t)USART3:
        line->USART = USART3;
        line->TX_DMA = DMA1;
        line->TX_DMA_STREAM = LL_DMA_STREAM_3;
        line->RX_DMA = DMA1;
        line->RX_DMA_STREAM = LL_DMA_STREAM_1;
        line->DE_GPIO = GPIOD;
        line->DE_PIN = UART3DE_GPIOD_PIN9;
        line->lineID = USART3_LINE;
        Protocol(&LineFunctions,line,role,USART3_TX_BUFFER,USART3_TX_BUFFER_SIZE,USART3_RX_BUFFER,USART3_RX_BUFFER_SIZE);
        break;
    #endif

    #if defined (UART4_USED)
      case (uint32_t)UART4:
        line->USART = UART4;
        line->TX_DMA = DMA1;
        line->TX_DMA_STREAM = LL_DMA_STREAM_4;
        line->RX_DMA = DMA1;
        line->RX_DMA_STREAM = LL_DMA_STREAM_2;
        line->DE_GPIO = GPIOA;
        line->DE_PIN = UART4DE_GPIOA_PIN15;
        line->lineID = UART4_LINE;
        Protocol(&LineFunctions,line,role,UART4_TX_BUFFER,UART4_TX_BUFFER_SIZE,UART4_RX_BUFFER,UART4_RX_BUFFER_SIZE);
        break;
    #endif

    #if defined (UART5_USED)
      case (uint32_t)UART5:
        line->USART = UART5;
        line->TX_DMA = DMA1;
        line->TX_DMA_STREAM = LL_DMA_STREAM_7;
        line->RX_DMA = DMA1;
        line->RX_DMA_STREAM = LL_DMA_STREAM_0;
        line->lineID = UART5_LINE;
        Protocol(&LineFunctions,line,role,UART5_TX_BUFFER,UART5_TX_BUFFER_SIZE,UART5_RX_BUFFER,UART5_RX_BUFFER_SIZE);
        break;
    #endif

    #if defined (USART6_USED)
      case (uint32_t)USART6:
        line->USART = USART6;
        line->TX_DMA = DMA2;
        line->TX_DMA_STREAM = LL_DMA_STREAM_6;
        line->RX_DMA = DMA2;
        line->RX_DMA_STREAM = LL_DMA_STREAM_2;
        line->DE_GPIO = GPIOC;
        line->DE_PIN = UART6DE_GPIOC_PIN7;
        line->lineID = USART6_LINE;
        Protocol(&LineFunctions,line,role,USART6_TX_BUFFER,USART6_TX_BUFFER_SIZE,USART6_RX_BUFFER,USART6_RX_BUFFER_SIZE);
        break;
    #endif

    default:

      break;
  }

  
}
 



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
