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
#include "hl_old.h"
#include "fifo_routine.h" 

#include "gpio_board_defines.h"
#include "stm32f4xx_ll_gpio.h"


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "usartx_timeout.h" 






struct packet_s{
    uint8_t cmd;
    uint8_t data[MAX_DATA_LENGTH];
};

struct TX_FIFO_element_s{
    struct packet_s packet;
};

struct RX_FIFO_element_s{
    struct packet_s packet;
    uint8_t packet_length;
};

static struct LineFunctions_s* LineFunctions;


static void Send_Response(USART_TypeDef* USART, uint8_t* data, uint8_t length)
{

}

static uint8_t Get_Request(USART_TypeDef* USART, struct HL_old_Request_s* request)
{
    struct LineInfo_s* line = LineFunctions->USARTx_GetLine(USART);

    if(FIFO_IsNotEmpty(&(line->RX_FIFO))){
        struct RX_FIFO_element_s RX_FIFO_element;
        FIFO_Remove(&(line->RX_FIFO),&RX_FIFO_element);
        request->cmd = RX_FIFO_element.packet.cmd;
        request->data_length = RX_FIFO_element.packet_length-1;
        memcpy(request->data,RX_FIFO_element.packet.data,RX_FIFO_element.packet_length-1);
        return 1;
    }

    return 0;
}


static void USART_RX_Timeout_Callback(void* line)
{
    
}

static void USART_TX_Complete_Callabck_Master(struct LineInfo_s* line)
{
   
}

static void USART_RX_Complete_Callabck_Master(struct LineInfo_s* line, uint32_t data_length)
{
    
}

static void USART_TX_Complete_Callabck_Slave(struct LineInfo_s* line)
{
	// LineFunctions->StopTX(line);
}

static void USART_RX_Complete_Callabck_Slave(struct LineInfo_s* line, uint32_t data_length)
{
    LineFunctions->StopRX(line);

    struct RX_FIFO_element_s* RX_FIFO_element;
    
    RX_FIFO_element = (struct RX_FIFO_element_s*)(line->RX_BUFFER);
    RX_FIFO_element->packet_length = data_length;

    if(FIFO_IsNotFull(&(line->RX_FIFO)))
	    FIFO_Add(&(line->RX_FIFO),RX_FIFO_element);

    LineFunctions->StartRX(line);
}



static void Protocol(struct LineFunctions_s* line_functions,struct LineInfo_s* line,uint8_t role, uint8_t* tx_line_buffer, uint32_t tx_line_buffer_size, uint8_t* rx_line_buffer, uint32_t rx_line_buffer_size)
{
    LineFunctions = line_functions;
	
    if((tx_line_buffer_size<2*sizeof(struct TX_FIFO_element_s))||
        (rx_line_buffer_size<2*sizeof(struct RX_FIFO_element_s)))
        return;

    line->TX_BUFFER = tx_line_buffer;
    line->RX_BUFFER = rx_line_buffer;

    line->TX_BUFFER_size = sizeof(struct packet_s);
    line->RX_BUFFER_size = sizeof(struct packet_s);

    uint8_t* TX_FIFO = (uint8_t*)((uint32_t)tx_line_buffer + sizeof(struct TX_FIFO_element_s));
    uint8_t* RX_FIFO = (uint8_t*)((uint32_t)rx_line_buffer + sizeof(struct RX_FIFO_element_s));

    uint32_t TX_FIFO_size = tx_line_buffer_size/sizeof(struct TX_FIFO_element_s) - 1;
    uint32_t RX_FIFO_size = rx_line_buffer_size/sizeof(struct RX_FIFO_element_s) - 1;

    FIFO_Init(&(line->TX_FIFO),TX_FIFO,TX_FIFO_size,sizeof(struct TX_FIFO_element_s));
    FIFO_Init(&(line->RX_FIFO),RX_FIFO,RX_FIFO_size,sizeof(struct RX_FIFO_element_s));

    line->lineState = LINE_FREE;
    line->USART_mode = FULL_DUPLEX;

    line->RX_Timeout_Callback = USART_RX_Timeout_Callback;

    switch (role)
    {
        case MASTER:
            line->TX_Complete_Callback = USART_TX_Complete_Callabck_Master;
            line->RX_Complete_Callback = USART_RX_Complete_Callabck_Master;
            break;

        case SLAVE:
            line->TX_Complete_Callback = USART_TX_Complete_Callabck_Slave;
            line->RX_Complete_Callback = USART_RX_Complete_Callabck_Slave;
            LineFunctions->StartRX(line);
            break;

        default:
            
            break;
    }

}

struct HL_old_s HL_old = {.Send_Response = Send_Response,
                            .Get_Request = Get_Request,
                            .Protocol = Protocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
