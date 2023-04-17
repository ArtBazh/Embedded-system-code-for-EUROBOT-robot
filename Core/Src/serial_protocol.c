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
#include "serial_protocol.h"
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




struct TX_FIFO_element_s{
    uint8_t bytes[SERIAL_MAX_BYTES_NUMBER];
    uint8_t bytes_number;
};

struct RX_FIFO_element_s{
    uint8_t bytes[SERIAL_MAX_BYTES_NUMBER];
    uint8_t bytes_number;
};

static struct LineFunctions_s* LineFunctions;

static uint8_t timeout_expired_flag;


static void Transmit_Bytes(struct LineInfo_s*  line, struct TX_FIFO_element_s* TX_FIFO_element);


void WriteBytes(USART_TypeDef* USART, uint8_t* bytes, uint8_t bytes_number)
{
    if(bytes_number>SERIAL_MAX_BYTES_NUMBER)
        return;

    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

    struct TX_FIFO_element_s TX_FIFO_element;

    memcpy(TX_FIFO_element.bytes, bytes, bytes_number);
    TX_FIFO_element.bytes_number = bytes_number;

    Transmit_Bytes(line,&TX_FIFO_element);
}

uint32_t ReadBytes(USART_TypeDef* USART, uint8_t* bytes, uint32_t timeout_ms)
{
    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

    if(timeout_ms){
        timeout_expired_flag = 0;
        LineFunctions->Set_RX_Timeout(timeout_ms*1000,line);
    }
    
    while(1)
    {
        if(FIFO_IsNotEmpty(&(line->RX_FIFO))){
            struct RX_FIFO_element_s RX_FIFO_element;
            FIFO_Remove(&(line->RX_FIFO),&RX_FIFO_element);
            if(timeout_ms)
                LineFunctions->Reset_RX_Timeout(line);
            memcpy(bytes, RX_FIFO_element.bytes, RX_FIFO_element.bytes_number);
            return RX_FIFO_element.bytes_number;
        }

        if((timeout_ms&&timeout_expired_flag)||(timeout_ms==0))
            return 0;

        taskYIELD();
    }
}

static void Transmit_Next_Queue_Element(struct LineInfo_s* line)
{
    FIFO_Remove(&(line->TX_FIFO),line->TX_BUFFER);
    LineFunctions->StartTX(line,((struct TX_FIFO_element_s*)(line->TX_BUFFER))->bytes_number);
}

static void Transmit_Bytes(struct LineInfo_s*  line, struct TX_FIFO_element_s* TX_FIFO_element)
{
    while(1)
    {
        taskENTER_CRITICAL();

        if(FIFO_IsNotFull(&(line->TX_FIFO)))
            break;

        taskEXIT_CRITICAL();
        taskYIELD();
    }
    
    FIFO_Add(&(line->TX_FIFO),TX_FIFO_element);

    if(line->lineState==LINE_FREE){
        line->lineState = LINE_BUSY;
        Transmit_Next_Queue_Element(line);
    }

    taskEXIT_CRITICAL();
}

static void USART_RX_Timeout_Callback(void* line)
{
    timeout_expired_flag = 1;
}

static void USART_TX_Complete_Callabck_MasterSlave(struct LineInfo_s* line)
{
   LineFunctions->StopTX(line);
    
    if(FIFO_IsNotEmpty(&(line->TX_FIFO)))
        Transmit_Next_Queue_Element(line);
    else
        line->lineState = LINE_FREE;
}

static void USART_RX_Complete_Callabck_MasterSlave(struct LineInfo_s* line, uint32_t data_length)
{
    LineFunctions->StopRX(line);

    if(FIFO_IsNotFull(&(line->RX_FIFO))){
        ((struct RX_FIFO_element_s*)(line->RX_BUFFER))->bytes_number = data_length;
        FIFO_Add(&(line->RX_FIFO),line->RX_BUFFER);
    }
	    
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

    line->TX_BUFFER_size = SERIAL_MAX_BYTES_NUMBER;
    line->RX_BUFFER_size = SERIAL_MAX_BYTES_NUMBER;

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
            
            break;

        case SLAVE:
            
            break;

        case MASTER_SLAVE:
            line->TX_Complete_Callback = USART_TX_Complete_Callabck_MasterSlave;
            line->RX_Complete_Callback = USART_RX_Complete_Callabck_MasterSlave;

        default:
            
            break;
    }

		LineFunctions->StartRX(line);
}

struct Serial_Protocol_s Serial = {.WriteBytes = WriteBytes,
                                    .ReadBytes = ReadBytes,
                                    .Protocol = Protocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
