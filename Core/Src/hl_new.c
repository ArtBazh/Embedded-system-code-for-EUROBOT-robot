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
#include "hl_new.h"
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





struct packet_data_s{
    uint8_t packet_length;
    uint8_t cmd;
    uint8_t data_crc[HL_NEW_MAX_DATA_LENGTH];
};

struct packet_s{
    uint8_t H1;
    uint8_t H2;
    struct packet_data_s packet_data;
};

struct TX_FIFO_element_s{
    struct packet_data_s packet_data;
};

struct RX_FIFO_element_s{
    struct packet_data_s packet_data;
};



static struct LineFunctions_s* LineFunctions;

static uint8_t CRC_Calc(struct packet_data_s* packet_data)
{
    uint16_t crc = packet_data->packet_length + packet_data->cmd;
	
	for(uint8_t ik=0; ik<(packet_data->packet_length-2); ik++)
		crc += packet_data->data_crc[ik];
	
	crc = ~crc;
	crc &= 0xFF;

    return (uint8_t)crc;
}

static void Transmit_Next(struct LineInfo_s* line)
{
    struct packet_s* packet = (struct packet_s*)line->TX_BUFFER;

    packet->H1 = 0xFF;
    packet->H2 = 0xFF;
    FIFO_Remove(&(line->TX_FIFO),&(packet->packet_data));

    LineFunctions->StartTX(line,packet->packet_data.packet_length+3);
}

static void Transmit_Cmd(USART_TypeDef* USART, struct TX_FIFO_element_s* TX_FIFO_element)
{
    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

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
        Transmit_Next(line);
    }

    taskEXIT_CRITICAL();
}

static void Send_Response(USART_TypeDef* USART, struct HL_new_Response_s* response)
{
    struct TX_FIFO_element_s TX_FIFO_element;

    TX_FIFO_element.packet_data.cmd = response->cmd;
    TX_FIFO_element.packet_data.packet_length = response->data_length+2;
    memcpy(TX_FIFO_element.packet_data.data_crc,response->data,response->data_length);
    TX_FIFO_element.packet_data.data_crc[response->data_length] = CRC_Calc(&(TX_FIFO_element.packet_data));

    Transmit_Cmd(USART,&TX_FIFO_element);
}

static uint8_t Get_Request(USART_TypeDef* USART, struct HL_new_Request_s* request)
{
    struct LineInfo_s* line = LineFunctions->USARTx_GetLine(USART);

    if(FIFO_IsNotEmpty(&(line->RX_FIFO))){
        struct RX_FIFO_element_s RX_FIFO_element;
        FIFO_Remove(&(line->RX_FIFO),&RX_FIFO_element);
        request->cmd = RX_FIFO_element.packet_data.cmd;
        request->data_length = RX_FIFO_element.packet_data.packet_length-2;
        memcpy(request->data,RX_FIFO_element.packet_data.data_crc,RX_FIFO_element.packet_data.packet_length-2);
        return 1;
    }

    return 0;
}

static uint8_t PacketErrorCheck(struct packet_s* packet, uint8_t packet_length)
{
    if((packet->H1 != 0xFF)||(packet->H2 != 0xFF)){
        // LL_GPIO_TogglePin(GPIOA, SERV6_GPIOA_PIN2);
		return 1;
    }

    if(packet_length<5){
        
        return 1;
    }

	if(packet_length != (packet->packet_data.packet_length + 3)){
        
		return 1;
    }

    uint8_t packet_crc = packet->packet_data.data_crc[packet->packet_data.packet_length - 2];
    uint8_t calc_crc = CRC_Calc(&(packet->packet_data));

    if(packet_crc != calc_crc){
        
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
	LineFunctions->StopTX(line);
    
    if(FIFO_IsNotEmpty(&(line->TX_FIFO)))
        Transmit_Next(line);
    else
        line->lineState = LINE_FREE;
}

static void USART_RX_Complete_Callabck_Slave(struct LineInfo_s* line, uint32_t data_length)
{
    LineFunctions->StopRX(line);

    struct packet_s* packet = (struct packet_s*)line->RX_BUFFER;

    if(PacketErrorCheck(packet,data_length)==0)
    {
        if(FIFO_IsNotFull(&(line->RX_FIFO)))
	        FIFO_Add(&(line->RX_FIFO),&(packet->packet_data));
    }
    
    memset(line->RX_BUFFER,0x55,sizeof(struct packet_s));

    LineFunctions->StartRX(line);
}



static void Protocol(struct LineFunctions_s* line_functions,struct LineInfo_s* line,uint8_t role, uint8_t* tx_line_buffer, uint32_t tx_line_buffer_size, uint8_t* rx_line_buffer, uint32_t rx_line_buffer_size)
{
    LineFunctions = line_functions;
	
    if((tx_line_buffer_size<(sizeof(struct TX_FIFO_element_s)+sizeof(struct packet_s)))||
        (rx_line_buffer_size<(sizeof(struct RX_FIFO_element_s)+sizeof(struct packet_s))))
        return;

    line->TX_BUFFER = tx_line_buffer;
    line->RX_BUFFER = rx_line_buffer;

    line->TX_BUFFER_size = sizeof(struct packet_s);
    line->RX_BUFFER_size = sizeof(struct packet_s);

    uint8_t* TX_FIFO = (uint8_t*)((uint32_t)tx_line_buffer + sizeof(struct packet_s));
    uint8_t* RX_FIFO = (uint8_t*)((uint32_t)rx_line_buffer + sizeof(struct packet_s));

    uint32_t TX_FIFO_size = (tx_line_buffer_size - sizeof(struct packet_s))/sizeof(struct TX_FIFO_element_s);
    uint32_t RX_FIFO_size = (rx_line_buffer_size - sizeof(struct packet_s))/sizeof(struct RX_FIFO_element_s);

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
            memset(line->RX_BUFFER,0x55,sizeof(struct packet_s));
            LineFunctions->StartRX(line);
            break;

        default:
            
            break;
    }

}

struct HL_new_s HL_new = {.Send_Response = Send_Response,
                            .Get_Request = Get_Request,
                            .Protocol = Protocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
