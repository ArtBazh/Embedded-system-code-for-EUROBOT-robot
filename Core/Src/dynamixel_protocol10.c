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
#include "dynamixel_protocol10.h"
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

#define MAX_RETRANSMIT_NUMBER	 2
#define TIMEOUT_US               2000




typedef struct {
    uint8_t H1;
    uint8_t H2;
    uint8_t packet_id;
    uint8_t length;
	uint8_t instruction_error;
	uint8_t param_crc[MAX_PACKET_SIZE-5];
} Packet_TypeDef;

typedef struct {
    Packet_TypeDef packet;
    struct Dynamixel_Protocol10_Response_s* response_buf;
    uint8_t responses_number;
    uint8_t retransmit_downcounter;
} TX_FIFO_Cell_TypeDef;

typedef struct {
    Packet_TypeDef packet;
    uint8_t responses_downcounter;
    uint8_t cumulative_error;
} RX_FIFO_Cell_TypeDef;




static struct LineFunctions_s* LineFunctions;




void Retransmit(struct LineInfo_s* line);
void Transmit_Next(struct LineInfo_s* line);
void Transmit_Cmd(struct LineInfo_s*  line, TX_FIFO_Cell_TypeDef* txFIFO_Cell);
uint8_t CRC_Calc(Packet_TypeDef* packet);



static void Ping_Cmd(USART_TypeDef* USART, uint8_t deviceID, struct Dynamixel_Protocol10_Response_s* response_buf)
{
    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

    TX_FIFO_Cell_TypeDef txFIFO_Cell;
    
    txFIFO_Cell.packet.H1 = 0xFF;
    txFIFO_Cell.packet.H2 = 0xFF;
    txFIFO_Cell.packet.packet_id = deviceID;
    txFIFO_Cell.packet.length = 2;
    txFIFO_Cell.packet.instruction_error = Ping_CMD;
    txFIFO_Cell.packet.param_crc[0] = CRC_Calc(&(txFIFO_Cell.packet));

    txFIFO_Cell.response_buf = response_buf;
    txFIFO_Cell.responses_number = 1;

    response_buf->data_ready_flag = 0;

    Transmit_Cmd(line,&txFIFO_Cell);
}

static void Write_Cmd(USART_TypeDef* USART, uint8_t deviceID, uint8_t reg_address, uint8_t* w_data, uint8_t data_length, struct Dynamixel_Protocol10_Response_s* response_buf)
{
    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

    TX_FIFO_Cell_TypeDef txFIFO_Cell;
    
    txFIFO_Cell.packet.H1 = 0xFF;
    txFIFO_Cell.packet.H2 = 0xFF;
    txFIFO_Cell.packet.packet_id = deviceID;
    txFIFO_Cell.packet.length = data_length+3;
    txFIFO_Cell.packet.instruction_error = Write_CMD;
    txFIFO_Cell.packet.param_crc[0] = reg_address;
    memcpy(&(txFIFO_Cell.packet.param_crc[1]), w_data, data_length);
    txFIFO_Cell.packet.param_crc[data_length+1] = CRC_Calc(&(txFIFO_Cell.packet));

    txFIFO_Cell.response_buf = response_buf;
    txFIFO_Cell.responses_number = 1;

    response_buf->data_ready_flag = 0;

    Transmit_Cmd(line,&txFIFO_Cell);
}


static void Read_Cmd(USART_TypeDef* USART, uint8_t deviceID, uint8_t reg_address, uint8_t data_length, struct Dynamixel_Protocol10_Response_s* response_buf)
{
    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

    TX_FIFO_Cell_TypeDef txFIFO_Cell;
    
    txFIFO_Cell.packet.H1 = 0xFF;
    txFIFO_Cell.packet.H2 = 0xFF;
    txFIFO_Cell.packet.packet_id = deviceID;
    txFIFO_Cell.packet.length = 4;
    txFIFO_Cell.packet.instruction_error = Read_CMD;
    txFIFO_Cell.packet.param_crc[0] = reg_address;
    txFIFO_Cell.packet.param_crc[1] = data_length;
    txFIFO_Cell.packet.param_crc[2] = CRC_Calc(&(txFIFO_Cell.packet));

    txFIFO_Cell.response_buf = response_buf;
    txFIFO_Cell.responses_number = 1;

    response_buf->data_ready_flag = 0;

    Transmit_Cmd(line,&txFIFO_Cell);
}

static void RegWrite_Cmd(USART_TypeDef* USART, uint8_t deviceID, uint8_t reg_address, uint8_t* w_data, uint8_t data_length, uint8_t* response_buf)
{

}
static void Action_Cmd(USART_TypeDef* USART, uint8_t deviceID, uint8_t* response_buf)
{

}
static void FactoryReset_Cmd(USART_TypeDef* USART, uint8_t deviceID, uint8_t* response_buf)
{

}
static void Reboot_Cmd(USART_TypeDef* USART, uint8_t deviceID, uint8_t* response_buf)
{

}

static void SyncWrite_Cmd(USART_TypeDef* USART, uint8_t* deviceID_list, uint8_t devices_number, uint8_t reg_address, uint8_t* w_data_list, uint8_t data_length, struct Dynamixel_Protocol10_Response_s* response_buf_list)
{
    struct LineInfo_s*  line = LineFunctions->USARTx_GetLine(USART);

    TX_FIFO_Cell_TypeDef txFIFO_Cell;
    
    txFIFO_Cell.packet.H1 = 0xFF;
    txFIFO_Cell.packet.H2 = 0xFF;
    txFIFO_Cell.packet.packet_id = BROADCAST_ID;
    txFIFO_Cell.packet.length = 4 + devices_number*(data_length+1);
    txFIFO_Cell.packet.instruction_error = SyncWrite_CMD;
    txFIFO_Cell.packet.param_crc[0] = reg_address;
    txFIFO_Cell.packet.param_crc[1] = data_length;

    uint8_t data_length_p1 = data_length + 1;

    do{
        uint8_t devices_number_m1 = devices_number - 1;
        txFIFO_Cell.packet.param_crc[2+data_length_p1*devices_number_m1] = deviceID_list[devices_number_m1];
        memcpy(&txFIFO_Cell.packet.param_crc[3+data_length_p1*devices_number_m1],&w_data_list[data_length*devices_number_m1],data_length);
        response_buf_list[devices_number_m1].data_ready_flag = 0;
    }while(devices_number--);
    
    txFIFO_Cell.packet.param_crc[txFIFO_Cell.packet.length-2] = CRC_Calc(&(txFIFO_Cell.packet));

    txFIFO_Cell.response_buf = response_buf_list;
    txFIFO_Cell.responses_number = 0;

    Transmit_Cmd(line,&txFIFO_Cell);
}

static void BulkRead_Cmd(USART_TypeDef* USART, uint8_t* deviceID_list, uint8_t* reg_address_list, uint8_t* data_length, struct Dynamixel_Protocol10_Response_s* response_buf_list)
{

}

static void Send_Response(USART_TypeDef* USART, uint8_t ID, uint8_t b, uint8_t* c, uint8_t d)
{

}
static void Receive_Request(uint8_t* a)
{

}




static uint8_t CRC_Calc(Packet_TypeDef* packet)
{
    uint16_t crc = packet->packet_id + packet->length + packet->instruction_error;
	
	for(uint8_t ik=0; ik<(packet->length-2); ik++)
		crc += packet->param_crc[ik];
	
	crc = ~crc;
	crc &= 0xFF;

    return (uint8_t)crc;
}

static uint8_t PacketErrorCheck(Packet_TypeDef* packet, uint8_t packet_length)
{
    if((packet->H1 != 0xFF)||(packet->H2 != 0xFF))
		return 1;

    if(packet_length<6)
        return 1;

	if(packet_length != (packet->length + 4))
		return 1;

    uint8_t packet_crc = packet->param_crc[packet->length - 2];
    uint8_t calc_crc = CRC_Calc(packet);

    if(packet_crc != calc_crc)
        return 1;
    
    return 0;
}

static void Retransmit(struct LineInfo_s* line)
{
    ((TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER))->retransmit_downcounter--;
    LineFunctions->StartTX(line,((TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER))->packet.length+4);
}

static void Transmit_Next(struct LineInfo_s* line)
{
    
    FIFO_Remove(&(line->TX_FIFO),line->TX_BUFFER);
    ((TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER))->retransmit_downcounter = MAX_RETRANSMIT_NUMBER;
    LineFunctions->StartTX(line,((TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER))->packet.length+4);
    
    
}

static void Transmit_Cmd(struct LineInfo_s*  line, TX_FIFO_Cell_TypeDef* txFIFO_Cell)
{
    while(1)
    {
        taskENTER_CRITICAL();

        if(FIFO_IsNotFull(&(line->TX_FIFO)))
            break;

        taskEXIT_CRITICAL();
        taskYIELD();
    }
    
    FIFO_Add(&(line->TX_FIFO),txFIFO_Cell);

    if(line->lineState==LINE_FREE){
        line->lineState = LINE_BUSY;
        Transmit_Next(line);
    }

    taskEXIT_CRITICAL();
}

static void Check_TX_FIFO(struct LineInfo_s* line)
{
    if(FIFO_IsNotEmpty(&(line->TX_FIFO)))
        Transmit_Next(line);
    else
        line->lineState = LINE_FREE;
}


static void USART_RX_Timeout_Callback(void* line)
{
    LineFunctions->StopRX((struct LineInfo_s*)line);

    TX_FIFO_Cell_TypeDef* TX_info = (TX_FIFO_Cell_TypeDef*)(((struct LineInfo_s*)line)->TX_BUFFER);

    if(TX_info->retransmit_downcounter){
        Retransmit(line);
        return;
    }

    if(TX_info->response_buf != NULL){
        do{
            TX_info->response_buf->bus_error = 1;
            TX_info->response_buf->data_ready_flag = 1;
        }while(TX_info->responses_number--);
     }

    Check_TX_FIFO(line);
}

static void USART_TX_Complete_Callabck_Master(struct LineInfo_s* line)
{
    LineFunctions->StopTX(line);

    if(((TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER))->responses_number){
        ((RX_FIFO_Cell_TypeDef*)(line->RX_BUFFER))->responses_downcounter = ((TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER))->responses_number;
        ((RX_FIFO_Cell_TypeDef*)(line->RX_BUFFER))->cumulative_error = 0;
        memset(&(((RX_FIFO_Cell_TypeDef*)(line->RX_BUFFER))->packet),0x55,sizeof(Packet_TypeDef));
        LineFunctions->StartRX(line);
        LineFunctions->Set_RX_Timeout(TIMEOUT_US,line);
        return;
    }

    Check_TX_FIFO(line);
}

static void USART_RX_Complete_Callabck_Master(struct LineInfo_s* line, uint32_t data_length)
{
    Packet_TypeDef* packet = (Packet_TypeDef*)&((RX_FIFO_Cell_TypeDef*)(line->RX_BUFFER))->packet;
    uint8_t packet_length = data_length;

    if(PacketErrorCheck(packet,packet_length))
        return;

    LineFunctions->Reset_RX_Timeout(line);
    LineFunctions->StopRX(line);

    RX_FIFO_Cell_TypeDef* RX_info = (RX_FIFO_Cell_TypeDef*)(line->RX_BUFFER);
    TX_FIFO_Cell_TypeDef* TX_info = (TX_FIFO_Cell_TypeDef*)(line->TX_BUFFER);

    RX_info->responses_downcounter--;

    RX_info->cumulative_error |= (packet->instruction_error)&(INSTRUCTION_ERROR|CHECKSUM_ERROR|RANGE_ERROR);

    if(RX_info->cumulative_error)
    {     
        if(TX_info->retransmit_downcounter)
        {
            if(RX_info->responses_downcounter)
                return;
            Retransmit(line);
            return;
        }
    }
        
    if(TX_info->response_buf != NULL){
        TX_info->response_buf->bus_error = 0;
        TX_info->response_buf->device_error = packet->instruction_error;
        TX_info->response_buf->data_length = packet->length-2;
        memcpy(TX_info->response_buf->data,packet->param_crc,packet->length-2);
    }
    

    if(RX_info->responses_downcounter){
        if(TX_info->response_buf != NULL)
            *((uint32_t*)(&(TX_info->response_buf))) += sizeof(struct Dynamixel_Protocol10_Response_s);
        LineFunctions->StartRX(line);
        return;
    }

    if(TX_info->response_buf != NULL)
        TX_info->response_buf->data_ready_flag = 1;

    Check_TX_FIFO(line);
}

static void USART_TX_Complete_Callabck_Slave(struct LineInfo_s* line)
{
	
}

static void USART_RX_Complete_Callabck_Slave(struct LineInfo_s* line, uint32_t data_length)
{
	
}



static void Protocol(struct LineFunctions_s* line_functions,struct LineInfo_s* line,uint8_t role, uint8_t* tx_line_buffer, uint32_t tx_line_buffer_size, uint8_t* rx_line_buffer, uint32_t rx_line_buffer_size)
{
    LineFunctions = line_functions;
	
    if((tx_line_buffer_size<2*sizeof(TX_FIFO_Cell_TypeDef))||
        (rx_line_buffer_size<2*sizeof(RX_FIFO_Cell_TypeDef)))
        return;

    line->TX_BUFFER = tx_line_buffer;
    line->RX_BUFFER = rx_line_buffer;

    line->TX_BUFFER_size = sizeof(Packet_TypeDef);
    line->RX_BUFFER_size = sizeof(Packet_TypeDef);

    uint8_t* TX_FIFO = (uint8_t*)((uint32_t)tx_line_buffer + sizeof(TX_FIFO_Cell_TypeDef));
    uint8_t* RX_FIFO = (uint8_t*)((uint32_t)rx_line_buffer + sizeof(RX_FIFO_Cell_TypeDef));

    uint32_t TX_FIFO_size = tx_line_buffer_size/sizeof(TX_FIFO_Cell_TypeDef) - 1;
    uint32_t RX_FIFO_size = rx_line_buffer_size/sizeof(RX_FIFO_Cell_TypeDef) - 1;

    FIFO_Init(&(line->TX_FIFO),TX_FIFO,TX_FIFO_size,sizeof(TX_FIFO_Cell_TypeDef));
    FIFO_Init(&(line->RX_FIFO),RX_FIFO,RX_FIFO_size,sizeof(RX_FIFO_Cell_TypeDef));

    line->lineState = LINE_FREE;
    line->USART_mode = HALF_DUPLEX;

    switch (role)
    {
        case MASTER:
            line->TX_Complete_Callback = USART_TX_Complete_Callabck_Master;
            line->RX_Complete_Callback = USART_RX_Complete_Callabck_Master;
            break;

        case SLAVE:
            line->TX_Complete_Callback = USART_TX_Complete_Callabck_Slave;
            line->RX_Complete_Callback = USART_RX_Complete_Callabck_Slave;
            break;

        default:
            
            break;
    }

    line->RX_Timeout_Callback = USART_RX_Timeout_Callback;
}

struct Dynamixel_Protocol10_s Dynamixel_Protocol10 = {.Ping = Ping_Cmd,
                                                        .Read = Read_Cmd,
                                                        .Write = Write_Cmd,
                                                        .RegWrite = RegWrite_Cmd,
                                                        .Action = Action_Cmd,
                                                        .FactoryReset = FactoryReset_Cmd,
                                                        .Reboot = Reboot_Cmd,
                                                        .SyncWrite = SyncWrite_Cmd,
                                                        .BulkRead = BulkRead_Cmd,
                                                        .Send_Response = Send_Response,
                                                        .Receive_Request = Receive_Request,
                                                        .Protocol = Protocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
