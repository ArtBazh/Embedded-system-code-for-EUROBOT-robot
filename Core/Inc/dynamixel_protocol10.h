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
#ifndef __DYNAMIXEL_PROTOCOL10_H
#define __DYNAMIXEL_PROTOCOL10_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "stm32f4xx_ll_usart.h"
#include "usartx_communication.h"

#define BROADCAST_ID            0xFE

/* Dynamixel Protocol 1.0 commands define ------------------------------------------------------------ */
#define Ping_CMD      				  0x01    /* Current Position (3 bytes)     */
#define Read_CMD      				  0x02    /* Current Position (3 bytes)     */
#define Write_CMD        			  0x03    /* Electrical Position (2 bytes) 	*/
#define RegWrite_CMD       			0x04    /* Electrical Position (2 bytes) 	*/
#define Action_CMD     				  0x05    /* Electrical Position (2 bytes) 	*/
#define FactoryReset_CMD    		0x06    /* Electrical Position (2 bytes) 	*/
#define Reboot_CMD       			  0x08    /* Electrical Position (2 bytes) 	*/

/*  NOT USED IN AX12A */
#define SyncWrite_CMD     			0x83    /* Electrical Position (2 bytes) 	*/
#define BulkRead_CMD        		0x92    /* Electrical Position (2 bytes) 	*/

/*	Dynamixel Protocol 1.0 ERROR codes define ------------------------------------------------------------ */
#define NO_DEVICE_ERROR         0    				 /* No error code    						*/
#define INSTRUCTION_ERROR       (1U<<6UL)    /* No error code    						*/
#define OVERLOAD_ERROR       		(1U<<5UL)    /* No error code    						*/
#define CHECKSUM_ERROR       		(1U<<4UL)    /* No error code    						*/
#define RANGE_ERROR       			(1U<<3UL)    /* No error code    						*/
#define OVERHEATING_ERROR       (1U<<2UL)    /* No error code    						*/
#define ANGLE_LIMIT_ERROR       (1U<<1UL)    /* No error code    						*/
#define INPUT_VOLTAGE_ERROR     (1U<<0UL)    /* No error code    						*/




#define MAX_PACKET_SIZE    16



struct Dynamixel_Protocol10_Response_s{
	uint8_t data_ready_flag;
  uint8_t bus_error;
  uint8_t device_error;
  uint8_t data_length;
  uint8_t data[MAX_PACKET_SIZE-6];
};



struct Dynamixel_Protocol10_s{
    void (*Ping)(USART_TypeDef* , uint8_t, struct Dynamixel_Protocol10_Response_s*);
    void (*Read)(USART_TypeDef*, uint8_t, uint8_t, uint8_t, struct Dynamixel_Protocol10_Response_s*);
    void (*Write)(USART_TypeDef*, uint8_t, uint8_t, uint8_t*, uint8_t, struct Dynamixel_Protocol10_Response_s*);
    void (*RegWrite)(USART_TypeDef*, uint8_t, uint8_t, uint8_t*, uint8_t, uint8_t*);
    void (*Action)(USART_TypeDef*, uint8_t, uint8_t*);
    void (*FactoryReset)(USART_TypeDef*, uint8_t, uint8_t*);
    void (*Reboot)(USART_TypeDef*, uint8_t, uint8_t*);
    void (*SyncWrite)(USART_TypeDef*, uint8_t*, uint8_t, uint8_t, uint8_t*, uint8_t, struct Dynamixel_Protocol10_Response_s*);
    void (*BulkRead)(USART_TypeDef*, uint8_t*, uint8_t*, uint8_t*, struct Dynamixel_Protocol10_Response_s*);
    void (*Send_Response)(USART_TypeDef*, uint8_t, uint8_t, uint8_t*, uint8_t);
    void (*Receive_Request)(uint8_t*);
    void (*Protocol)(struct LineFunctions_s*, struct LineInfo_s*, uint8_t, uint8_t*, uint32_t, uint8_t*, uint32_t);
};

extern struct Dynamixel_Protocol10_s Dynamixel_Protocol10;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
