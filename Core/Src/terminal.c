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



#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "gpio_robot_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 
#include "hl_new.h" 
#include "terminal.h" 




void Send_Response(struct terminal_response_s* terminal_response)
{
  HL_new.Send_Response(USART1,(struct HL_new_Response_s*)terminal_response);
}

uint8_t Get_Request(struct terminal_request_s* terminal_request)
{
  return HL_new.Get_Request(USART1,(struct HL_new_Request_s*)terminal_request);
}


extern struct terminal_s Terminal = {.Send_Response = Send_Response,
                                        .Get_Request = Get_Request};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
