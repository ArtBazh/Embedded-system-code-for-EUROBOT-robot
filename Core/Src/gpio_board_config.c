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


#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_system.h"
#include "gpio_board_defines.h"
#include "gpio_board_config.h"


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Board_Config(void)
{
	LL_GPIO_InitTypeDef GPIO_Struct;

	/*	RS-485 & USARTx ------------------------------------------------------------------*/

	/*	UART1 RX pin	*/
	GPIO_Struct.Pin = UART1RX_GPIOA_PIN10;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_7;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);
	
	/*	UART1 TX pin	*/
	GPIO_Struct.Pin = UART1TX_GPIOA_PIN9;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_7;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	USART2TXRX RS-485	*/
	GPIO_Struct.Pin = UART2TXRX_GPIOD_PIN5;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	//GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	USART2DE RS-485	*/
	LL_GPIO_ResetOutputPin(GPIOD, UART2DE_GPIOD_PIN6);
	
	GPIO_Struct.Pin = UART2DE_GPIOD_PIN6;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	USART4TXRX RS-485	*/
	GPIO_Struct.Pin = UART4TXRX_GPIOC_PIN10;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	USART4DE RS-485	*/
	LL_GPIO_ResetOutputPin(GPIOA, UART4DE_GPIOA_PIN15);
	
	GPIO_Struct.Pin = UART4DE_GPIOA_PIN15;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	UART3 TX/RX DYNAMIXEL	*/
	GPIO_Struct.Pin = UART3TXRX_GPIOD_PIN8;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_7;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	UART3 DE DYNAMIXEL	*/
	LL_GPIO_ResetOutputPin(GPIOD, UART3DE_GPIOD_PIN9);
	
	GPIO_Struct.Pin = UART3DE_GPIOD_PIN9;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	UART5 RX pin	*/
	GPIO_Struct.Pin = UART5RX_GPIOD_PIN2;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	UART5 TX pin	*/
	GPIO_Struct.Pin = UART5TX_GPIOC_PIN12;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	UART6TXRX RS-485	*/
	GPIO_Struct.Pin = UART6TXRX_GPIOC_PIN6;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	UART6DE RS-485	*/
	LL_GPIO_ResetOutputPin(GPIOC, UART6DE_GPIOC_PIN7);
	
	GPIO_Struct.Pin = UART6DE_GPIOC_PIN7;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	

	
	/*	Shift Register Control  ------------------------------------------------------------------*/

	/*	STCP	*/
	LL_GPIO_SetOutputPin(GPIOA, STCP_GPIOA_PIN4);
	
	GPIO_Struct.Pin = STCP_GPIOA_PIN4;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	SPI1	*/
	GPIO_Struct.Pin = SPI1_SCK_GPIOA_PIN5|SPI1_MISO_GPIOA_PIN6|SPI1_MOSI_GPIOA_PIN7;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_5;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);


	/*	Stepper Motors Control ------------------------------------------------------------------*/
	
	/*	NCS1	*/
	LL_GPIO_SetOutputPin(GPIOE, NCS1_GPIOE_PIN7);
	
	GPIO_Struct.Pin = NCS1_GPIOE_PIN7;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	NCS2	*/
	LL_GPIO_SetOutputPin(GPIOD, NCS2_GPIOD_PIN10);
	
	GPIO_Struct.Pin = NCS2_GPIOD_PIN10;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	NCS3	*/
	LL_GPIO_SetOutputPin(GPIOD, NCS3_GPIOD_PIN11);
	
	GPIO_Struct.Pin = NCS3_GPIOD_PIN11;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	NCS4	*/
	LL_GPIO_SetOutputPin(GPIOC, NCS4_GPIOC_PIN9);
	
	GPIO_Struct.Pin = NCS4_GPIOC_PIN9;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);

	/*	STCK1	*/
	GPIO_Struct.Pin = STCK1_GPIOE_PIN9;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	STCK2	*/
	GPIO_Struct.Pin = STCK2_GPIOE_PIN11;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	STCK3	*/
	GPIO_Struct.Pin = STCK3_GPIOE_PIN13;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	STCK4	*/
	GPIO_Struct.Pin = STCK4_GPIOE_PIN14;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	DIR1	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR1_GPIOE_PIN8);
	
	GPIO_Struct.Pin = DIR1_GPIOE_PIN8;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	DIR2	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR2_GPIOE_PIN10);
	
	GPIO_Struct.Pin = DIR2_GPIOE_PIN10;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	DIR3	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR3_GPIOE_PIN12);
	
	GPIO_Struct.Pin = DIR3_GPIOE_PIN12;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	DIR4	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR4_GPIOE_PIN15);
	
	GPIO_Struct.Pin = DIR4_GPIOE_PIN15;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	
	/*	RC Serovs ------------------------------------------------------------------*/
	
	/*	SERV0	*/
	GPIO_Struct.Pin = SERV0_GPIOD_PIN15;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	SERV1	*/
	GPIO_Struct.Pin = SERV1_GPIOD_PIN14;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	SERV2	*/
	GPIO_Struct.Pin = SERV2_GPIOD_PIN13;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	SERV3	*/
	GPIO_Struct.Pin = SERV3_GPIOD_PIN12;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	SERV4 pin	*/
	GPIO_Struct.Pin = SERV4_GPIOA_PIN0;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	SERV5 pin	*/
	GPIO_Struct.Pin = SERV5_GPIOA_PIN1;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	SERV6 pin	*/
	GPIO_Struct.Pin = SERV6_GPIOA_PIN2;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	SERV7 pin	*/
	GPIO_Struct.Pin = SERV7_GPIOA_PIN3;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);


	/*	WS2812B ------------------------------------------------------------------*/
	
	/*	WS2812B pin	*/
	GPIO_Struct.Pin = WS2812B_GPIOC_PIN8;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_UP;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_3;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
