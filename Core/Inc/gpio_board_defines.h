/**
  ******************************************************************************
  * @file    	GPIOs_Defines.h
  * @brief   HAL configuration file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_BOARD_DEFINES_H
#define __GPIO_BOARD_DEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_ll_gpio.h"



/*  RS-485 & USARTx  */
#define	UART1RX_GPIOA_PIN10				LL_GPIO_PIN_10
#define	UART1TX_GPIOA_PIN9				LL_GPIO_PIN_9

#define	UART2TXRX_GPIOD_PIN5			LL_GPIO_PIN_5
#define	UART2DE_GPIOD_PIN6				LL_GPIO_PIN_6

#define	UART3TXRX_GPIOD_PIN8			LL_GPIO_PIN_8
#define	UART3DE_GPIOD_PIN9				LL_GPIO_PIN_9
	 
#define	UART4TXRX_GPIOC_PIN10			LL_GPIO_PIN_10
#define	UART4DE_GPIOA_PIN15				LL_GPIO_PIN_15

#define	UART5RX_GPIOD_PIN2				LL_GPIO_PIN_2
#define	UART5TX_GPIOC_PIN12				LL_GPIO_PIN_12

#define	UART6TXRX_GPIOC_PIN6			LL_GPIO_PIN_6
#define	UART6DE_GPIOC_PIN7				LL_GPIO_PIN_7

/*  Stepper Motors  */
#define	NCS4_GPIOC_PIN9						LL_GPIO_PIN_9
#define	STCK4_GPIOE_PIN14					LL_GPIO_PIN_14
#define	DIR4_GPIOE_PIN15					LL_GPIO_PIN_15

#define	NCS3_GPIOD_PIN11					LL_GPIO_PIN_11
#define	STCK3_GPIOE_PIN13					LL_GPIO_PIN_13
#define	DIR3_GPIOE_PIN12					LL_GPIO_PIN_12

#define	NCS2_GPIOD_PIN10					LL_GPIO_PIN_10
#define	STCK2_GPIOE_PIN11					LL_GPIO_PIN_11
#define	DIR2_GPIOE_PIN10					LL_GPIO_PIN_10

#define	NCS1_GPIOE_PIN7						LL_GPIO_PIN_7
#define	STCK1_GPIOE_PIN9					LL_GPIO_PIN_9
#define	DIR1_GPIOE_PIN8						LL_GPIO_PIN_8

/*  SPI1 for the Shift Register  */
#define	SPI1_SCK_GPIOA_PIN5				LL_GPIO_PIN_5
#define	SPI1_MISO_GPIOA_PIN6			LL_GPIO_PIN_6
#define	SPI1_MOSI_GPIOA_PIN7			LL_GPIO_PIN_7

/*  SPI2  */
#define	SPI2_SCK_GPIOB_PIN13			LL_GPIO_PIN_13
#define	SPI2_MISO_GPIOB_PIN14			LL_GPIO_PIN_14
#define	SPI2_MOSI_GPIOB_PIN15			LL_GPIO_PIN_15
#define	SPI2_NSS_GPIOB_PIN12			LL_GPIO_PIN_12

/*  SPI3  */
#define	SPI3_SCK_GPIOB_PIN3				LL_GPIO_PIN_3
#define	SPI3_MISO_GPIOB_PIN4			LL_GPIO_PIN_4
#define	SPI3_MOSI_GPIOB_PIN5			LL_GPIO_PIN_5
#define	SPI3_NSS_GPIOB_PIN6			  LL_GPIO_PIN_6

/*  I2C1  */
#define	I2C1_SCL_GPIOB_PIN8				LL_GPIO_PIN_8
#define	I2C1_SDA_GPIOB_PIN9			  LL_GPIO_PIN_9

/*  I2C2  */
#define	I2C1_SCL_GPIOB_PIN10			LL_GPIO_PIN_10
#define	I2C1_SDA_GPIOB_PIN11			LL_GPIO_PIN_11

/*  STCP Signal for the Shift Register  */
#define	STCP_GPIOA_PIN4						LL_GPIO_PIN_4

/*  RC SERVOs  */
#define	SERV0_GPIOD_PIN15					LL_GPIO_PIN_15
#define	SERV1_GPIOD_PIN14					LL_GPIO_PIN_14
#define	SERV2_GPIOD_PIN13					LL_GPIO_PIN_13
#define	SERV3_GPIOD_PIN12					LL_GPIO_PIN_12
#define	SERV4_GPIOA_PIN0					LL_GPIO_PIN_0
#define	SERV5_GPIOA_PIN1					LL_GPIO_PIN_1
#define	SERV6_GPIOA_PIN2					LL_GPIO_PIN_2
#define	SERV7_GPIOA_PIN3					LL_GPIO_PIN_3

/*  WS2812B LED Strip  */
#define	WS2812B_GPIOC_PIN8				LL_GPIO_PIN_8


/*  28 GPIO HEADER DEFINES */
#define GPIO1_PD12                SERV3_GPIOD_PIN12
#define GPIO2_PD15                SERV0_GPIOD_PIN15 
#define GPIO3_PA8                 LL_GPIO_PIN_8 
#define GPIO4_PD1                 LL_GPIO_PIN_1 
#define GPIO5_PB9                 I2C1_SDA_GPIOB_PIN9 
#define GPIO6_PD4                 LL_GPIO_PIN_4
#define GPIO7_PD7                 LL_GPIO_PIN_7 
#define GPIO8_PB5                 SPI3_MOSI_GPIOB_PIN5 
#define GPIO9_PB6                 SPI3_NSS_GPIOB_PIN6 
#define GPIO10_PE1                LL_GPIO_PIN_1
#define GPIO11_PE2                LL_GPIO_PIN_2
#define GPIO12_PE5                LL_GPIO_PIN_5 
#define GPIO13_PE6                LL_GPIO_PIN_6 
#define GPIO14_PC15               LL_GPIO_PIN_15

#define GPIO15_PC14               LL_GPIO_PIN_14 
#define GPIO16_PC13               LL_GPIO_PIN_13
#define GPIO17_PE4                LL_GPIO_PIN_4 
#define GPIO18_PE3                LL_GPIO_PIN_3 
#define GPIO19_PE0                LL_GPIO_PIN_0 
#define GPIO20_PB7                LL_GPIO_PIN_7 
#define GPIO21_PB4                SPI3_MISO_GPIOB_PIN4
#define GPIO22_PB3                SPI3_SCK_GPIOB_PIN3 
#define GPIO23_PD3                LL_GPIO_PIN_3 
#define GPIO24_PB8                I2C1_SCL_GPIOB_PIN8 
#define GPIO25_PD0                LL_GPIO_PIN_0
#define GPIO26_PC11               LL_GPIO_PIN_11
#define GPIO27_PD14               SERV1_GPIOD_PIN14 
#define GPIO28_PD13               SERV2_GPIOD_PIN13 



#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_HAL_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
