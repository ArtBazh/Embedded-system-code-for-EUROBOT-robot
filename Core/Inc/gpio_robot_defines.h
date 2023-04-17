/**
  ******************************************************************************
  * @file    	GPIOs_Defines.h
  * @brief   HAL configuration file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_ROBOT_DEFINES_H
#define __GPIO_ROBOT_DEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "gpio_board_defines.h"


/*  Common Robot Pins  */
#define	STARTING_CORD_BTN_GPIO14_PC15			GPIO14_PC15
#define	PLAYGROUND_SIDE_BTN_GPIO13_PE6	  GPIO13_PE6
#define	STRATEGY_BTN_GPIO12_PE5				    GPIO12_PE5
#define	MAXON_ENABLE_BTN_GPIO11_PE2				GPIO11_PE2


/*  Specific Robot GUIDO Pins  */
#if defined (ROBOT_GUIDO)

#define	CART_FRONT_BTN_GPIOD_PIN4			LL_GPIO_PIN_4
#define	CART_BACK_BTN_GPIOD_PIN3			LL_GPIO_PIN_3
#define	STATUE_BTN_GPIOD_PIN7				  LL_GPIO_PIN_7

#endif

/*  Specified Robot BIG Pins  */
#if defined (ROBOT_BIG)

#define	STEPPER_LEFT_BTN_GPIOD_PIN4			LL_GPIO_PIN_4
#define	STEPPER_RIGHT_BTN_GPIOD_PIN3		LL_GPIO_PIN_3
#define	GRIPPER_BTN_GPIOD_PIN7				  LL_GPIO_PIN_7

#endif





/* Exported macro ------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_HAL_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
