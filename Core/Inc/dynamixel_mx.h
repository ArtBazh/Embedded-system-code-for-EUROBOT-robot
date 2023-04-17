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
#ifndef __DYNAMIXEL_MX_H
#define __DYNAMIXEL_MX_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "stm32f4xx_ll_usart.h"

	 

/*	MX EEPROM Area registers define ------------------------------------------------------------ */
#define MX_ModelNumber           	0    /* Current Position (3 bytes)     */
#define MX_FirmwareVersion       	2    /* Electrical Position (2 bytes) 	*/
#define MX_ID              				3    /* Mark Position (3 bytes) 				*/
#define MX_BaudRate              	4    /* Reference Current  						*/
#define MX_ReturnDelayTime       	5    /* Fast Decay/Fall Step Time  		*/
#define MX_CW_AngleLimit     			6    /* Minimum ON Time  							*/
#define MX_CCW_AngleLimit     		8    /* Minimum ON Time  							*/
#define MX_TemperatureLimit      	11    /* Minimum OFF Time               */
#define MX_Min_VoltageLimit  			12    /* ADC Output     								*/
#define MX_Max_VoltageLimit  			13    /* ADC Output     								*/
#define MX_MaxTorque           		14    /* OCD Threshold     							*/
#define MX_StatusReturnLevel     	16    /* Step Mode     									*/
#define MX_AlarmLED          			17    /* Alarms Enables     						*/
#define MX_Shutdown            		18    /* IC Configuration (2 bytes) 		*/
#define MX_MultiTurnOffset        20    /* Alarms Enables     						*/
#define MX_ResolutionDivider      22    /* IC Configuration (2 bytes) 		*/


/*	MX RAM Area registers define ------------------------------------------------------------ */
#define MX_TorqueEnable          	24    /* Status (2 bytes)    						*/
#define MX_LED           					25    /* Current Position (3 bytes)     */
#define MX_D_Gain   	            26    /* Electrical Position (2 bytes) 	*/
#define MX_I_Gain  	              27    /* Electrical Position (2 bytes) 	*/
#define MX_P_Gain    	            28    /* Mark Position (3 bytes) 				*/
#define MX_GoalPosition          	30    /* Reference Current  						*/
#define MX_MovingSpeed           	32    /* Fast Decay/Fall Step Time  		*/
#define MX_TorqueLimit           	34    /* Minimum ON Time  							*/
#define MX_PresentPosition       	36    /* Minimum OFF Time               */
#define MX_PresentSpeed         	38    /* ADC Output     								*/
#define MX_PresentLoad           	40    /* OCD Threshold     							*/
#define MX_PresentVoltage        	42    /* Step Mode     									*/
#define MX_PresentTemperature    	43    /* Alarms Enables     						*/
#define MX_Registered  	          44    /* IC Configuration (2 bytes) 		*/
#define MX_Moving            			46    /* Status (2 bytes)    						*/
#define MX_Lock            				47    /* IC Configuration (2 bytes) 		*/
#define MX_Punch            			48    /* Status (2 bytes)    						*/
#define MX_RealtimeTick    	      50    /* Alarms Enables     						*/
#define MX_Current  	            68    /* IC Configuration (2 bytes) 		*/
#define MX_TorqueCtrlModeEnable   70    /* Status (2 bytes)    						*/
#define MX_GoalTorque            	71    /* IC Configuration (2 bytes) 		*/
#define MX_GoalAcceleration       73    /* Status (2 bytes)    						*/



struct Dynamixel_MX_s{
    uint8_t (*Ping)(USART_TypeDef* , uint8_t);
    void (*SetID)(USART_TypeDef* , uint8_t, uint8_t);
    void (*TorqueEnable)(USART_TypeDef* , uint8_t);
    void (*TorqueDisable)(USART_TypeDef* , uint8_t);
    void (*SetAngle)(USART_TypeDef* , uint8_t, uint16_t);
    uint8_t (*GetAngle)(USART_TypeDef*, uint8_t, uint16_t*);
    void (*SetWheelMode)(USART_TypeDef* , uint8_t);
    void (*SetJointMode)(USART_TypeDef* , uint8_t);
    void (*SetMovingSpeed)(USART_TypeDef* , uint8_t, int16_t);
    uint8_t (*GetPresentLoad)(USART_TypeDef*, uint8_t, int16_t*);
    void (*EnableLED)(USART_TypeDef* , uint8_t);
    void (*DisableLED)(USART_TypeDef* , uint8_t);
}; 

extern struct Dynamixel_MX_s Dynamixel_MX;



#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
