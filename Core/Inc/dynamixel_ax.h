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
#ifndef __DYNAMIXEL_AX_H
#define __DYNAMIXEL_AX_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "stm32f4xx_ll_usart.h"

	 
/*	AX12A EEPROM Area registers define ------------------------------------------------------------ */
#define AX_ModelNumber           	0    /* Current Position (3 bytes)     */
#define AX_FirmwareVersion       	2    /* Electrical Position (2 bytes) 	*/
#define AX_ID              				3    /* Mark Position (3 bytes) 				*/
#define AX_BaudRate              	4    /* Reference Current  						*/
#define AX_ReturnDelayTime       	5    /* Fast Decay/Fall Step Time  		*/
#define AX_CW_AngleLimit     			6    /* Minimum ON Time  							*/
#define AX_CCW_AngleLimit     			8    /* Minimum ON Time  							*/
#define AX_TemperatureLimit      	11    /* Minimum OFF Time               */
#define AX_Min_VoltageLimit  			12    /* ADC Output     								*/
#define AX_Max_VoltageLimit  			13    /* ADC Output     								*/
#define AX_MaxTorque           		14    /* OCD Threshold     							*/
#define AX_StatusReturnLevel     	16    /* Step Mode     									*/
#define AX_AlarmLED          			17    /* Alarms Enables     						*/
#define AX_Shutdown            		18    /* IC Configuration (2 bytes) 		*/


/*	AX12A RAM Area registers define ------------------------------------------------------------ */
#define AX_TorqueEnable          	24    /* Status (2 bytes)    						*/
#define AX_LED           					25    /* Current Position (3 bytes)     */
#define AX_CW_ComplianceMargin   	26    /* Electrical Position (2 bytes) 	*/
#define AX_CCW_ComplianceMargin  	27    /* Electrical Position (2 bytes) 	*/
#define AX_CW_ComplianceSlope    	28    /* Mark Position (3 bytes) 				*/
#define AX_CCW_ComplianceSlope   	29    /* Mark Position (3 bytes) 				*/
#define AX_GoalPosition          	30    /* Reference Current  						*/
#define AX_MovingSpeed           	32    /* Fast Decay/Fall Step Time  		*/
#define AX_TorqueLimit           	34    /* Minimum ON Time  							*/
#define AX_PresentPosition       	36    /* Minimum OFF Time               */
#define AX_PresentSpeed         		38    /* ADC Output     								*/
#define AX_PresentLoad           	40    /* OCD Threshold     							*/
#define AX_PresentVoltage        	42    /* Step Mode     									*/
#define AX_PresentTemperature    	43    /* Alarms Enables     						*/
#define AX_RegisteredInstruction 	44    /* IC Configuration (2 bytes) 		*/
#define AX_Moving            			46    /* Status (2 bytes)    						*/
#define AX_Lock            				47    /* IC Configuration (2 bytes) 		*/
#define AX_Punch            				48    /* Status (2 bytes)    						*/



struct Dynamixel_AX_s{
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

extern struct Dynamixel_AX_s Dynamixel_AX;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
