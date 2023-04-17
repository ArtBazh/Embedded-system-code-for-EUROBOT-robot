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
#ifndef __TERMINAL_CMD_TABLE_H
#define __TERMINAL_CMD_TABLE_H

#ifdef __cplusplus
 extern "C" {
#endif
	 

#define TERMINAL_CMD_TABLE_SIZE     64

#define BOARD_Ping            	    0X00    /* Status (2 bytes)    						*/
#define BOARD_Zhanibek              0X01    /* IC Configuration (2 bytes) 		*/
#define BOARD_Punch            	    0X02    /* Status (2 bytes)    						*/
#define BOARD_AX_SetAngle           0X03    /* Status (2 bytes)    						*/

#define BOARD_SetSpeed            	0X04    /* Status (2 bytes)    						*/
#define BOARD_GetSpeed              0X05    /* Status (2 bytes)    						*/
#define BOARD_SetCoord            	0X06    /* Status (2 bytes)    						*/
#define BOARD_GetCoord              0X07    /* Status (2 bytes)    						*/
#define BOARD_AX_GetAngle           0X08    /* Status (2 bytes)    						*/
#define BOARD_GrubPile              0X09    /* Status (2 bytes)               */
#define BOARD_ReleasePile           0X0A    /* Status (2 bytes)               */
#define BOARD_GrubPileStatus        0X0B    /* Status (2 bytes)               */
#define BOARD_ReleasePileStatus     0X0D    /* Status (2 bytes)               */
#define BOARD_AX_GetLoad            0X0E    /* Status (2 bytes)    						*/
#define BOARD_AX_SetWheelMode       0X0F    /* Status (2 bytes)    						*/
#define BOARD_AX_SetJointMode       0X10    /* Status (2 bytes)    						*/
#define BOARD_AX_SetMovingSpeed     0X11    /* Status (2 bytes)    						*/
#define BOARD_SetPWM                0X12    /* Status (2 bytes)    						*/
#define BOARD_Set_stepper_angle     0X13    /* Status (2 bytes)    						*/
#define BOARD_Stop_stepper          0X14    /* Status (2 bytes)    						*/
#define BOARD_Experiment            0X15    /* Status (2 bytes)    						*/
#define BOARD_DC                    0X16    /* Status (2 bytes)    						*/
#define BOARD_DCConst               0X17    /* Status (2 bytes)    						*/
#define BOARD_Encoder               0X18    /* Status (2 bytes)    						*/
#define BOARD_DC_Sort               0X19    /* Status (2 bytes)    						*/
#define BOARD_Sort                  0X1A    /* Status (2 bytes)    						*/
#define BOARD_Sort_Status           0X1B    /* Status (2 bytes)    						*/
#define BOARD_Optimal_Sort          0X1C    /* Status (2 bytes)    						*/
#define BOARD_Zero_State            0X1D    /* Status (2 bytes)    						*/
#define BOARD_Open_Gate             0X1E    /* Status (2 bytes)    						*/
#define BOARD_Switch_Emp            0X1F    /* Status (2 bytes)    						*/
#define BOARD_GripperCalibration    0X20    /* Status (2 bytes)    						*/
#define BOARD_Start_Game            0X21    /* Status (2 bytes)    						*/

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
