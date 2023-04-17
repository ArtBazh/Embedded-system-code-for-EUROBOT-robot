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
#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "l6474_drvr.h" 



struct stepper_s{
    void (*SetZeroPosisition)(enum L6474H_motor_s motor);
    void (*StepToPosisition)(enum L6474H_motor_s motor, int32_t steps);
    int32_t (*GetPresentPosisition)(enum L6474H_motor_s motor);
    void (*Stop)(enum L6474H_motor_s motor);
    void (*Enable)(enum L6474H_motor_s motor);
    void (*Disable)(enum L6474H_motor_s motor);
    void (*Init)(enum L6474H_motor_s motor);
}; 

extern struct stepper_s Stepper;

void TIM1_Steppers_PWM_CC_IRQ_Process(void);

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
