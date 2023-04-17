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
#include "rc_servo.h"
#include "h_bridge.h"
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"


#define H_BRIDGE1_P1  (enum rc_servo_e)RC_SERVO_0
#define H_BRIDGE1_P2  (enum rc_servo_e)RC_SERVO_1

#define H_BRIDGE2_P1  (enum rc_servo_e)RC_SERVO_2
#define H_BRIDGE2_P2  (enum rc_servo_e)RC_SERVO_3


enum rc_servo_e H_BRIDGE_P1[2] = {H_BRIDGE1_P1, H_BRIDGE2_P1};
enum rc_servo_e H_BRIDGE_P2[2] = {H_BRIDGE1_P2, H_BRIDGE2_P2};

static void SetMotorSpeed(enum h_bridge_e h_bridge, int32_t speed)
{
  uint32_t duty_cycle_p1, duty_cycle_p2;

  if((speed>100)||(speed<-100))
    return;

  if(speed>=0){
    duty_cycle_p1 = speed;
    duty_cycle_p2 = 100;
  }else{
    duty_cycle_p1 = 100;
    duty_cycle_p2 = -speed;
  }

  RC_Servo.SetPWMDutyCycle(H_BRIDGE_P1[h_bridge], duty_cycle_p1);
  RC_Servo.SetPWMDutyCycle(H_BRIDGE_P2[h_bridge], duty_cycle_p2);
}

static void Enable(enum h_bridge_e h_bridge)
{
  RC_Servo.Enable(H_BRIDGE_P1[h_bridge]);
  RC_Servo.Enable(H_BRIDGE_P2[h_bridge]);
}

static void Disable(enum h_bridge_e h_bridge)
{
  RC_Servo.Disable(H_BRIDGE_P1[h_bridge]);
  RC_Servo.Disable(H_BRIDGE_P2[h_bridge]);
}



struct h_bridge_s H_Bridge = {.SetMotorSpeed = SetMotorSpeed,
                              .Enable = Enable,
                              .Disable = Disable};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
