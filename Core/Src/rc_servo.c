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
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"


TIM_TypeDef* TIM[SERVOS_NUMBER] = {TIM4,TIM4,TIM4,TIM4,TIM5,TIM5,TIM5,TIM5};

uint32_t TIM_CHANNEL[SERVOS_NUMBER] = {LL_TIM_CHANNEL_CH4,LL_TIM_CHANNEL_CH3,LL_TIM_CHANNEL_CH2,LL_TIM_CHANNEL_CH1,
                                        LL_TIM_CHANNEL_CH1,LL_TIM_CHANNEL_CH2,LL_TIM_CHANNEL_CH3,LL_TIM_CHANNEL_CH4};

volatile uint32_t* TIM_CCR[SERVOS_NUMBER] = {&TIM4->CCR4,&TIM4->CCR3,&TIM4->CCR2,&TIM4->CCR1,&TIM5->CCR1,&TIM5->CCR2,&TIM5->CCR3,&TIM5->CCR4};
volatile uint32_t* TIM_CCER[SERVOS_NUMBER] = {&TIM4->CCER,&TIM4->CCER,&TIM4->CCER,&TIM4->CCER,&TIM5->CCER,&TIM5->CCER,&TIM5->CCER,&TIM5->CCER};

GPIO_TypeDef* SERVO_GPIO[SERVOS_NUMBER] = {GPIOD,GPIOD,GPIOD,GPIOD,GPIOA,GPIOA,GPIOA,GPIOA};
uint32_t SERVO_PIN[SERVOS_NUMBER] = {SERV0_GPIOD_PIN15,SERV1_GPIOD_PIN14,SERV2_GPIOD_PIN13,SERV3_GPIOD_PIN12,
                                      SERV4_GPIOA_PIN0,SERV5_GPIOA_PIN1,SERV6_GPIOA_PIN2,SERV7_GPIOA_PIN3};



static void TIM_OC_SetCompareCH(uint32_t servo, uint32_t CompareValue)
{
  WRITE_REG(*TIM_CCR[servo], CompareValue);
}

static void TIM_CC_EnableChannel(uint32_t servo)
{
  SET_BIT(*TIM_CCER[servo], TIM_CHANNEL[servo]);
}

static void TIM_CC_DisableChannel(uint32_t servo)
{
  CLEAR_BIT(*TIM_CCER[servo], TIM_CHANNEL[servo]);
}

static void SetPWMDutyCycle(enum rc_servo_e servo, uint32_t duty_cycle)
{
  if(duty_cycle>100)
    return;

  uint32_t cc_value = ((LL_TIM_GetAutoReload(TIM[servo])+1)*duty_cycle)/100;

  TIM_OC_SetCompareCH(servo, cc_value);
}

static void SetMotorSpeed(enum rc_servo_e servo, uint32_t speed)
{
  // uint32_t cc_value = 1000 + speed*10;
  uint32_t cc_value = speed*10;
  if (servo == (enum rc_servo_e)0 | servo == (enum rc_servo_e)1 | servo == (enum rc_servo_e)2 | servo == (enum rc_servo_e)3)
    speed /= 10;

  TIM_OC_SetCompareCH(servo, cc_value);
}

static void SetServoAngle(enum rc_servo_e servo, uint32_t pulse_min_us, uint32_t pulse_max_us, uint32_t angle_range, uint32_t angle)
{
  uint32_t cc_value = pulse_min_us + (angle*(pulse_max_us-pulse_min_us))/angle_range;

  TIM_OC_SetCompareCH(servo, cc_value);
}

static void Enable(enum rc_servo_e servo)
{
  LL_GPIO_InitTypeDef GPIO_Struct;

  GPIO_Struct.Pin = SERVO_PIN[servo];
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_2;
	
	LL_GPIO_Init(SERVO_GPIO[servo], &GPIO_Struct);

  TIM_CC_EnableChannel(servo);
  LL_TIM_GenerateEvent_UPDATE(TIM[servo]);
}

static void Disable(enum rc_servo_e servo)
{
  LL_GPIO_InitTypeDef GPIO_Struct;

  GPIO_Struct.Pin = SERVO_PIN[servo];
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(SERVO_GPIO[servo], &GPIO_Struct);

  TIM_CC_DisableChannel(servo);
  LL_TIM_GenerateEvent_UPDATE(TIM[servo]);
}

struct rc_servo_s RC_Servo = {.SetPWMDutyCycle = SetPWMDutyCycle,
                              .SetMotorSpeed = SetMotorSpeed,
                              .SetServoAngle = SetServoAngle,
                              .Enable = Enable,
                              .Disable = Disable};

// void RC_Servo_SetAngle(uint8_t servo, uint32_t angle)
// {
//   //  angle 0 -> 1000 us
//   //  angle 180 -> 2000 us

//   uint32_t cc_value = 1000 + (angle*1000)/180;

//   switch (servo)
//   {
//     case  RC_SERVO_0:
//       LL_TIM_OC_SetCompareCH4(TIM4, cc_value);
//       LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
//       break;
//     case  RC_SERVO_1:
//       LL_TIM_OC_SetCompareCH3(TIM4, cc_value);
//       LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
//       break;
//     case  RC_SERVO_2:
//       LL_TIM_OC_SetCompareCH2(TIM4, cc_value);
//       LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
//       break;
//     case  RC_SERVO_3:
//       LL_TIM_OC_SetCompareCH1(TIM4, cc_value);
//       LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
//       break;
//     case  RC_SERVO_4:
//       LL_TIM_OC_SetCompareCH1(TIM5, cc_value);
//       LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
//       break;
//     case  RC_SERVO_5:
//       LL_TIM_OC_SetCompareCH2(TIM5, cc_value);
//       LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
//       break;
//     case  RC_SERVO_6:
//       LL_TIM_OC_SetCompareCH3(TIM5, cc_value);
//       LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
//       break;
//     case  RC_SERVO_7:
//       LL_TIM_OC_SetCompareCH4(TIM5, cc_value);
//       LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);
//       break;
//   }
// }




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
