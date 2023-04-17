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
#include "stepper.h" 
#include "l6474_drvr.h"
#include "stm32f4xx_ll_tim.h"
#include "gpio_board_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "spi_gpio_defines.h"



uint32_t LL_TIM_CHANNEL[MOTORS_NUMBER] = {LL_TIM_CHANNEL_CH1,LL_TIM_CHANNEL_CH2,LL_TIM_CHANNEL_CH3,LL_TIM_CHANNEL_CH4};

int32_t goal_position[MOTORS_NUMBER];
int32_t present_position[MOTORS_NUMBER];


static uint32_t TIM_IsActiveFlag_CC(TIM_TypeDef *TIMx, uint32_t motor)
{
	return ((READ_BIT(TIMx->SR, (1<<(motor+1))) == (1<<(motor+1))) ? 1UL : 0UL);
}

static void TIM_ClearFlag_CC(TIM_TypeDef *TIMx, uint32_t motor)
{
	WRITE_REG(TIMx->SR, ~ (1<<(motor+1)));
}

static void TIM_DisableIT_CC(TIM_TypeDef *TIMx, uint32_t motor)
{
  	CLEAR_BIT(TIMx->DIER, (1<<(motor+1)));
}

static void TIM_EnableIT_CC(TIM_TypeDef *TIMx, uint32_t motor)
{
  	SET_BIT(TIMx->DIER, (1<<(motor+1)));
}

static uint32_t TIM_CC_IsEnabledChannel(TIM_TypeDef *TIMx, uint32_t motor)
{
  	return ((READ_BIT(TIMx->CCER, LL_TIM_CHANNEL[motor]) == (LL_TIM_CHANNEL[motor])) ? 1UL : 0UL);
}

static void TIM_CC_DisableChannel(TIM_TypeDef *TIMx, uint32_t motor)
{
  	CLEAR_BIT(TIMx->CCER, LL_TIM_CHANNEL[motor]);
}

static void TIM_CC_EnableChannel(TIM_TypeDef *TIMx, uint32_t motor)
{
  	SET_BIT(TIMx->CCER, LL_TIM_CHANNEL[motor]);
}

void TIM1_Steppers_PWM_CC_IRQ_Process(void)
{
	for(uint8_t motor_index=0; motor_index<MOTORS_NUMBER; motor_index++)
	{
		if(TIM_IsActiveFlag_CC(TIM1,motor_index))
		{
			TIM_ClearFlag_CC(TIM1,motor_index);

			if(TIM_CC_IsEnabledChannel(TIM1, motor_index))
			{
				
				if(goal_position[motor_index]>present_position[motor_index]){
					goal_position[motor_index]--;
					continue;
				}

				if(goal_position[motor_index]<present_position[motor_index]){
					goal_position[motor_index]++;
					continue;
				}

				if(goal_position[motor_index]==present_position[motor_index]){
					TIM_CC_DisableChannel(TIM1, motor_index);
					LL_TIM_GenerateEvent_COM(TIM1);
					TIM_DisableIT_CC(TIM1,motor_index);
				}
			}
		}
	}
}

static void SetZeroPosisition(enum L6474H_motor_s motor)
{
	present_position[motor] = 0;
	goal_position[motor] = 0;
}

/*  Step on N Steps  */
static void StepToPosisition(enum L6474H_motor_s motor, int32_t position)
{
	if(present_position[motor]==position)
		return;

	goal_position[motor] = position;

	if(goal_position[motor]>present_position[motor])
		L6474H.SetDirection(motor,DIRECTION_FORWARD);

	if(goal_position[motor]<present_position[motor])
		L6474H.SetDirection(motor,DIRECTION_BACKWARD);

	TIM_CC_EnableChannel(TIM1, motor);
	LL_TIM_GenerateEvent_COM(TIM1);
	TIM_EnableIT_CC(TIM1, motor);
}

static int32_t GetPresentPosisition(enum L6474H_motor_s motor)
{
	return present_position[motor];
}

static void Enable(enum L6474H_motor_s motor)
{
	L6474H.Enable(motor);
}

static void Disable(enum L6474H_motor_s motor)
{
	L6474H.Disable(motor);
}

/*  Stop Stepper Motors  */
static void Stop(enum L6474H_motor_s motor)
{
	TIM_CC_DisableChannel(TIM1, motor);
	LL_TIM_GenerateEvent_COM(TIM1);
	TIM_DisableIT_CC(TIM1, motor);
}

static void Motor1_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_1);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_1, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x00<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_1, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_1, L6474_STEP_MODE_8);

	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_1, 2800);
		
    L6474H.SetDirection(STEPPER_MOTOR_1, DIRECTION_FORWARD);
}

static void Motor2_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_2);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_2, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x01<<L6474_POW_SR_POS)
					|(0x01<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_2, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_2, L6474_STEP_MODE_8);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_2, 2000);
		
    L6474H.SetDirection(STEPPER_MOTOR_2, DIRECTION_FORWARD);
}

static void Motor3_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_3);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_3, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x01<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_3, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_3, L6474_STEP_MODE_16);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_3, 800);
		
    L6474H.SetDirection(STEPPER_MOTOR_3, DIRECTION_FORWARD);
}

static void Motor4_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_4);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_4, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x01<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_4, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_4, L6474_STEP_MODE_16);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_4, 800);
		
    L6474H.SetDirection(STEPPER_MOTOR_4, DIRECTION_FORWARD);
}

static void (*MotorInit[MOTORS_NUMBER])() = {Motor1_Init, Motor2_Init, Motor3_Init, Motor4_Init};

static void Init(enum L6474H_motor_s motor)
{
	MotorInit[motor]();
}

struct stepper_s Stepper = {.SetZeroPosisition = SetZeroPosisition,
							.StepToPosisition = StepToPosisition,
							.GetPresentPosisition = GetPresentPosisition,
							.Stop = Stop,
							.Enable = Enable,
							.Disable = Disable,
							.Init = Init};

// switch (MOTORx)
// 	{
// 		case STEPPER_MOTOR_1:
// 			Motor1_Init();
// 			break;
// 		case STEPPER_MOTOR_2:
// 			Motor2_Init();
// 			break;
// 		case STEPPER_MOTOR_3:
// 			Motor3_Init();
// 			break;
// 		case STEPPER_MOTOR_4:
// 			Motor4_Init();
// 			break;
// 	}

// if(MOTORx&STEPPER_MOTOR_1){
		
		
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_2){
		
		
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_3){
		
		
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_4){
		
		
// 	}

// if(MOTORx&STEPPER_MOTOR_1){
// 		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4); //1<<12 3
//     	LL_TIM_DisableIT_CC4(TIM1); //1<<4 
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_2){
// 		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3); //1<<8 2 
//     	LL_TIM_DisableIT_CC3(TIM1); //1<<3 CLEAR_BIT(TIMx->DIER, TIM_DIER_CC3IE);
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_3){
// 		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2); //1<<4 1
//     	LL_TIM_DisableIT_CC2(TIM1); //1<<2
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_4){
// 		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1); //1<<0 0
//     	LL_TIM_DisableIT_CC1(TIM1); //1<<1
// 	}

// if(MOTORx&STEPPER_MOTOR_1){
		
// 		if(steps>0){
// 			L6474_SetDirection(STEPPER_MOTOR_1,DIRECTION_FORWARD);
// 			steppers_struct.dir[STEPPER_MOTOR_1] = DIRECTION_FORWARD;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_1] = steps;
//     	}
    
// 		if(steps<0){
// 			L6474_SetDirection(STEPPER_MOTOR_1,DIRECTION_REVERSE);
// 			steppers_struct.dir[STEPPER_MOTOR_1] = DIRECTION_REVERSE;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_1] = -steps;
// 		}

// 		if(steps!=0){
// 			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
// 			LL_TIM_EnableIT_CC4(TIM1);
// 		}
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_2){
		
// 		if(steps>0){
// 			L6474_SetDirection(STEPPER_MOTOR_2,DIRECTION_FORWARD);
// 			steppers_struct.dir[STEPPER_MOTOR_2] = DIRECTION_FORWARD;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_2] = steps;
// 		}
    
// 		if(steps<0){
// 			L6474_SetDirection(STEPPER_MOTOR_2,DIRECTION_REVERSE);
// 			steppers_struct.dir[STEPPER_MOTOR_2] = DIRECTION_REVERSE;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_2] = -steps;
// 		}

// 		if(steps!=0){
// 			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
// 			LL_TIM_EnableIT_CC3(TIM1);
// 		}
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_3){
		
// 		if(steps>0){
// 			L6474_SetDirection(STEPPER_MOTOR_3,DIRECTION_FORWARD);
// 			steppers_struct.dir[STEPPER_MOTOR_3] = DIRECTION_FORWARD;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_3] = steps;
// 		}
    
//     	if(steps<0){
// 			L6474_SetDirection(STEPPER_MOTOR_3,DIRECTION_REVERSE);
// 			steppers_struct.dir[STEPPER_MOTOR_3] = DIRECTION_REVERSE;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_3] = -steps;
//     	}

// 		if(steps!=0){
// 			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
// 			LL_TIM_EnableIT_CC2(TIM1);
// 		}
		
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_4){
		
// 		if(steps>0){
// 			L6474_SetDirection(STEPPER_MOTOR_4,DIRECTION_FORWARD);
// 			steppers_struct.dir[STEPPER_MOTOR_4] = DIRECTION_FORWARD;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_4] = steps;
// 		}
    
// 		if(steps<0){
// 			L6474_SetDirection(STEPPER_MOTOR_4,DIRECTION_REVERSE);
// 			steppers_struct.dir[STEPPER_MOTOR_4] = DIRECTION_REVERSE;
// 			steppers_struct.rel_pos[STEPPER_MOTOR_4] = -steps;
// 		}

// 		if(steps!=0){
// 			LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
// 			LL_TIM_EnableIT_CC1(TIM1); //SET_BIT(TIMx->DIER, TIM_DIER_CC1IE);
// 		}
// 	}

// if(MOTORx&STEPPER_MOTOR_1){
// 		steppers_struct.abs_pos[STEPPER_MOTOR_1] = 0;
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_2){
// 		steppers_struct.abs_pos[STEPPER_MOTOR_2] = 0;
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_3){
// 		steppers_struct.abs_pos[STEPPER_MOTOR_3] = 0;
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_4){
// 		steppers_struct.abs_pos[STEPPER_MOTOR_4] = 0;
// 	}

// 	if(MOTORx&STEPPER_MOTOR_1){
		
// 		switch (direction)
// 		{
// 		case DIRECTION_FORWARD:
// 			LL_GPIO_SetOutputPin(GPIOE, DIR1_GPIOE_PIN15);
// 			steppers_struct.dir[STEPPER_MOTOR_1] = DIRECTION_FORWARD;
// 			break;
// 		case DIRECTION_REVERSE:
// 			LL_GPIO_ResetOutputPin(GPIOE, DIR1_GPIOE_PIN15);
// 			steppers_struct.dir[STEPPER_MOTOR_1] = DIRECTION_REVERSE;
// 			break;
// 		default:
// 			return;
// 			break;
// 		}
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_2){
		
// 		switch (direction)
// 		{
// 		case DIRECTION_FORWARD:
// 			LL_GPIO_SetOutputPin(GPIOE, DIR2_GPIOE_PIN12);
// 			steppers_struct.dir[STEPPER_MOTOR_2] = DIRECTION_FORWARD;
// 			break;
// 		case DIRECTION_REVERSE:
// 			LL_GPIO_ResetOutputPin(GPIOE, DIR2_GPIOE_PIN12);
// 			steppers_struct.dir[STEPPER_MOTOR_2] = DIRECTION_REVERSE;
// 			break;
// 		default:
// 			return;
// 			break;
// 		}
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_3){
		
// 		switch (direction)
// 		{
// 		case DIRECTION_FORWARD:
// 			LL_GPIO_SetOutputPin(GPIOE, DIR3_GPIOE_PIN10);
// 			steppers_struct.dir[STEPPER_MOTOR_3] = DIRECTION_FORWARD;
// 			break;
// 		case DIRECTION_REVERSE:
// 			LL_GPIO_ResetOutputPin(GPIOE, DIR3_GPIOE_PIN10);
// 			steppers_struct.dir[STEPPER_MOTOR_3] = DIRECTION_REVERSE;
// 			break;
// 		default:
// 			return;
// 			break;
// 		}
// 	}
	
// 	if(MOTORx&STEPPER_MOTOR_4){
		
// 		switch (direction)
// 		{
// 		case DIRECTION_FORWARD:
// 			LL_GPIO_SetOutputPin(GPIOE, DIR4_GPIOE_PIN8);
// 			steppers_struct.dir[STEPPER_MOTOR_4] = DIRECTION_FORWARD;
// 			break;
// 		case DIRECTION_REVERSE:
// 			LL_GPIO_ResetOutputPin(GPIOE, DIR4_GPIOE_PIN8);
// 			steppers_struct.dir[STEPPER_MOTOR_4] = DIRECTION_REVERSE;
// 			break;
// 		default:
// 			return;
// 			break;
// 		}
// 	}
	
// }

//   	if(LL_TIM_IsActiveFlag_CC1(TIM1)){
// 		LL_TIM_ClearFlag_CC1(TIM1);

// 		if(LL_TIM_CC_IsEnabledChannel(TIM1, LL_TIM_CHANNEL_CH1)){

// 		steppers_struct.rel_pos[STEPPER_MOTOR_4]--;

// 		if(steppers_struct.rel_pos[STEPPER_MOTOR_4]==0){
// 			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
// 			LL_TIM_DisableIT_CC1(TIM1);
// 		}

// 		if(steppers_struct.dir[STEPPER_MOTOR_4] == DIRECTION_FORWARD)
// 			steppers_struct.abs_pos[STEPPER_MOTOR_4] ++;
// 		else
// 			steppers_struct.abs_pos[STEPPER_MOTOR_4] --;

// 		}
// 	}

//   	if(LL_TIM_IsActiveFlag_CC2(TIM1)){
// 		LL_TIM_ClearFlag_CC2(TIM1);

// 		if(LL_TIM_CC_IsEnabledChannel(TIM1, LL_TIM_CHANNEL_CH2)){

// 		steppers_struct.rel_pos[STEPPER_MOTOR_3]--;

// 		if(steppers_struct.rel_pos[STEPPER_MOTOR_3]==0){
// 			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
// 			LL_TIM_DisableIT_CC2(TIM1);
// 		}

// 		if(steppers_struct.dir[STEPPER_MOTOR_3] == DIRECTION_FORWARD)
// 			steppers_struct.abs_pos[STEPPER_MOTOR_3] ++;
// 		else
// 			steppers_struct.abs_pos[STEPPER_MOTOR_3] --;

// 		}
// 	}

//   	if(LL_TIM_IsActiveFlag_CC3(TIM1)){ //1<<3
// 		LL_TIM_ClearFlag_CC3(TIM1);

// 		if(LL_TIM_CC_IsEnabledChannel(TIM1, LL_TIM_CHANNEL_CH3)){

// 		steppers_struct.rel_pos[STEPPER_MOTOR_2]--;

// 		if(steppers_struct.rel_pos[STEPPER_MOTOR_2]==0){
// 			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
// 			LL_TIM_DisableIT_CC3(TIM1);
// 		}

// 		if(steppers_struct.dir[STEPPER_MOTOR_2] == DIRECTION_FORWARD)
// 			steppers_struct.abs_pos[STEPPER_MOTOR_2] ++;
// 		else
// 			steppers_struct.abs_pos[STEPPER_MOTOR_2] --;

// 		}
// 	}

//   	if(LL_TIM_IsActiveFlag_CC4(TIM1)){ //1<<4
// 		LL_TIM_ClearFlag_CC4(TIM1);

// 		if(LL_TIM_CC_IsEnabledChannel(TIM1, LL_TIM_CHANNEL_CH4)){

// 		steppers_struct.rel_pos[STEPPER_MOTOR_1]--;

// 		if(steppers_struct.rel_pos[STEPPER_MOTOR_1]==0){
// 			LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);
// 			LL_TIM_DisableIT_CC4(TIM1);
// 		}

// 		if(steppers_struct.dir[STEPPER_MOTOR_1] == DIRECTION_FORWARD)
// 			steppers_struct.abs_pos[STEPPER_MOTOR_1] ++;
// 		else
// 			steppers_struct.abs_pos[STEPPER_MOTOR_1] --;

// 		}
// 	}

// }





/*  Set Speed in Steps per Second  */
// void StepperMotors_SetSpeed(uint32_t speed_sps)
// {
//   uint32_t tim1_arr;

//   if(speed_sps>TIM1_STEPPER_AUTORELOAD_VALUE)
//     speed_sps = TIM1_STEPPER_AUTORELOAD_VALUE;

//   tim1_arr = TIM1_STEPPER_AUTORELOAD_VALUE - speed_sps;

//   LL_TIM_DisableCounter(TIM1);

//   LL_TIM_SetAutoReload(TIM1, tim1_arr);

//   LL_TIM_OC_SetCompareCH4(TIM1, (tim1_arr>>1));
// 	LL_TIM_OC_SetCompareCH3(TIM1, (tim1_arr>>1));
// 	LL_TIM_OC_SetCompareCH2(TIM1, (tim1_arr>>1));
// 	LL_TIM_OC_SetCompareCH1(TIM1, (tim1_arr>>1));

//   LL_TIM_EnableCounter(TIM1);
// }









/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
