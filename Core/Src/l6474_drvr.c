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
#include "l6474_drvr.h" 
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "spi_gpio_defines.h"
#include "spi_gpio.h" 

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


uint32_t SPI_NRST_GPIO[MOTORS_NUMBER] = {SPI_L6474_NRST1,SPI_L6474_NRST2,SPI_L6474_NRST3,SPI_L6474_NRST4};
GPIO_TypeDef* SPI_NCS_GPIO[MOTORS_NUMBER] = {GPIOE,GPIOD,GPIOD,GPIOC};
uint32_t SPI_NCS_PIN[MOTORS_NUMBER] = {NCS1_GPIOE_PIN7,NCS2_GPIOD_PIN10,NCS3_GPIOD_PIN11,NCS4_GPIOC_PIN9};
GPIO_TypeDef* DIR_GPIO[MOTORS_NUMBER] = {GPIOE,GPIOE,GPIOE,GPIOE};
uint32_t DIR_PIN[MOTORS_NUMBER] = {DIR1_GPIOE_PIN8,DIR2_GPIOE_PIN10,DIR3_GPIOE_PIN12,DIR4_GPIOE_PIN15};


static void SetParam(enum L6474H_motor_s motor, uint8_t address, uint32_t data, uint8_t length)
{
	taskENTER_CRITICAL();

	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_SetParam_CMD|(address&0x1F));
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	while(length>0){
		LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
		length--;
		LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
		
		LL_SPI_TransmitData8(SPI1, (data>>(length*8))&0xFF);
		while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	}
	
	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	taskEXIT_CRITICAL();
}

static uint32_t GetParam(enum L6474H_motor_s motor, uint8_t address, uint8_t length)
{
	uint32_t temp;
	
	temp = 0;

	taskENTER_CRITICAL();
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_GetParam_CMD|(address&0x1F));
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	while(LL_SPI_IsActiveFlag_RXNE(SPI1)!=0)
		LL_SPI_ReceiveData8(SPI1);
	
	while(length>0){
		LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
		length--;
		LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
		
		LL_SPI_TransmitData8(SPI1, L6474_NOP_CMD);
		while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
		
		temp |= ((uint32_t)LL_SPI_ReceiveData8(SPI1))<<(length*8);
	}

	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	taskEXIT_CRITICAL();
	
	return temp;
}

static uint32_t GetStatus(enum L6474H_motor_s motor)
{
	uint32_t temp;

	taskENTER_CRITICAL();
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_GetStatus_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	while(LL_SPI_IsActiveFlag_RXNE(SPI1)!=0)
		LL_SPI_ReceiveData8(SPI1);
	
	
	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_NOP_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	temp = ((uint32_t)LL_SPI_ReceiveData8(SPI1))<<8;
	
	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_NOP_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	
	temp |= LL_SPI_ReceiveData8(SPI1);
	
	
	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	taskEXIT_CRITICAL();
	
	return temp;
}

static void Enable(enum L6474H_motor_s motor)
{
	taskENTER_CRITICAL();

	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_ENABLE_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	taskEXIT_CRITICAL();
}

static void Disable(enum L6474H_motor_s motor)
{
	taskENTER_CRITICAL();

	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_DISABLE_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	LL_GPIO_SetOutputPin(SPI_NCS_GPIO[motor], SPI_NCS_PIN[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	taskEXIT_CRITICAL();
}

static void SetStepMode(enum L6474H_motor_s motor, uint32_t step_mode)
{
	SetParam(motor, L6474_STEP_MODE, (step_mode&L6474_STEP_SEL_MASK)|0x88, L6474_STEP_MODE_LENGTH);
}

static void SetCurrentLimit(enum L6474H_motor_s motor, uint32_t current_limit)
{
	SetParam(motor, L6474_TVAL, current_limit&L6474_TVAL_MASK,1);
}

static void SetCurrentLimit_mA(enum L6474H_motor_s motor, float current_limit_mA)
{
	uint8_t temp;
	if(current_limit_mA>=31.25f){
		temp = (uint8_t)((current_limit_mA - 31.25f)/31.25f);
	}else{
		temp = 0;
	}
	SetParam(motor, L6474_TVAL, temp&L6474_TVAL_MASK, L6474_TVAL_LENGTH);
}

static void SetDirection(enum L6474H_motor_s motor, enum L6474H_motor_dir_s direction)
{
	if(direction)
		LL_GPIO_SetOutputPin(DIR_GPIO[motor], DIR_PIN[motor]);
	else
		LL_GPIO_ResetOutputPin(DIR_GPIO[motor], DIR_PIN[motor]);
}

static void HardReset(enum L6474H_motor_s motor)
{
	SPI_GPIO_ResetOutputPin(SPI_NRST_GPIO[motor]);
	vTaskDelay(10);
	SPI_GPIO_SetOutputPin(SPI_NRST_GPIO[motor]);
	vTaskDelay(10);
}

struct L6474H_s L6474H = {.SetParam = SetParam,
							.GetParam = GetParam,
							.GetStatus = GetStatus,
							.Enable = Enable,
							.Disable = Disable,
							.SetStepMode = SetStepMode,
							.SetCurrentLimit = SetCurrentLimit,
							.SetCurrentLimit_mA = SetCurrentLimit_mA,
							.SetDirection = SetDirection,
							.HardReset = HardReset};
							
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
