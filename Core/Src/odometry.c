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

#include "odometry.h" 
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "byte2word.h" 
#include "maxon.h" 

#include "kinematics.h"
#include "bus_device_def.h"




struct coord_s path;

static void Init(void)
{
	path.x = 0.0f;
	path.y = 0.0f;
	path.phi = 0.0f;
  Maxon.SetTotalTicks(MAXON_MOTORS[0].USART,MAXON_MOTORS[0].ID,0);
  Maxon.SetTotalTicks(MAXON_MOTORS[1].USART,MAXON_MOTORS[1].ID,0);
  Maxon.SetTotalTicks(MAXON_MOTORS[2].USART,MAXON_MOTORS[2].ID,0);
}

static void SetCoord(struct coord_s* coord)
{
  path.x = coord->x;
  path.y = coord->y;
  path.phi = coord->phi;
}

static void GetCoord(struct coord_s* coord)
{
  coord->x = path.x;
  coord->y = path.y;
  coord->phi = path.phi;
}

static void SetSpeed(struct speed_s* speed)
{
  int32_t tpm[3];

  Kinematics.Speed_To_TPM(speed,tpm);

  Maxon.SetTPM(MAXON_MOTORS[0].USART,MAXON_MOTORS[0].ID,tpm[0]);
  Maxon.SetTPM(MAXON_MOTORS[1].USART,MAXON_MOTORS[1].ID,tpm[1]);
  Maxon.SetTPM(MAXON_MOTORS[2].USART,MAXON_MOTORS[2].ID,tpm[2]);
}

static void GetSpeed(struct speed_s* speed)
{
  int32_t tpm[3];

  Maxon.GetTPM(MAXON_MOTORS[0].USART,MAXON_MOTORS[0].ID,tpm);
  Maxon.GetTPM(MAXON_MOTORS[1].USART,MAXON_MOTORS[1].ID,tpm+1);
  Maxon.GetTPM(MAXON_MOTORS[2].USART,MAXON_MOTORS[2].ID,tpm+2);

  Kinematics.TPM_To_Speed(tpm,speed);
}

static void UpdatePath(void)
{
  static int32_t ticks_old[3] = {0};

  int32_t ticks_new[3];
  int32_t d_ticks[3];
  uint8_t error = 0;

  error |= Maxon.GetTotalTicks(MAXON_MOTORS[0].USART,MAXON_MOTORS[0].ID,ticks_new);
  error |= Maxon.GetTotalTicks(MAXON_MOTORS[1].USART,MAXON_MOTORS[1].ID,ticks_new+1);
  error |= Maxon.GetTotalTicks(MAXON_MOTORS[2].USART,MAXON_MOTORS[2].ID,ticks_new+2);


  if(error)
    return;

  // LL_GPIO_TogglePin(GPIOA, SERV4_GPIOA_PIN0);

  d_ticks[0] = ticks_new[0] - ticks_old[0];
  d_ticks[1] = ticks_new[1] - ticks_old[1];
  d_ticks[2] = ticks_new[2] - ticks_old[2];

  Kinematics.UpdatePath(d_ticks,&path);

  memcpy(ticks_old,ticks_new,12);
}


struct odometry_s Odometry = {.Init = Init,
                              .SetCoord = SetCoord,
                              .GetCoord = GetCoord,
                              .SetSpeed = SetSpeed,
                              .GetSpeed = GetSpeed,
                              .UpdatePath = UpdatePath};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
