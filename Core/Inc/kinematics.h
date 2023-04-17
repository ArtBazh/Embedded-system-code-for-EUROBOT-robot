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
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
	 
struct speed_s{
  float dx;
  float dy;
  float dphi;
};

struct coord_s{
  float x;
  float y;
  float phi;
};


struct kinematics_s{
    void (*MatrixInit)(void);
    void (*Speed_To_TPM)(struct speed_s* speed, int32_t* tpm);
    void (*TPM_To_Speed)(int32_t* tpm, struct speed_s* speed);
    void (*UpdatePath)(int32_t* d_ticks, struct coord_s* path);
};

extern struct kinematics_s Kinematics;


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
