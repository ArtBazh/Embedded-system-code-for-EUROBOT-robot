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
#ifndef __USARTX_TIMEOUT_H
#define __USARTX_TIMEOUT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
void Set_Timeout(uint32_t delay_us, uint32_t channel, void (*Timeout_Callback)(void*), void* callback_param);
void Reset_Timeout(uint32_t channel);
void USARTx_Timeout_IRQ_Process(void);


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
