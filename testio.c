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
//#include "functions.h"
//#include "irq_process.h" 
//#include "board_hw_control.h" 
//#include "l6474_drvr.h" 

typedef struct
{                
  int32_t (* Init)    (void* pConf); /**< Pointer to HCI TL function for the IO Bus initialization */
  int32_t (* DeInit)  (void); /**< Pointer to HCI TL function for the IO Bus de-initialization */  
  int32_t (* Reset)   (void); /**< Pointer to HCI TL function for the IO Bus reset */    
  int32_t (* Receive) (uint8_t*, uint16_t); /**< Pointer to HCI TL function for the IO Bus data reception */
  int32_t (* Send)    (uint8_t*, uint16_t); /**< Pointer to HCI TL function for the IO Bus data transmission */
  int32_t (* DataAck) (uint8_t*, uint16_t* len); /**< Pointer to HCI TL function for the IO Bus data ack reception */	
  int32_t (* GetTick) (void); /**< Pointer to BSP function for getting the HAL time base timestamp */    
} tHciIO;

int32_t HCI_TL_SPI_Init(void* pConf)
{
	
}

void hci_register_io_bus(tHciIO* fops)
{
  /* Register bus function */
  hciContext.io.Init    = fops->Init; 
  hciContext.io.Receive = fops->Receive;  
  hciContext.io.Send    = fops->Send;
  hciContext.io.GetTick = fops->GetTick;
  hciContext.io.Reset   = fops->Reset;    
}

void hci_tl_lowlevel_init()
{
	tHciIO fops;
	
	/* Register IO bus services */
  fops.Init    = HCI_TL_SPI_Init;
  fops.DeInit  = HCI_TL_SPI_DeInit;
  fops.Send    = HCI_TL_SPI_Send;
  fops.Receive = HCI_TL_SPI_Receive;
  fops.Reset   = HCI_TL_SPI_Reset;
  fops.GetTick = BSP_GetTick;
  
  hci_register_io_bus (&fops);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
