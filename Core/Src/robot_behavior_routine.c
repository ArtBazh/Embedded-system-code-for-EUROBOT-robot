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
#include "robot_behavior_routine.h" 
#include "gpio_robot_defines.h"
#include "bus_device_def.h"
#include "maxon.h" 
#include "flags.h"
#include "DC_Motor.h"

struct robot_ctrl_s robot_ctrl;


#if defined (ROBOT_GUIDO)

  	

#endif

#if defined ROBOT_BIG

  	

#endif

void MaxonsEnable(void)
{
	if(robot_ctrl.maxons_state != MAXONS_ENABLED)
		return;

	Maxon.Enable(MAXON_MOTORS[0].USART, MAXON_MOTORS[0].ID);
  	Maxon.Enable(MAXON_MOTORS[1].USART, MAXON_MOTORS[1].ID);
  	Maxon.Enable(MAXON_MOTORS[2].USART, MAXON_MOTORS[2].ID);
}

void MaxonsDisable(void)
{
	Maxon.Disable(MAXON_MOTORS[0].USART, MAXON_MOTORS[0].ID);
  	Maxon.Disable(MAXON_MOTORS[1].USART, MAXON_MOTORS[1].ID);
  	Maxon.Disable(MAXON_MOTORS[2].USART, MAXON_MOTORS[2].ID);
}

void MaxonsStop(void)
{
	Maxon.SetTPM(MAXON_MOTORS[0].USART, MAXON_MOTORS[0].ID, 0);
  	Maxon.SetTPM(MAXON_MOTORS[1].USART, MAXON_MOTORS[1].ID, 0);
  	Maxon.SetTPM(MAXON_MOTORS[2].USART, MAXON_MOTORS[2].ID, 0);
}


void StopAllActuators(void)
{
	MaxonsDisable();
}



void ControlButtons_Callback(enum buttons_e button_id, enum button_event_e button_event)
{
	switch (button_id)
	{
		case STARTING_CORD_BTN:
			if(button_event == BUTTON_EVENT_PRESSED)
				robot_ctrl.starting_cord = STARTING_CORD_ATTACHED;
			if(button_event == BUTTON_EVENT_RELEASED)
				robot_ctrl.starting_cord = STARTING_CORD_RELEASED;
			break;

		case PLAYGROUND_SIDE_BTN:
			if(button_event == BUTTON_EVENT_PRESSED)
				robot_ctrl.playground_side = RIGHT_SIDE;
			if(button_event == BUTTON_EVENT_RELEASED)
				robot_ctrl.playground_side = LEFT_SIDE;
			break;

		case STRATEGY_BTN:
			if(button_event == BUTTON_EVENT_CLICK)
			{
				if(robot_ctrl.strategy<STRATEGIES_NUMBER)
					robot_ctrl.strategy++;
				else
					robot_ctrl.strategy = (enum strategy_e)0;
			}
			break;

		case MAXON_ENABLE_BTN:
			if(button_event == BUTTON_EVENT_PRESSED){
				robot_ctrl.maxons_state = MAXONS_ENABLED;
				MaxonsEnable();
			}
			if(button_event == BUTTON_EVENT_RELEASED){
				robot_ctrl.maxons_state = MAXONS_DISABLED;
				MaxonsDisable();
			}
			break;

		case START_GAME:
			if(button_event == BUTTON_EVENT_PRESSED)
				*flag.start_game = 1;
			if(button_event == BUTTON_EVENT_RELEASED)
				*flag.start_game = 0;
			break;
		case UP_BTN:
			if(button_event == BUTTON_EVENT_PRESSED)
				driveDC(2, 0, 100);
			if(button_event == BUTTON_EVENT_RELEASED)
				driveDC(2, 0, 0);
			break;
		case DOWN_BTN:
			if(button_event == BUTTON_EVENT_PRESSED)
				driveDC(2, 1, 100);
			if(button_event == BUTTON_EVENT_RELEASED)
				driveDC(2, 1, 0);
			break;
		default:
			break;
	}
	// LL_GPIO_TogglePin(GPIOA, SERV7_GPIOA_PIN3);
}


// void Starting_Cord_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {

// }

// void Playground_Side_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {

// }

// void Strategy_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {
	
// }

// void Maxon_Enable_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {
// 	if(button_event == BUTTON_EVENT_PRESSED)
// 		MaxonsEnable();
// 	if(button_event == BUTTON_EVENT_RELEASED)
// 		MaxonsDisable();
// }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
