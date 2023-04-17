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
#include "button.h"
#include "stm32f4xx_ll_gpio.h"



#define PRESSED_TIME_ms     4
#define HOLD_TIME_ms        1000


struct button_info_s{
    GPIO_TypeDef* gpio;
    uint32_t pin;
    enum button_polarity_e polarity;
    void (*ButtonCallback)(enum buttons_e button_id, enum button_event_e button_event);
    enum button_state_e state;
    uint32_t shift_reg;
    uint32_t press_counter_ms;
};

struct button_info_s button_info[BUTTONS_NUMBER];


static enum button_state_e GetState(enum buttons_e button_id)
{
    return button_info[button_id].state;
}

static void AddButton(enum buttons_e button_id, GPIO_TypeDef* button_gpio, uint32_t button_pin, enum button_polarity_e button_polarity, void (*ButtonCallback)(enum buttons_e button_id, enum button_event_e button_event))
{
    button_info[button_id].gpio = button_gpio;
    button_info[button_id].pin = button_pin;
    button_info[button_id].polarity = button_polarity;
    button_info[button_id].ButtonCallback = ButtonCallback;

    button_info[button_id].state = BUTTON_STATE_UNKNOWN;
    button_info[button_id].shift_reg = 0x55;
}

static void PollButtons(void)
{
    for(uint8_t ik=0; ik<BUTTONS_NUMBER; ik++)
    {
        uint32_t pin_state = LL_GPIO_IsInputPinSet(button_info[ik].gpio, button_info[ik].pin)^button_info[ik].polarity;

        /*  BUTTON BOUNCING FILTER  */
        button_info[ik].shift_reg = (button_info[ik].shift_reg<<1)|pin_state;

        /*  DETERMINE STATE    */
        switch (button_info[ik].state)
        {
            case (uint32_t)BUTTON_STATE_PRESSED:
                if((button_info[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == 0)
                {
                    button_info[ik].state = BUTTON_STATE_RELEASED;
                    if(button_info[ik].ButtonCallback != NULL)
                        button_info[ik].ButtonCallback((enum buttons_e)ik,BUTTON_EVENT_RELEASED);
                }
                break;
            case (uint32_t)BUTTON_STATE_RELEASED:
                if((button_info[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == (uint32_t)(0xFFFFFFFF>>(32-PRESSED_TIME_ms)))
                {
                    button_info[ik].state = BUTTON_STATE_PRESSED;
                    if(button_info[ik].ButtonCallback != NULL)
                        button_info[ik].ButtonCallback((enum buttons_e)ik,BUTTON_EVENT_PRESSED);
                }
                break;
            default:
                if((button_info[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == 0)
                {
                    button_info[ik].state = BUTTON_STATE_RELEASED;
                    if(button_info[ik].ButtonCallback != NULL)
                        button_info[ik].ButtonCallback((enum buttons_e)ik,BUTTON_EVENT_RELEASED);
                    break;
                }
                if((button_info[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == (uint32_t)(0xFFFFFFFF>>(32-PRESSED_TIME_ms)))
                {
                    button_info[ik].state = BUTTON_STATE_PRESSED;
                    if(button_info[ik].ButtonCallback != NULL)
                        button_info[ik].ButtonCallback((enum buttons_e)ik,BUTTON_EVENT_PRESSED);
                    break;
                }
                break;
        }


        /*  ACTION DETECTION    */
        switch(button_info[ik].state)
        {
            case BUTTON_STATE_PRESSED:
                if(button_info[ik].press_counter_ms==HOLD_TIME_ms)
                {
                    if(button_info[ik].ButtonCallback != NULL)
                        button_info[ik].ButtonCallback((enum buttons_e)ik,BUTTON_EVENT_HOLD);
                }
                if(button_info[ik].press_counter_ms<=HOLD_TIME_ms)
                    button_info[ik].press_counter_ms++;
                break;

            case BUTTON_STATE_RELEASED:
                if((button_info[ik].press_counter_ms)&&(button_info[ik].press_counter_ms<=HOLD_TIME_ms))
                {
                    if(button_info[ik].ButtonCallback != NULL)
                        button_info[ik].ButtonCallback((enum buttons_e)ik,BUTTON_EVENT_CLICK);
                }
                button_info[ik].press_counter_ms = 0;
                break;

            default:
                button_info[ik].press_counter_ms = 0;
                break;
        }
    }
}

struct button_s Button = {.AddButton = AddButton,
                            .GetState = GetState,
                            .PollButtons = PollButtons};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
