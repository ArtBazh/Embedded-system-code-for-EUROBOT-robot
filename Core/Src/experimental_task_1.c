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

#include "experimental_task_1.h" 
#include "dynamixel_protocol10.h"
#include "stm32f4xx_ll_gpio.h"
#include "gpio_board_defines.h"
#include "gpio_robot_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "stm32f4xx_ll_tim.h"
#include "terminal.h"
#include "h_bridge.h"
#include "task_DC.h" 
#include "serial_protocol.h"


#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
  // vTaskSuspend(TaskHandle);

	const TickType_t xDelay = 500;
	struct Dynamixel_Protocol10_Response_s resp, resp2, resp3, resp4;
	uint8_t data[8];
	int32_t rpm; // = Float_To_Fixed(3000.0f);
	Word32_To_Byte(&rpm,data);

  TickType_t xLastWakeTime;
  
  // data[0] = 0x25;
  // Dynamixel_Protocol10.Write(UART4,BROADCAST_ID,MAXON_ID,data,1,&resp2); 
  data[0] = 1;
  // Dynamixel_Protocol10.Write(UART4,0x25,MAXON_Enable,data,1,&resp2); 
  int32_t tt = 0;
  Word32_To_Byte(&tt,data);
  // Dynamixel_Protocol10.Write(UART4,0x25,MAXON_TotalTicks,data,4,&resp2); 
  uint32_t low = 0;
  uint32_t high = 10000*4096;
  Word32_To_Byte(&low,data);
  Word32_To_Byte(&high,data+4);
  // Dynamixel_Protocol10.Write(UART4,0x25,MAXON_RangeTPM,data,8,&resp2); 
  int32_t tpm = 3000*4096;
  Word32_To_Byte(&tpm,data);
  // Dynamixel10.Write(UART4,BROADCAST_ID,MAXON_ID,data,1,&resp2); 
    int32_t speed = 1000;
    speed *= 4096;
  // Maxon.SetID(UART4,0x01,0x23);
  vTaskDelay(100);
  // Maxon.SetRangeTPM(UART4,0x23,0,10000*4096);
  vTaskDelay(100);

  // Maxon.SetID(USART6,BROADCAST_ID,0x23);
  // vTaskDelay(100);
  // Maxon.SetRangeTPM(USART6,0x23,0,10000*4096);
  // vTaskDelay(100);
  // Maxon.Enable(USART6,0x23);
  // Maxon.SetTPM(USART6,0x23,speed);
	// vTaskSuspend(TaskHandle);


  // Maxon.Enable(USART6,0x21);
  // Maxon.Enable(USART6,0x22);
  // Maxon.Enable(USART6,0x23);
  // Maxon.SetTPM(USART6,0x21,speed);
  // Maxon.SetTPM(USART6,0x22,speed);
  // Maxon.SetTPM(USART6,0x23,speed);

  // Maxon.Disable(USART6,0x21);
  // Maxon.Disable(USART6,0x22);
  // Maxon.Disable(USART6,0x23);

  // Stepper.Init(STEPPER_MOTOR_1);
  // Stepper.Enable(STEPPER_MOTOR_1);
  // Stepper.SetZeroPosisition(STEPPER_MOTOR_1);
  // Stepper.StepToPosisition(STEPPER_MOTOR_1,4000);
  

  // if(LL_TIM_CC_IsEnabledChannel(TIM1, LL_TIM_CHANNEL_CH3))
  //   LL_GPIO_SetOutputPin(GPIOA, SERV4_GPIOA_PIN0|SERV5_GPIOA_PIN1|SERV6_GPIOA_PIN2|SERV7_GPIOA_PIN3);
// while(1){
//   while(LL_TIM_IsActiveFlag_CC3(TIM1)==0);
//   LL_TIM_ClearFlag_CC3(TIM1);
//   LL_GPIO_TogglePin(GPIOA, SERV4_GPIOA_PIN0|SERV5_GPIOA_PIN1|SERV6_GPIOA_PIN2|SERV7_GPIOA_PIN3);
// }
  // Dynamixel_AX.EnableLED(USART3,0x03);

  // Dynamixel_AX.SetID(USART3,BROADCAST_ID,0x10);
// vTaskDelay(200);

  // Dynamixel_AX.TorqueEnable(USART3,0x02);
  // Dynamixel_AX.SetAngle(USART3,0x02,0);
  

  // RC_Servo.Enable(RC_SERVO_5);
  // RC_Servo.SetServoAngle(RC_SERVO_5,500,2500,180,180);
  // RC_Servo.Disable(RC_SERVO_5);
  LL_GPIO_SetOutputPin(GPIOA, SERV4_GPIOA_PIN0);

  // RC_Servo.Enable(RC_SERVO_1);
  // RC_Servo.SetServoAngle(RC_SERVO_1,1100,1900,120,60);
  // RC_Servo.Disable(RC_SERVO_4);

  // Dynamixel_AX.SetAngle(USART3,0x10,200);

  H_Bridge.Enable(H_BRIDGE1);
  H_Bridge.SetMotorSpeed(H_BRIDGE1, 60);

  H_Bridge.Enable(H_BRIDGE2);
  H_Bridge.SetMotorSpeed(H_BRIDGE2, -40);
  
	uint8_t encoder_number = 1;
	while(1)
	{

	vTaskResume(DC_Task.GetTaskHandle());
	//Serial.WriteBytes(UART5,&encoder_number,1);
	vTaskDelay(100);
	
	// Dynamixel_AX.EnableLED(USART3,0x10);
  // vTaskDelay(1000);
  // Dynamixel_AX.DisableLED(USART3,0x10);
  // vTaskDelay(1000);
    // if(LL_GPIO_IsInputPinSet(GPIOE, PLAYGROUND_SIDE_BTN_GPIO13_PE6))
    //   LL_GPIO_SetOutputPin(GPIOA, SERV7_GPIOA_PIN3);
    // else
    //   LL_GPIO_ResetOutputPin(GPIOA, SERV7_GPIOA_PIN3);
    // LL_GPIO_TogglePin(GPIOA, SERV4_GPIOA_PIN0);
    // LL_GPIO_SetOutputPin(GPIOD, SERV0_GPIOD_PIN15|SERV1_GPIOD_PIN14|SERV2_GPIOD_PIN13|SERV3_GPIOD_PIN12);
    // LL_GPIO_SetOutputPin(GPIOD, SERV0_GPIOD_PIN15|SERV1_GPIOD_PIN14|SERV2_GPIOD_PIN13|SERV3_GPIOD_PIN12);
    // LL_GPIO_SetOutputPin(GPIOA, SERV4_GPIOA_PIN0|SERV5_GPIOA_PIN1|SERV6_GPIOA_PIN2|SERV7_GPIOA_PIN3);
    // xLastWakeTime = xTaskGetTickCount ();
		// LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
    // taskENTER_CRITICAL();
    // __disable_irq();
    // Dynamixel10.Write(UART4,0x25,MAXON_TPM,data,4,&resp2); 
    // Dynamixel_Protocol10.Read(UART4,0x25,MAXON_TPM,4,&resp3); 
    // while(resp3.data_ready_flag==0);
    // int32_t tpm;
    // Byte_To_Word32(resp3.data,&tpm);
    // tpm /= 4096;
    // // if(tpm==2995)
    // //   LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);

    // Dynamixel_Protocol10.Read(UART4,0x25,MAXON_TotalTicks,4,&resp3);
    // while(resp3.data_ready_flag==0);
    // Byte_To_Word32(resp3.data,&tpm);
    // // if(tpm>28*4096)
    // //   LL_GPIO_SetOutputPin(GPIOD, SERV0_GPIOD_PIN15);
    // // if(tpm<-28*4096)
    // //   LL_GPIO_ResetOutputPin(GPIOD, SERV0_GPIOD_PIN15);
    // Maxon.Ping(UART4,0x01);

    // Dynamixel_Protocol10.Read(UART4,0x25,MAXON_TPM,8,&resp4);
    // while(resp4.data_ready_flag==0);
    // Byte_To_Word32(resp4.data,&tpm);
    // Byte_To_Word32(resp4.data+4,&tt);
    // tpm /= 4096;
    // if(tpm>100)
    //   LL_GPIO_SetOutputPin(GPIOD, SERV0_GPIOD_PIN15);
    // if(tpm<-100)
    //   LL_GPIO_ResetOutputPin(GPIOD, SERV0_GPIOD_PIN15); //SERV2_GPIOD_PIN13 SERV1_GPIOD_PIN14

    // if(tt>-40000)
    //   LL_GPIO_SetOutputPin(GPIOD, SERV3_GPIOD_PIN12);
    // if(tt<-50000)
    //   LL_GPIO_ResetOutputPin(GPIOD, SERV3_GPIOD_PIN12);

    
    // Word32_To_Byte(&speed,data);
    // Dynamixel_Protocol10.Write(UART4,0x25,MAXON_TPM,data,4,&resp2); 

    // if(Maxon.Ping(UART4,0x25)==0)
    //   LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
    int32_t TPM, ticks0, ticks1, ticks2;
    uint32_t bot, top;
    // if(Maxon.GetRangeTPM(UART4,0x25,&bot, &top)==0)
    //   LL_GPIO_TogglePin(GPIOD, SERV3_GPIOD_PIN12);
    // if(Maxon.GetTPM(UART4,0x25,&TPM)==0)
    //   LL_GPIO_TogglePin(GPIOD, SERV3_GPIOD_PIN12);

    //if((Maxon.GetTotalTicks(UART4,0x23,&ticks0)==0)&&(Maxon.GetTotalTicks(UART4,0x24,&ticks1)==0)&&(Maxon.GetTotalTicks(UART4,0x25,&ticks2)==0))
      //LL_GPIO_TogglePin(GPIOD, SERV3_GPIOD_PIN12);

    // if(bot==0)
    //   LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
    // TPM /= 4096;
    // if((TPM>-2990)&&(TPM<-2980))
    //   LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);

    // Maxon.SetTPM(UART4,0x25,speed);

    // Maxon.Enable(USART6,0x25);
    // Maxon.SetTPM(USART6,0x25,speed);
    // Maxon.SetTPM(USART6,0x25,speed);
    // Dynamixel_AX.Ping(USART3,0x02);
    // speed *= (-1);
		// Dynamixel10.Write(USART6,0x25,MAXON_RPM_Q19_12,data,4,&resp); 
    // AX12A.SetAngle(USART3,0x02,160);
    // __enable_irq();
    // taskEXIT_CRITICAL();
    // vTaskDelay(1000/ODOMETRY_UPDATE_RATE_HZ);
    // vTaskDelayUntil(&xLastWakeTime,xFrequency);
    // taskYIELD();
    // int32_t pos = Stepper.GetPresentPosisition(STEPPER_MOTOR_3);
    // Stepper.StepToPosisition(STEPPER_MOTOR_3,Stepper.GetPresentPosisition(STEPPER_MOTOR_3)-100);
    // vTaskDelay(6000);
    // Maxon.Enable(USART6,0x25);
    // Maxon.SetTPM(USART6,0x25,-speed);
    // vTaskDelay(6000);
    // Stepper.Stop(STEPPER_MOTOR_3); speed/2
    // Stepper.Disable(STEPPER_MOTOR_3);
    // Stepper.StepToPosisition(STEPPER_MOTOR_3,-100);
    // vTaskDelay(1000);

    // struct terminal_response_s terminal_response;

    // terminal_response.cmd = 0x03;
    // terminal_response.data_length = 0;

    // Terminal.Send_Response(&terminal_response);
    taskYIELD();
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "Experimental_Task_1", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


struct experimental_task_1_s Experimental_Task_1 = {.CreateTask = CreateTask,
                                                    .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
