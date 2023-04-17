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

#include "terminal_cmd_handler.h" 
#include "dynamixel_protocol10.h"
#include "byte2word.h"

#include "gpio_board_defines.h"
#include "spi_gpio.h"
#include "spi_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "terminal.h"

#include "dynamixel_ax.h" 
#include "dynamixel_mx.h" 
#include "stm32f4xx_ll_usart.h"
#include "odometry_task.h"
#include "kinematics.h"
#include "odometry.h" 

#include "grub_pile_task.h"
#include "release_pile_task.h"
#include "stepper.h"

#include "flags.h"
#include "rc_servo.h"
#include "l6474_drvr.h"

#include "robot_behavior_routine.h"
#include "task_DC.h" 
#include "serial_protocol.h"
#include "encoder_board_drvr.h"
#include "DC_Motor.h"
#include "task_DC0.h" 
#include "task_DC1.h" 
#include "task_DC2.h" 

#include "task_sort.h" 
#include "task_sort_optimize.h" 
#include "task_zero_state.h" 
#include "task_gripper_calibration.h" 
#include "task_vacuum.h"


static void Ping(uint8_t* data, uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
}

static void Zhanibek(uint8_t* data, uint8_t data_length)
{

  struct terminal_response_s terminal_response;

  RC_Servo.SetMotorSpeed(RC_SERVO_0, 50);

  terminal_response.cmd = BOARD_Zhanibek;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void Punch(uint8_t* data, uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOA, SERV6_GPIOA_PIN2);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Punch;
  terminal_response.data[0] = 0x05;
  terminal_response.data[1] = 0x15;
  terminal_response.data[2] = 0x25;
  terminal_response.data_length = 3;

  Terminal.Send_Response(&terminal_response);
}

static void AX_SetAngle(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];
  uint16_t angle;
  
  Byte_To_Word16(data+1,&angle);

  Dynamixel_AX.TorqueEnable(USART3,ID);
  Dynamixel_AX.SetAngle(USART3,ID,angle);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_SetAngle;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void AX_GetAngle  (uint8_t* data, uint8_t data_length)
{
  uint8_t ID = data[0];
  struct terminal_response_s terminal_response;

  uint16_t angle;

  Dynamixel_AX.GetAngle(USART3,ID,&angle);

  Word16_To_Byte(&angle,terminal_response.data);

  terminal_response.cmd = BOARD_AX_GetAngle;
  terminal_response.data_length = 2;

  Terminal.Send_Response(&terminal_response);
}

static void SetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;
  
  Byte_To_Word32(data,&speed.dx);
  Byte_To_Word32(data+4,&speed.dy);
  Byte_To_Word32(data+8,&speed.dphi);

  Odometry.SetSpeed(&speed);

  terminal_response.cmd = BOARD_SetSpeed;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void GetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;

  Odometry.GetSpeed(&speed);

  Word32_To_Byte(&speed.dx,terminal_response.data);
  Word32_To_Byte(&speed.dy,terminal_response.data+4);
  Word32_To_Byte(&speed.dphi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetSpeed;
  terminal_response.data_length = 12;

  Terminal.Send_Response(&terminal_response);
}

static void SetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;
  
  Byte_To_Word32(data,&coord.x);
  Byte_To_Word32(data+4,&coord.y);
  Byte_To_Word32(data+8,&coord.phi);

  Odometry.SetCoord(&coord);

  terminal_response.cmd = BOARD_SetCoord;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void GetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;

  Odometry.GetCoord(&coord);

  Word32_To_Byte(&coord.x,terminal_response.data);
  Word32_To_Byte(&coord.y,terminal_response.data+4);
  Word32_To_Byte(&coord.phi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetCoord;
  terminal_response.data_length = 12;
  
  Terminal.Send_Response(&terminal_response);
}

static void GrubPile(uint8_t* data, uint8_t data_length)
{
  flag.side = &data[0];

  struct terminal_response_s terminal_response;

  vTaskResume(Grub_Pile_Task.GetTaskHandle());

  terminal_response.cmd = BOARD_GrubPile;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void ReleasePile(uint8_t* data, uint8_t data_length)
{
  flag.side = &data[0];

  struct terminal_response_s terminal_response;

  vTaskResume(Release_Pile_Task.GetTaskHandle());

  terminal_response.cmd = BOARD_GrubPile;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void AX_GetLoad(uint8_t* data, uint8_t data_length)
{
  uint8_t ID = data[0];
  struct terminal_response_s terminal_response;

  uint16_t load;

  Dynamixel_AX.GetPresentLoad(USART3,ID,&load);

  Word16_To_Byte(&load,terminal_response.data);

  terminal_response.cmd = BOARD_AX_GetLoad;
  terminal_response.data_length = 2;

  Terminal.Send_Response(&terminal_response);
}

static void GrubPileSatus(uint8_t* data, uint8_t data_length)
{
  uint8_t side = data[0];
  struct terminal_response_s terminal_response;

  uint16_t angle1;
  uint16_t angle2;
  uint8_t ID1;
  uint8_t ID2;

  if (side == 1)
      ID2 = 5;
    else if (side == 2)
      ID2 = 6;
    else
      ID2 = 4;
    ID1 = side;
  
  Dynamixel_AX.GetAngle(USART3,ID1,&angle1);
  Dynamixel_AX.GetAngle(USART3,ID2,&angle2);
  uint8_t resp = 0;
  if (angle1 >= 38 & angle1 <= 42 & angle2 >= 153 & angle2 <= 157)
    resp = 1;

  Word8_To_Byte(&resp,terminal_response.data);

  terminal_response.cmd = BOARD_GrubPileStatus;
  terminal_response.data_length = 1;

  Terminal.Send_Response(&terminal_response);
}

static void ReleasePileSatus(uint8_t* data, uint8_t data_length)
{
  uint8_t side = data[0];
  struct terminal_response_s terminal_response;

  uint16_t angle1;
  uint16_t angle2;
  uint8_t ID1;
  uint8_t ID2;

  if (side == 1)
      ID2 = 5;
    else if (side == 2)
      ID2 = 6;
    else
      ID2 = 4;
    ID1 = side;
  
  Dynamixel_AX.GetAngle(USART3,ID1,&angle1);
  Dynamixel_AX.GetAngle(USART3,ID2,&angle2);
  uint8_t resp = 0;
  if (angle1 >= 23 & angle1 <= 27 & angle2 >= 132 & angle2 <= 137)
    resp = 1;

  Word8_To_Byte(&resp,terminal_response.data);

  terminal_response.cmd = BOARD_ReleasePileStatus;
  terminal_response.data_length = 1;

  Terminal.Send_Response(&terminal_response);
}

static void AX_SetWheelMode(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];

  Dynamixel_AX.TorqueEnable(USART3,ID);
  Dynamixel_AX.SetWheelMode(USART3,ID);
  Dynamixel_AX.SetMovingSpeed(USART3,ID,512); 

  // Dynamixel_AX.SetMovingSpeed(USART3,ID,512); 
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_SetWheelMode;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void AX_SetJointMode(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];

  Dynamixel_AX.TorqueEnable(USART3,ID);
  Dynamixel_AX.SetJointMode(USART3,ID);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_SetJointMode;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void AX_SetMovingSpeed(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];
  uint16_t speed;
  Byte_To_Word16(data+1,&speed);
  Dynamixel_AX.TorqueEnable(USART3,ID);
  Dynamixel_AX.SetMovingSpeed(USART3,ID,speed);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_SetMovingSpeed;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void SetPWM(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];
  uint16_t speed;
  Byte_To_Word16(data+1,&speed);
  if (speed == 0)
    RC_Servo.Disable((enum rc_servo_e)ID);
  else{
    RC_Servo.Enable((enum rc_servo_e)ID);
    RC_Servo.SetMotorSpeed((enum rc_servo_e)ID, speed);
  }
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_SetPWM;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void Set_stepper_angle(int8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];
  int32_t angle;
  Stepper.Init((enum L6474H_motor_s)ID);
  Byte_To_Word32_s(data+1,&angle);
  if (angle == 0)
    Stepper.Disable((enum L6474H_motor_s)ID);
  else{
    Stepper.Enable((enum L6474H_motor_s)ID);
    Stepper.SetZeroPosisition((enum L6474H_motor_s)ID);
    Stepper.StepToPosisition((enum L6474H_motor_s)ID,angle);
  }

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Set_stepper_angle;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void Stop_stepper(int8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, SERV0_GPIOD_PIN15);
  uint8_t ID = data[0];
  Stepper.Stop((enum L6474H_motor_s)ID);
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Set_stepper_angle;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void Experiment(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;
  
  Byte_To_Word32(data,&speed.dx);
  Byte_To_Word32(data+4,&speed.dy);
  Byte_To_Word32(data+8,&speed.dphi);
  MaxonsEnable();
  Odometry.SetSpeed(&speed);

  if (speed.dx == 0 && speed.dy == 0 && speed.dphi == 0)
    MaxonsDisable();

  terminal_response.cmd = BOARD_SetSpeed;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void DC(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;
  uint8_t ID = data[0];
  uint8_t dir = data[1];
  uint16_t speed;
  Byte_To_Word16(data+2, &speed);
  driveDC(ID, dir, speed);
  terminal_response.cmd = BOARD_DC;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void DCConst(uint8_t* data, uint8_t data_length)
{
  *flag.ID_DC = data[0];
  Byte_To_Word32_s(data + 1, flag.angle_DC);
  vTaskResume(DC_Task.GetTaskHandle());
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_DC;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void CheckEncoder(uint8_t* data, uint8_t data_length)
{
  // vTaskResume(DC_Task.GetTaskHandle());
  int32_t encoder_counts;
  uint8_t encoder_number = *data;
  struct terminal_response_s terminal_response;
  if(Encoder_Board.GetEncoderCounts(UART5,encoder_number,&encoder_counts)==ENCODER_DRVR_OK){
		// do something, everything is OK
			// LL_GPIO_TogglePin(GPIOA, SERV7_GPIOA_PIN3);
		}else{
		// either of ENCODER_DRVR_TIMEOUT_ERROR or ENCODER_DRVR_ENCODER_NUMBER_ERROR errors occured
	}
  Word8_To_Byte(&encoder_number,terminal_response.data);
  Word32_To_Byte_s(&encoder_counts,terminal_response.data + 1);
  terminal_response.cmd = BOARD_DC;
  terminal_response.data_length = 5;
  Terminal.Send_Response(&terminal_response);
}

static void DCSort(uint8_t* data, uint8_t data_length)
{
  uint8_t ID = data[0];
  if (ID == 0){
    Byte_To_Word32_s(data + 1, flag.angle_DC0);
    *flag.angle_DC0 += 32000;
    vTaskResume(DC0_Task.GetTaskHandle());
  }
  else if (ID == 1){
    Byte_To_Word32_s(data + 1, flag.angle_DC1);
    *flag.angle_DC1 += 32000;
    vTaskResume(DC1_Task.GetTaskHandle());
  }
  else if (ID == 2){
    Byte_To_Word32_s(data + 1, flag.angle_DC2);
    *flag.angle_DC2 += 32000;
    vTaskResume(DC2_Task.GetTaskHandle());  
  }
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_DC_Sort;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void Sort(uint8_t* data, uint8_t data_length)
{
  vTaskResume(Sort_Task.GetTaskHandle());

  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_Sort;
  terminal_response.data_length = 0;
  Terminal.Send_Response(&terminal_response);
}

static void SortSatus(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  Word8_To_Byte(flag.sort,terminal_response.data);

  terminal_response.cmd = BOARD_Sort_Status;
  terminal_response.data_length = 1;
  Terminal.Send_Response(&terminal_response);
}

static void OptimalSort(uint8_t* data, uint8_t data_length)
{
  vTaskResume(Sort_Optimize_Task.GetTaskHandle());

  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_Optimal_Sort;
  terminal_response.data_length = 0;
  Terminal.Send_Response(&terminal_response);
}

static void ZeroState(uint8_t* data, uint8_t data_length)
{
  vTaskResume(Zero_State_Task.GetTaskHandle());

  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_Zero_State;
  terminal_response.data_length = 0;
  Terminal.Send_Response(&terminal_response);
}
static void OpenGate(uint8_t* data, uint8_t data_length){
  uint8_t ID = 10;
  uint8_t pos = data[0];

  uint16_t OPEN_ANGLE = 184;
  uint16_t CLOSE_ANGLE = 240;

  if (pos == 0){
    Dynamixel_AX.TorqueEnable(USART3,ID);
    Dynamixel_AX.SetAngle(USART3,ID,CLOSE_ANGLE);
  }
  else if (pos == 1){
    Dynamixel_AX.TorqueEnable(USART3,ID);
    Dynamixel_AX.SetAngle(USART3,ID,OPEN_ANGLE);
  }

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Open_Gate;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void SwitchEmp(uint8_t* data, uint8_t data_length)
{
  uint8_t state = data[0];
  if (state == 1){
    SPI_GPIO_SetOutputPin(SPI_POWER_OUTPUT_1);
    SPI_GPIO_SetOutputPin(SPI_POWER_OUTPUT_2);
  }
  else if(state == 0){
    SPI_GPIO_ResetOutputPin(SPI_POWER_OUTPUT_1);
    SPI_GPIO_ResetOutputPin(SPI_POWER_OUTPUT_2);
  }
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_Switch_Emp;
  terminal_response.data_length = 0;
  Terminal.Send_Response(&terminal_response);
}

static void GripperCalibration(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOA, SERV7_GPIOA_PIN3);
  vTaskResume(Gripper_Calibration_Task.GetTaskHandle());

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GripperCalibration;
  terminal_response.data_length = 0;

  Terminal.Send_Response(&terminal_response);
}

static void StartGame(uint8_t* data, uint8_t data_length)
{

  struct terminal_response_s terminal_response;

  // uint8_t a = 5;
  // if(LL_GPIO_IsInputPinSet(GPIOE, GPIO17_PE4))
  //   *flag.start_game = 1;
  // else
  //   *flag.start_game = 0;
  Word8_To_Byte(flag.start_game,terminal_response.data);

  terminal_response.cmd = BOARD_Start_Game;
  terminal_response.data_length = 1;

  Terminal.Send_Response(&terminal_response);
}

static void Vacuum(uint8_t* data, uint8_t data_length)
{
  *flag.vacuum_on = data[0];
  vTaskResume(Vacuum_Task.GetTaskHandle());

  struct terminal_response_s terminal_response;

  // uint8_t a = 5;
  // if(LL_GPIO_IsInputPinSet(GPIOE, GPIO17_PE4))
  //   *flag.start_game = 1;
  // else
  //   *flag.start_game = 0;
  Word8_To_Byte(flag.start_game,terminal_response.data);

  terminal_response.cmd = BOARD_Start_Game;
  terminal_response.data_length = 1;

  Terminal.Send_Response(&terminal_response);
}

// static void get_stepper_angle(uint8_t* data, uint8_t data_length)
// uint8_t hg[5] = {[1] = 5}; (data_length>>2)-1 LL_GPIO_SetOutputPin(GPIOD, SERV0_GPIOD_PIN15); IDs[ik]

// static void (*cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length);

// void Get_Terminal_Cmd_Handlers(void (**Cmd_Handler)(uint8_t* data, uint8_t data_length))
// {
//   cmd_handlers[BOARD_Ping] = Ping;
//   cmd_handlers[BOARD_Zhanibek] = Zhanibek;

//   // Cmd_Handler = 
// }

// void (*Terminal_Cmd_Handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {[BOARD_Ping] = Ping,
//                                                                                               [BOARD_Zhanibek] = Zhanibek};

void (*Terminal_Cmd_Handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {NULL};

void Terminal_Cmd_Handlers_Init(void)
{
  Terminal_Cmd_Handlers[BOARD_Ping] = Ping;
  Terminal_Cmd_Handlers[BOARD_Zhanibek] = Zhanibek;
  Terminal_Cmd_Handlers[BOARD_Punch] = Punch;
  Terminal_Cmd_Handlers[BOARD_AX_SetAngle] = AX_SetAngle;

  Terminal_Cmd_Handlers[BOARD_SetSpeed] = SetRobotSpeed;
  Terminal_Cmd_Handlers[BOARD_GetSpeed] = GetRobotSpeed;
  Terminal_Cmd_Handlers[BOARD_SetCoord] = SetRobotCoord;
  Terminal_Cmd_Handlers[BOARD_GetCoord] = GetRobotCoord;

  Terminal_Cmd_Handlers[BOARD_AX_GetAngle] = AX_GetAngle;
  Terminal_Cmd_Handlers[BOARD_GrubPile] = GrubPile;
  Terminal_Cmd_Handlers[BOARD_ReleasePile] = ReleasePile;
  Terminal_Cmd_Handlers[BOARD_AX_GetLoad] = AX_GetLoad;
  Terminal_Cmd_Handlers[BOARD_GrubPileStatus] = GrubPileSatus;
  Terminal_Cmd_Handlers[BOARD_ReleasePileStatus] = ReleasePileSatus;
  Terminal_Cmd_Handlers[BOARD_AX_SetWheelMode] = AX_SetWheelMode;
  Terminal_Cmd_Handlers[BOARD_AX_SetJointMode] = AX_SetJointMode;
  Terminal_Cmd_Handlers[BOARD_AX_SetMovingSpeed] = AX_SetMovingSpeed;
  Terminal_Cmd_Handlers[BOARD_SetPWM] = SetPWM;
  Terminal_Cmd_Handlers[BOARD_Set_stepper_angle] = Set_stepper_angle;
  Terminal_Cmd_Handlers[BOARD_Stop_stepper] = Stop_stepper;
  Terminal_Cmd_Handlers[BOARD_Experiment] = Experiment;
  Terminal_Cmd_Handlers[BOARD_DC] = DC;
  Terminal_Cmd_Handlers[BOARD_DCConst] = DCConst;
  Terminal_Cmd_Handlers[BOARD_Encoder] = CheckEncoder;
  Terminal_Cmd_Handlers[BOARD_DC_Sort] = DCSort;
  Terminal_Cmd_Handlers[BOARD_Sort] = Sort;
  Terminal_Cmd_Handlers[BOARD_Sort_Status] = SortSatus;
  Terminal_Cmd_Handlers[BOARD_Optimal_Sort] = OptimalSort;
  Terminal_Cmd_Handlers[BOARD_Zero_State] = ZeroState;
  Terminal_Cmd_Handlers[BOARD_Open_Gate] = OpenGate;
  Terminal_Cmd_Handlers[BOARD_Switch_Emp] = SwitchEmp;
  Terminal_Cmd_Handlers[BOARD_GripperCalibration] = GripperCalibration;
  Terminal_Cmd_Handlers[BOARD_Start_Game] = StartGame;
}

// struct terminal_cmd_handler_s Terminal_Cmd_Handler = {.Init = Init,
//                                                       .Cmd_Handler = cmd_handler};

// piska
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
