#include "DC_Motor.h"
#include "terminal_cmd_handler.h" 
#include "dynamixel_protocol10.h"
#include "byte2word.h"

#include "gpio_board_defines.h"
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

void driveDC (uint8_t ID, uint8_t dir, uint16_t speed){
    if (ID == 0){
        if (speed == 0){
        RC_Servo.Disable((enum rc_servo_e)ID);
        LL_GPIO_SetOutputPin(GPIOC, GPIO14_PC15);
        LL_GPIO_SetOutputPin(GPIOC, GPIO15_PC14);
        }
        else{
        RC_Servo.Enable((enum rc_servo_e)ID);
        RC_Servo.SetMotorSpeed((enum rc_servo_e)ID, speed);
        if (dir == 0){
            LL_GPIO_ResetOutputPin(GPIOC, GPIO14_PC15);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO15_PC14);
            LL_GPIO_SetOutputPin(GPIOC, GPIO14_PC15);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO15_PC14);
        } else if (dir == 1){
            LL_GPIO_ResetOutputPin(GPIOC, GPIO14_PC15);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO15_PC14);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO14_PC15);
            LL_GPIO_SetOutputPin(GPIOC, GPIO15_PC14);
        }
        }
    }else if (ID == 1){
        if (speed == 0){
        RC_Servo.Disable((enum rc_servo_e)ID);
        LL_GPIO_SetOutputPin(GPIOE, GPIO13_PE6);
        LL_GPIO_SetOutputPin(GPIOC, GPIO16_PC13);
        }
        else{
        RC_Servo.Enable((enum rc_servo_e)ID);
        RC_Servo.SetMotorSpeed((enum rc_servo_e)ID, speed);
        if (dir == 0){
            LL_GPIO_ResetOutputPin(GPIOE, GPIO13_PE6);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO16_PC13);
            LL_GPIO_SetOutputPin(GPIOE, GPIO13_PE6);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO16_PC13);
        } else if (dir == 1){
            LL_GPIO_ResetOutputPin(GPIOE, GPIO13_PE6);
            LL_GPIO_ResetOutputPin(GPIOC, GPIO16_PC13);
            LL_GPIO_ResetOutputPin(GPIOE, GPIO13_PE6);
            LL_GPIO_SetOutputPin(GPIOC, GPIO16_PC13);
        }
        }
    }else if (ID == 2){
        if (speed == 0){
            RC_Servo.Disable((enum rc_servo_e)ID);
            LL_GPIO_SetOutputPin(GPIOE, GPIO12_PE5);
            // LL_GPIO_SetOutputPin(GPIOE, GPIO17_PE4);
        }
        else{
            RC_Servo.Enable((enum rc_servo_e)ID);
            RC_Servo.SetMotorSpeed((enum rc_servo_e)ID, speed);
        if (dir == 1){
            LL_GPIO_ResetOutputPin(GPIOE, GPIO12_PE5);
            // LL_GPIO_ResetOutputPin(GPIOE, GPIO17_PE4);
            LL_GPIO_SetOutputPin(GPIOE, GPIO12_PE5);
            // LL_GPIO_ResetOutputPin(GPIOE, GPIO17_PE4);
        } else if (dir == 0){
            LL_GPIO_ResetOutputPin(GPIOE, GPIO12_PE5);
            // LL_GPIO_ResetOutputPin(GPIOE, GPIO17_PE4);
            LL_GPIO_ResetOutputPin(GPIOE, GPIO12_PE5);
            // LL_GPIO_SetOutputPin(GPIOE, GPIO17_PE4);
        }
        }
    }
}

void setPosition(uint8_t ID, int32_t angle){  
    int32_t encoder_counts;
    uint32_t encoder_counts_old;
    int32_t angle_tick = angle;
    Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
    int32_t err = angle_tick - encoder_counts;
    int32_t err_old;
    float Pp = 0.1;
    float Pd = 10;
    uint16_t speed;
    if (*flag.ID_DC == 2){
        Pp = 2;
        Pd = 10;
    while(abs(err) > 50){
        if(err < 0)
            driveDC(ID, 1, 2000);
        else
            driveDC(ID, 0, 2000);
        Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    while(abs(err) > 3){
        if(err < 0){
            speed = -(err * Pp) - (err - err_old) * Pd;
            if (speed < 300)
                speed = 300;
            driveDC(ID, 1, speed);
        }
        else{
            speed = err * Pp + (err - err_old) * Pd;
            if (speed < 300)
                speed = 300;
            driveDC(ID, 0, speed);
        }
        Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    driveDC(ID, 0, 0);
    }
    else {
    while(abs(err) > 200){
        if(err < 0)
            driveDC(ID, 1, 2000);
        else
            driveDC(ID, 0, 2000);
        Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    while(abs(err) > 3){
        if(err < 0){
            speed = -(err * Pp) - (err - err_old) * Pd;
            if (speed < 100)
                speed = 100;
            else if (speed > 2000)
                speed = 2000;
            driveDC(ID, 1, speed);
        }
        else{
            speed = err * Pp + (err - err_old) * Pd;
            if (speed < 100)
                speed = 100;
            else if (speed > 2000)
                speed = 2000;
            driveDC(ID, 0, speed);
        }
        Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    driveDC(ID, 0, 0);
    }
}
void setPosition01motor(int32_t angle0, int32_t angle1){

    int32_t encoder_counts0;
    uint32_t encoder_counts_old0;
    int32_t angle_tick0 = angle0;
    int32_t encoder_counts1;
    uint32_t encoder_counts_old1;
    int32_t angle_tick1 = angle1;

    Encoder_Board.GetEncoderCounts(UART5, 0, &encoder_counts0);
    Encoder_Board.GetEncoderCounts(UART5, 1, &encoder_counts1);

    int32_t err0 = angle_tick0 - encoder_counts0;
    int32_t err_old0;

    int32_t err1 = angle_tick1 - encoder_counts1;
    int32_t err_old1;

    float Pp = 0.1;
    float Pd = 10;
    uint16_t speed0;
    uint16_t speed1;

    while(abs(err0) > 200 & abs(err1) > 200){
        if(err0 < 0)
            driveDC(0, 1, 2000);
        else
            driveDC(0, 0, 2000);
        if(err1 < 0)
            driveDC(1, 1, 2000);
        else
            driveDC(1, 0, 2000);

        Encoder_Board.GetEncoderCounts(UART5, 0, &encoder_counts0);
        Encoder_Board.GetEncoderCounts(UART5, 1, &encoder_counts1);

        err_old0 = err0;
        err0 = angle_tick0 - encoder_counts0;

        err_old1 = err1;
        err1 = angle_tick1 - encoder_counts1;
    }
    while(abs(err0) > 3 & abs(err1) > 3){
        if (abs(err0) > 3){
            if(err0 < 0){
                speed0 = -(err0 * Pp) - (err0 - err_old0) * Pd;
                if (speed0 < 100)
                    speed0 = 100;
                else if (speed0 > 2000)
                    speed0 = 2000;
                driveDC(0, 1, speed0);
            }
            else{
                speed0 = err0 * Pp + (err0 - err_old0) * Pd;
                if (speed0 < 100)
                    speed0 = 100;
                else if (speed0 > 2000)
                    speed0 = 2000;
                driveDC(0, 0, speed0);
            }
            Encoder_Board.GetEncoderCounts(UART5, 0, &encoder_counts0);
        }
        else
            driveDC(0, 0, 0);
        if (abs(err1) > 3){
            if(err1 < 0){
                speed1 = -(err1 * Pp) - (err1 - err_old1) * Pd;
                if (speed1 < 100)
                    speed1 = 100;
                else if (speed1 > 2000)
                    speed1 = 2000;
                driveDC(1, 1, speed1);
            }
            else{
                speed1 = err1 * Pp + (err1 - err_old1) * Pd;
                if (speed1 < 100)
                    speed1 = 100;
                else if (speed1 > 2000)
                    speed1 = 2000;
                driveDC(1, 0, speed1);
            }
            Encoder_Board.GetEncoderCounts(UART5, 1, &encoder_counts1);
        }
        else
            driveDC(1, 0, 0);

        err_old0 = err0;
        err0 = angle_tick0 - encoder_counts0;

        err_old1 = err1;
        err1 = angle_tick1 - encoder_counts1;
    }
}

void setPosition0motor(int32_t angle){
    int32_t encoder_counts;
    int32_t encoder_counts_old;
    int32_t angle_tick = angle;
    // Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
    encoder_counts = *flag.DC0_current_pos;
    int32_t err = angle_tick - encoder_counts;
    int32_t err_old;
    float Pp = 0.1;
    float Pd = 10;
    uint16_t speed;
    while(abs(err) > 3){
        if(err < 0){
            speed = -(err * Pp) - (err - err_old) * Pd;
            if (speed < 20)
                speed = 20;
            else if (speed > 200)
                speed = 200;
            driveDC(0, 0, speed);
        }
        else{
            speed = err * Pp + (err - err_old) * Pd;
            if (speed < 20)
                speed = 20;
            else if (speed > 200)
                speed = 200;
            driveDC(0, 1, speed);
        }
        // Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        encoder_counts = *flag.DC0_current_pos;
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    driveDC(0, 0, 0);
}

void setPosition1motor(int32_t angle){
    int32_t encoder_counts;
    int32_t encoder_counts_old;
    int32_t angle_tick = angle;
    // Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
    encoder_counts = *flag.DC1_current_pos;
    int32_t err = angle_tick - encoder_counts;
    int32_t err_old;
    float Pp = 0.1;
    float Pd = 10;
    uint16_t speed;
    while(abs(err) > 3){
        if(err < 0){
            speed = -(err * Pp) - (err - err_old) * Pd;
            if (speed < 20)
                speed = 20;
            else if (speed > 200)
                speed = 200;
            driveDC(1, 0, speed);
        }
        else{
            speed = err * Pp + (err - err_old) * Pd;
            if (speed < 20)
                speed = 20;
            else if (speed > 200)
                speed = 200;
            driveDC(1, 1, speed);
        }
        // Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        encoder_counts = *flag.DC1_current_pos;
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    driveDC(1, 0, 0);
}

void setPosition2motor(int32_t angle){  
    int32_t encoder_counts = *flag.DC2_current_pos;
    int32_t encoder_counts_old;
    int32_t angle_tick = angle;
    // Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
    int32_t err = angle_tick - encoder_counts;
    int32_t err_old;
    float Pp = 2;
    float Pd = 10;
    uint16_t speed;
    while(abs(err) > 1){
        if(err < 0){
            speed = -(err * Pp) - (err - err_old) * Pd;
            if (speed < 20)
                speed = 20;
            driveDC(2, 1, speed);
        }
        else{
            speed = err * Pp + (err - err_old) * Pd;
            if (speed < 20)
                speed = 20;
            driveDC(2, 0, speed);
        }
        // Encoder_Board.GetEncoderCounts(UART5, ID, &encoder_counts);
        encoder_counts = *flag.DC2_current_pos;
        err_old = err;
        err = angle_tick - encoder_counts;
    }
    driveDC(2, 0, 0);
}

