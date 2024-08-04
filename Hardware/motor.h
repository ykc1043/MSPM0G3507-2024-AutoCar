#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"
#include "math.h"


#define MotorIn_A_Set() DL_GPIO_setPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_0_L_PIN);
#define MotorIn_A_Reset() DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_0_L_PIN);

#define MotorIn_B_Set() DL_GPIO_setPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_1_R_PIN);
#define MotorIn_B_Reset() DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_1_R_PIN);

int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Set_PWM(int pwma,int pwmb);
#endif