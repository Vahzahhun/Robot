/*
 * motor_encoder.h
 */

#ifndef MOTOR_ENCODER_H_
#define MOTOR_ENCODER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx.h"
#include "stdlib.h"
#include "math.h"

#define motor0 TIM9->CCR1
#define motor1 TIM9->CCR2
#define motor2 TIM13->CCR1
//#define motor3 TIM10->CCR1

#define encoder0 TIM2->CNT
#define encoder1 TIM1->CNT
#define encoder2 TIM3->CNT
//#define encoder3 TIM8->CNT

#define kp_motor 10
#define ki_motor 5
#define kd_motor 2

extern GPIO_InitTypeDef GPIO_InitStructure;
extern TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
extern TIM_OCInitTypeDef TIM_OCInitStructure;

//=================
//-----Safety motor
//=================
uint32_t motor_timer;
char motor_status;

//=============================================
//-----Set Point and Process Speed Variable
//=============================================
short int pv_motor[4], sp_motor[4];

void motor0_initialization(void);
void motor1_initialization(void);
void motor2_initialization(void);
void motor3_initialization(void);

void encoder0_initialization(void);
void encoder1_initialization(void);
void encoder2_initialization(void);
void encoder3_initialization(void);

void motor_control(void);

void move(short int vx, short int vy, short int vtheta);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_H_ */
