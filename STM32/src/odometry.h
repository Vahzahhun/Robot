/*
 * odometry.h
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define odometry0 TIM4->CNT
#define odometry1 TIM5->CNT

#define odometry_to_cm 0.0195004145

extern GPIO_InitTypeDef GPIO_InitStructure;
extern TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
extern TIM_OCInitTypeDef TIM_OCInitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;
extern USART_InitTypeDef USART_InitStructure;
extern DMA_InitTypeDef DMA_InitStructure;

extern void buzzer(short int time, short int count);

extern float x_buffer_position, y_buffer_position;
extern float x_offset_position, y_offset_position;
extern float x_position, y_position;

extern char usart3_status;
extern char usart3_data;

extern char gyro_status;
extern char gyro_send[7];
extern char gyro_receive[7];

extern float gyro_buffer, gyro_offset;
extern float gyro_angle, gyro_radian;

void odometry0_initialization(void);
void odometry1_initialization(void);
void gyro_initialization(void);

void calculate_odometry(void);

//Start RECEIVE data FROM gyro
void USART3_IRQHandler(void);
//End RECEIVE data FROM gyro
void DMA1_Stream1_IRQHandler(void);
//End SEND data TO gyro
void DMA1_Stream3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H_ */
