#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "delay.h"
#include "lcd.h"
#include "motor_encoder.h"
#include "odometry.h"

#include "ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#define BUTTON0 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
#define BUTTON1 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
#define BUTTON2 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)
#define BUTTON3 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)
#define BUTTON4 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)
#define TOGGLE0 GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)
#define TOGGLE1 GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)
#define TOGGLE2 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)

ADC_CommonInitTypeDef ADC_CommonInitStructure;
ADC_InitTypeDef ADC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
I2C_InitTypeDef I2C_InitStructure;
USART_InitTypeDef USART_InitStructure;
SPI_InitTypeDef SPI_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

//==============
//----------MISC
//==============
char lcd[21];

//================
//----------BUZZER
//================
char buzzer_status = 0;

short int buzzer_iteration = 0;
short int buzzer_time = 0;
short int buzzer_count = 0;

//==================
//----------VELOCITY
//==================
short int x_velocity;
short int y_velocity;
short int angular_velocity;
int status_move;

//=============
//----------ROS
//=============
ros::NodeHandle rosNH;
unsigned long int t0_ros;
unsigned long int t1_ros;
std_msgs::UInt8 msg_button;
geometry_msgs::Pose2D msg_odometry_buffer;

//======================
//---------ROS PUBLISHER
//======================
ros::Publisher pub_button("stm2pc_Button", &msg_button);
ros::Publisher pub_odometry_buffer("stm2pc_odometry_buffer", &msg_odometry_buffer);

//================================
//---------ROS SUBSCRIBER CALLBACK
//================================
void cllbck_sub_velocity(const geometry_msgs::Twist &msg)
{
	x_velocity = msg.linear.x;
	y_velocity = msg.linear.y;
	angular_velocity = msg.angular.z;

	motor_timer = 0;
}

void cllbck_sub_buzzer(const std_msgs::Int16MultiArray &msg)
{
	buzzer(msg.data[0], msg.data[1]);
}

void cllbck_sub_odometry_offset(const geometry_msgs::Pose2D &msg)
{
	x_offset_position = msg.x;
	y_offset_position = msg.y;
	gyro_offset = msg.theta;
}

//=======================
//---------ROS SUBSCRIBER
//=======================
ros::Subscriber<geometry_msgs::Twist> sub_velocity("pc2stm_velocity", cllbck_sub_velocity);
ros::Subscriber<std_msgs::Int16MultiArray> sub_buzzer("pc2stm_buzzer", cllbck_sub_buzzer);
ros::Subscriber<geometry_msgs::Pose2D> sub_odometry_offset("pc2stm_odometry_offset", cllbck_sub_odometry_offset);

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	inisialisasi_gpio();
	adc_initialization();
	tim6_initialization();
	tim7_initialization();

	tick_init();
	lcd_init(20, 4);

	inisialisasi_encoder0();
	inisialisasi_encoder1();
	inisialisasi_encoder2();
	inisialisasi_encoder3();
	inisialisasi_motor0();
	inisialisasi_motor1();
	inisialisasi_motor2();
	inisialisasi_motor3();
	inisialisasi_odometry0();
	inisialisasi_odometry1();
	inisialisasi_gyro();

	rosNH.initNode();

	rosNH.advertise(pub_button);
	rosNH.advertise(pub_odometry_buffer);

	rosNH.subscribe(sub_velocity);
	rosNH.subscribe(sub_buzzer);
	rosNH.subscribe(sub_odometry_offset);

	buzzer(22, 22);

	while (1)
	{
		t1_ros = millis();
		int delay = millis();
		int delay_i;
		if (t1_ros - t0_ros > 20)
		{
			t0_ros = t1_ros;

			pub_button.publish(&msg_button);
			pub_odometry_buffer.publish(&msg_odometry_buffer);
		}

		rosNH.spinOnce();

		switch(status_move)
		{
			case 0:
			move(0,30,0);
			if (delay - delay_i >1000)
			{
				delay_i = delay;
				status_move = 1;
			}
			break;

			case 1:
			move(30,0,0);
			if (delay - delay_i >1000)
			{
				delay_i = delay;
				status_move = 0;
			}
			break;
		}

		sprintf(lcd, "x:%+05.0f dX:%+03d", x_position, x_velocity);
		lcd_print(0, 0, lcd);
		sprintf(lcd, "Y:%+05.0f dY:%+03d", y_position, y_velocity);
		lcd_print(0, 1, lcd);
		sprintf(lcd, "A: %+04.0f  W:%+03d", gyro_angle, angular_velocity);
		lcd_print(0, 2, lcd);

		sprintf(lcd, "%+3d %+3d %+3d %+3d", pv_motor[0], pv_motor[1],pv_motor[2], pv_motor[3]);
		lcd_print(0,3,lcd);

	}
}

//===========
//-----Buzzer
//===========
void buzzer(short int time, short int count)
{
	buzzer_status = 1;
	buzzer_iteration = 0;
	buzzer_time = time;
	buzzer_count = count * 2;

	GPIO_SetBits(GPIOE, GPIO_Pin_2);
}

//===============
//-----1kHz
//===============
extern "C" void TIM6_DAC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update))
	{
		//=========================
		//-----Buzzer sound control
		//=========================
		if (buzzer_status == 1 && buzzer_count > 0 && buzzer_iteration++ == buzzer_time)
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_2);

			buzzer_iteration = 0;
			buzzer_status = 0;
			buzzer_count--;
		}
		else if (buzzer_status == 0 && buzzer_count > 1 && buzzer_iteration++ == buzzer_time)
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_2);

			buzzer_iteration = 0;
			buzzer_status = 1;
			buzzer_count--;
		}

		//=================
		//-----Safety motor
		//=================
//		if (++motor_timer > 1000)
//			motor_status = 1;
//		else
//			motor_status = 0;
//		motor_status = TOGGLE2;

		//=================
		//-----manual movement
		//=================
		move(x_velocity, y_velocity, angular_velocity);
//		move(0, 0, 10);
//		sp_motor[0]=10;
//		sp_motor[1]=10;
//		sp_motor[2]=10;
//		sp_motor[3]=10;


		//====================
		//-----Calculate odometry
		//====================
		calculate_odometry();
		msg_odometry_buffer.x = x_buffer_position;
		msg_odometry_buffer.y = y_buffer_position;
		msg_odometry_buffer.theta = gyro_buffer;

		//===========
		//-----Button
		//===========
		unsigned char Button_buffer = 0;
		Button_buffer |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) << 0; //Button 0
		Button_buffer |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) << 1; //Button 1
		Button_buffer |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) << 2; //Button 2
		Button_buffer |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) << 3; //Button 3
		Button_buffer |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) << 4; //Button 4
		Button_buffer |= GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) << 5; //Toggle 0
		Button_buffer |= GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) << 6; //Toggle 1
		Button_buffer |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) << 7; //Toggle 2
		msg_button.data = Button_buffer;

	}
}

//================
//-----50Hz
//================
extern "C" void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update))
	{
		//============================
		//-----Motor speed control
		//============================
		motor_control();

		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

void inisialisasi_gpio(void)
{
	//Buzzer
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//Button
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource2);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void adc_initialization(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);

	ADC_Cmd(ADC1, ENABLE);

	ADC_DMACmd(ADC1, ENABLE);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &sensor_bola;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	DMA_Cmd(DMA2_Stream0, ENABLE);

	ADC_SoftwareStartConv(ADC1);
}

void tim6_initialization(void)
{
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM6, ENABLE);
}

void tim7_initialization(void)
{
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1680 - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM7, ENABLE);
}
