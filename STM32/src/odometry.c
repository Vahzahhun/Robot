/*
 * odometry.c
 */

#include "odometry.h"

float x_buffer_position = 0, y_buffer_position = 0;
float x_offset_position = 0, y_offset_position = 0;
float x_position = 0, y_position = 0;

char usart3_status = 0;
char usart3_data = 0;

char gyro_status = 0;
char gyro_send[7] =
{ 'i', 't', 's' };
char gyro_receive[7] =
{ 'i', 't', 's' };

float gyro_buffer = 0, gyro_offset = 90;
float gyro_angle = 90, gyro_radian = 1.5707963268;

void odometry0_initialization(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

	TIM_Cmd(TIM4, ENABLE);
}

void odometry1_initialization(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

	TIM_Cmd(TIM5, ENABLE);
}

void gyro_initialization(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &gyro_send;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = sizeof(gyro_send);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &gyro_receive + 3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(gyro_receive) - 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);

	USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);

	//USART Interrupt USART_RX
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//DMA Interrupt USART_TX
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//DMA Interrupt USART_RX
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream1, ENABLE);

	USART_Cmd(USART3, ENABLE);
}

void calculate_odometry(void)
{
	short int odometry0_speed = odometry0;
	odometry0 = 0;
	short int odometry1_speed = odometry1;
	odometry1 = 0;

	float buffer_x[2];
	float buffer_y[2];

	buffer_x[0] = odometry0_speed * cosf(gyro_radian + 0.785398);
	buffer_x[1] = odometry1_speed * cosf(gyro_radian + 2.356190);

	buffer_y[0] = odometry0_speed * sinf(gyro_radian + 0.785398);
	buffer_y[1] = odometry1_speed * sinf(gyro_radian + 2.356190);

	x_buffer_position += (buffer_x[0] + buffer_x[1]) * odometry_to_cm;
	y_buffer_position -= (buffer_y[0] + buffer_y[1]) * odometry_to_cm;

	x_position = x_buffer_position - x_offset_position;
	y_position = y_buffer_position - y_offset_position;
}

//Start RECEIVE data FROM gyro
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE))
	{
		usart3_data = USART_ReceiveData(USART3);

		if (usart3_status == 0 && usart3_data == 'i')
			usart3_status++;
		else if (usart3_status == 1 && usart3_data == 't')
			usart3_status++;
		else if (usart3_status == 2 && usart3_data == 's')
		{
			usart3_status = 0;

			USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		}
		else
			usart3_status = 0;

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}
//End RECEIVE data FROM gyro
void DMA1_Stream1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
	{
		if (gyro_status == RESET)
		{
			gyro_status = SET;
			buzzer(22, 22);
		}

		memcpy(&gyro_buffer, gyro_receive + 3, 4);

		gyro_angle = (gyro_offset - gyro_buffer);
		gyro_radian = (gyro_offset - gyro_buffer) * 0.01745329252;

		while (gyro_angle > 180)
			gyro_angle -= 360;
		while (gyro_angle < -180)
			gyro_angle += 360;

		while (gyro_radian > 3.14159265359)
			gyro_radian -= 6.28318530718;
		while (gyro_radian < -3.14159265359)
			gyro_radian += 6.28318530718;

		USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
	}
}
//End SEND data TO gyro
void DMA1_Stream3_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
	{
		USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);

		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
	}
}
