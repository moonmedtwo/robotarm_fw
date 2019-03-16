#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "math.h"
#include "stdio.h"

#include <configuration.h>
#include <controller.h>

#define MAX_JOINT_SPEED_IN_RPM 		(uint16_t)		60

extern dcServoMotor_t BASE, SHLDR, ELBOW, PITCH, ROLL;
extern robotGripper_t GRIPPER;

uint32_t Multiplier = 0; // https://stm32f4-discovery.net/2014/09/precise-delay-counter/

void LED_DEBUG_Config(void)
{
	GPIO_InitTypeDef GPIO_DIR;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
	GPIO_DIR.GPIO_Pin 	= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_DIR.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_DIR.GPIO_OType = GPIO_OType_PP;
	GPIO_DIR.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_DIR.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_DIR);
}

void Interupt_Routine_Config(void)
{	//Initialize TIM7 for sampling
	NVIC_InitTypeDef nvicStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; 					//1MHz -> T = 1us
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 5000; // 5ms
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM7, ENABLE);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	nvicStructure.NVIC_IRQChannel = TIM7_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

void InitializeMotorParam(void)
{	// Initialize Encoder & Motor Param
	GRIPPER.gripTimer = GRIPPER_TIMER;
	
	// BASE Initialize
	BASE.Encoder.Param.pulsePerRound 	= 1 * 4 * 139 * (120 / 20);
	BASE.DcMotor.Param.maxSpeed 			= MAX_JOINT_SPEED_IN_RPM  * 360 / 60;
	BASE.DcMotor.Param.maxPulse 			= 300;
	BASE.DcMotor.Param.acceptedError 	= 5;
	// SHOULDER Initialize
	SHLDR.Encoder.Param.pulsePerRound = 200 * 4 * 1;
	SHLDR.DcMotor.Param.maxSpeed 			= MAX_JOINT_SPEED_IN_RPM  * 360 / 60;
	SHLDR.DcMotor.Param.maxPulse 			= 700;
	SHLDR.DcMotor.Param.acceptedError = 2;
	// ELBOW Initialize
	ELBOW.Encoder.Param.pulsePerRound = 13 * 4 * 264 * (2 / 1);
	ELBOW.DcMotor.Param.maxSpeed 			= MAX_JOINT_SPEED_IN_RPM  * 360 / 60;
	ELBOW.DcMotor.Param.maxPulse 			= 600;
	ELBOW.DcMotor.Param.acceptedError = 2;
	// PITCH Initialize 													
	PITCH.Encoder.Param.pulsePerRound 	= 11 * 4 * 226;
	PITCH.DcMotor.Param.maxSpeed 				= MAX_JOINT_SPEED_IN_RPM  * 360 / 60;
	PITCH.DcMotor.Param.maxPulse 				= 800;
	PITCH.DcMotor.Param.acceptedError 	= 2;
	// ROLL Initialize
	ROLL.Encoder.Param.pulsePerRound 	= 11 * 4 * 34;
	ROLL.DcMotor.Param.maxSpeed 			= MAX_JOINT_SPEED_IN_RPM  * 360 / 60;
	ROLL.DcMotor.Param.maxPulse 			= 500;
	ROLL.DcMotor.Param.acceptedError 	= 5;
}
/* Encoder Config */
void TIM2_ENC_BASE_Config(void)
{	// PA15 - PB3	Checked
	GPIO_InitTypeDef ENCODER_BASE;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
	
	ENCODER_BASE.GPIO_Mode  = GPIO_Mode_AF; 
	ENCODER_BASE.GPIO_Speed = GPIO_Speed_100MHz;
	ENCODER_BASE.GPIO_OType = GPIO_OType_PP;
	ENCODER_BASE.GPIO_PuPd  = GPIO_PuPd_DOWN;
	ENCODER_BASE.GPIO_Pin   = GPIO_Pin_15;
	GPIO_Init(GPIOA, &ENCODER_BASE);

	ENCODER_BASE.GPIO_Mode  = GPIO_Mode_AF; 
	ENCODER_BASE.GPIO_Speed = GPIO_Speed_100MHz;
	ENCODER_BASE.GPIO_OType = GPIO_OType_PP;
	ENCODER_BASE.GPIO_PuPd  = GPIO_PuPd_DOWN;
	ENCODER_BASE.GPIO_Pin   = GPIO_Pin_3;
	GPIO_Init(GPIOB, &ENCODER_BASE);	

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(TIM2, ENABLE);	
}

void TIM3_ENC_SHLDR_Config(void)
{	// PB4 - PB5 Checked
	GPIO_InitTypeDef ENCODER_SHLDR;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	
	ENCODER_SHLDR.GPIO_Mode  = GPIO_Mode_AF; 
	ENCODER_SHLDR.GPIO_Speed = GPIO_Speed_100MHz;
	ENCODER_SHLDR.GPIO_OType = GPIO_OType_PP;
	ENCODER_SHLDR.GPIO_PuPd  = GPIO_PuPd_DOWN;
	ENCODER_SHLDR.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &ENCODER_SHLDR);	

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM3, ENABLE);	
}

void TIM4_ENC_ELBOW_Config(void)
{	// PB6 - PB7 Checked
	GPIO_InitTypeDef ENCODER_ELBOW;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	ENCODER_ELBOW.GPIO_Mode  = GPIO_Mode_AF; 
	ENCODER_ELBOW.GPIO_Speed = GPIO_Speed_100MHz;
	ENCODER_ELBOW.GPIO_OType = GPIO_OType_PP;
	ENCODER_ELBOW.GPIO_PuPd  = GPIO_PuPd_DOWN;
	ENCODER_ELBOW.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &ENCODER_ELBOW);	

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM4, ENABLE);
}

void TIM5_ENC_PITCH_Config(void)
{	// PA0 - PA1 Checked
	GPIO_InitTypeDef ENCODER_PITCH;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	
	ENCODER_PITCH.GPIO_Mode  = GPIO_Mode_AF; 
	ENCODER_PITCH.GPIO_Speed = GPIO_Speed_100MHz;
	ENCODER_PITCH.GPIO_OType = GPIO_OType_PP;
	ENCODER_PITCH.GPIO_PuPd  = GPIO_PuPd_DOWN;
	ENCODER_PITCH.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &ENCODER_PITCH);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_Cmd(TIM5, ENABLE);
}

void TIM1_ENC_ROLL_Config(void)
{	// PA8 - PA9 Checked // Remove capacitor C49 connected to PA9
	GPIO_InitTypeDef ENCODER_ROLL;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	
	ENCODER_ROLL.GPIO_Mode  = GPIO_Mode_AF; 
	ENCODER_ROLL.GPIO_Speed = GPIO_Speed_100MHz;
	ENCODER_ROLL.GPIO_OType = GPIO_OType_PP;
	ENCODER_ROLL.GPIO_PuPd  = GPIO_PuPd_DOWN;
	ENCODER_ROLL.GPIO_Pin   = GPIO_Pin_9| GPIO_Pin_8;
	GPIO_Init(GPIOA, &ENCODER_ROLL);
	
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
	TIM_CCPreloadControl(TIM1, ENABLE);
	
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM1, ENABLE);
}


/* PWM Config */
void TIM9_PWM_BASE_Config(void)
{ // PE5 - PE6 | CH1 - CH2 Checked
	GPIO_InitTypeDef 		PWM_PIN;
	TIM_TimeBaseInitTypeDef PWM_TimeBase;
	TIM_OCInitTypeDef 		PWM_OCInit;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
	
	PWM_PIN.GPIO_Mode 	= GPIO_Mode_AF;
	PWM_PIN.GPIO_Speed 	= GPIO_Speed_100MHz;
	PWM_PIN.GPIO_OType 	= GPIO_OType_PP;
	PWM_PIN.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	PWM_PIN.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &PWM_PIN);

	PWM_TimeBase.TIM_Prescaler 		= 83;		// f_prescaler = 1MHz
	PWM_TimeBase.TIM_Period 		= 1000;		// T_period  = 0.5ms => f = 1kHz
	PWM_TimeBase.TIM_ClockDivision 	= 0;
	PWM_TimeBase.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM9, &PWM_TimeBase);

	PWM_OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	PWM_OCInit.TIM_OutputState 	= TIM_OutputState_Enable;
	PWM_OCInit.TIM_Pulse 		= 0;
	PWM_OCInit.TIM_OCPolarity 	= TIM_OCPolarity_High;

	TIM_OC1Init(TIM9, &PWM_OCInit);
	TIM_OC2Init(TIM9, &PWM_OCInit);

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM9, ENABLE);

	TIM_Cmd(TIM9, ENABLE);
}

void TIM10_TIM11_PWM_SHLDR_Config(void)
{	// PB8 - PB9 | TIM10_CH1 - TIM11_CH1 Checked
	GPIO_InitTypeDef 		PWM_PIN;
	TIM_TimeBaseInitTypeDef PWM_TimeBase;
	TIM_OCInitTypeDef 		PWM_OCInit;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);

	PWM_PIN.GPIO_Mode 	= GPIO_Mode_AF;
	PWM_PIN.GPIO_Speed 	= GPIO_Speed_100MHz;
	PWM_PIN.GPIO_OType 	= GPIO_OType_PP;
	PWM_PIN.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	PWM_PIN.GPIO_Pin 	= GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB, &PWM_PIN);

	PWM_TimeBase.TIM_Prescaler 		= 83;		// f_prescaler = 1MHz
	PWM_TimeBase.TIM_Period 		= 1000;		// T_period  = 0.5ms => f = 1kHz
	PWM_TimeBase.TIM_ClockDivision 	= 0;
	PWM_TimeBase.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &PWM_TimeBase);
	TIM_TimeBaseInit(TIM11, &PWM_TimeBase);

	PWM_OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	PWM_OCInit.TIM_OutputState 	= TIM_OutputState_Enable;
	PWM_OCInit.TIM_Pulse 		= 0;
	PWM_OCInit.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OC1Init(TIM10, &PWM_OCInit);
	TIM_OC1Init(TIM11, &PWM_OCInit);
	
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM10, ENABLE);
	TIM_ARRPreloadConfig(TIM11, ENABLE);

	TIM_Cmd(TIM10, ENABLE);
	TIM_Cmd(TIM11, ENABLE);	
}

void TIM13_TIM14_PWM_ELBOW_Config(void)
{	// PA6 - PA7 | TIM13_CH1 - TIM14_CH1 Checked
	GPIO_InitTypeDef 		PWM_PIN;
	TIM_TimeBaseInitTypeDef PWM_TimeBase;
	TIM_OCInitTypeDef 		PWM_OCInit;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);

	PWM_PIN.GPIO_Mode 	= GPIO_Mode_AF;
	PWM_PIN.GPIO_Speed 	= GPIO_Speed_100MHz;
	PWM_PIN.GPIO_OType 	= GPIO_OType_PP;
	PWM_PIN.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	PWM_PIN.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &PWM_PIN);

	PWM_TimeBase.TIM_Prescaler 		= 83;		// f_prescaler = 1MHz
	PWM_TimeBase.TIM_Period 		= 1000;		// T_period  = 0.5ms => f = 1kHz
	PWM_TimeBase.TIM_ClockDivision 	= 0;
	PWM_TimeBase.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM13, &PWM_TimeBase);
	TIM_TimeBaseInit(TIM14, &PWM_TimeBase);

	PWM_OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	PWM_OCInit.TIM_OutputState 	= TIM_OutputState_Enable;
	PWM_OCInit.TIM_Pulse 		= 0;
	PWM_OCInit.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OC1Init(TIM13, &PWM_OCInit);
	TIM_OC1Init(TIM14, &PWM_OCInit);
	
	TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM13, ENABLE);
	TIM_ARRPreloadConfig(TIM14, ENABLE);

	TIM_Cmd(TIM13, ENABLE);
	TIM_Cmd(TIM14, ENABLE);	
}

void TIM8_PWM_PITCH_ROLL_Config(void)
{	// PC8 - PC9 | TIM8_CH3 - TIM8 - CH4 => PITCH	// PC6 - PC7 | TIM8_CH1 - TIM8 - CH2 => ROLL Checked
	GPIO_InitTypeDef 				PWM_PIN;
	TIM_TimeBaseInitTypeDef PWM_TimeBase;
	TIM_OCInitTypeDef 			PWM_OCInit;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);	
	
	PWM_PIN.GPIO_Mode 	= GPIO_Mode_AF;
	PWM_PIN.GPIO_Speed 	= GPIO_Speed_100MHz;
	PWM_PIN.GPIO_OType 	= GPIO_OType_PP;
	PWM_PIN.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	PWM_PIN.GPIO_Pin 		= GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &PWM_PIN);

	PWM_TimeBase.TIM_Prescaler 			= 83;			// f_prescaler = 1MHz
	PWM_TimeBase.TIM_Period 				= 1000;		// T_period  = 0.5ms => f = 1kHz
	PWM_TimeBase.TIM_ClockDivision 	= 0;
	PWM_TimeBase.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &PWM_TimeBase);

	PWM_OCInit.TIM_OCMode 			= TIM_OCMode_PWM1;
	PWM_OCInit.TIM_OutputState 	= TIM_OutputState_Enable;
	PWM_OCInit.TIM_Pulse 				= 0;
	PWM_OCInit.TIM_OCPolarity 	= TIM_OCPolarity_High;

	TIM_OC1Init(TIM8, &PWM_OCInit);
	TIM_OC2Init(TIM8, &PWM_OCInit);
	TIM_OC3Init(TIM8, &PWM_OCInit);
	TIM_OC4Init(TIM8, &PWM_OCInit);
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	
	/* Config for Advanced Timer */
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStruct);
	TIM_CCPreloadControl(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	
	TIM_Cmd(TIM8, ENABLE);
}

void TIM12_GPIO_GRIPPER_Config(void) // CHECKKKKKKKKKKKKKKKKK
{	// PB14 - TIM_12_PWM_CH1 || PD8 PD10 - GPIO DIR1 + DIR2

	/* PWM GRIPPER INIT - TIM12_CH1 */
	GPIO_InitTypeDef 		PWM_PIN;
	TIM_TimeBaseInitTypeDef PWM_TimeBase;
	TIM_OCInitTypeDef 		PWM_OCInit;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14 , GPIO_AF_TIM12);
	
	PWM_PIN.GPIO_Mode 	= GPIO_Mode_AF;
	PWM_PIN.GPIO_Speed 	= GPIO_Speed_100MHz;
	PWM_PIN.GPIO_OType 	= GPIO_OType_PP;
	PWM_PIN.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	PWM_PIN.GPIO_Pin 	= GPIO_Pin_14;
	GPIO_Init(GPIOB, &PWM_PIN);

	PWM_TimeBase.TIM_Prescaler 		= 83;		// f_prescaler = 1MHz
	PWM_TimeBase.TIM_Period 			= 1000;		// T_period  = 0.5ms => f = 1kHz
	PWM_TimeBase.TIM_ClockDivision 	= 0;
	PWM_TimeBase.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM12, &PWM_TimeBase);

	PWM_OCInit.TIM_OCMode 			= TIM_OCMode_PWM1;
	PWM_OCInit.TIM_OutputState 	= TIM_OutputState_Enable;
	PWM_OCInit.TIM_Pulse 				= 0;
	PWM_OCInit.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OC1Init(TIM12, &PWM_OCInit);

	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM12, ENABLE);

	TIM_Cmd(TIM12, ENABLE);	
	
	/* DIR GRIPPER INIT - GPIO */
	GPIO_InitTypeDef GPIO_DIR;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
	GPIO_DIR.GPIO_Pin 	= GPIO_Pin_8 | GPIO_Pin_10;
	GPIO_DIR.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_DIR.GPIO_OType = GPIO_OType_PP;
	GPIO_DIR.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_DIR.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_DIR);
}

void USART2_Init(void)
{	// PD5 - PD6 | TX - RX

	/* Pinout Config */
	GPIO_InitTypeDef GPIO_USART;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	GPIO_USART.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_USART.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_USART.GPIO_OType = GPIO_OType_PP;
	GPIO_USART.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_USART.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD  , &GPIO_USART);
	
	
	/* USART Config */
	USART_InitTypeDef USART_CONFIG;
	USART_CONFIG.USART_BaudRate 				= 921600;
	USART_CONFIG.USART_HardwareFlowControl 		= USART_HardwareFlowControl_None;
	USART_CONFIG.USART_Mode 						= USART_Mode_Tx | USART_Mode_Rx;
	USART_CONFIG.USART_Parity 					= USART_Parity_No;
	USART_CONFIG.USART_StopBits 				= USART_StopBits_1;
	USART_CONFIG.USART_WordLength 			= USART_WordLength_8b;
	
	USART_Init(USART2, &USART_CONFIG);
	
	USART_Cmd(USART2, ENABLE);

	/* USART Receive Data Interupt Config */	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_InitTypeDef NVIC_RX_INTERUPT;
	NVIC_RX_INTERUPT.NVIC_IRQChannel 						= USART2_IRQn;
	NVIC_RX_INTERUPT.NVIC_IRQChannelCmd 				= ENABLE;
	NVIC_RX_INTERUPT.NVIC_IRQChannelPreemptionPriority 	= 0;
	NVIC_RX_INTERUPT.NVIC_IRQChannelSubPriority 				= 0;
	NVIC_Init(&NVIC_RX_INTERUPT);	
}
