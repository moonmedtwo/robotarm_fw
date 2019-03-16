/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <user_timing.h>

#include "invKin.h"
#include "configuration.h"
#include "controller.h"
#include "matrix.h"
#include "user_uart.h"

uint32_t msTicks_Ctrl = 0, Int_Cnt = 0;

uint8_t DEBUG = 0;

/* Structure to store data of Servos. */
dcServoMotor_t BASE, SHLDR, ELBOW, PITCH, ROLL;
robotGripper_t GRIPPER;

/* Destination Homogenous Matrix and set of joint angles. */
jointSet_t 		jointSet, 		jointMeasured ;
homoMatrix_t 	dstPosition, 	meaPosition;

/* Global Command. */
protocolCmd_t GlobalCommand = CMD_ERROR;

/* Private function prototypes -----------------------------------------------*/
void TIM7_IRQHandler(void);					

/* Private functions ---------------------------------------------------------*/
void TIM7_IRQHandler(void)
{ // Sampling routine at TIM7 - 20ms
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		
		// Toggle LED PD13 - Entering Interupt for DEBUG:
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		
		// Manual control each JOINT by STM32 Studio - DEBUGGING:
		update_manual_desiredPosition(&jointSet);
		
		if (GlobalCommand == CMD_STOP)
		{
			update_holdPosition(&jointSet, &jointMeasured);
		}

		getUpdateMotorState(&BASE, TIM2->CNT);
		getUpdateMotorState(&SHLDR, *(int16_t*)(&(TIM3->CNT)));
		getUpdateMotorState(&ELBOW, *(int16_t*)(&(TIM4->CNT)));
		getUpdateMotorState(&PITCH, TIM5->CNT);		
		getUpdateMotorState(&ROLL, *(int16_t*)(&(TIM1->CNT)));

//		if (DEBUG == 0)
//		{
//			if ( (BASE.DcMotor.State.measuredPosition > T1MAX) || (BASE.DcMotor.State.measuredPosition < T1MIN) ||
//					 (SHLDR.DcMotor.State.measuredPosition > T2MAX) || (SHLDR.DcMotor.State.measuredPosition < T2MIN) ||
//					 (ELBOW.DcMotor.State.measuredPosition > T3MAX) || (ELBOW.DcMotor.State.measuredPosition < T3MIN) ||
//					 (PITCH.DcMotor.State.measuredPosition > T4MAX) || (PITCH.DcMotor.State.measuredPosition < T4MIN) ||
//					 (ROLL.DcMotor.State.measuredPosition > T5MAX) || (ROLL.DcMotor.State.measuredPosition < T5MIN) )
//				GlobalCommand == CMD_STOP;
//		}		
		
		// Control Motor 1 - Theta 1		
		motorPathPlanning_BASE(&BASE, 0.8);
		motorStrPositionController_BASE(&BASE);
		generateControlSignal(&BASE,TIM9,CH1,TIM9,CH2);

		// Control Motor 2 - Theta 2
		motorPathPlanning_SHOULDER(&SHLDR, 1);
		motorStrPositionController_SHOULDER(&SHLDR);
		generateControlSignal(&SHLDR,TIM10,CH1,TIM11,CH1);
	
		// Control Motor 3 - Theta 2 + Theta 3
		motorPathPlanning_ELBOW(&ELBOW, 1);
		motorStrPositionController_ELBOW(&ELBOW);
		generateControlSignal(&ELBOW,TIM14,CH1,TIM13,CH1);
		
		// Control Motor 4 - Theta 4
		motorPathPlanning_PITCH(&PITCH, 1);
		motorStrPositionController_PITCH(&PITCH);
		generateControlSignal(&PITCH,TIM8,CH3,TIM8,CH4);
		
		// Control Motor 5 - Theta 5
		motorPathPlanning_ROLL(&ROLL, 2);
		motorStrPositionController_ROLL(&ROLL);
		generateControlSignal(&ROLL,TIM8,CH2,TIM8,CH1);
		
		// Executing gripper command:
		if 			(GlobalCommand == CMD_GRAB)		GRIPPER.gripperAction = GRIPPER_GRAB;
		else if 	(GlobalCommand == CMD_DROP)		GRIPPER.gripperAction = GRIPPER_DROP;	
		controlGripper(&GRIPPER);
		
		/* Calculate forKin for current position. */
		update_measuredPosition(&meaPosition, &jointMeasured);
		
		/* DEBUG VAR. */
		Int_Cnt += 5;
	}
}

void USART2_IRQHandler(void)
	{	// USART2 Received Interupt:
		
    USART_STATUS = USART2->SR;
		
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{ // Enter interrupt when STM32 receice data.
    uint8_t ReceivedData;
			
		/* Receive each byte and put in the received package order. */
    ReceivedData = USART_ReceiveData(USART2);
		USART_RX_Handler(ReceivedData,&USART2_Ctrl);
		
		/* Clear interupt pendung bit. */
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART_STATUS = USART2->SR;
    }
}

void SysTick_Handler(void) 
{ 
		// Toggle LED PD14 - Entering SysTick for DEBUG:
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
	
		msTicks++;
		msTicks_Ctrl++;
}
