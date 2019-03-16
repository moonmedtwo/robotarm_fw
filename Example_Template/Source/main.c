#include "stm32f4xx.h"
#include "system_timetick.h"
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_gpio.h"

#include "controller.h"
#include "configuration.h"
#include "user_uart.h"
#include "util.h"
#include "invKin.h"
#include "user_timing.h"
#include "eeprom.h"

/* Multiplier for Main Loop Delay. */
extern uint32_t Multiplier;

/* Destination Homogenous Matrix. */
extern jointSet_t 		jointSet, 		jointMeasured ;
extern homoMatrix_t 	dstPosition, 	meaPosition;

/* GlobalCommand takes from PC */
extern protocolCmd_t GlobalCommand;
double sqr_euclideDistance;

uint16_t VirtAddVarTab[NB_OF_VAR];

uint8_t TEST = 0, CAL = 0;

/* Main Program */
int main(void)
{
	LED_DEBUG_Config();

	/* Encoder Config for reading Encoder */
	TIM2_ENC_BASE_Config();
	TIM3_ENC_SHLDR_Config();
	TIM4_ENC_ELBOW_Config();
	TIM5_ENC_PITCH_Config();
	TIM1_ENC_ROLL_Config();

	
	/* PWM Config to control Actuators */
	TIM9_PWM_BASE_Config();
	TIM10_TIM11_PWM_SHLDR_Config();
	TIM13_TIM14_PWM_ELBOW_Config();
	TIM8_PWM_PITCH_ROLL_Config();
	TIM12_GPIO_GRIPPER_Config();

	/* Init Clock for Main Loop Delay */	DelayInit();
	/* Communication USART Config */			USART2_Init();
	/* Controller of USART Comm Data */		usart_Ctrl_Init(&USART2_Ctrl,USART2);
	
	calculate_ForKin(&dstPosition, &jointSet);
	
	/* Interupt Config. */
	InitializeMotorParam();		
	Interupt_Routine_Config();
	SysTick_Config(SystemCoreClock / 1000); // SysTick Period = 1ms
	
	GPIO_SetBits(GPIOD,GPIO_Pin_15);
	
	while (1)
	{
			
		/* Check the CRC Byte and upate to the flag. */
		if( user_checkFlag(&(USART2_Ctrl.Status),UART_CHECKCRC) )
		{
			uint8_t * pData = NULL;
			
			/* Parse Command from Received Data */
			GlobalCommand = parseCommand_DataUSART(&USART2_Ctrl,&pData);
			
			/* Check if Global command is right. */
			if (GlobalCommand != CMD_ERROR)
			{
				GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

				if(pData != NULL)
				{
					// Toggle LED PD12 - Command Received for DEBUG:
					GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
					
					/* Send back the ACK Package. */
					uint8_t ACK_PACKAGE[2] = {*pData, ACK};	// CMD and ACK
					sendDataPackage_USART(ACK_PACKAGE, 2, &USART2_Ctrl);
				
					/* Check the Command. */
					/* Update the state and param only. The actuators control part is processed in stm32f4xx_it.c. */
					switch (GlobalCommand)
					{
						case CMD_MOVETOXYZ:
						{
							/* 	Calculate inverse Kinematic: If Success, put in the set of destination for each joint.	If failed, send back the ERROR OUT_OF_RANGE Package.		*/
							dstPosition.P.x = Util_parseFloat(pData+1) * 1000;
							dstPosition.P.y = Util_parseFloat(pData+5) * 1000;
							dstPosition.P.z = Util_parseFloat(pData+9) * 1000;
														
							/* Set the approach from above. Can set to approach from horizontal. */
							set_UpwardApproach(&dstPosition);
							
							if ( update_desiredPosition(&dstPosition, &jointSet) == _NULL)
							{
									DelayMs(100);
									// Send out of range error.
									uint8_t OUTOFRANGE_PACKAGE[1] = {CMD_OUTOFRANGE}; 	
									sendDataPackage_USART(OUTOFRANGE_PACKAGE, 1, &USART2_Ctrl);
							}
						}
							break;
						default:
							break;
					}
						
				/* Free the processed data. */
				free(pData);
				}
			}
			/* Clear the CRC check flag. */
			user_clearFlag(&(USART2_Ctrl.Status),UART_CHECKCRC);
		}
	
		/* Test Inverse Kinematic by STM32 Studio - DEBUGGING*/
		
		sqr_euclideDistance = (dstPosition.P.x - meaPosition.P.x)*(dstPosition.P.x - meaPosition.P.x) + \
													(dstPosition.P.y - meaPosition.P.y)*(dstPosition.P.y - meaPosition.P.y) + \
													(dstPosition.P.z - meaPosition.P.z)*(dstPosition.P.z - meaPosition.P.z);
				
		if (TEST == 1)
		{
				CAL = 0;
				set_UpwardApproach(&dstPosition);
				if ( update_desiredPosition(&dstPosition, &jointSet) == _NULL )
				{
						CAL = 1;
				}		
				TEST = 0;
		}
		
		/* Send END_POINT PACKAGE. */
		static uint8_t Cnt = 0;		
		if(Cnt++ == 50)	// 20ms * 50 = 1s
		{
			uint8_t ENDPOINT_PACKAGE[13] = {CMD_ENDPOINT}; 
			Util_bufferFloat(ENDPOINT_PACKAGE + 1,meaPosition.P.x / 1000);
			Util_bufferFloat(ENDPOINT_PACKAGE + 5,meaPosition.P.y / 1000);
			Util_bufferFloat(ENDPOINT_PACKAGE + 9,meaPosition.P.z / 1000);
			sendDataPackage_USART(ENDPOINT_PACKAGE, 13, &USART2_Ctrl);
			
			Cnt = 0;
		}

		DelayMs(20);
	}
}
