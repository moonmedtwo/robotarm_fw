#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#ifdef __cplusplus
    extern "C" {
#endif 
	
void InitializeMotorParam(void);			// Initialize Param For Motor Control
void Interupt_Routine_Config(void);			// Interrupt Sampling

// Encoder Reading:			
void TIM2_ENC_BASE_Config(void);
void TIM3_ENC_SHLDR_Config(void);
void TIM4_ENC_ELBOW_Config(void);
void TIM5_ENC_PITCH_Config(void);
void TIM1_ENC_ROLL_Config(void);	
// Pulse Width Modulation:
void TIM9_PWM_BASE_Config(void);
void TIM10_TIM11_PWM_SHLDR_Config(void);
void TIM13_TIM14_PWM_ELBOW_Config(void);
void TIM8_PWM_PITCH_ROLL_Config(void);

// GRIPPER:
void TIM12_GPIO_GRIPPER_Config(void);

// USART Comm: 
void USART2_Init(void);

// Blinky LED for debugging purpose: 
void LED_DEBUG_Config(void);

// Main Loop Delay for Receive data processing:
void DelayInit(void);
void DelayMs(uint32_t Millis);
void DelayUs(uint32_t Micros);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIGURATION_H */
