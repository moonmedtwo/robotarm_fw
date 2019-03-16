#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
    extern "C" {
#endif 

#include "stm32f4xx.h"
			
#define INIT_ENABLE	 		(uint8_t)	  0
#define INIT_DISABLE 		(uint8_t)	  1			

#define MODE_SELFHOLDING	(uint8_t)	  0
#define MODE_PROCESSING		(uint8_t)	  1
#define MODE_OPERATING		(uint8_t)	  2

#define GRIPPER_NONE		(uint8_t)	  0
#define GRIPPER_GRAB		(uint8_t)	  1
#define GRIPPER_DROP		(uint8_t)	  2

#define GRIPPER_PULSEW	(uint16_t)	150
#define GRIPPER_TIMER		(uint16_t)	220

#define CH1					(uint8_t)	  1
#define CH2					(uint8_t)	  2
#define CH3					(uint8_t)	  3
#define CH4					(uint8_t)	  4

typedef	struct {
	double desiredPosition[2];// Degree
	double plannedPosition;		// Pulses
	double measuredPosition;	// Degree				 // = pulseCount[0] * 360 / pulsePerRound
	double measuredSpeed;			// Pulses per Sec
	double measuredError;
	double PulseWidth;	
} dcMotorState_t;

typedef struct {
	double	maxSpeed;			// Pulse Per Sec // = pulsePerRound * 25 RPM / 60 = 
	double	maxPulse;			// 1000 Cycles;
	double	minPulse;			// 0 Cycles;
	double acceptedError;		// Degree
} dcMotorParam_t;

typedef	struct {
	int32_t	pulsePerRound;		// in Pulses	// = EncoderPulse * Mode x4 * GearRatio * MechRatio
} encoderParam_t;

typedef	struct {
	int32_t pulseCount[2];
} encoderState_t;

typedef struct {
	dcMotorState_t State;
	dcMotorParam_t Param;
} DcMotor_t;

typedef struct {
	encoderParam_t Param;
	encoderState_t State;
} Encoder_t;

typedef struct {
	DcMotor_t DcMotor;
	Encoder_t Encoder;
}	dcServoMotor_t;

typedef struct {
	uint8_t gripperAction, gripStatus, remAction;
	uint16_t gripTimer;
}	robotGripper_t;

void motorPathPlanning_BASE(dcServoMotor_t * , double );

void motorStrPositionController_BASE(dcServoMotor_t * );

void motorPathPlanning_SHOULDER(dcServoMotor_t * , double );

void motorStrPositionController_SHOULDER(dcServoMotor_t * );

void motorPathPlanning_ELBOW(dcServoMotor_t * , double );

void motorStrPositionController_ELBOW(dcServoMotor_t * );

void motorPathPlanning_PITCH(dcServoMotor_t * , double );

void motorStrPositionController_PITCH(dcServoMotor_t * );

void motorPathPlanning_ROLL(dcServoMotor_t * , double );

void motorStrPositionController_ROLL(dcServoMotor_t * );

void getUpdateMotorState(dcServoMotor_t * dcServo, int32_t encoderCount); 

void generateControlSignal(dcServoMotor_t * MOTOR, TIM_TypeDef * TIMrght, uint8_t chRGHT, TIM_TypeDef * TIMleft, uint8_t chLEFT);

void controlGripper(robotGripper_t * gripper);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROLLER_H */
