#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_it.h"

#include <controller.h>
#include <matrix.h>
#include <user_uart.h>

#define a1m  0.9757
#define a2m  0.7203
#define a3m  1.596e-05
#define b1m  0.04638
#define b2m  0.001349
#define b3m  -1.387e-21

extern uint32_t 			msTicks_Ctrl;
extern protocolCmd_t 	GlobalCommand;

// Path Planning:

void motorPathPlanning_BASE(dcServoMotor_t * MOTOR, double optTIME)
{
	static double xf, x0, x01, x02, v01, vmax, sTicks;
	static uint8_t optSTATUS; /* 0> idle, hold position   1> Set point have just updated, calculate params	  2> Tracking trajectory */

	if ((*MOTOR).DcMotor.State.desiredPosition[0] != (*MOTOR).DcMotor.State.desiredPosition[1])
			optSTATUS = MODE_PROCESSING;

	(*MOTOR).DcMotor.State.desiredPosition[1] = (*MOTOR).DcMotor.State.desiredPosition[0];			

	switch (optSTATUS) 
	{
		case MODE_SELFHOLDING: /* 0> Idle - Self holding position */ 	  
			{
				break;
			}
		case MODE_PROCESSING: /* 1> Set point is updated, calculate params */
			{	
				xf  = (*MOTOR).DcMotor.State.desiredPosition[0];
				x0  = (*MOTOR).DcMotor.State.measuredPosition;
				v01 = 0; //(*MOTOR).DcMotor.State.measuredSpeed;

				vmax = (xf - x0) / (0.8f * optTIME) - v01 / 8;
				
				if (fabs(vmax) >= (*MOTOR).DcMotor.Param.maxSpeed)
				{
					(*MOTOR).DcMotor.State.plannedPosition = (*MOTOR).DcMotor.State.measuredPosition;
					(*MOTOR).DcMotor.State.desiredPosition[0] = (*MOTOR).DcMotor.State.measuredPosition;
					optSTATUS = MODE_SELFHOLDING;
					break;
				}
				else
				{
					x01 = x0  +  v01 * (1 * optTIME / 5) + 0.5f * (vmax - v01) * (optTIME / 5);
					x02 = x01 + vmax * (3 * optTIME / 5);
					optSTATUS = MODE_OPERATING;
					msTicks_Ctrl = 0;
				}
			}
		case MODE_OPERATING: /* 2> Tracking trajectory */
			{ 
				sTicks = (double) msTicks_Ctrl/1000;
				
				if 		(sTicks <= 	1*optTIME/5) 	  					
					(*MOTOR).DcMotor.State.plannedPosition = x0  +  v01 * sTicks + 2.5f * (vmax - v01) * sTicks * sTicks / optTIME;
				else if (sTicks <=  4*optTIME/5)						
					(*MOTOR).DcMotor.State.plannedPosition = x01 + vmax * (sTicks - optTIME/5);
				else if (sTicks <=    optTIME)							
					(*MOTOR).DcMotor.State.plannedPosition = x02 + vmax * (sTicks - 4*optTIME/5) - 0.5f * (5*vmax/optTIME) * (sTicks - 4*optTIME/5) * (sTicks - 4*optTIME/5);
				else	optSTATUS = MODE_SELFHOLDING;

				break;
			}
	}
}

void motorPathPlanning_SHOULDER(dcServoMotor_t * MOTOR, double optTIME)
{
	static double xf, x0, x01, x02, v01, vmax, sTicks;
	static uint8_t optSTATUS; /* 0> idle, hold position   1> Set point have just updated, calculate params	  2> Tracking trajectory */

	if ((*MOTOR).DcMotor.State.desiredPosition[0] != (*MOTOR).DcMotor.State.desiredPosition[1])
			optSTATUS = MODE_PROCESSING;

	(*MOTOR).DcMotor.State.desiredPosition[1] = (*MOTOR).DcMotor.State.desiredPosition[0];			

	switch (optSTATUS) 
	{
		case MODE_SELFHOLDING: /* 0> Idle - Self holding position */ 	  
			{
				break;
			}
		case MODE_PROCESSING: /* 1> Set point is updated, calculate params */
			{	
				xf  = (*MOTOR).DcMotor.State.desiredPosition[0];
				x0  = (*MOTOR).DcMotor.State.measuredPosition;
				v01 = 0; //(*MOTOR).DcMotor.State.measuredSpeed;

				vmax = (xf - x0) / (0.8f * optTIME) - v01 / 8;
				if  (( ((xf - x0) > 0) && ((xf - x0) < 20)) 
					  || ((xf - x0) < 0) )
				{
					(*MOTOR).DcMotor.State.plannedPosition = xf;
					break;
				}
				
				if (fabs(vmax) >= (*MOTOR).DcMotor.Param.maxSpeed)
				{
					(*MOTOR).DcMotor.State.plannedPosition = (*MOTOR).DcMotor.State.measuredPosition;
					(*MOTOR).DcMotor.State.desiredPosition[0] = (*MOTOR).DcMotor.State.measuredPosition;
					optSTATUS = MODE_SELFHOLDING;
					break;
				}
				else
				{
					x01 = x0  +  v01 * (1 * optTIME / 5) + 0.5f * (vmax - v01) * (optTIME / 5);
					x02 = x01 + vmax * (3 * optTIME / 5);
					optSTATUS = MODE_OPERATING;
					msTicks_Ctrl = 0;
				}
			}
		case MODE_OPERATING: /* 2> Tracking trajectory */
			{ 
				sTicks = (double) msTicks_Ctrl/1000;
				
				if 		(sTicks <= 	1*optTIME/5) 	  					
					(*MOTOR).DcMotor.State.plannedPosition = x0  +  v01 * sTicks + 2.5f * (vmax - v01) * sTicks * sTicks / optTIME;
				else if (sTicks <=  4*optTIME/5)						
					(*MOTOR).DcMotor.State.plannedPosition = x01 + vmax * (sTicks - optTIME/5);
				else if (sTicks <=    optTIME)							
					(*MOTOR).DcMotor.State.plannedPosition = x02 + vmax * (sTicks - 4*optTIME/5) - 0.5f * (5*vmax/optTIME) * (sTicks - 4*optTIME/5) * (sTicks - 4*optTIME/5);
				else	optSTATUS = MODE_SELFHOLDING;
			}
	}
}

void motorPathPlanning_ELBOW(dcServoMotor_t * MOTOR, double optTIME)
{
	static double xf, x0, x01, x02, v01, vmax, sTicks;
	static uint8_t optSTATUS; /* 0> idle, hold position   1> Set point have just updated, calculate params	  2> Tracking trajectory */

	if ((*MOTOR).DcMotor.State.desiredPosition[0] != (*MOTOR).DcMotor.State.desiredPosition[1])
			optSTATUS = MODE_PROCESSING;

	(*MOTOR).DcMotor.State.desiredPosition[1] = (*MOTOR).DcMotor.State.desiredPosition[0];			

	switch (optSTATUS) 
	{
		case MODE_SELFHOLDING: /* 0> Idle - Self holding position */ 	  
			{
				break;
			}
		case MODE_PROCESSING: /* 1> Set point is updated, calculate params */
			{	
				xf  = (*MOTOR).DcMotor.State.desiredPosition[0];
				x0  = (*MOTOR).DcMotor.State.measuredPosition;
				v01 = 0; //(*MOTOR).DcMotor.State.measuredSpeed;

				vmax = (xf - x0) / (0.8f * optTIME) - v01 / 8;

				if (fabs(vmax) >= (*MOTOR).DcMotor.Param.maxSpeed)
				{
					(*MOTOR).DcMotor.State.plannedPosition = (*MOTOR).DcMotor.State.measuredPosition;
					(*MOTOR).DcMotor.State.desiredPosition[0] = (*MOTOR).DcMotor.State.measuredPosition;
					optSTATUS = MODE_SELFHOLDING;
					break;
				}
				else
				{
					x01 = x0  +  v01 * (1 * optTIME / 5) + 0.5f * (vmax - v01) * (optTIME / 5);
					x02 = x01 + vmax * (3 * optTIME / 5);
					optSTATUS = MODE_OPERATING;
					msTicks_Ctrl = 0;
				}
			}
		case MODE_OPERATING: /* 2> Tracking trajectory */
			{ 
				sTicks = (double) msTicks_Ctrl/1000;
				
				if 		(sTicks <= 	1*optTIME/5) 	  					
					(*MOTOR).DcMotor.State.plannedPosition = x0  +  v01 * sTicks + 2.5f * (vmax - v01) * sTicks * sTicks / optTIME;
				else if (sTicks <=  4*optTIME/5)						
					(*MOTOR).DcMotor.State.plannedPosition = x01 + vmax * (sTicks - optTIME/5);
				else if (sTicks <=    optTIME)							
					(*MOTOR).DcMotor.State.plannedPosition = x02 + vmax * (sTicks - 4*optTIME/5) - 0.5f * (5*vmax/optTIME) * (sTicks - 4*optTIME/5) * (sTicks - 4*optTIME/5);
				else	optSTATUS = MODE_SELFHOLDING;
			}
	}
}

void motorPathPlanning_PITCH(dcServoMotor_t * MOTOR, double optTIME)
{
	static double xf, x0, x01, x02, v01, vmax, sTicks;
	static uint8_t optSTATUS; /* 0> idle, hold position   1> Set point have just updated, calculate params	  2> Tracking trajectory */

	if ((*MOTOR).DcMotor.State.desiredPosition[0] != (*MOTOR).DcMotor.State.desiredPosition[1])
			optSTATUS = MODE_PROCESSING;

	(*MOTOR).DcMotor.State.desiredPosition[1] = (*MOTOR).DcMotor.State.desiredPosition[0];			

	switch (optSTATUS) 
	{
		case MODE_SELFHOLDING: /* 0> Idle - Self holding position */ 	  
			{
				break;
			}
		case MODE_PROCESSING: /* 1> Set point is updated, calculate params */
			{	
				xf  = (*MOTOR).DcMotor.State.desiredPosition[0];
				x0  = (*MOTOR).DcMotor.State.measuredPosition;
				v01 = 0; //(*MOTOR).DcMotor.State.measuredSpeed;

				vmax = (xf - x0) / (0.8f * optTIME) - v01 / 8;

				if (fabs(vmax) >= (*MOTOR).DcMotor.Param.maxSpeed)
				{
					(*MOTOR).DcMotor.State.plannedPosition = (*MOTOR).DcMotor.State.measuredPosition;
					(*MOTOR).DcMotor.State.desiredPosition[0] = (*MOTOR).DcMotor.State.measuredPosition;
					optSTATUS = MODE_SELFHOLDING;
					break;
				}
				else
				{
					x01 = x0  +  v01 * (1 * optTIME / 5) + 0.5f * (vmax - v01) * (optTIME / 5);
					x02 = x01 + vmax * (3 * optTIME / 5);
					optSTATUS = MODE_OPERATING;
					msTicks_Ctrl = 0;
				}
			}
		case MODE_OPERATING: /* 2> Tracking trajectory */
			{ 
				sTicks = (double) msTicks_Ctrl/1000;
				
				if 		(sTicks <= 	1*optTIME/5) 	  					
					(*MOTOR).DcMotor.State.plannedPosition = x0  +  v01 * sTicks + 2.5f * (vmax - v01) * sTicks * sTicks / optTIME;
				else if (sTicks <=  4*optTIME/5)						
					(*MOTOR).DcMotor.State.plannedPosition = x01 + vmax * (sTicks - optTIME/5);
				else if (sTicks <=    optTIME)							
					(*MOTOR).DcMotor.State.plannedPosition = x02 + vmax * (sTicks - 4*optTIME/5) - 0.5f * (5*vmax/optTIME) * (sTicks - 4*optTIME/5) * (sTicks - 4*optTIME/5);
				else	optSTATUS = MODE_SELFHOLDING;

				break;
			}
	}
}

void motorPathPlanning_ROLL(dcServoMotor_t * MOTOR, double optTIME)
{
	static double xf, x0, x01, x02, v01, vmax, sTicks;
	static uint8_t optSTATUS; /* 0> idle, hold position   1> Set point have just updated, calculate params	  2> Tracking trajectory */

	if ((*MOTOR).DcMotor.State.desiredPosition[0] != (*MOTOR).DcMotor.State.desiredPosition[1])
			optSTATUS = MODE_PROCESSING;

	(*MOTOR).DcMotor.State.desiredPosition[1] = (*MOTOR).DcMotor.State.desiredPosition[0];			

	switch (optSTATUS) 
	{
		case MODE_SELFHOLDING: /* 0> Idle - Self holding position */ 	  
			{
				break;
			}
		case MODE_PROCESSING: /* 1> Set point is updated, calculate params */
			{	
				xf  = (*MOTOR).DcMotor.State.desiredPosition[0];
				x0  = (*MOTOR).DcMotor.State.measuredPosition;
				v01 = 0; //(*MOTOR).DcMotor.State.measuredSpeed;

				vmax = (xf - x0) / (0.8f * optTIME) - v01 / 8;

				if (fabs(vmax) >= (*MOTOR).DcMotor.Param.maxSpeed)
				{
					(*MOTOR).DcMotor.State.plannedPosition = (*MOTOR).DcMotor.State.measuredPosition;
					(*MOTOR).DcMotor.State.desiredPosition[0] = (*MOTOR).DcMotor.State.measuredPosition;
					optSTATUS = MODE_SELFHOLDING;
					break;
				}
				else
				{
					x01 = x0  +  v01 * (1 * optTIME / 5) + 0.5f * (vmax - v01) * (optTIME / 5);
					x02 = x01 + vmax * (3 * optTIME / 5);
					optSTATUS = MODE_OPERATING;
					msTicks_Ctrl = 0;
				}
			}
		case MODE_OPERATING: /* 2> Tracking trajectory */
			{ 
				sTicks = (double) msTicks_Ctrl/1000;
				
				if 		(sTicks <= 	1*optTIME/5) 	  					
					(*MOTOR).DcMotor.State.plannedPosition = x0  +  v01 * sTicks + 2.5f * (vmax - v01) * sTicks * sTicks / optTIME;
				else if (sTicks <=  4*optTIME/5)						
					(*MOTOR).DcMotor.State.plannedPosition = x01 + vmax * (sTicks - optTIME/5);
				else if (sTicks <=    optTIME)							
					(*MOTOR).DcMotor.State.plannedPosition = x02 + vmax * (sTicks - 4*optTIME/5) - 0.5f * (5*vmax/optTIME) * (sTicks - 4*optTIME/5) * (sTicks - 4*optTIME/5);
				else	optSTATUS = MODE_SELFHOLDING;

				break;
			}
	}
}

// Object Control:

void generateControlSignal(dcServoMotor_t * MOTOR, TIM_TypeDef* TIMrght, uint8_t chRGHT, TIM_TypeDef* TIMleft, uint8_t chLEFT)
{
	uint32_t PulseW;
	PulseW = (uint32_t) fabs( (*MOTOR).DcMotor.State.PulseWidth );

	if ((*MOTOR).DcMotor.State.PulseWidth >= 0 )
	{
			switch (chRGHT) 
			{
				case 1: {TIMrght->CCR1 = PulseW; break; }
				case 2: {TIMrght->CCR2 = PulseW; break; }
				case 3: {TIMrght->CCR3 = PulseW; break; }
				case 4: {TIMrght->CCR4 = PulseW; break; }
			}
			switch (chLEFT) 
			{
				case 1: {TIMleft->CCR1 = 0; break; }
				case 2: {TIMleft->CCR2 = 0; break; }
				case 3: {TIMleft->CCR3 = 0; break; }
				case 4: {TIMleft->CCR4 = 0; break; }
			}			
	}
	else if ((*MOTOR).DcMotor.State.PulseWidth < 0 )
	{
			switch (chRGHT) 
			{
				case 1: {TIMrght->CCR1 = 0; break; }
				case 2: {TIMrght->CCR2 = 0; break; }
				case 3: {TIMrght->CCR3 = 0; break; }
				case 4: {TIMrght->CCR4 = 0; break; }
			}
			switch (chLEFT) 
			{
				case 1: {TIMleft->CCR1 = PulseW; break; }
				case 2: {TIMleft->CCR2 = PulseW; break; }
				case 3: {TIMleft->CCR3 = PulseW; break; }
				case 4: {TIMleft->CCR4 = PulseW; break; }
			}	
	}
}

void getUpdateMotorState(dcServoMotor_t * dcServo, int32_t encoderCount)
{
	(*dcServo).Encoder.State.pulseCount[1] = (*dcServo).Encoder.State.pulseCount[0];
	(*dcServo).Encoder.State.pulseCount[0] = encoderCount;
	
	(*dcServo).DcMotor.State.measuredPosition = (double) (*dcServo).Encoder.State.pulseCount[0] * 360 / (*dcServo).Encoder.Param.pulsePerRound;	
	(*dcServo).DcMotor.State.measuredError    = (double) (*dcServo).DcMotor.State.desiredPosition[0] - (*dcServo).DcMotor.State.measuredPosition;
}

void controlGripper(robotGripper_t * gripper)
{
	if  (((*gripper).gripStatus == GRIPPER_NONE) && ((*gripper).gripperAction != (*gripper).remAction))
	{
		/* Executing gripper action. */
		switch ((*gripper).gripperAction)
		{
			case (GRIPPER_NONE):
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_8);
				GPIO_SetBits(GPIOD, GPIO_Pin_10);
				TIM12->CCR1 = 0;
				
				(*gripper).gripTimer  	= GRIPPER_TIMER;
				(*gripper).gripStatus 	= GRIPPER_NONE;
				break;
			}
			case (GRIPPER_GRAB):
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_8);
				GPIO_SetBits(GPIOD, GPIO_Pin_10);
				TIM12->CCR1 = GRIPPER_PULSEW;
				 
				(*gripper).remAction 		= GRIPPER_GRAB;
				(*gripper).gripStatus 	= GRIPPER_GRAB;
				break;				
			}
			case (GRIPPER_DROP):
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_8);
				GPIO_ResetBits(GPIOD, GPIO_Pin_10);
				TIM12->CCR1 = GRIPPER_PULSEW;		
				
				(*gripper).remAction 	= GRIPPER_DROP;
				(*gripper).gripStatus = GRIPPER_DROP;				
				break;				
			}
			default:
				break;
		}
	}	
	
	/* Checking the timer when gripper action is not NOCMD. */ 
	if ( ( ((*gripper).gripperAction == GRIPPER_GRAB) || ((*gripper).gripperAction == GRIPPER_DROP) ) && ((*gripper).gripStatus != GRIPPER_NONE) )
	{
		if ((*gripper).gripTimer != 0)
		{
			(*gripper).gripTimer--;
		}
		else
		{
			(*gripper).gripTimer = GRIPPER_TIMER;

			(*gripper).gripperAction = GRIPPER_NONE;
			(*gripper).gripStatus 	 = GRIPPER_NONE;
			TIM12->CCR1 = 0;
		}
	}	
}

// STR Control:

void motorStrPositionController_BASE(dcServoMotor_t * MOTOR)
{
	// Local variables declaration
		
		static double s0, s1, s2, t0, t1, t2, r1, r2;
		static double err_val[1], set_pos[4], out_val[5], mea_pos[5];
		
		static double lamda, lamda_predict, phiTPphi, epsilon;
		static double phi[6][6], L[6][6], phiT[6][6], temp[6][6], temp2[6][6], denominator[6][6], numerator[6][6], P[6][6], theta[6][6];
		
		static uint8_t firstCall;
		
	// Initialize

		if (firstCall == INIT_ENABLE)
		{	
			firstCall = INIT_DISABLE;
			
			lamda = 0.95; lamda_predict = 100; phiTPphi = 0; epsilon = 0;
			Clear_Matrix(L);
			Clear_Matrix(P);
			Clear_Matrix(theta);
			for(int i = 0; i < 6; i++) P[i][i] = 1;
			for(int i = 0; i < 3; i++)
			{
				set_pos[i] = 0;
				out_val[i] = 0;
				mea_pos[i] = 0;
			}
			theta[0][0] = -1;
		}
	
	// Estimation Algorithm

		phi[0][0] = - mea_pos[2];			
		phi[1][0] = - mea_pos[3];			
		phi[2][0] = - mea_pos[4];
		phi[3][0] = out_val[2];				
		phi[4][0] = out_val[3];				
		phi[5][0] = out_val[4];
		
		Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
		
		Multiply_Matrix(phiT,1,6,theta,6,1,temp);				// temp = phiT(k)*theta(k-1)
			epsilon = mea_pos[0] - temp[0][0];

			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
		Multiply_Matrix(P,6,6,phi,6,1,numerator);				// (P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P,6,6,temp);					// temp = phiT(k)*P(k-1)
		Multiply_Matrix(temp,1,6,phi,6,1,denominator);			// phiT(k)*P(k-1)*phi(k)

			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;			//(1 + phiT(k)*P(k-1)*phi(k))
		
		Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			lamda = (lamda > 1.00f) ? 1.00f : lamda; 
			lamda = (lamda < 0.95f) ? 0.95f : lamda;

			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		Multiply_Matrix(L,6,1,temp,1,6,temp2);	
		Sub_Matrix(P,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P); P[0][0] = 0;
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta,temp,6,1,theta);

		theta[3][0] = (theta[3][0] == 0) ? 1 : theta[3][0];
		
			/*Recursion*/
		set_pos[3] = set_pos[2];			set_pos[2] = set_pos[1];			set_pos[1] = set_pos[0];
		out_val[4] = out_val[3];			out_val[3] = out_val[2];			out_val[2] = out_val[1];			out_val[1] = out_val[0];
		mea_pos[4] = mea_pos[3];			mea_pos[3] = mea_pos[2];			mea_pos[2] = mea_pos[1];			mea_pos[1] = mea_pos[0];
	
	// Compensator

		set_pos[0] = (double) (*MOTOR).DcMotor.State.plannedPosition;
		mea_pos[0] = (double) (*MOTOR).DcMotor.State.measuredPosition;

		err_val[0] = set_pos[0] - mea_pos[0];

	// Position Model
	
		r1 = (double) theta[4][0] / theta[3][0];
		r2 = (double) theta[5][0] / theta[3][0];
		t0 = (double) a1m / theta[3][0];
		t1 = (double) a2m / theta[3][0];
		t2 = (double) a3m / theta[3][0];
		s0 = (double) (b1m - theta[0][0]) / theta[3][0];
		s1 = (double) (b2m - theta[1][0]) / theta[3][0];
		s2 = (double) (b3m - theta[2][0]) / theta[3][0];
		
		out_val[0] = - (r1 * out_val[1]) - (r2 * out_val[2])  												\
								 + (t0 * set_pos[0]) + (t1 * set_pos[1]) + (t2 * set_pos[2]) 			\
								 - (s0 * mea_pos[0]) - (s1 * mea_pos[1]) - (s2 * mea_pos[2]);

		out_val[0] = (out_val[0] >  (*MOTOR).DcMotor.Param.maxPulse) ?  (*MOTOR).DcMotor.Param.maxPulse : out_val[0];
		out_val[0] = (out_val[0] < -(*MOTOR).DcMotor.Param.maxPulse) ? -(*MOTOR).DcMotor.Param.maxPulse : out_val[0];		
		
		// First condition is to check whether error is too small, the motor stops.
		if (fabs(err_val[0]) < 0.3)
				out_val[0] = 0;
		// Second condition is to check if error < about 5 or 10 degree, multiply the control value with a ratio to prevent shaking in small movement.
		else if (fabs(err_val[0]) < (*MOTOR).DcMotor.Param.acceptedError)
				out_val[0] *= 0.6;
		
		// 
		if (((*MOTOR).DcMotor.State.desiredPosition[0] -  mea_pos[0]) * out_val[0] < 0)
		{
			out_val[0] *= -.8;
			//firstCall = INIT_ENABLE;
		}
		
		if (GlobalCommand == CMD_STOP)
		{
				for(int i = 0; i < 5; i++)	out_val[i] = 0;
		}

		(*MOTOR).DcMotor.State.PulseWidth = out_val[0];
}

void motorStrPositionController_SHOULDER(dcServoMotor_t * MOTOR)
{
	// Local variables declaration
		
		static double s0, s1, s2, t0, t1, t2, r1, r2;
		static double err_val[1], set_pos[4], out_val[6], mea_pos[6];
		
		static double lamda, lamda_predict, phiTPphi, epsilon;
		static double phi[6][6], L[6][6], phiT[6][6], temp[6][6], temp2[6][6], denominator[6][6], numerator[6][6], P[6][6], theta[6][6];
		
		static uint8_t firstCall;

		// Initialize

		if (firstCall == INIT_ENABLE)
		{	
			firstCall = INIT_DISABLE;
			
			lamda = 0.95; lamda_predict = 100; phiTPphi = 0; epsilon = 0;
			Clear_Matrix(L);
			Clear_Matrix(P);
			Clear_Matrix(theta);
			for(int i = 0; i < 6; i++) P[i][i] = 1;
			for(int i = 0; i < 4; i++)
			{
				set_pos[i] = 0;
				out_val[i] = 0;
				mea_pos[i] = 0;
			}
			theta[0][0] = -1;
		}
	
	// Estimation Algorithm
		
		phi[0][0] = - mea_pos[3];			
		phi[1][0] = - mea_pos[4];			
		phi[2][0] = - mea_pos[5];
		phi[3][0] = out_val[3];				
		phi[4][0] = out_val[4];				
		phi[5][0] = out_val[5];
		
		Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
		
		Multiply_Matrix(phiT,1,6,theta,6,1,temp);				// temp = phiT(k)*theta(k-1)
			epsilon = mea_pos[0] - temp[0][0];

			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
		Multiply_Matrix(P,6,6,phi,6,1,numerator);				// (P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P,6,6,temp);					// temp = phiT(k)*P(k-1)
		Multiply_Matrix(temp,1,6,phi,6,1,denominator);			// phiT(k)*P(k-1)*phi(k)

			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;			//(1 + phiT(k)*P(k-1)*phi(k))
		
		Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			lamda = (lamda > 1.00f) ? 1.00f : lamda; 
			lamda = (lamda < 0.95f) ? 0.95f : lamda;

			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		Multiply_Matrix(L,6,1,temp,1,6,temp2);	
		Sub_Matrix(P,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P); 
		P[0][0] = 0;

			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta,temp,6,1,theta);

		theta[3][0] = (theta[3][0] == 0) ? 1 : theta[3][0];
		
			/*Recursion*/
		set_pos[3] = set_pos[2];			set_pos[2] = set_pos[1];			set_pos[1] = set_pos[0];
		out_val[5] = out_val[4];			out_val[4] = out_val[3];			out_val[3] = out_val[2];			out_val[2] = out_val[1];			out_val[1] = out_val[0];
		mea_pos[5] = mea_pos[4];			mea_pos[4] = mea_pos[3];			mea_pos[3] = mea_pos[2];			mea_pos[2] = mea_pos[1];			mea_pos[1] = mea_pos[0];
	
	// Compensator

		set_pos[0] = (double) (*MOTOR).DcMotor.State.plannedPosition;
		mea_pos[0] = (double) (*MOTOR).DcMotor.State.measuredPosition;

		err_val[0] = set_pos[0] - mea_pos[0];
			if (fabs(err_val[0]) <= 1.5) 	err_val[0] = 0;
	// Position Model
	
		r1 = (double) theta[4][0] / theta[3][0];
		r2 = (double) theta[5][0] / theta[3][0];
		t0 = (double) a1m / theta[3][0];
		t1 = (double) a2m / theta[3][0];
		t2 = (double) a3m / theta[3][0];
		s0 = (double) (b1m - theta[0][0]) / theta[3][0];
		s1 = (double) (b2m - theta[1][0]) / theta[3][0];
		s2 = (double) (b3m - theta[2][0]) / theta[3][0];
		
		out_val[0] = - (r1 * out_val[1]) - (r2 * out_val[2])  												\
								 + (t0 * set_pos[0]) + (t1 * set_pos[1]) + (t2 * set_pos[2]) 			\
								 - (s0 * mea_pos[0]) - (s1 * mea_pos[1]) - (s2 * mea_pos[2]);

		out_val[0] = (out_val[0] >  (*MOTOR).DcMotor.Param.maxPulse) ?  (*MOTOR).DcMotor.Param.maxPulse : out_val[0];
		out_val[0] = (out_val[0] < -(*MOTOR).DcMotor.Param.maxPulse) ? -(*MOTOR).DcMotor.Param.maxPulse : out_val[0];		
		
		if (fabs(err_val[0]) <= .5)
				out_val[0] = 0;
		else if ( (fabs(err_val[0]) < (*MOTOR).DcMotor.Param.acceptedError) )
		{
				if ((*MOTOR).DcMotor.State.desiredPosition[0] -  mea_pos[0]) out_val[0] *= 0.8;
				else								out_val[0] *= 0.7;
		}
		
		if (((*MOTOR).DcMotor.State.desiredPosition[0] -  mea_pos[0]) * out_val[0] < 0)
		{
			out_val[0] *= -.8;
			//firstCall = INIT_ENABLE;
		}
		
		if (GlobalCommand == CMD_STOP)
		{
				for(int i = 0; i < 5; i++)	out_val[i] = 0;
		}
		
		(*MOTOR).DcMotor.State.PulseWidth = out_val[0];
}

void motorStrPositionController_ELBOW(dcServoMotor_t * MOTOR)
{
	// Local variables declaration
		
		static double s0, s1, s2, t0, t1, t2, r1, r2;
		static double err_val[1], set_pos[5], out_val[6], mea_pos[6];
		
		static double lamda, lamda_predict, phiTPphi, epsilon;
		static double phi[6][6], L[6][6], phiT[6][6], temp[6][6], temp2[6][6], denominator[6][6], numerator[6][6], P[6][6], theta[6][6];
		
		static uint8_t firstCall;
		
	// Initialize

		if (firstCall == INIT_ENABLE)
		{	
			firstCall = INIT_DISABLE;
			
			lamda = 0.95; lamda_predict = 100; phiTPphi = 0; epsilon = 0;
			Clear_Matrix(L);
			Clear_Matrix(P);
			Clear_Matrix(theta);
			for(int i = 0; i < 6; i++) P[i][i] = 1;
			for(int i = 0; i < 3; i++)
			{
				set_pos[i] = 0;
				out_val[i] = 0;
				mea_pos[i] = 0;
			}
			theta[0][0] = -1;
		}
	
	// Estimation Algorithm

		phi[0][0] = - mea_pos[2];			
		phi[1][0] = - mea_pos[3];			
		phi[2][0] = - mea_pos[4];
		phi[3][0] = out_val[2];				
		phi[4][0] = out_val[3];				
		phi[5][0] = out_val[4];
		
		Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
		
		Multiply_Matrix(phiT,1,6,theta,6,1,temp);				// temp = phiT(k)*theta(k-1)
			epsilon = mea_pos[0] - temp[0][0];

			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
		Multiply_Matrix(P,6,6,phi,6,1,numerator);				// (P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P,6,6,temp);					// temp = phiT(k)*P(k-1)
		Multiply_Matrix(temp,1,6,phi,6,1,denominator);			// phiT(k)*P(k-1)*phi(k)

			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;			//(1 + phiT(k)*P(k-1)*phi(k))
		
		Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			lamda = (lamda > 1.00f) ? 1.00f : lamda; 
			lamda = (lamda < 0.95f) ? 0.95f : lamda;

			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		Multiply_Matrix(L,6,1,temp,1,6,temp2);	
		Sub_Matrix(P,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P); 
		P[0][0] = 0;
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta,temp,6,1,theta);

		theta[3][0] = (theta[3][0] == 0) ? 1 : theta[3][0];
		
			/*Recursion*/
		set_pos[3] = set_pos[2];			set_pos[2] = set_pos[1];			set_pos[1] = set_pos[0];
		out_val[5] = out_val[4];			out_val[4] = out_val[3];			out_val[3] = out_val[2];			out_val[2] = out_val[1];			out_val[1] = out_val[0];
		mea_pos[5] = mea_pos[4];			mea_pos[4] = mea_pos[3];			mea_pos[3] = mea_pos[2];			mea_pos[2] = mea_pos[1];			mea_pos[1] = mea_pos[0];
	
	// Compensator

		set_pos[0] = (double) (*MOTOR).DcMotor.State.plannedPosition;
		mea_pos[0] = (double) (*MOTOR).DcMotor.State.measuredPosition;
		//	if (fabs(err_val[0]) <= 1) 	err_val[0] = 0;

		err_val[0] = set_pos[0] - mea_pos[0];

	// Position Model
	
		r1 = (double) theta[4][0] / theta[3][0];
		r2 = (double) theta[5][0] / theta[3][0];
		t0 = (double) a1m / theta[3][0];
		t1 = (double) a2m / theta[3][0];
		t2 = (double) a3m / theta[3][0];
		s0 = (double) (b1m - theta[0][0]) / theta[3][0];
		s1 = (double) (b2m - theta[1][0]) / theta[3][0];
		s2 = (double) (b3m - theta[2][0]) / theta[3][0];
		
		out_val[0] = - (r1 * out_val[1]) - (r2 * out_val[2])  												\
								 + (t0 * set_pos[0]) + (t1 * set_pos[1]) + (t2 * set_pos[2]) 			\
								 - (s0 * mea_pos[0]) - (s1 * mea_pos[1]) - (s2 * mea_pos[2]);

		out_val[0] = (out_val[0] >  (*MOTOR).DcMotor.Param.maxPulse) ?  (*MOTOR).DcMotor.Param.maxPulse : out_val[0];
		out_val[0] = (out_val[0] < -(*MOTOR).DcMotor.Param.maxPulse) ? -(*MOTOR).DcMotor.Param.maxPulse : out_val[0];		
		
		if (fabs(err_val[0]) <= 0.5)
				out_val[0] = 0;
		else if (fabs(err_val[0]) < (*MOTOR).DcMotor.Param.acceptedError)
		{
				out_val[0] *= 0.9;
		}
		
		if (((*MOTOR).DcMotor.State.desiredPosition[0] -  mea_pos[0]) * out_val[0] < 0)
		{
			out_val[0] *= -.5;
		}
		
		if (GlobalCommand == CMD_STOP)
		{
				for(int i = 0; i < 5; i++)	out_val[i] = 0;
		}		
		
		(*MOTOR).DcMotor.State.PulseWidth = out_val[0];
}

void motorStrPositionController_PITCH(dcServoMotor_t * MOTOR)
{
	// Local variables declaration
		
		static double s0, s1, s2, t0, t1, t2, r1, r2;
		static double err_val[1], set_pos[4], out_val[5], mea_pos[5];
		
		static double lamda, lamda_predict, phiTPphi, epsilon;
		static double phi[6][6], L[6][6], phiT[6][6], temp[6][6], temp2[6][6], denominator[6][6], numerator[6][6], P[6][6], theta[6][6];
		
		static uint8_t firstCall;
		
	// Initialize

		if (firstCall == INIT_ENABLE)
		{	
			firstCall = INIT_DISABLE;
			
			lamda = 0.95; lamda_predict = 100; phiTPphi = 0; epsilon = 0;
			Clear_Matrix(L);
			Clear_Matrix(P);
			Clear_Matrix(theta);
			for(int i = 0; i < 6; i++) P[i][i] = 1;
			for(int i = 0; i < 3; i++)
			{
				set_pos[i] = 0;
				out_val[i] = 0;
				mea_pos[i] = 0;
			}
			theta[0][0] = -1;
		}
	
	// Estimation Algorithm

		phi[0][0] = - mea_pos[2];			
		phi[1][0] = - mea_pos[3];			
		phi[2][0] = - mea_pos[4];
		phi[3][0] = out_val[2];				
		phi[4][0] = out_val[3];				
		phi[5][0] = out_val[4];
		
		Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
		
		Multiply_Matrix(phiT,1,6,theta,6,1,temp);				// temp = phiT(k)*theta(k-1)
			epsilon = mea_pos[0] - temp[0][0];

			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
		Multiply_Matrix(P,6,6,phi,6,1,numerator);				// (P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P,6,6,temp);					// temp = phiT(k)*P(k-1)
		Multiply_Matrix(temp,1,6,phi,6,1,denominator);			// phiT(k)*P(k-1)*phi(k)

			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;			//(1 + phiT(k)*P(k-1)*phi(k))
		
		Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			lamda = (lamda > 1.00f) ? 1.00f : lamda; 
			lamda = (lamda < 0.95f) ? 0.95f : lamda;

			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		Multiply_Matrix(L,6,1,temp,1,6,temp2);	
		Sub_Matrix(P,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P); 
		P[0][0] = 0;
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta,temp,6,1,theta);

		theta[3][0] = (theta[3][0] == 0) ? 1 : theta[3][0];
		
			/*Recursion*/
		set_pos[3] = set_pos[2];			set_pos[2] = set_pos[1];			set_pos[1] = set_pos[0];
		out_val[4] = out_val[3];			out_val[3] = out_val[2];			out_val[2] = out_val[1];			out_val[1] = out_val[0];
		mea_pos[4] = mea_pos[3];			mea_pos[3] = mea_pos[2];			mea_pos[2] = mea_pos[1];			mea_pos[1] = mea_pos[0];
	
	// Compensator

		set_pos[0] = (double) (*MOTOR).DcMotor.State.plannedPosition;
		mea_pos[0] = (double) (*MOTOR).DcMotor.State.measuredPosition;

		err_val[0] = set_pos[0] - mea_pos[0];

	// Position Model
	
		r1 = (double) theta[4][0] / theta[3][0];
		r2 = (double) theta[5][0] / theta[3][0];
		t0 = (double) a1m / theta[3][0];
		t1 = (double) a2m / theta[3][0];
		t2 = (double) a3m / theta[3][0];
		s0 = (double) (b1m - theta[0][0]) / theta[3][0];
		s1 = (double) (b2m - theta[1][0]) / theta[3][0];
		s2 = (double) (b3m - theta[2][0]) / theta[3][0];
		
		out_val[0] = - (r1 * out_val[1]) - (r2 * out_val[2])  												\
								 + (t0 * set_pos[0]) + (t1 * set_pos[1]) + (t2 * set_pos[2]) 			\
								 - (s0 * mea_pos[0]) - (s1 * mea_pos[1]) - (s2 * mea_pos[2]);

		out_val[0] = (out_val[0] >  (*MOTOR).DcMotor.Param.maxPulse) ?  (*MOTOR).DcMotor.Param.maxPulse : out_val[0];
		out_val[0] = (out_val[0] < -(*MOTOR).DcMotor.Param.maxPulse) ? -(*MOTOR).DcMotor.Param.maxPulse : out_val[0];		
		
		if (fabs(err_val[0]) < (*MOTOR).DcMotor.Param.acceptedError)
				out_val[0] *= 0.3f;
		
		if (/*(*MOTOR).DcMotor.State.desiredPosition[0] -  mea_pos[0])*/ err_val[0]* out_val[0] < 0)
		{
			out_val[0] *= -0.2;
			if ((fabs(err_val[0]) > 10) && (fabs(out_val[0]) > 200))
				firstCall = INIT_ENABLE;
		}
		
		if (GlobalCommand == CMD_STOP)
		{
				for(int i = 0; i < 5; i++)	out_val[i] = 0;
		}
		
		(*MOTOR).DcMotor.State.PulseWidth = out_val[0];
}

void motorStrPositionController_ROLL(dcServoMotor_t * MOTOR)
{
	// Local variables declaration
		
		static double s0, s1, s2, t0, t1, t2, r1, r2;
		static double err_val[1], set_pos[4], out_val[5], mea_pos[5];
		
		static double lamda, lamda_predict, phiTPphi, epsilon;
		static double phi[6][6], L[6][6], phiT[6][6], temp[6][6], temp2[6][6], denominator[6][6], numerator[6][6], P[6][6], theta[6][6];
		
		static uint8_t firstCall;
		
	// Initialize

		if (firstCall == INIT_ENABLE)
		{	
			firstCall = INIT_DISABLE;
			
			lamda = 0.95; lamda_predict = 100; phiTPphi = 0; epsilon = 0;
			Clear_Matrix(L);
			Clear_Matrix(P);
			Clear_Matrix(theta);
			for(int i = 0; i < 6; i++) P[i][i] = 1;
			for(int i = 0; i < 3; i++)
			{
				set_pos[i] = 0;
				out_val[i] = 0;
				mea_pos[i] = 0;
			}
			theta[0][0] = -1;
		}
	
	// Estimation Algorithm

		phi[0][0] = - mea_pos[2];			
		phi[1][0] = - mea_pos[3];			
		phi[2][0] = - mea_pos[4];
		phi[3][0] = out_val[2];				
		phi[4][0] = out_val[3];				
		phi[5][0] = out_val[4];
		
		Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
		
		Multiply_Matrix(phiT,1,6,theta,6,1,temp);				// temp = phiT(k)*theta(k-1)
			epsilon = mea_pos[0] - temp[0][0];

			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
		Multiply_Matrix(P,6,6,phi,6,1,numerator);				// (P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P,6,6,temp);					// temp = phiT(k)*P(k-1)
		Multiply_Matrix(temp,1,6,phi,6,1,denominator);			// phiT(k)*P(k-1)*phi(k)

			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;			//(1 + phiT(k)*P(k-1)*phi(k))
		
		Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			lamda = (lamda > 1.00f) ? 1.00f : lamda; 
			lamda = (lamda < 0.95f) ? 0.95f : lamda;

			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		Multiply_Matrix(L,6,1,temp,1,6,temp2);	
		Sub_Matrix(P,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P); 
		P[0][0] = 0;
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta,temp,6,1,theta);

		theta[3][0] = (theta[3][0] == 0) ? 1 : theta[3][0];
		
			/*Recursion*/
		set_pos[3] = set_pos[2];			set_pos[2] = set_pos[1];			set_pos[1] = set_pos[0];
		out_val[4] = out_val[3];			out_val[3] = out_val[2];			out_val[2] = out_val[1];			out_val[1] = out_val[0];
		mea_pos[4] = mea_pos[3];			mea_pos[3] = mea_pos[2];			mea_pos[2] = mea_pos[1];			mea_pos[1] = mea_pos[0];
	
	// Compensator

		set_pos[0] = (double) (*MOTOR).DcMotor.State.plannedPosition;
		mea_pos[0] = (double) (*MOTOR).DcMotor.State.measuredPosition;

		err_val[0] = set_pos[0] - mea_pos[0];

	// Position Model
	
		r1 = (double) theta[4][0] / theta[3][0];
		r2 = (double) theta[5][0] / theta[3][0];
		t0 = (double) a1m / theta[3][0];
		t1 = (double) a2m / theta[3][0];
		t2 = (double) a3m / theta[3][0];
		s0 = (double) (b1m - theta[0][0]) / theta[3][0];
		s1 = (double) (b2m - theta[1][0]) / theta[3][0];
		s2 = (double) (b3m - theta[2][0]) / theta[3][0];
		
		out_val[0] = - (r1 * out_val[1]) - (r2 * out_val[2])  												\
								 + (t0 * set_pos[0]) + (t1 * set_pos[1]) + (t2 * set_pos[2]) 			\
								 - (s0 * mea_pos[0]) - (s1 * mea_pos[1]) - (s2 * mea_pos[2]);

		out_val[0] = (out_val[0] >  (*MOTOR).DcMotor.Param.maxPulse) ?  (*MOTOR).DcMotor.Param.maxPulse : out_val[0];
		out_val[0] = (out_val[0] < -(*MOTOR).DcMotor.Param.maxPulse) ? -(*MOTOR).DcMotor.Param.maxPulse : out_val[0];		
		
		if (fabs(err_val[0]) < (*MOTOR).DcMotor.Param.acceptedError)
				out_val[0] *= 0.5f;
		else if (fabs(out_val[0]) < 2)
				out_val[0] = 0;		

		if (((*MOTOR).DcMotor.State.desiredPosition[0] -  mea_pos[0]) * out_val[0] < 0)
		{
			out_val[0] *= -.5;
		}
		
		if (GlobalCommand == CMD_STOP)
		{
				for(int i = 0; i < 5; i++)	out_val[i] = 0;
		}		
		
		(*MOTOR).DcMotor.State.PulseWidth = out_val[0];
}
