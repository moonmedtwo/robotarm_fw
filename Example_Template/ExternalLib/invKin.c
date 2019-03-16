#include <stdio.h>
#include <stdint.h>
#include <math.h>	

#include "invKin.h"
#include "controller.h"

extern jointSet_t 		jointSet, 		jointMeasured ;
extern homoMatrix_t 	dstPosition, 	meaPosition;

extern dcServoMotor_t BASE, SHLDR, ELBOW, PITCH, ROLL;	

void update_measuredPosition(homoMatrix_t * MEAS, jointSet_t * result)
{
	(*result).JOINT1 = + BASE.DcMotor.State.measuredPosition;
	(*result).JOINT2 = - SHLDR.DcMotor.State.measuredPosition;
	(*result).JOINT3 = SHLDR.DcMotor.State.measuredPosition + ELBOW.DcMotor.State.measuredPosition;
	(*result).JOINT4 = - PITCH.DcMotor.State.measuredPosition;
	(*result).JOINT5 = + ROLL.DcMotor.State.measuredPosition;
	
	calculate_ForKin(MEAS, result);
}

int update_desiredPosition(homoMatrix_t * DEST, jointSet_t * result)
{
	uint8_t calculateStatus = _NULL;
	
	calculateStatus = calculate_InvKin(DEST, result);
	
	if (calculateStatus == _DONE)
	{	/* Update to input of each joint controller. */
		BASE.DcMotor.State.desiredPosition[0] 	= + (*result).JOINT1;
		SHLDR.DcMotor.State.desiredPosition[0] 	= - (*result).JOINT2 ;
		ELBOW.DcMotor.State.desiredPosition[0] 	= (*result).JOINT2 + (*result).JOINT3;
		PITCH.DcMotor.State.desiredPosition[0] 	= - (*result).JOINT4;
		ROLL.DcMotor.State.desiredPosition[0] 	= + (*result).JOINT5;
		return _DONE;
	}
	else
	{
		return _NULL;
	}
}

void update_holdPosition(jointSet_t * setJoint, jointSet_t * meaJoint)
{
	/* Set all the desiredPosition of each joint to the current measuredPosition. */
	(*setJoint).JOINT1 = (*meaJoint).JOINT1; 
	(*setJoint).JOINT2 = (*meaJoint).JOINT2; 
	(*setJoint).JOINT3 = (*meaJoint).JOINT3; 
	(*setJoint).JOINT4 = (*meaJoint).JOINT4; 
	(*setJoint).JOINT5 = (*meaJoint).JOINT5; 
	
	BASE.DcMotor.State.desiredPosition[0] 	= BASE.DcMotor.State.measuredPosition;
	BASE.DcMotor.State.plannedPosition 	= BASE.DcMotor.State.measuredPosition;
		
	SHLDR.DcMotor.State.desiredPosition[0] 	= SHLDR.DcMotor.State.measuredPosition;
	SHLDR.DcMotor.State.plannedPosition 	= SHLDR.DcMotor.State.measuredPosition;

	ELBOW.DcMotor.State.desiredPosition[0] 	= ELBOW.DcMotor.State.measuredPosition;
	ELBOW.DcMotor.State.plannedPosition 	= ELBOW.DcMotor.State.measuredPosition;

	PITCH.DcMotor.State.desiredPosition[0] 	= PITCH.DcMotor.State.measuredPosition;
	PITCH.DcMotor.State.plannedPosition 	= PITCH.DcMotor.State.measuredPosition;

	ROLL.DcMotor.State.desiredPosition[0] 	= ROLL.DcMotor.State.measuredPosition;
	ROLL.DcMotor.State.plannedPosition 	= ROLL.DcMotor.State.measuredPosition;
}

void update_manual_desiredPosition(jointSet_t * setPoint)
{
	BASE.DcMotor.State.desiredPosition[0] 	= + (*setPoint).JOINT1;
	SHLDR.DcMotor.State.desiredPosition[0] 	= - (*setPoint).JOINT2 ;
	ELBOW.DcMotor.State.desiredPosition[0] 	= (*setPoint).JOINT2 + (*setPoint).JOINT3;
	PITCH.DcMotor.State.desiredPosition[0] 	= - (*setPoint).JOINT4;
	ROLL.DcMotor.State.desiredPosition[0] 	= + (*setPoint).JOINT5;	
}


int calculate_InvKin(homoMatrix_t * DEST, jointSet_t * result)
{	
	/*
		Output = Joint Set (Set of joint angles)
		Input  = homogenous Matrix (desired P + Orientation) + robot Parameters

		If dest position is out of range => Return _NULL
		Else						  	 => Return _DONE  & Update the new joint set.
	*/

	// Upper Arm Configuration in Calculating	
	float Px, Py, Pz, nx ,ny, sx, sy, ax, ay, az;
	float q1, q2, q3, q4, q5;
	float S5, C5, S234, C234, K1, K2, C3, S2, C2, C4, S4;

	Px = (*DEST).P.x;	Py = (*DEST).P.y; 	Pz = (*DEST).P.z;
	nx = (*DEST).n.x;	ny = (*DEST).n.y;
	sx = (*DEST).s.x;	sy = (*DEST).s.y;	
	ax = (*DEST).a.x;	ay = (*DEST).a.y;	az = (*DEST).a.z;
	
	// JOINT 1
	if ((Py != 0) ||  (Px != 0))
	{
		q1 = atan2(Py,Px);
	}
	else
	{
		return _NULL;
	}

	// JOINT 5
	S5 = nx*sin(q1) - ny*cos(q1);
	C5 = sx*sin(q1) + sy*cos(q1);

	if (fabs(S5*S5 + C5*C5 - 1) <= 0.01)
	{
		q5 = atan2(S5,C5);
	}
	else
	{
		return _NULL;
	}
	
	// JOINT3
	S234 = - cos(q1)*ax - sin(q1)*ay;
	C234 = - az;
	K1 =  Px*cos(q1) + Py*sin(q1) - d2 + d5*S234; 
	K2 = -Pz + d1 - d5*C234;
	
	C3 = (K1*K1 + K2*K2 - d4*d4 - d3*d3) / (2*d3*d4);
	if (fabs(C3) <= 1.1)
		{
		if (fabs(C3) > 1)	C3 = (C3 > 0)? 1: -1;
		q3 = acos(C3);
		}
	else
	{
		return _NULL;
	}
			
	// JOINT2
	S2 = (K2*d4*cos(q3) + K2*d3 - K1*d4*sin(q3)) /(d4*d4 + d3*d3 + 2*d3*d4*cos(q3));
	C2 = (K1*d4*cos(q3) + K1*d3 + K2*d4*sin(q3)) /(d4*d4 + d3*d3 + 2*d3*d4*cos(q3));
	
	if (fabs(S2*S2 + C2*C2 - 1) <= 0.01)
		q2 = atan2(S2 ,C2);
	else
	{
		return _NULL;
	}

	// JOINT 4
	C4 = (sin(q2+q3)*S234 + cos(q2+q3)*C234);
	S4 = (cos(q2+q3)*S234 - sin(q2+q3)*C234);
	
	if (fabs(S4*S4 + C4*C4 - 1) <= 0.01)
	{
		q4 = atan2(S4 ,C4);
	}
	else
	{
		return _NULL;
	}

	// Convert angle unit form radian to degree AND check limit condition
	q1 = q1*180/_PI;		q2 = q2*180/_PI;		q3 = q3*180/_PI;		q4 = q4*180/_PI;		q5 = q5*180/_PI;
	
	if (   (T1MIN <= q1) && (q1 <= T1MAX) && (T2MIN <= q2) && (q2 <= T2MAX) 
		&& (T3MIN <= q3) && (q3 <= T3MAX) && (T4MIN <= q4) && (q4 <= T4MAX) 
		&& (T5MIN <= q5) && (q5 <= T5MAX) )
	{	// All joint values satisfied the limit -> Update new set point -> Next Set
		(*result).JOINT1 = q1;	(*result).JOINT2 = q2;	
		(*result).JOINT3 = q3;	(*result).JOINT4 = q4;	
		(*result).JOINT5 = q5;
		return _DONE;
	}
	else
	{	// There is any joint not satisfied the limit -> Hold the old set point -> Next Set
		return _NULL;
	}

	// All joints have been calculated successfully:
}

int calculate_ForKin(homoMatrix_t * DEST, jointSet_t * curr)
{
	/*
		Output = homogenous Matrix (P + Orientation)
		Input  = currrent state (currrent joint angles) in Degree;
	*/
	float C1, S1, C2, S2, C5, S5, C23, S23, C234, S234;
	float q1, q2, q3, q4, q5;

	q1 = (*curr).JOINT1*_PI/180;	q2 = (*curr).JOINT2*_PI/180;	q3 = (*curr).JOINT3*_PI/180;
	q4 = (*curr).JOINT4*_PI/180;	q5 = (*curr).JOINT5*_PI/180;

	S1 = sin(q1); C1 = cos(q1); S2 = sin(q2); C2 = cos(q2); S5 = sin(q5); C5 = cos(q5); 
	S23 = sin(q2+q3); 		C23 = cos(q2+q3); 
	S234 = sin(q2+q3+q4); 	C234 = cos(q2+q3+q4);
	
	/* Orientation [n, s, a] */
	(*DEST).n.x =  C1*C234*C5 + S1*S5;	(*DEST).s.x = -C1*C234*S5 + S1*C5;	(*DEST).a.x = -C1*S234;
	(*DEST).n.y =  S1*C234*C5 - C1*S5;	(*DEST).s.y = -S1*C234*S5 - C1*C5;	(*DEST).a.y = -S1*S234;
	(*DEST).n.z = -S234*C5;				(*DEST).s.z =  S234*S5;				(*DEST).a.z = -C234;
	/* Position */
	(*DEST).P.x =  C1*(d4*C23 + d3*C2 - d5*S234 + d2);
	(*DEST).P.y =  S1*(d4*C23 + d3*C2 - d5*S234 + d2);
	(*DEST).P.z =  d1 - d4*S23 - d3*S2 - d5*C234;

	return _DONE;
}

int set_HorizontalApproach(homoMatrix_t * DEST)
{
	float q1;
	
	if (((*DEST).P.y != 0) ||  ((*DEST).P.x != 0))
	{
		q1 = atan2((*DEST).P.y,(*DEST).P.x);
		
		(*DEST).n.x = 0;				(*DEST).n.y = 0; 				(*DEST).n.z = 1;
		(*DEST).s.x = sin(q1);	(*DEST).s.y = cos(q1);	(*DEST).s.z = 0;
		(*DEST).a.x = cos(q1);	(*DEST).a.y = sin(q1);	(*DEST).a.z = 0;		
		
		return _DONE;
	}
	else
	{
		return _NULL;
	}
}

int set_UpwardApproach(homoMatrix_t *DEST)
{
	float q1;
	
	if (((*DEST).P.y != 0) ||  ((*DEST).P.x != 0))
	{
		q1 = atan2((*DEST).P.y,(*DEST).P.x);
		
		(*DEST).n.x = cos(q1);	(*DEST).n.y = sin(q1); 	(*DEST).n.z = 0;
		(*DEST).s.x = sin(q1);	(*DEST).s.y = cos(q1);	(*DEST).s.z = 0;
		(*DEST).a.x = 0;				(*DEST).a.y = 0;				(*DEST).a.z = -1;		
		
		return _DONE;
	}
	else
	{
		return _NULL;
	}	
}
