#ifndef __INVKIN_H
#define __INVKIN_H

#ifdef __cplusplus
    extern "C" {
#endif 


#define _DONE 		0
#define _NULL  		1
#define _PI 			3.14159265f

#ifndef SCORBOT_PARAM // Length in mm; Angle in degree
	#define d1		((float) 272)
	#define d2		((float)  20)
	#define d3		((float) 174)
	#define d4		((float) 175)
	#define d5		((float) 210)
	
	#define T1MIN	((float) 	-90)
	#define T1MAX	((float)   90)
	
	#define T2MIN	((float)  -70)	// GOING UP
	#define T2MAX	((float)    5)	// GOING DOWN
	
	#define T3MIN	((float)  -91)	// GOING UP
	#define T3MAX	((float)   91)	// GOING DOWN
	
	#define T4MIN	((float) -181)	// GOING UP
	#define T4MAX	((float)    1)	// GOING DOWN
	
	#define T5MIN	((float) -181)
	#define T5MAX	((float)  181)
#endif

typedef struct {
	float x;
	float y;
	float z;
}	Coor_t;

typedef struct { // Coor of orientation vectors and point in mm;
	Coor_t n;
	Coor_t s;
	Coor_t a;
	Coor_t P;
}	homoMatrix_t;

typedef struct { // Joint angle in Degree
	float JOINT1;
	float JOINT2;
	float JOINT3;
	float JOINT4;
	float JOINT5;
}	jointSet_t;

void 	update_measuredPosition(homoMatrix_t *, jointSet_t *);
int 	update_desiredPosition(homoMatrix_t * , jointSet_t *);
void 	update_holdPosition(jointSet_t *, jointSet_t *);
void 	update_manual_desiredPosition(jointSet_t *);

int 	calculate_InvKin(homoMatrix_t *, jointSet_t *);
int 	calculate_ForKin(homoMatrix_t *, jointSet_t *);

int set_HorizontalApproach(homoMatrix_t *);
int set_UpwardApproach(homoMatrix_t *);

#ifdef __cplusplus
}
#endif

#endif /* __INVKIN_H */


