#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "matrix.h"

void Multiply_Matrix(double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6])
{
	double TEMP;
	for (int i = 0 ; i < m ; i++ ) {
    	for (int j = 0 ; j < q ; j++ ) {
        	for (int k = 0 ; k < p ; k++ ) {
          		TEMP += x[i][k] * y[k][j];
        	}
        	z[i][j] = TEMP;
        	TEMP = 0;
    	}
    }
}
void Add_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(int i=0;i<m;i++)
    	for(int j=0;j<n;j++)
        	z[i][j]=x[i][j]+y[i][j];
}
void Sub_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(int i=0;i<m;i++)
    	for(int j=0;j<n;j++)
        	z[i][j]=x[i][j]-y[i][j];
}
void Multiply_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
	for(int i=0;i<m;i++)
    	for(int j=0;j<n;j++)
        	z[i][j]=x[i][j]*y;
}
void Divide_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
	for(int i=0;i<m;i++)
    	for(int j=0;j<n;j++)
        	z[i][j]=x[i][j]/y;
}
void Tranpose_Matrix(double x[6][6],unsigned char m, unsigned char n,  double y[6][6])
{
	for(int i=0;i<n;i++)
    	for(int j=0;j<m;j++)
        	y[i][j]=x[j][i];
}

void Clear_Matrix(double z[6][6])
{
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
				z[i][j] = 0;
		}
	}
}
