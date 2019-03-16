#ifndef __MATRIX_H
#define __MATRIX_H

#ifdef __cplusplus
    extern "C" {
#endif 

void Multiply_Matrix(double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6]);

void Add_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);

void Sub_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);

void Multiply_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6]);

void Divide_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6]);

void Tranpose_Matrix(double x[6][6],unsigned char m, unsigned char n,  double y[6][6]);

void Clear_Matrix(double z[6][6]);
			
#ifdef __cplusplus
}
#endif

#endif /* __MATRIX_H */


