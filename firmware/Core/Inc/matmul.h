/*
 * matmul.h
 *
 *  Created on: Jan 2, 2024
 *      Author: kylem
 */

#ifndef INC_MATMUL_H_
#define INC_MATMUL_H_

#include "stdint.h"

void Mat4x4_Init(float mat[4][4]);
void Mat3x3_Init(float mat[3][3]);

void Vec_Init(float *vec, uint8_t row);
void Vec3_Init(float *vec);

void Mat4x4_Mul(float a[4][4], float b[4][4], float and[4][4]);
void Mat3x3_Mul(float a[3][3], float b[3][3], float and[3][3]);

void Mat3x3_Vec3_Mul(float mat[3][3], float vec[3], float ans[3]);
void Mat4x4_Vec4_Mul(float mat[3][3], float vec[3], float ans[3]);

void Mat4x4_I(float mat[4][4]);
void Mat3x3_I(float mat[3][3]);

void Mat4x4_Copy(float a[4][4], float b[4][4]);
void Mat3x3_Copy(float a[3][3], float b[3][3]);

void Mat3x3_T(float a[3][3], float a_T[3][3]);
#endif /* INC_MATMUL_H_ */
