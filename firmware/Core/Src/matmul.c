/*
 * matmul.c
 *
 *  Created on: Jan 2, 2024
 *      Author: kylem
 */
#include "matmul.h"

void Mat4x4_Init(float mat[4][4])
{
	uint8_t n = 4;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			mat[r][c] = 0;
		}
	}
}
void Mat3x3_Init(float mat[3][3])
{
	uint8_t n = 3;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			mat[r][c] = 0;
		}
	}
}

void Vec_Init(float *vec, uint8_t row)
{
	for(uint8_t r = 0; r < row; r++)
	{
		vec[r] = 0;
	}
}
void Vec3_Init(float *vec)
{
	Vec_Init(vec, 3);
}

void Mat4x4_Mul(float a[4][4], float b[4][4], float ans[4][4])
{
	uint8_t n = 4;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			float mat_u = 0;
			for(uint8_t u = 0; u < n; u++)
			{
				mat_u += a[r][u] * b[u][c];
			}
			ans[r][c] = mat_u;
		}
	}
}
void Mat3x3_Mul(float a[3][3], float b[3][3], float ans[3][3])
{
	uint8_t n = 3;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			float mat_u = 0;
			for(uint8_t u = 0; u < n; u++)
			{
				mat_u += a[r][u] * b[u][c];
			}
			ans[r][c] = mat_u;
		}
	}
}

void Mat3x3_Vec3_Mul(float mat[3][3], float vec[3], float ans[3])
{
	uint8_t n = 3;
	for(uint8_t r = 0; r < n; r++)
	{
		float mat_u = 0;
		for(uint8_t u = 0; u < n; u++)
		{
			mat_u += mat[r][u] * vec[u];
		}
		ans[r] = mat_u;
	}
}

void Mat4x4_Vec4_Mul(float mat[3][3], float vec[3], float ans[3])
{
	uint8_t n = 4;
	for(uint8_t r = 0; r < n; r++)
	{
		float mat_u = 0;
		for(uint8_t u = 0; u < n; u++)
		{
			mat_u += mat[r][u] * vec[u];
		}
		ans[r] = mat_u;
	}
}

void Mat4x4_I(float mat[4][4])
{
	Mat4x4_Init(mat);
	uint8_t n = 4;
	for(uint8_t u = 0; u < n; u++)
	{
		mat[u][u] = 1;
	}
}
void Mat3x3_I(float mat[3][3])
{
	Mat3x3_Init(mat);
	uint8_t n = 3;
	for(uint8_t u = 0; u < n; u++)
	{
		mat[u][u] = 1;
	}
}

void Mat4x4_Copy(float a[4][4], float b[4][4])
{
	uint8_t n = 4;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			b[r][c] = a[r][c];
		}
	}
}
void Mat3x3_Copy(float a[3][3], float b[3][3])
{
	uint8_t n = 3;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			b[r][c] = a[r][c];
		}
	}
}

void Mat3x3_T(float a[3][3], float a_T[3][3])
{
	uint8_t n = 3;
	for(uint8_t r = 0; r < n; r++)
	{
		for(uint8_t c = 0; c < n; c++)
		{
			a_T[c][r] = a[r][c];
		}
	}
}
