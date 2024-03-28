/*
 * transform.c
 *
 *  Created on: Jan 2, 2024
 *      Author: kylem
 */
#include "transform.h"

float degree_to_radian(float degree)
{
	return degree * (M_PI / 180);
}
float radian_to_degree(float radian)
{
	return radian / (M_PI / 180);
}

void T_init(float T[4][4])
{
	Mat4x4_I(T);
}

void R_and_t_to_T(float R[3][3], float t[3], float T[4][4])
{
	uint8_t row = 3;
	uint8_t col = 3;
	for(uint8_t r = 0; r < row; r++)
	{
		for(uint8_t c = 0; c < col; c++)
		{
			T[r][c] = R[r][c];
		}
	}
	T[0][3] = t[0];
	T[1][3] = t[1];
	T[2][3] = t[2];
}
void T_to_R_and_t(float T[4][4], float R[3][3], float t[3])
{
	uint8_t row = 3;
	uint8_t col = 3;
	for(uint8_t r = 0; r < row; r++)
	{
		for(uint8_t c = 0; c < col; c++)
		{
			R[r][c] = T[r][c];
		}
	}
	t[0] = T[0][3];
	t[1] = T[1][3];
	t[2] = T[2][3];
}

void rotate_x(float T[4][4], float degree)
{
	float ans[4][4];
	float T_rx[4][4];
	float radian = degree_to_radian(degree);
	T_init(T_rx);

	T_rx[1][1] = cosf(radian);
	T_rx[1][2] = -sinf(radian);
	T_rx[2][1] = sinf(radian);
	T_rx[2][2] = cosf(radian);

	Mat4x4_Mul(T_rx, T, ans);
	Mat4x4_Copy(ans, T);
}
void rotate_y(float T[4][4], float degree)
{
	float ans[4][4];
	float T_ry[4][4];
	float radian = degree_to_radian(degree);
	T_init(T_ry);

	T_ry[0][0] = cosf(radian);
	T_ry[0][2] = sinf(radian);
	T_ry[2][0] = -sinf(radian);
	T_ry[2][2] = cosf(radian);

	Mat4x4_Mul(T_ry, T, ans);
	Mat4x4_Copy(ans, T);
}
void rotate_z(float T[4][4], float degree)
{
	float ans[4][4];
	float T_rz[4][4];
	float radian = degree_to_radian(degree);
	T_init(T_rz);

	T_rz[0][0] = cosf(radian);
	T_rz[0][1] = -sinf(radian);
	T_rz[1][0] = sinf(radian);
	T_rz[1][1] = cosf(radian);

	Mat4x4_Mul(T_rz, T, ans);
	Mat4x4_Copy(ans, T);
}
void translate(float T[4][4], float x, float y, float z)
{
	float ans[4][4];
	float T_t[4][4];
	T_init(T_t);

	T_t[0][3] = x;
	T_t[1][3] = y;
	T_t[2][3] = z;

	Mat4x4_Mul(T_t, T, ans);
	Mat4x4_Copy(ans, T);
}
void pose_to_translation_matrix(float pos[3], float rot[3], float T[4][4])
{
	T_init(T);
	rotate_x(T, rot[0]);
	rotate_y(T, rot[1]);
	rotate_z(T, rot[2]);
	T[0][3] = pos[0];
	T[1][3] = pos[1];
	T[2][3] = pos[2];
}
void translation_matrix_to_pose(float T[4][4], float pos[3], float rot[3])
{
	pos[0] = T[0][3];
	pos[1] = T[1][3];
	pos[2] = T[2][3];
	rot[0] = radian_to_degree(atan2f(T[2][1], T[2][2]));
	rot[1] = radian_to_degree(asinf(-T[2][0]));
	rot[2] = radian_to_degree(atan2f(T[1][0], T[0][0]));
}

void DH_Parser(float d, float theta, float r, float alpha, float T[4][4])
{
	T_init(T);
	rotate_x(T, alpha);
	translate(T, r, 0, d);
	rotate_z(T, theta);
}
