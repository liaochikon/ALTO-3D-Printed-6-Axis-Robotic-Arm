/*
 * FK.c
 *
 *  Created on: Jan 2, 2024
 *      Author: kylem
 */
#include "FK.h"

//void Mat_Print(float mat[4][4])
//{
//	uint8_t n = 4;
//	for(uint8_t r = 0; r < n; r++)
//	{
//		for(uint8_t c = 0; c < n; c++)
//		{
//			CDC_TXData[0] = r;
//			CDC_TXData[1] = c;
//			CDC_Transmit_FS(CDC_TXData, 2);
//			osDelay(5);
//			float m = mat[r][c];
//			int16_t m_int = (int16_t)m;
//			uint16_t m_dec = (uint16_t)((m - m_int) * 10000);
//			CDC_TXData[0] = (uint8_t)(m_int >> 8);
//			CDC_TXData[1] = (uint8_t)(m_int >> 0);
//			CDC_TXData[2] = (uint8_t)(m_dec >> 8);
//			CDC_TXData[3] = (uint8_t)(m_dec >> 0);
//			CDC_TXData[4] = 0x6b;
//			CDC_Transmit_FS(CDC_TXData, 5);
//			osDelay(5);
//		}
//	}
//}

void FK()
{
	float T_TCP[4][4];
	T_init(T_TCP);
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		float ans[4][4];
		float _T[4][4];
		DH_Parser(
			Alto.DH.Joints[index].d,
			Alto.Current_JointAngle[index] + Alto.DH.Joints[index].theta,
			Alto.DH.Joints[index].r,
			Alto.DH.Joints[index].alpha, _T);

		Mat4x4_Mul(T_TCP, _T, ans);
		Mat4x4_Copy(ans, T_TCP);
	}
	//Mat_Print(T_TCP);
	translation_matrix_to_pose(T_TCP, Alto.Current_TCP_Position, Alto.Current_TCP_Rotation);
}
