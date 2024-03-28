/*
 * IK.c
 *
 *  Created on: Jan 5, 2024
 *      Author: kylem
 */
#include "IK.h"

bool Position(float T_TCP[4][4], float theta_pos[3])
{
	float R_06[3][3];
	float t_06[3];
	T_to_R_and_t(T_TCP, R_06, t_06);

	float d6 = Alto.DH.Joints[5].d;
	float d6_vec[3] = {0, 0, d6};
	float d6_vec_rot[3];
	Mat3x3_Vec3_Mul(R_06, d6_vec, d6_vec_rot);
	float W[3];
	W[0] = t_06[0] - d6_vec_rot[0];
	W[1] = t_06[1] - d6_vec_rot[1];
	W[2] = t_06[2] - d6_vec_rot[2];

	float r = sqrtf(W[0] * W[0] + W[1] * W[1]) - Alto.DH.Joints[0].r;
	float s = W[2] - Alto.DH.Joints[0].d;
	float a_s = sqrtf(
			Alto.DH.Joints[2].r * Alto.DH.Joints[2].r +
			Alto.DH.Joints[3].d * Alto.DH.Joints[3].d);
	float a_2 = Alto.DH.Joints[1].r;
	float c = (r * r + s * s - a_2 * a_2 - a_s * a_s) / (2 * a_2 * a_s);
	if(c > 1)//Ik position failed.
		return false;

	float theta3_delta = atan2f(Alto.DH.Joints[2].r, Alto.DH.Joints[3].d);
	float thetas = -acosf(c);
	float theta3 = thetas - theta3_delta + M_PI / 2;
	float theta2 =
			atan2f(s, r) -
			atan2f((a_s * sinf(thetas)), (a_2 + a_s * cosf(thetas)));
	float theta1 = atan2f(W[1], W[0]);

	theta_pos[0] = radian_to_degree(theta1);
	theta_pos[1] = radian_to_degree(theta2);
	theta_pos[2] = radian_to_degree(theta3);
	return true;
}

void Rot_Sol_1(float R_30[3][3], float R_06[3][3], float theta_rot[6])
{
	float c4s5 =
			R_30[0][0] * R_06[0][2] +
			R_30[0][1] * R_06[1][2] +
			R_30[0][2] * R_06[2][2];
	float s4s5 =
			R_30[1][0] * R_06[0][2] +
			R_30[1][1] * R_06[1][2] +
			R_30[1][2] * R_06[2][2];

	float c5 =
			R_30[2][0] * R_06[0][2] +
			R_30[2][1] * R_06[1][2] +
			R_30[2][2] * R_06[2][2];
	float s5 = sqrtf(1 - c5 * c5);

	float s5c6 =
			-(R_30[2][0] * R_06[0][0] +
			  R_30[2][1] * R_06[1][0] +
			  R_30[2][2] * R_06[2][0]);
	float s5s6 =
			R_30[2][0] * R_06[0][1] +
			R_30[2][1] * R_06[1][1] +
			R_30[2][2] * R_06[2][1];

	float theta4 = atan2f(s4s5, c4s5);
	float theta5 = atan2f(s5, c5);
	float theta6 = atan2f(s5s6, s5c6);

	theta_rot[3] = radian_to_degree(theta4);
	theta_rot[4] = radian_to_degree(theta5);
	theta_rot[5] = radian_to_degree(theta6);
}

void Rot_Sol_2(float R_30[3][3], float R_06[3][3], float theta_rot[6])
{
	float c4s5 =
			R_30[0][0] * R_06[0][2] +
			R_30[0][1] * R_06[1][2] +
			R_30[0][2] * R_06[2][2];
	float s4s5 =
			R_30[1][0] * R_06[0][2] +
			R_30[1][1] * R_06[1][2] +
			R_30[1][2] * R_06[2][2];

	float c5 =
			R_30[2][0] * R_06[0][2] +
			R_30[2][1] * R_06[1][2] +
			R_30[2][2] * R_06[2][2];
	float s5 = sqrtf(1 - c5 * c5);

	float s5c6 =
			-(R_30[2][0] * R_06[0][0] +
			  R_30[2][1] * R_06[1][0] +
			  R_30[2][2] * R_06[2][0]);
	float s5s6 =
			R_30[2][0] * R_06[0][1] +
			R_30[2][1] * R_06[1][1] +
			R_30[2][2] * R_06[2][1];

	float theta4 = atan2f(-s4s5, -c4s5);
	float theta5 = atan2f(-s5, c5);
	float theta6 = atan2f(-s5s6, -s5c6);

	theta_rot[3] = radian_to_degree(theta4);
	theta_rot[4] = radian_to_degree(theta5);
	theta_rot[5] = radian_to_degree(theta6);
}

bool Rotation(float T_TCP[4][4], float theta_pos[3], float theta_rot1[6], float theta_rot2[6])
{
	float R_06[3][3];
	float t_06[3];
	T_to_R_and_t(T_TCP, R_06, t_06);

	float T_03[4][4];
	T_init(T_03);

	float alpha1 = Alto.DH.Joints[0].alpha;
	float alpha2 = Alto.DH.Joints[1].alpha;
	float alpha3 = Alto.DH.Joints[2].alpha;
	float theta1 = theta_pos[0];
	float theta2 = theta_pos[1];
	float theta3 = theta_pos[2];

	rotate_x(T_03, alpha3);
	rotate_z(T_03, theta3);
	rotate_x(T_03, alpha2);
	rotate_z(T_03, theta2);
	rotate_x(T_03, alpha1);
	rotate_z(T_03, theta1);

	float R_03[3][3];
	float t_03[3];
	T_to_R_and_t(T_03, R_03, t_03);

	float R_30[3][3];
	Mat3x3_T(R_03, R_30);

	Rot_Sol_1(R_30, R_06, theta_rot1);
	Rot_Sol_2(R_30, R_06, theta_rot2);
	return true;
}

bool IK(float target_tcp_position[3], float target_tcp_rotation[3])
{
	float T_TCP[4][4];
	float thetas_pos[3];
	pose_to_translation_matrix(target_tcp_position, target_tcp_rotation, T_TCP);
	bool ret_pos = Position(T_TCP, thetas_pos);
	if (ret_pos == false)
		return false;

	float thetas1[6];
	float thetas2[6];
	bool ret_rot = Rotation(T_TCP, thetas_pos, thetas1, thetas2);
	if (ret_rot == false)
		return false;

	thetas1[0] = thetas_pos[0];
	thetas1[1] = thetas_pos[1];
	thetas1[2] = thetas_pos[2];
	thetas2[0] = thetas_pos[0];
	thetas2[1] = thetas_pos[1];
	thetas2[2] = thetas_pos[2];

	//Joint offset adjust
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		thetas1[index] = (thetas1[index] - Alto.DH.Joints[index].theta);
		thetas2[index] = (thetas2[index] - Alto.DH.Joints[index].theta);
	}

	//IK solution rating
	float total_dist1 = 0;
	float total_dist2 = 0;
	bool is_theta1_failed = false;
	bool is_theta2_failed = false;
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		if(thetas1[index] > Alto.Upper_JointAngle_Limit[index] ||
				thetas1[index] < Alto.Lower_JointAngle_Limit[index])
			is_theta1_failed = true;

		if(thetas2[index] > Alto.Upper_JointAngle_Limit[index] ||
				thetas2[index] < Alto.Lower_JointAngle_Limit[index])
			is_theta2_failed = true;

		total_dist1 += fabsf(Alto.Current_JointAngle[index] - thetas1[index]) * Alto.DH.Joints[index].ik_joint_weight;
		total_dist2 += fabsf(Alto.Current_JointAngle[index] - thetas2[index]) * Alto.DH.Joints[index].ik_joint_weight;
	}
	if(is_theta1_failed && is_theta2_failed)
		return false;

	//Assign IK solution to Target_JointAngle
	if(is_theta1_failed || is_theta2_failed)
	{
		if(is_theta1_failed)
		{
			for(uint8_t index = 0; index < AXIS_NUM; index++)
				Alto.Target_JointAngle[index] = thetas1[index];
		}
		if(is_theta2_failed)
		{
			for(uint8_t index = 0; index < AXIS_NUM; index++)
				Alto.Target_JointAngle[index] = thetas2[index];
		}
	}
	if(!is_theta1_failed && !is_theta2_failed)
	{
		//Choose solution 1
		if(total_dist1 < total_dist2)
		{
			for(uint8_t index = 0; index < AXIS_NUM; index++)
				Alto.Target_JointAngle[index] = thetas1[index];
		}
		//Choose solution 2
		if(total_dist1 >= total_dist2)
		{
			for(uint8_t index = 0; index < AXIS_NUM; index++)
				Alto.Target_JointAngle[index] = thetas2[index];
		}
	}
	return true;
}
