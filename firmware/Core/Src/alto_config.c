/*
 * alto_config.c
 *
 *  Created on: Dec 24, 2023
 *      Author: kylem
 */
#include "alto_config.h"
#include "matmul.h"

Robot Alto;
Request_Handle Emm_Request_Handle;

uint8_t Data_Temp[DATA_TEMP_LENGTH];
uint8_t CAN_TXData[CAN_DATA_LENGTH];
uint8_t CAN_RXData[CAN_DATA_LENGTH];
uint8_t CDC_TXData[CDC_DATA_LENGTH];
uint8_t CDC_RXData[CDC_DATA_LENGTH];

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;

void Robot_Config_Init(Robot *a)
{
	//Alto robot parameters config start:
	a->Mode = DISABLE_MODE;
	a->Start_Flag = false;
	a->Stop_Flag = false;
	a->Start_Entry_Flag = false;
	a->Home_Flag = false;

	a->Homing_State = HITTING_LIMIT_ENTEY;
	a->Homing_Joint = 3;

	a->Execute = DO;
	a->Mov = MOVP;
	a->Coor = BASE;

	a->Speed = DEFAULT_FAST_SPEED;
	a->Acc = DEFAULT_ACC;
	//Alto robot parameters config end.

	//Steppers parameters config start:
	for(int i = 0; i < AXIS_NUM; i++)
	{
		a->Steppers[i].id = (uint8_t)(i + 1);
		a->Steppers[i].Speed = a->Speed;
		a->Steppers[i].Acc = a->Acc;
		a->Steppers[i].Clk = 0;
		a->Steppers[i].Angle = 0;
		a->Steppers[i].Error = 0;
		a->Steppers[i].En = true;
		a->Steppers[i].Block = false;
		a->Steppers[i].Clk_Delta = 0;
		a->Steppers[i].Target_Clk = 0;
		a->Steppers[i].Target_En = true;
		a->Steppers[i].isStop = false;
		a->Steppers[i].Set_Zero_Requset = false;
		for(uint8_t index = 0; index < CLK_DELTA_SAMPLE_LENGTH; index++)
			a->Steppers[i]._Clk_Delta[index] = 0;
	}
	//Steppers parameters config end.

	//Steppers IK and DH parameters config start:
	Vec_Init(a->Current_JointAngle, 6);
	Vec3_Init(a->Current_TCP_Position);
	Vec3_Init(a->Current_TCP_Rotation);
	Vec_Init(a->Target_JointAngle, 6);
	Vec3_Init(a->Target_TCP_Position);
	Vec3_Init(a->Target_TCP_Rotation);

	a->Upper_JointAngle_Limit[0] = JOINT1_ANGLE_UPPER_LIMIT;
	a->Upper_JointAngle_Limit[1] = JOINT2_ANGLE_UPPER_LIMIT;;
	a->Upper_JointAngle_Limit[2] = JOINT3_ANGLE_UPPER_LIMIT;;
	a->Upper_JointAngle_Limit[3] = JOINT4_ANGLE_UPPER_LIMIT;;
	a->Upper_JointAngle_Limit[4] = JOINT5_ANGLE_UPPER_LIMIT;;
	a->Upper_JointAngle_Limit[5] = JOINT6_ANGLE_UPPER_LIMIT;;

	a->Lower_JointAngle_Limit[0] = JOINT1_ANGLE_LOWER_LIMIT;
	a->Lower_JointAngle_Limit[1] = JOINT2_ANGLE_LOWER_LIMIT;
	a->Lower_JointAngle_Limit[2] = JOINT3_ANGLE_LOWER_LIMIT;
	a->Lower_JointAngle_Limit[3] = JOINT4_ANGLE_LOWER_LIMIT;
	a->Lower_JointAngle_Limit[4] = JOINT5_ANGLE_LOWER_LIMIT;
	a->Lower_JointAngle_Limit[5] = JOINT6_ANGLE_LOWER_LIMIT;

	a->DH.Joints[0].d = d_1;
	a->DH.Joints[0].r = r_1;
	a->DH.Joints[0].alpha = alpha_1;
	a->DH.Joints[0].theta = theta_1;
	a->DH.Joints[0].orientation = orientation_1;
	a->DH.Joints[0].reduction_ratio = reduction_ratio_1;
	a->DH.Joints[0].ik_joint_weight = ik_joint_weight_1;

	a->DH.Joints[1].d = d_2;
	a->DH.Joints[1].r = r_2;
	a->DH.Joints[1].alpha = alpha_2;
	a->DH.Joints[1].theta = theta_2;
	a->DH.Joints[1].orientation = orientation_2;
	a->DH.Joints[1].reduction_ratio = reduction_ratio_2;
	a->DH.Joints[1].ik_joint_weight = ik_joint_weight_2;

	a->DH.Joints[2].d = d_3;
	a->DH.Joints[2].r = r_3;
	a->DH.Joints[2].alpha = alpha_3;
	a->DH.Joints[2].theta = theta_3;
	a->DH.Joints[2].orientation = orientation_3;
	a->DH.Joints[2].reduction_ratio = reduction_ratio_3;
	a->DH.Joints[2].ik_joint_weight = ik_joint_weight_3;

	a->DH.Joints[3].d = d_4;
	a->DH.Joints[3].r = r_4;
	a->DH.Joints[3].alpha = alpha_4;
	a->DH.Joints[3].theta = theta_4;
	a->DH.Joints[3].orientation = orientation_4;
	a->DH.Joints[3].reduction_ratio = reduction_ratio_4;
	a->DH.Joints[3].ik_joint_weight = ik_joint_weight_4;

	a->DH.Joints[4].d = d_5;
	a->DH.Joints[4].r = r_5;
	a->DH.Joints[4].alpha = alpha_5;
	a->DH.Joints[4].theta = theta_5;
	a->DH.Joints[4].orientation = orientation_5;
	a->DH.Joints[4].reduction_ratio = reduction_ratio_5;
	a->DH.Joints[4].ik_joint_weight = ik_joint_weight_5;

	a->DH.Joints[5].d = d_6;
	a->DH.Joints[5].r = r_6;
	a->DH.Joints[5].alpha = alpha_6;
	a->DH.Joints[5].theta = theta_6;
	a->DH.Joints[5].orientation = orientation_6;
	a->DH.Joints[5].reduction_ratio = reduction_ratio_6;
	a->DH.Joints[5].ik_joint_weight = ik_joint_weight_6;
	//Steppers IK and DH parameters config end.
}

void Emm_Request_Handle_Init(Request_Handle *emm)
{
	emm->REQUEST_FLAG = GET_STEP_REQUEST_FLAG;
	emm->REQUEST_ID = 1;
}

void PushBack(int32_t *Buf, uint8_t len, int16_t val)
{
	for(uint8_t i = 1; i < len; i++)
	{
		Buf[i - 1] = Buf[i];
	}
	Buf[len - 1] = val;
}

int16_t CLK_Delta_Total(int32_t *Buf, uint8_t len)
{
	int16_t val = 0;
	for(uint8_t i = 1; i < len; i++)
	{
		val += abs(Buf[i]);
	}
	return val;
}

float AngleRaw_to_Angle(int32_t ang_raw)
{
	return (float)ang_raw / 65536 * 360;
}

int32_t Angle_to_Step(float ang)
{
	return (int32_t)(ang * 0.1125);
}
