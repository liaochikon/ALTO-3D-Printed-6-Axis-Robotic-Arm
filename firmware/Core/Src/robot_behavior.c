/*
 * robot_behavior.c
 *
 *  Created on: Dec 17, 2023
 *      Author: Liaochikon
 */
#include "cmsis_os.h"
#include "Emm_can.h"
#include "robot_behavior.h"
#include "main.h"
#include "FK.h"
#include "IK.h"

void Robot_Callback(CAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef msg = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RXData);
	if(msg != HAL_OK)
		Error_Handler();

	uint8_t index = Emm_Request_Handle.REQUEST_ID - 1;
	uint8_t cmd = CAN_RXData[0];

	if(Emm_Request_Handle.REQUEST_FLAG == GET_STEP_REQUEST_FLAG && cmd == 0x32)
	{
		int32_t clk =
				(int32_t)(((int32_t)CAN_RXData[2] << 24) |
						  ((int32_t)CAN_RXData[3] << 16) |
						  ((int32_t)CAN_RXData[4] << 8) |
						  ((int32_t)CAN_RXData[5] << 0));
		if ((bool)CAN_RXData[1])
			clk = -clk;
		int32_t old_clk = Alto.Steppers[index].Clk;
		PushBack(Alto.Steppers[index]._Clk_Delta, CLK_DELTA_SAMPLE_LENGTH, (int32_t)(clk - old_clk));
		Alto.Steppers[index].Clk_Delta = CLK_Delta_Total(Alto.Steppers[index]._Clk_Delta, CLK_DELTA_SAMPLE_LENGTH);
		Alto.Steppers[index].Clk = clk;

		Emm_Request_Handle.REQUEST_FLAG = GET_ANGLE_REQUEST_FLAG;
	}
	if(Emm_Request_Handle.REQUEST_FLAG == GET_ANGLE_REQUEST_FLAG && cmd == 0x36)
	{
		int32_t ang_raw =
				(int32_t)(((int32_t)CAN_RXData[2] << 24) |
						  ((int32_t)CAN_RXData[3] << 16) |
						  ((int32_t)CAN_RXData[4] << 8) |
						  ((int32_t)CAN_RXData[5] << 0));
		if ((bool)CAN_RXData[1])
			ang_raw = -ang_raw;
		Alto.Steppers[index].Angle = (float)ang_raw / 65536 * 360;
		Alto.Current_JointAngle[index] =
				Alto.Steppers[index].Angle / Alto.DH.Joints[index].reduction_ratio * Alto.DH.Joints[index].orientation;

		Emm_Request_Handle.REQUEST_FLAG = GET_ERROR_REQUEST_FLAG;
	}
	if(Emm_Request_Handle.REQUEST_FLAG == GET_ERROR_REQUEST_FLAG && cmd == 0x37)
	{
		int32_t error =
				(int32_t)(((int32_t)CAN_RXData[2] << 24) |
						  ((int32_t)CAN_RXData[3] << 16) |
						  ((int32_t)CAN_RXData[4] << 8) |
						  ((int32_t)CAN_RXData[5] << 0));
		if ((bool)CAN_RXData[1])
			error = -error;
		Alto.Steppers[index].Error = (float)error / 65536 * 360;

		Emm_Request_Handle.REQUEST_FLAG = GET_FLAG_REQUEST_FLAG;
	}
	if(Emm_Request_Handle.REQUEST_FLAG == GET_FLAG_REQUEST_FLAG && cmd == 0x3A)
	{
		uint8_t flag = CAN_RXData[1];
		Alto.Steppers[index].En = (bool)(flag & 0x01);
		Alto.Steppers[index].Block = (bool)(flag & 0x04);

		Emm_Request_Handle.REQUEST_FLAG = GET_STEP_REQUEST_FLAG;
	}
}

void Robot_Master(CAN_HandleTypeDef *hcan, uint8_t *cmd)
{
	System_Base(hcan, cmd);
	switch(Alto.Mode)
	{
		case OPERATING_MODE:
			Operating_Mode();
			break;
		case HOMING_MODE:
			Homing_Mode();
			break;
		case DISABLE_MODE:
			Disable_Mode();
			break;
	}
}

void Joint_Activation(bool en)
{
	for(uint8_t index = 0; index < AXIS_NUM; index++)
		Alto.Steppers[index].Target_En = en;
}

void Operating_Mode()
{
	Joint_Activation(true);
	FK();
	if(Alto.Start_Flag)
	{
		switch(Alto.Mov)
		{
			case MOVP:
				MovP_Manager();
				break;
//			case MOVL:
//				MovL_Manager();
//				break;
			case JOINT:
				Joint_Manager();
				break;
		}
	}
}

void Homing_Mode()
{
	Joint_Activation(true);
	uint8_t id;
	uint8_t index;
	id = Alto.Homing_Joint;
	index = id - 1;
	switch(Alto.Homing_Joint)
	{
		case 3:
			Home_Manager(Limit_Switch_3_Pin, index,
					JOINT3_HITTING_LIMIT_DIST,
					JOINT3_BACK_TO_ZERO_DIST + JOINT3_BACK_TO_ZERO_DIST_OFFSET);
			if(Alto.Homing_State == SET_JOINT_ZERO)
			{
				Alto.Homing_Joint = 2;
				Alto.Homing_State = HITTING_LIMIT_ENTEY;
			}

			break;
		case 2:
			Home_Manager(Limit_Switch_2_Pin, index,
					JOINT2_HITTING_LIMIT_DIST,
					JOINT2_BACK_TO_ZERO_DIST + JOINT2_BACK_TO_ZERO_DIST_OFFSET);
			if(Alto.Homing_State == SET_JOINT_ZERO)
			{
				Alto.Homing_Joint = 5;
				Alto.Homing_State = HITTING_LIMIT_ENTEY;
			}
			break;
		case 5:
			Home_Manager(Limit_Switch_5_Pin, index,
					JOINT5_HITTING_LIMIT_DIST,
					JOINT5_BACK_TO_ZERO_DIST + JOINT5_BACK_TO_ZERO_DIST_OFFSET);
			if(Alto.Homing_State == SET_JOINT_ZERO)
			{
				Alto.Homing_Joint = 6;
				Alto.Homing_State = HITTING_LIMIT_ENTEY;
			}
			break;
		case 6:
			Home_Manager(Limit_Switch_6_Pin, index,
					JOINT6_HITTING_LIMIT_DIST,
					JOINT6_BACK_TO_ZERO_DIST + JOINT6_BACK_TO_ZERO_DIST_OFFSET);
			if(Alto.Homing_State == SET_JOINT_ZERO)
			{
				Alto.Homing_Joint = 4;
				Alto.Homing_State = HITTING_LIMIT_ENTEY;
			}
			break;
		case 4:
			Home_Manager(Limit_Switch_4_Pin, index,
					JOINT4_HITTING_LIMIT_DIST,
					JOINT4_BACK_TO_ZERO_DIST + JOINT4_BACK_TO_ZERO_DIST_OFFSET);
			if(Alto.Homing_State == SET_JOINT_ZERO)
			{
				Alto.Homing_Joint = 1;
				Alto.Homing_State = HITTING_LIMIT_ENTEY;
			}
			break;
		case 1:
			Home_Manager(Limit_Switch_1_Pin, index,
					JOINT1_HITTING_LIMIT_DIST,
					JOINT1_BACK_TO_ZERO_DIST + JOINT1_BACK_TO_ZERO_DIST_OFFSET);
			if(Alto.Homing_State == SET_JOINT_ZERO)
			{
				Robot_Config_Init(&Alto);
				Alto.Mode = OPERATING_MODE;
				Alto.Home_Flag = true;
			}
			break;
	}
}

void Disable_Mode()
{
	Joint_Activation(false);
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		Alto.Target_JointAngle[index] = Alto.Current_JointAngle[index];
		Alto.Steppers[index].Target_Clk =
						(int32_t)(Alto.Target_JointAngle[index] * Alto.DH.Joints[index].reduction_ratio / 0.1125) * Alto.DH.Joints[index].orientation;
	}
	FK();
}

void System_Base(CAN_HandleTypeDef *hcan, uint8_t *cmd)
{
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		Info_Manager(hcan, cmd, index);
		Motion_Manager(hcan, cmd, index);
	}
}

void Home_Manager(uint16_t limit_sw, uint8_t index, int32_t hitting_limit_clk, int32_t back_to_zero_clk)
{
	switch(Alto.Homing_State)
	{
		case HITTING_LIMIT_ENTEY:
			Alto.Steppers[index].Speed = DEFAULT_FAST_SPEED;
			Alto.Steppers[index].Target_Clk += hitting_limit_clk;
			Alto.Homing_State = HITTING_LIMIT;
			break;
		case HITTING_LIMIT:
			if(HAL_GPIO_ReadPin(GPIOE, limit_sw) == 0)
			{
				Alto.Steppers[index].isStop = true;
				Alto.Homing_State = CORRECTING_1_LIMIT_ENTEY;
			}
			break;
		case CORRECTING_1_LIMIT_ENTEY:
			Alto.Steppers[index].Speed = DEFAULT_SLOW_SPEED;
			Alto.Steppers[index].Target_Clk -= hitting_limit_clk * 2;
			Alto.Homing_State = CORRECTING_1_LIMIT;
		case CORRECTING_1_LIMIT:
			if(HAL_GPIO_ReadPin(GPIOE, limit_sw) == 1)
			{
				Alto.Steppers[index].isStop = true;
				Alto.Homing_State = CORRECTING_2_LIMIT_ENTEY;
			}
			break;
		case CORRECTING_2_LIMIT_ENTEY:
			Alto.Steppers[index].Target_Clk += hitting_limit_clk * 2;
			Alto.Homing_State = CORRECTING_2_LIMIT;
			break;
		case CORRECTING_2_LIMIT:
			if(HAL_GPIO_ReadPin(GPIOE, limit_sw) == 0)
			{
				Alto.Steppers[index].isStop = true;
				Alto.Homing_State = BACK_TO_ZERO_ENTEY;
			}
			break;
		case BACK_TO_ZERO_ENTEY:
			Alto.Steppers[index].Speed = DEFAULT_FAST_SPEED;
			Alto.Steppers[index].Target_Clk += back_to_zero_clk;
			Alto.Homing_State = BACK_TO_ZERO;
			break;
		case BACK_TO_ZERO:
			if(Alto.Steppers[index].Target_Clk == Alto.Steppers[index].Clk)
			{
				Alto.Homing_State = SET_JOINT_ZERO_ENTEY;
				Alto.Steppers[index].Set_Zero_Requset = true;
			}
			break;
		case SET_JOINT_ZERO_ENTEY:
			if(Alto.Steppers[index].Set_Zero_Requset == false)
				Alto.Homing_State = SET_JOINT_ZERO;
			break;
	}
}

void Set_Zero_Manager(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t index)
{
	uint8_t id = index + 1;
	Alto.Steppers[index].Clk = 0;
	Alto.Steppers[index].Angle = 0;
	Alto.Steppers[index].Block = 0;
	for(uint8_t i = 0; i < CLK_DELTA_SAMPLE_LENGTH; i++)
		Alto.Steppers[i]._Clk_Delta[i] = 0;
	Alto.Steppers[index].Clk_Delta = 0;
	Alto.Steppers[index].Target_Clk = 0;
	Alto.Steppers[index].Set_Zero_Requset = false;
	Stepper_Set_Zero(hcan, cmd, id);
	osDelay(DELAY_MS);
}

void Info_Manager(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t index)
{
	uint8_t id = index + 1;
	Emm_Request_Handle.REQUEST_ID = id;

	while(Emm_Request_Handle.REQUEST_FLAG == GET_STEP_REQUEST_FLAG)
	{
		Get_Step_Request(hcan, cmd, id);
		osDelay(DELAY_MS);
	}

	while(Emm_Request_Handle.REQUEST_FLAG == GET_ANGLE_REQUEST_FLAG)
	{
		Get_Angle_Request(hcan, cmd, id);
		osDelay(DELAY_MS);
	}

	while(Emm_Request_Handle.REQUEST_FLAG == GET_ERROR_REQUEST_FLAG)
	{
		Get_Error_Request(hcan, cmd, id);
		osDelay(DELAY_MS);;
	}

	while(Emm_Request_Handle.REQUEST_FLAG == GET_FLAG_REQUEST_FLAG)
	{
		Get_Flag_Request(hcan, cmd, id);
		osDelay(DELAY_MS);
	}
}

void Speed_Average_Manager()
{
	int32_t dist[AXIS_NUM] = {0};
	int32_t max_dist = 0;
	for (uint8_t index = 0; index < AXIS_NUM; index++)
	{
		dist[index] = abs(Alto.Steppers[index].Target_Clk - Alto.Steppers[index].Clk);
		if (dist[index] > max_dist)
		{
			max_dist = dist[index];
		}
	}

	for (uint8_t index = 0; index < AXIS_NUM; index++)
	{
		Alto.Steppers[index].Acc = Alto.Acc;
		Alto.Steppers[index].Speed = (uint8_t)((float)dist[index] / max_dist * Alto.Speed);
	}
}

void MovP_Manager()
{
	if(Alto.Start_Entry_Flag)
	{
		bool ret = IK(Alto.Target_TCP_Position, Alto.Target_TCP_Rotation);
		if(ret)
		{
			for(uint8_t index = 0; index < AXIS_NUM; index++)
			{
				Alto.Steppers[index].Target_Clk =
								(int32_t)(Alto.Target_JointAngle[index] * Alto.DH.Joints[index].reduction_ratio / 0.1125) * Alto.DH.Joints[index].orientation;
			}
			Speed_Average_Manager();
		}
		Alto.Start_Entry_Flag = false;
	}

	bool Position_Flag = true;//1 : In target position, 0 : NOt in target position.
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		if (fabsf(Alto.Target_JointAngle[index] - Alto.Current_JointAngle[index]) > CLK_DISTANCE_TOL)
		{
			Position_Flag = false;
			break;
		}
	}
	if(Position_Flag)
		Alto.Start_Flag = false;
}

//void MovL_Manager()
//{
//
//}

void Joint_Manager()
{
	if(Alto.Start_Entry_Flag)
	{
		for(uint8_t index = 0; index < AXIS_NUM; index++)
		{
			//Target_JointAngle limitation check
			if(Alto.Target_JointAngle[index] > Alto.Upper_JointAngle_Limit[index])
				Alto.Target_JointAngle[index] = Alto.Upper_JointAngle_Limit[index];
			if(Alto.Target_JointAngle[index] < Alto.Lower_JointAngle_Limit[index])
				Alto.Target_JointAngle[index] = Alto.Lower_JointAngle_Limit[index];

			Alto.Steppers[index].Target_Clk =
					(int32_t)(Alto.Target_JointAngle[index] * Alto.DH.Joints[index].reduction_ratio / 0.1125) * Alto.DH.Joints[index].orientation;
		}
		Speed_Average_Manager();
		Alto.Start_Entry_Flag = false;
	}

	bool Position_Flag = true;//1 : In target position, 0 : NOt in target position.
	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		if (fabsf(Alto.Target_JointAngle[index] - Alto.Current_JointAngle[index]) > CLK_DISTANCE_TOL)
		{
			Position_Flag = false;
			break;
		}
	}
	if(Position_Flag)
		Alto.Start_Flag = false;
}

void Motion_Manager(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t index)
{
	uint8_t id = index + 1;
	//stop
	if(Alto.Steppers[index].isStop)
	{
		Stepper_Stop(hcan, cmd, id, false);
		osDelay(DELAY_MS);
		Alto.Steppers[index].Target_Clk = (int32_t)(Alto.Steppers[index].Angle / 0.1125);
		Alto.Steppers[index].isStop = false;
		return;
	}

	//busy
	if(Alto.Steppers[index].Clk_Delta != 0)
		return;

	//set zero
	if(Alto.Steppers[index].Set_Zero_Requset)
	{
		Set_Zero_Manager(hcan, cmd, index);
		return;
	}

	//enable
	if(Alto.Steppers[index].En != Alto.Steppers[index].Target_En)
	{
		Stepper_Enable_Control(hcan, cmd, id, Alto.Steppers[index].Target_En, false);
		osDelay(DELAY_MS);
	}
	else
	{
		if(Alto.Steppers[index].En == false)
		{
			Alto.Steppers[index].Target_Clk = (int32_t)(Alto.Steppers[index].Angle / 0.1125);
			return;
		}
	}

	//move
	uint16_t spd = Alto.Steppers[index].Speed;
	uint8_t acc = Alto.Steppers[index].Acc;
	int32_t clk_mov = Alto.Steppers[index].Target_Clk;
	uint8_t dir = (uint8_t)(clk_mov < 0);
	uint32_t clk_mov_abs = (uint32_t)abs(clk_mov);
	if(Alto.Steppers[index].Target_Clk != Alto.Steppers[index].Clk)
	{
		Stepper_Position_Control(hcan, cmd, id, dir, spd, acc, clk_mov_abs, true, false);
		osDelay(DELAY_MS);
		return;
	}
}
