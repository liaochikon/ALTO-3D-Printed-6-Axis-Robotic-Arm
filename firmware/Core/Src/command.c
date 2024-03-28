/*
 * command.c
 *
 *  Created on: Dec 7, 2023
 *      Author: Liaochikon
 */

#include "command.h"
#include "usbd_cdc_if.h"
#include "main.h"

void Command_State_Machine(uint8_t* Buf, uint32_t Len)
{
	switch(Buf[0])
	{
		//single stepper command
		case ALTO_STEPPER_ABS:
			stepper_abs(Buf);
			break;
		case ALTO_STEPPER_JOG:
			stepper_jog(Buf);
			break;
		case ALTO_STEPPER_STOP:
			stepper_stop(Buf);
			break;

		case ALTO_STEPPER_SET_ZERO:
			stepper_set_zero(Buf);
			break;
		case ALTO_STEPPER_SET_EN:
			stepper_set_en(Buf);
			break;

		case ALTO_STEPPER_GET_ANGLE:
			stepper_get_angle(Buf);
			break;
		case ALTO_STEPPER_GET_STEP:
			stepper_get_step(Buf);
			break;
		case ALTO_STEPPER_GET_ERROR:
			stepper_get_error(Buf);
			break;
		case ALTO_STEPPER_GET_EN:
			stepper_get_en(Buf);
			break;
		case ALTO_STEPPER_GET_BLOCK:
			stepper_get_block(Buf);
			break;
		case ALTO_STEPPER_GET_SPEED:
			stepper_get_speed(Buf);
			break;
		case ALTO_STEPPER_GET_ACC:
			stepper_get_acc(Buf);
			break;
		case ALTO_STEPPER_GET_ISBUSY:
			stepper_get_isbusy(Buf);
			break;

		case ALTO_STEPPER_SET_SPEED:
			stepper_set_speed(Buf);
			break;
		case ALTO_STEPPER_SET_ACC:
			stepper_set_acc(Buf);
			break;

		//robot command
		case ALTO_GET_MODE:
			get_mode(Buf);
			break;
		case ALTO_GET_POSITION_FLAG:
			get_position_flag(Buf);
			break;
		case ALTO_GET_BUSY_FLAG:
			get_busy_flag(Buf);
			break;
		case ALTO_GET_HOME_FLAG:
			get_home_flag(Buf);
			break;
		case ALTO_GET_TCP_X:
			get_tcp_x(Buf);
			break;
		case ALTO_GET_TCP_Y:
			get_tcp_y(Buf);
			break;
		case ALTO_GET_TCP_Z:
			get_tcp_z(Buf);
			break;
		case ALTO_GET_TCP_RX:
			get_tcp_rx(Buf);
			break;
		case ALTO_GET_TCP_RY:
			get_tcp_ry(Buf);
			break;
		case ALTO_GET_TCP_RZ:
			get_tcp_rz(Buf);
			break;
		case ALTO_GET_TARGET_TCP_X:
			get_target_tcp_x(Buf);
			break;
		case ALTO_GET_TARGET_TCP_Y:
			get_target_tcp_y(Buf);
			break;
		case ALTO_GET_TARGET_TCP_Z:
			get_target_tcp_z(Buf);
			break;
		case ALTO_GET_TARGET_TCP_RX:
			get_target_tcp_rx(Buf);
			break;
		case ALTO_GET_TARGET_TCP_RY:
			get_target_tcp_ry(Buf);
			break;
		case ALTO_GET_TARGET_TCP_RZ:
			get_target_tcp_rz(Buf);
			break;
		case ALTO_GET_JOINT_ANGLE:
			get_joint_angle(Buf);
			break;
		case ALTO_GET_TARGET_JOINT_ANGLE:
			get_target_joint_angle(Buf);
			break;

		case ALTO_OPERATE:
			operate(Buf);
			break;
		case ALTO_HOME:
			home(Buf);
			break;
		case ALTO_DISABLE:
			disable(Buf);
			break;

		case ALTO_START:
			start(Buf);
			break;
		case ALTO_STOP:
			stop(Buf);
			break;

		case ALTO_SET_JOINT_HOME_OFFSET:
			set_joint_home_offset(Buf);
			break;

		case ALTO_SET_JOINT_ANGLE:
			set_joint_angle(Buf);
			break;

		case ALTO_SET_COOR:
			set_coor(Buf);
			break;
		case ALTO_SET_SPEED:
			set_speed(Buf);
			break;
		case ALTO_SET_ACC:
			set_acc(Buf);
			break;
		case ALTO_SET_MOV:
			set_mov(Buf);
			break;
		case ALTO_SET_X:
			set_x(Buf);
			break;
		case ALTO_SET_Y:
			set_y(Buf);
			break;
		case ALTO_SET_Z:
			set_z(Buf);
			break;
		case ALTO_SET_RX:
			set_rx(Buf);
			break;
		case ALTO_SET_RY:
			set_ry(Buf);
			break;
		case ALTO_SET_RZ:
			set_rz(Buf);
			break;

		case ALTO_GO_X:
			go_x(Buf);
			break;
		case ALTO_GO_Y:
			go_y(Buf);
			break;
		case ALTO_GO_Z:
			go_z(Buf);
			break;
		case ALTO_GO_RX:
			go_rx(Buf);
			break;
		case ALTO_GO_RY:
			go_ry(Buf);
			break;
		case ALTO_GO_RZ:
			go_rz(Buf);
			break;

		case ALTO_GO_JOINT:
			go_joint(Buf);
			break;

		case ALTO_GET_MESSAGE:
			get_msg(Buf);
			break;
	}
}

void error_return(uint8_t c)
{
	CDC_TXData[0] = c;
	CDC_TXData[1] = ERROR_HEX;
	CDC_Transmit_FS(CDC_TXData, 2);
}

void done_return(uint8_t c)
{
	CDC_TXData[0] = c;
	CDC_TXData[1] = DONE_HEX;
	CDC_Transmit_FS(CDC_TXData, 2);
}

void stepper_abs(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	int32_t clk = (int32_t)(
			((int32_t)Buf[2] << 24) |
			((int32_t)Buf[3] << 16) |
			((int32_t)Buf[4] << 8) |
			((int32_t)Buf[5] << 0));

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_ABS);
		return;
	}

	Alto.Steppers[index].Target_Clk = clk;

	done_return(ALTO_STEPPER_ABS);
}

void stepper_jog(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	int32_t clk = (int32_t)(
			((int32_t)Buf[2] << 24) |
			((int32_t)Buf[3] << 16) |
			((int32_t)Buf[4] << 8) |
			((int32_t)Buf[5] << 0));

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_JOG);
		return;
	}

	Alto.Steppers[index].Target_Clk += clk;

	done_return(ALTO_STEPPER_JOG);
}

void stepper_stop(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_STOP);
		return;
	}

	Alto.Steppers[index].isStop = true;

	done_return(ALTO_STEPPER_STOP);
}

void stepper_set_zero(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_SET_ZERO);
		return;
	}

	Alto.Steppers[index].Set_Zero_Requset = true;

	done_return(ALTO_STEPPER_SET_ZERO);
}

void stepper_set_en(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	bool target_en = (bool)Buf[2];

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_SET_EN);
		return;
	}

	Alto.Steppers[index].Target_En = target_en;

	done_return(ALTO_STEPPER_SET_EN);
}

void stepper_get_angle(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_ANGLE);
		return;
	}

	float m = Alto.Steppers[index].Angle;
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_STEPPER_GET_ANGLE;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(m_int >> 8);
	CDC_TXData[3] = (uint8_t)(m_int >> 0);
	CDC_TXData[4] = (uint8_t)(m_dec >> 8);
	CDC_TXData[5] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 6);
}

void stepper_get_step(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_STEP);
		return;
	}

	CDC_TXData[0] = ALTO_STEPPER_GET_STEP;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(Alto.Steppers[index].Clk >> 24);
	CDC_TXData[3] = (uint8_t)(Alto.Steppers[index].Clk >> 16);
	CDC_TXData[4] = (uint8_t)(Alto.Steppers[index].Clk >> 8);
	CDC_TXData[5] = (uint8_t)(Alto.Steppers[index].Clk >> 0);
	CDC_Transmit_FS(CDC_TXData, 6);
}

void stepper_get_error(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_ERROR);
		return;
	}

	float m = Alto.Steppers[index].Error;
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_STEPPER_GET_ERROR;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(m_int >> 8);
	CDC_TXData[3] = (uint8_t)(m_int >> 0);
	CDC_TXData[4] = (uint8_t)(m_dec >> 8);
	CDC_TXData[5] = (uint8_t)(m_dec >> 0);
	CDC_Transmit_FS(CDC_TXData, 6);
}

void stepper_get_en(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_EN);
		return;
	}

	CDC_TXData[0] = ALTO_STEPPER_GET_EN;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(Alto.Steppers[index].En >> 0);
	CDC_Transmit_FS(CDC_TXData, 3);
}

void stepper_get_block(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_BLOCK);
		return;
	}

	CDC_TXData[0] = ALTO_STEPPER_GET_BLOCK;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(Alto.Steppers[index].Block >> 0);
	CDC_Transmit_FS(CDC_TXData, 3);
}

void stepper_get_speed(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_SPEED);
		return;
	}

	CDC_TXData[0] = ALTO_STEPPER_GET_SPEED;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(Alto.Steppers[index].Speed >> 8);
	CDC_TXData[3] = (uint8_t)(Alto.Steppers[index].Speed >> 0);
	CDC_Transmit_FS(CDC_TXData, 4);
}

void stepper_get_acc(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_ACC);
		return;
	}

	CDC_TXData[0] = ALTO_STEPPER_GET_ACC;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(Alto.Steppers[index].Acc >> 0);
	CDC_Transmit_FS(CDC_TXData, 3);
}

void stepper_get_isbusy(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_GET_ISBUSY);
		return;
	}

	CDC_TXData[0] = ALTO_STEPPER_GET_ISBUSY;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(Alto.Steppers[index].Clk_Delta != 0);
	CDC_Transmit_FS(CDC_TXData, 3);
}

void stepper_set_speed(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	uint16_t spd = (uint16_t)(
					((uint16_t)Buf[2] << 8) |
					((uint16_t)Buf[3] << 0));

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_SET_SPEED);
		return;
	}

	Alto.Steppers[index].Speed = spd;
	done_return(ALTO_STEPPER_SET_SPEED);
}

void stepper_set_acc(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	uint8_t acc = Buf[2];

	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_STEPPER_SET_ACC);
		return;
	}

	Alto.Steppers[index].Acc = acc;
	done_return(ALTO_STEPPER_SET_ACC);
}

void get_mode(uint8_t* Buf)
{
	CDC_TXData[0] = ALTO_GET_MODE;
	CDC_TXData[1] = (uint8_t)(Alto.Mode >> 0);
	CDC_Transmit_FS(CDC_TXData, 2);
}

void get_position_flag(uint8_t* Buf)
{
	bool Position_Flag = true;//1 : In target position, 0 : NOt in target position.

	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		if (fabsf(Alto.Target_JointAngle[index] - Alto.Current_JointAngle[index]) > CLK_DISTANCE_TOL)
		{
			Position_Flag = false;
			break;
		}
	}

	CDC_TXData[0] = ALTO_GET_POSITION_FLAG;
	CDC_TXData[1] = (uint8_t)(Position_Flag >> 0);
	CDC_Transmit_FS(CDC_TXData, 2);
}

void get_busy_flag(uint8_t* Buf)
{
	bool Busy_Flag = 1;//1 : Moving, 0 : Static.

	for(uint8_t index = 0; index < AXIS_NUM; index++)
	{
		if(Alto.Steppers[index].Clk_Delta != 0)
		{
			Busy_Flag = 1;
		}
	}
	CDC_TXData[0] = ALTO_GET_BUSY_FLAG;
	CDC_TXData[1] = (uint8_t)(Busy_Flag >> 0);
	CDC_Transmit_FS(CDC_TXData, 2);
}

void get_home_flag(uint8_t* Buf)
{
	CDC_TXData[0] = ALTO_GET_HOME_FLAG;
	CDC_TXData[1] = (uint8_t)(Alto.Home_Flag >> 0);
	CDC_Transmit_FS(CDC_TXData, 2);
}

void get_tcp_x(uint8_t* Buf)
{
	float m = Alto.Current_TCP_Position[0];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TCP_X;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_tcp_y(uint8_t* Buf)
{
	float m = Alto.Current_TCP_Position[1];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TCP_Y;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_tcp_z(uint8_t* Buf)
{
	float m = Alto.Current_TCP_Position[2];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TCP_Z;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_tcp_rx(uint8_t* Buf)
{
	float m = Alto.Current_TCP_Rotation[0];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TCP_RX;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_tcp_ry(uint8_t* Buf)
{
	float m = Alto.Current_TCP_Rotation[1];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TCP_RY;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_tcp_rz(uint8_t* Buf)
{
	float m = Alto.Current_TCP_Rotation[2];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TCP_RZ;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_target_tcp_x(uint8_t* Buf)
{
	float m = Alto.Target_TCP_Position[0];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_TCP_X;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_target_tcp_y(uint8_t* Buf)
{
	float m = Alto.Target_TCP_Position[1];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_TCP_Y;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_target_tcp_z(uint8_t* Buf)
{
	float m = Alto.Target_TCP_Position[2];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_TCP_Z;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_target_tcp_rx(uint8_t* Buf)
{
	float m = Alto.Target_TCP_Rotation[0];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_TCP_RX;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_target_tcp_ry(uint8_t* Buf)
{
	float m = Alto.Target_TCP_Rotation[1];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_TCP_RY;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_target_tcp_rz(uint8_t* Buf)
{
	float m = Alto.Target_TCP_Rotation[2];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_TCP_RZ;
	CDC_TXData[1] = (uint8_t)(m_int >> 8);
	CDC_TXData[2] = (uint8_t)(m_int >> 0);
	CDC_TXData[3] = (uint8_t)(m_dec >> 8);
	CDC_TXData[4] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 5);
}

void get_joint_angle(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	if (id > AXIS_NUM || id == 0)
	{
		return;
	}

	float m = Alto.Current_JointAngle[index];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_JOINT_ANGLE;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(m_int >> 8);
	CDC_TXData[3] = (uint8_t)(m_int >> 0);
	CDC_TXData[4] = (uint8_t)(m_dec >> 8);
	CDC_TXData[5] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 6);
}

void get_target_joint_angle(uint8_t* Buf)
{
	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	if (id > AXIS_NUM || id == 0)
	{
		return;
	}

	float m = Alto.Target_JointAngle[index];
	int16_t m_int = (int16_t)m;
	int16_t m_dec = (int16_t)((m - m_int) * 10000);

	CDC_TXData[0] = ALTO_GET_TARGET_JOINT_ANGLE;
	CDC_TXData[1] = id;
	CDC_TXData[2] = (uint8_t)(m_int >> 8);
	CDC_TXData[3] = (uint8_t)(m_int >> 0);
	CDC_TXData[4] = (uint8_t)(m_dec >> 8);
	CDC_TXData[5] = (uint8_t)(m_dec >> 0);

	CDC_Transmit_FS(CDC_TXData, 6);
}

void operate(uint8_t* Buf)
{
	Alto.Mode = OPERATING_MODE;
	done_return(ALTO_OPERATE);
}

void home(uint8_t* Buf)
{
	Alto.Mode = HOMING_MODE;
	done_return(ALTO_HOME);
}

void disable(uint8_t* Buf)
{
	Alto.Mode = DISABLE_MODE;
	done_return(ALTO_DISABLE);
}

void start(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_START);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;
	done_return(ALTO_START);
}

void stop(uint8_t* Buf)
{
	for(uint8_t index = 0; index < AXIS_NUM; index++)
		Alto.Steppers[index].isStop = true;
	done_return(ALTO_START);
}

void set_joint_home_offset(uint8_t* Buf)
{

}

void set_joint_angle(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_JOINT_ANGLE);
		return;
	}

	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_SET_JOINT_ANGLE);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[2] << 8 | (int16_t)Buf[3] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[4] << 8 | (int16_t)Buf[5] << 0);
	Alto.Target_JointAngle[index] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_JOINT_ANGLE);
}

void set_coor(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_COOR);
		return;
	}

	switch(Buf[1])
	{
		case BASE:
			Alto.Coor = BASE;
			done_return(ALTO_SET_COOR);
			break;
		case TOOL:
			Alto.Coor = TOOL;
			done_return(ALTO_SET_COOR);
			break;
		default:
			error_return(ALTO_SET_COOR);
			break;
	}
}

void set_speed(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_SPEED);
		return;
	}

	Alto.Speed = Buf[1];
	done_return(ALTO_SET_SPEED);
}

void set_acc(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_ACC);
		return;
	}

	Alto.Acc = Buf[1];
	done_return(ALTO_SET_ACC);
}

void set_mov(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_MOV);
		return;
	}

	switch(Buf[1])
	{
		case MOVP:
			Alto.Mov = MOVP;
			done_return(ALTO_SET_MOV);
			break;
//		case MOVL:
//			Alto.Mov = MOVL;
//			done_return(ALTO_SET_MOV);
//			break;
		case JOINT:
			Alto.Mov = JOINT;
			done_return(ALTO_SET_MOV);
			break;
		default:
			error_return(ALTO_SET_MOV);
			break;
	}
}

void set_x(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_X);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	Alto.Target_TCP_Position[0] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_X);
}

void set_y(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_Y);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	Alto.Target_TCP_Position[1] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_Y);
}

void set_z(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_Z);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	Alto.Target_TCP_Position[2] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_Z);
}

void set_rx(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_RX);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	Alto.Target_TCP_Rotation[0] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_RX);
}

void set_ry(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_RY);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	Alto.Target_TCP_Rotation[1] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_RY);
}

void set_rz(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_SET_RZ);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	Alto.Target_TCP_Rotation[2] = (float)((float)m_int + (float)m_dec / 10000);

	done_return(ALTO_SET_RZ);
}

void go_x(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_X);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	float x = (float)((float)m_int + (float)m_dec / 10000);

	Alto.Target_TCP_Position[0] = Alto.Current_TCP_Position[0] + x;
	Alto.Target_TCP_Position[1] = Alto.Current_TCP_Position[1];
	Alto.Target_TCP_Position[2] = Alto.Current_TCP_Position[2];
	Alto.Target_TCP_Rotation[0] = Alto.Current_TCP_Rotation[0];
	Alto.Target_TCP_Rotation[1] = Alto.Current_TCP_Rotation[1];
	Alto.Target_TCP_Rotation[2] = Alto.Current_TCP_Rotation[2];
	Alto.Mov = MOVP;

	done_return(ALTO_GO_X);
}

void go_y(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_Y);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	float y = (float)((float)m_int + (float)m_dec / 10000);

	Alto.Target_TCP_Position[0] = Alto.Current_TCP_Position[0];
	Alto.Target_TCP_Position[1] = Alto.Current_TCP_Position[1] + y;
	Alto.Target_TCP_Position[2] = Alto.Current_TCP_Position[2];
	Alto.Target_TCP_Rotation[0] = Alto.Current_TCP_Rotation[0];
	Alto.Target_TCP_Rotation[1] = Alto.Current_TCP_Rotation[1];
	Alto.Target_TCP_Rotation[2] = Alto.Current_TCP_Rotation[2];
	Alto.Mov = MOVP;

	done_return(ALTO_GO_Y);
}

void go_z(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_Z);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	float z = (float)((float)m_int + (float)m_dec / 10000);

	Alto.Target_TCP_Position[0] = Alto.Current_TCP_Position[0];
	Alto.Target_TCP_Position[1] = Alto.Current_TCP_Position[1];
	Alto.Target_TCP_Position[2] = Alto.Current_TCP_Position[2] + z;
	Alto.Target_TCP_Rotation[0] = Alto.Current_TCP_Rotation[0];
	Alto.Target_TCP_Rotation[1] = Alto.Current_TCP_Rotation[1];
	Alto.Target_TCP_Rotation[2] = Alto.Current_TCP_Rotation[2];
	Alto.Mov = MOVP;

	done_return(ALTO_GO_Z);
}

void go_rx(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_RX);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	float rx = (float)((float)m_int + (float)m_dec / 10000);

	float T_TCP[4][4];
	float T_rx[4][4];
	pose_to_translation_matrix(Alto.Current_TCP_Position, Alto.Current_TCP_Rotation, T_TCP);
	T_init(T_rx);
	rotate_x(T_rx, rx);
	float T_ans[4][4];
	Mat4x4_Mul(T_TCP, T_rx, T_ans);
	translation_matrix_to_pose(T_ans, Alto.Target_TCP_Position, Alto.Target_TCP_Rotation);
	Alto.Mov = MOVP;

	done_return(ALTO_GO_RX);
}

void go_ry(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_RY);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	float ry = (float)((float)m_int + (float)m_dec / 10000);

	float T_TCP[4][4];
	float T_ry[4][4];
	pose_to_translation_matrix(Alto.Current_TCP_Position, Alto.Current_TCP_Rotation, T_TCP);
	T_init(T_ry);
	rotate_y(T_ry, ry);
	float T_ans[4][4];
	Mat4x4_Mul(T_TCP, T_ry, T_ans);
	translation_matrix_to_pose(T_ans, Alto.Target_TCP_Position, Alto.Target_TCP_Rotation);
	Alto.Mov = MOVP;

	done_return(ALTO_GO_RY);
}

void go_rz(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_RZ);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	int16_t m_int = (int16_t)((int16_t)Buf[1] << 8 | (int16_t)Buf[2] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[3] << 8 | (int16_t)Buf[4] << 0);
	float rz = (float)((float)m_int + (float)m_dec / 10000);

	float T_TCP[4][4];
	float T_rz[4][4];
	pose_to_translation_matrix(Alto.Current_TCP_Position, Alto.Current_TCP_Rotation, T_TCP);
	T_init(T_rz);
	rotate_z(T_rz, rz);
	float T_ans[4][4];
	Mat4x4_Mul(T_TCP, T_rz, T_ans);
	translation_matrix_to_pose(T_ans, Alto.Target_TCP_Position, Alto.Target_TCP_Rotation);
	Alto.Mov = MOVP;

	done_return(ALTO_GO_RZ);
}

void go_joint(uint8_t* Buf)
{
	if(Alto.Start_Flag)
	{
		error_return(ALTO_GO_JOINT);
		return;
	}

	Alto.Start_Flag = true;
	Alto.Start_Entry_Flag = true;

	uint8_t id = Buf[1];
	uint8_t index = id - 1;
	if (id > AXIS_NUM || id == 0)
	{
		error_return(ALTO_GO_JOINT);
		return;
	}

	int16_t m_int = (int16_t)((int16_t)Buf[2] << 8 | (int16_t)Buf[3] << 0);
	int16_t m_dec = (int16_t)((int16_t)Buf[4] << 8 | (int16_t)Buf[5] << 0);
	float ang = (float)((float)m_int + (float)m_dec / 10000);
	Alto.Target_JointAngle[index] = Alto.Current_JointAngle[index] + ang;
	Alto.Mov = JOINT;

	done_return(ALTO_GO_JOINT);
}

void get_msg(uint8_t* Buf)
{

}
