/*
 * Emm_can.c
 *
 *  Created on: Dec 21, 2023
 *      Author: kylem
 */
#include <Emm_can.h>

void CAN_Send_Cmd(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t len)
{
	uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
	j = len - 2;

	while(i < j)
	{
		k = j - i;
		TxHeader.StdId = cmd[0];
		TxHeader.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		CAN_TXData[0] = cmd[1];
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.RTR = CAN_RTR_DATA;
		if(k < 8)
		{
			for(l=0; l < k; l++,i++)
				CAN_TXData[l + 1] = cmd[i + 2];
			TxHeader.DLC = k + 1;
		}
		else
		{
			for(l=0; l < 7; l++,i++)
				CAN_TXData[l + 1] = cmd[i + 2];
			TxHeader.DLC = 8;
		}

		HAL_StatusTypeDef msg = HAL_CAN_AddTxMessage(hcan, &TxHeader, CAN_TXData, &TxMailbox);
		if (msg != HAL_OK)
		   Error_Handler ();
		++packNum;
	}
}

void Get_Angle_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr)
{
	cmd[0] = addr;
	cmd[1] = 0x36;
	cmd[2] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 3);
}

void Get_Step_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr)
{
	cmd[0] = addr;
	cmd[1] = 0x32;
	cmd[2] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 3);
}

void Get_Error_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr)
{
	cmd[0] = addr;
	cmd[1] = 0x37;
	cmd[2] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 3);
}

void Get_Flag_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr)
{
	cmd[0] = addr;
	cmd[1] = 0x3A;
	cmd[2] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 3);
}

void Stepper_Set_Zero(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr)
{
	cmd[0] = addr;
	cmd[1] = 0x0A;
	cmd[2] = 0x6D;
	cmd[3] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 4);
}

void Stepper_Disable_Block(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr)
{
	cmd[0] = addr;
	cmd[1] = 0x0E;
	cmd[2] = 0x52;
	cmd[3] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 4);
}

void Stepper_Enable_Control(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr, bool en, bool sync)
{
	cmd[0] = addr;
	cmd[1] = 0xF3;
	cmd[2] = 0xAB;
	cmd[3] = (uint8_t)en;
	cmd[4] = (uint8_t)sync;
	cmd[5] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 6);
}

void Stepper_Position_Control(
		CAN_HandleTypeDef *hcan,
		uint8_t *cmd,
		uint8_t addr,
		bool dir,
		uint16_t vel,
		uint8_t acc,
		uint32_t clk,
		bool abs,
		bool sync)
{
	if (vel > MAX_SPEED)
		vel = MAX_SPEED;

	if (acc > MAX_ACC)
		acc = MAX_ACC;

	if (clk > MAX_STEP)
		clk = MAX_STEP;

	cmd[0] = addr;
	cmd[1] = 0xFD;
	cmd[2] = (uint8_t)dir;
	cmd[3] = (uint8_t)(vel >> 8);
	cmd[4] = (uint8_t)(vel >> 0);
	cmd[5] = acc;
	cmd[6] = (uint8_t)(clk >> 24);
	cmd[7] = (uint8_t)(clk >> 16);
	cmd[8] = (uint8_t)(clk >> 8);
	cmd[9] = (uint8_t)(clk >> 0);
	cmd[10] = (uint8_t)abs;
	cmd[11] = (uint8_t)sync;
	cmd[12] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 13);
}

void Stepper_Stop(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr, bool sync)
{
	cmd[0] = addr;
	cmd[1] = 0xFE;
	cmd[2] = 0x98;
	cmd[3] = (uint8_t)sync;
	cmd[4] = 0x6B;

	CAN_Send_Cmd(hcan, cmd, 5);
}
