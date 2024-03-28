/*
 * Emm_can.h
 *
 *  Created on: Dec 21, 2023
 *      Author: kylem
 */

#ifndef INC_EMM_CAN_H_
#define INC_EMM_CAN_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "alto_config.h"

void CAN_Send_Cmd(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t len);

void Get_Angle_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr);
void Get_Step_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr);
void Get_Error_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr);
void Get_Flag_Request(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr);

void Stepper_Set_Zero(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr);
void Stepper_Disable_Block(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr);
void Stepper_Enable_Control(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr, bool en, bool sync);
void Stepper_Position_Control(
		CAN_HandleTypeDef *hcan,
		uint8_t *cmd,
		uint8_t addr,
		bool dir,
		uint16_t vel,
		uint8_t acc,
		uint32_t clk,
		bool abs,
		bool sync);
void Stepper_Stop(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t addr, bool sync);

#endif /* INC_EMM_CAN_H_ */
