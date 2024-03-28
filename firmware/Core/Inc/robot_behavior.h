/*
 * robot_behavior.h
 *
 *  Created on: Dec 17, 2023
 *      Author: Liaochikon
 */

#ifndef INC_ROBOT_BEHAVIOR_H_
#define INC_ROBOT_BEHAVIOR_H_

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "main.h"

void Robot_Callback(CAN_HandleTypeDef *hcan);
void Robot_Master(CAN_HandleTypeDef *hcan, uint8_t *cmd);

void Joint_Activation(bool en);

void Operating_Mode();
void Homing_Mode();
void Disable_Mode();

void System_Base(CAN_HandleTypeDef *hcan, uint8_t *cmd);

void Home_Manager(uint16_t limit_sw, uint8_t index, int32_t hitting_limit_clk, int32_t back_to_zero_clk);
void Set_Zero_Manager(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t index);
void Info_Manager(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t index);
void Speed_Average_Manager();
void MovP_Manager();
//void MovL_Manager();
void Joint_Manager();
void Motion_Manager(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t index);

#endif /* INC_ROBOT_BEHAVIOR_H_ */
