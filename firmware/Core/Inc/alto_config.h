/*
 * alto_config.h
 *
 *  Created on: Dec 24, 2023
 *      Author: kylem
 */

#ifndef INC_ALTO_CONFIG_H_
#define INC_ALTO_CONFIG_H_

#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include "main.h"

#define CLK_DELTA_SAMPLE_LENGTH 10
#define AXIS_NUM 6
#define DELAY_MS 1
#define DATA_TEMP_LENGTH 20
#define CAN_DATA_LENGTH 8
#define CDC_DATA_LENGTH 8

#define MAX_SPEED 0xff
#define MAX_ACC 0xff
#define MAX_STEP 0xffffff

#define DEFAULT_FAST_SPEED 80
#define DEFAULT_SLOW_SPEED 30
#define DEFAULT_ACC 255

#define GO_DIST 10.0
#define GO_ANG 5.0

//Joint homing configs start:
#define JOINT1_HITTING_LIMIT_DIST 160000
#define JOINT2_HITTING_LIMIT_DIST 80000
#define JOINT3_HITTING_LIMIT_DIST 48000
#define JOINT4_HITTING_LIMIT_DIST 96000
#define JOINT5_HITTING_LIMIT_DIST 64000
#define JOINT6_HITTING_LIMIT_DIST 96000

#define JOINT1_BACK_TO_ZERO_DIST -80000
#define JOINT2_BACK_TO_ZERO_DIST -40000
#define JOINT3_BACK_TO_ZERO_DIST -24000
#define JOINT4_BACK_TO_ZERO_DIST -48000
#define JOINT5_BACK_TO_ZERO_DIST -32000
#define JOINT6_BACK_TO_ZERO_DIST -48000

#define JOINT1_BACK_TO_ZERO_DIST_OFFSET 1334
#define JOINT2_BACK_TO_ZERO_DIST_OFFSET 2666
#define JOINT3_BACK_TO_ZERO_DIST_OFFSET 666
#define JOINT4_BACK_TO_ZERO_DIST_OFFSET 1333
#define JOINT5_BACK_TO_ZERO_DIST_OFFSET 933
#define JOINT6_BACK_TO_ZERO_DIST_OFFSET 2400
//Joint homing configs end.

//Joint IK config start:
#define CLK_DISTANCE_TOL 0.1

#define JOINT1_ANGLE_UPPER_LIMIT 180.0
#define JOINT1_ANGLE_LOWER_LIMIT -180.0
#define JOINT2_ANGLE_UPPER_LIMIT 90.0
#define JOINT2_ANGLE_LOWER_LIMIT -90.0
#define JOINT3_ANGLE_UPPER_LIMIT 90.0
#define JOINT3_ANGLE_LOWER_LIMIT -90.0
#define JOINT4_ANGLE_UPPER_LIMIT 180.0
#define JOINT4_ANGLE_LOWER_LIMIT -180.0
#define JOINT5_ANGLE_UPPER_LIMIT 120.0
#define JOINT5_ANGLE_LOWER_LIMIT -120.0
#define JOINT6_ANGLE_UPPER_LIMIT 180.0
#define JOINT6_ANGLE_LOWER_LIMIT -180.0
//Joint IK config end.

//DH Parameter configs start:
#define d_1 160
#define theta_1 0
#define r_1 35
#define alpha_1 90
#define orientation_1 -1
#define reduction_ratio_1 50
#define ik_joint_weight_1 1

#define d_2 0
#define theta_2 90
#define r_2 200
#define alpha_2 0
#define orientation_2 1
#define reduction_ratio_2 50
#define ik_joint_weight_2 1

#define d_3 0
#define theta_3 0
#define r_3 85
#define alpha_3 90
#define orientation_3 1
#define reduction_ratio_3 30
#define ik_joint_weight_3 1

#define d_4 155
#define theta_4 0
#define r_4 0
#define alpha_4 -90
#define orientation_4 1
#define reduction_ratio_4 30
#define ik_joint_weight_4 3

#define d_5 0
#define theta_5 0
#define r_5 0
#define alpha_5 90
#define orientation_5 1
#define reduction_ratio_5 30
#define ik_joint_weight_5 2

#define d_6 143.5
#define theta_6 0
#define r_6 0
#define alpha_6 0
#define orientation_6 1
#define reduction_ratio_6 30
#define ik_joint_weight_6 1
//DH Parameter configs end.

typedef enum
{
	GET_ANGLE_REQUEST_FLAG = 0U,
	GET_STEP_REQUEST_FLAG,
	GET_ERROR_REQUEST_FLAG,
	GET_FLAG_REQUEST_FLAG,
} _Request_Handle_State;

typedef enum
{
	OPERATING_MODE = 0U,
	HOMING_MODE,
	DISABLE_MODE,
} _Alto_Mode;

typedef enum
{
	MOVP,
//	MOVL,
	JOINT,
} _Mov_Mode;

typedef enum
{
	DO,
	GO,
} _Execute_Mode;

typedef enum
{
	BASE,
	TOOL,

} _Coor_Mode;

typedef enum
{
	HITTING_LIMIT_ENTEY = 0U,
	HITTING_LIMIT,
	CORRECTING_1_LIMIT_ENTEY,
	CORRECTING_1_LIMIT,
	CORRECTING_2_LIMIT_ENTEY,
	CORRECTING_2_LIMIT,
	BACK_TO_ZERO_ENTEY,
	BACK_TO_ZERO,
	SET_JOINT_ZERO_ENTEY,
	SET_JOINT_ZERO,
} _Homing_State;

typedef struct Request_Handle_sturct
{
	uint8_t REQUEST_FLAG;
	uint8_t REQUEST_ID;
}Request_Handle;


typedef struct Steppers_struct
{
	uint8_t id;

	uint16_t Speed;
	uint8_t Acc;

	int32_t Clk;
	float Angle;
	float Error;
	bool En;
	bool Block;

	int32_t _Clk_Delta[CLK_DELTA_SAMPLE_LENGTH];
	int32_t Clk_Delta;

	int32_t Target_Clk;
	bool Target_En;

	bool isStop;
	bool Set_Zero_Requset;
} Stepper;

typedef struct DH_Unit_struct
{
  float d;
  float theta;
  float r;
  float alpha;
  int8_t orientation;
  uint8_t reduction_ratio;
  uint8_t ik_joint_weight;
}DH_Unit;

typedef struct DH_Parameter_struct
{
	DH_Unit Joints[AXIS_NUM];
} DH_Parameter;

typedef struct Robot_struct
{
	Stepper Steppers[AXIS_NUM];
	DH_Parameter DH;

	float Upper_JointAngle_Limit[6];
	float Lower_JointAngle_Limit[6];

	float Current_JointAngle[6];
	float Current_TCP_Position[3];
	float Current_TCP_Rotation[3];
	float Target_JointAngle[6];
	float Target_TCP_Position[3];
	float Target_TCP_Rotation[3];

	uint8_t Execute;
	uint8_t Mov;
	uint8_t Coor;
	uint8_t Speed;
	uint8_t Acc;

	uint8_t Mode;
	bool Start_Flag;
	bool Start_Entry_Flag;
	bool Stop_Flag;
	bool Home_Flag;//1 : Homed, 0 : Not homed yet.
	uint8_t Homing_State;
	uint8_t Homing_Joint;
} Robot;

void Stepper_Init(Stepper stp, uint8_t id);
void Robot_Config_Init(Robot *a);
void Emm_Request_Handle_Init(Request_Handle *emm);

void PushBack(int32_t *Buf, uint8_t len, int16_t val);
int16_t CLK_Delta_Total(int32_t *Buf, uint8_t len);

float AngleRaw_to_Angle(int32_t ang_raw);
int32_t Angle_to_Step(float ang);

extern Robot Alto;
extern Request_Handle Emm_Request_Handle;

extern uint8_t Data_Temp[DATA_TEMP_LENGTH];
extern uint8_t CAN_TXData[CAN_DATA_LENGTH];
extern uint8_t CAN_RXData[CAN_DATA_LENGTH];
extern uint8_t CDC_TXData[CDC_DATA_LENGTH];
extern uint8_t CDC_RXData[CDC_DATA_LENGTH];

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;

#endif /* INC_ALTO_CONFIG_H_ */
