/*
 * command.h
 *
 *  Created on: Dec 7, 2023
 *      Author: Liaochikon
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "stdint.h"
#include "math.h"
#include "transform.h"

#define END_HEX 0x6B
#define ERROR_HEX 0xEE
#define DONE_HEX 0xDD

typedef enum
{
//single stepper command
ALTO_STEPPER_ABS = 0U,
ALTO_STEPPER_JOG,
ALTO_STEPPER_STOP,

ALTO_STEPPER_SET_ZERO,
ALTO_STEPPER_SET_EN,

ALTO_STEPPER_GET_ANGLE,
ALTO_STEPPER_GET_STEP,
ALTO_STEPPER_GET_ERROR,
ALTO_STEPPER_GET_EN,
ALTO_STEPPER_GET_BLOCK,
ALTO_STEPPER_GET_SPEED,
ALTO_STEPPER_GET_ACC,
ALTO_STEPPER_GET_ISBUSY,

ALTO_STEPPER_SET_SPEED,
ALTO_STEPPER_SET_ACC,

//robot command
ALTO_GET_MODE,
ALTO_GET_POSITION_FLAG,
ALTO_GET_BUSY_FLAG,
ALTO_GET_HOME_FLAG,
ALTO_GET_TCP_X,
ALTO_GET_TCP_Y,
ALTO_GET_TCP_Z,
ALTO_GET_TCP_RX,
ALTO_GET_TCP_RY,
ALTO_GET_TCP_RZ,
ALTO_GET_TARGET_TCP_X,
ALTO_GET_TARGET_TCP_Y,
ALTO_GET_TARGET_TCP_Z,
ALTO_GET_TARGET_TCP_RX,
ALTO_GET_TARGET_TCP_RY,
ALTO_GET_TARGET_TCP_RZ,
ALTO_GET_JOINT_ANGLE,
ALTO_GET_TARGET_JOINT_ANGLE,

ALTO_OPERATE,
ALTO_HOME,
ALTO_DISABLE,

ALTO_START,
ALTO_STOP,

ALTO_SET_JOINT_HOME_OFFSET,

ALTO_SET_JOINT_ANGLE,

ALTO_SET_COOR,
ALTO_SET_SPEED,
ALTO_SET_ACC,
ALTO_SET_MOV,
ALTO_SET_X,
ALTO_SET_Y,
ALTO_SET_Z,
ALTO_SET_RX,
ALTO_SET_RY,
ALTO_SET_RZ,

ALTO_GO_X,
ALTO_GO_Y,
ALTO_GO_Z,
ALTO_GO_RX,
ALTO_GO_RY,
ALTO_GO_RZ,

ALTO_GO_JOINT,

ALTO_GET_MESSAGE,
} _Command;

void Command_State_Machine(uint8_t* Buf, uint32_t Len);

//single stepper command
void stepper_abs(uint8_t* Buf);
void stepper_jog(uint8_t* Buf);
void stepper_stop(uint8_t* Buf);

void stepper_set_zero(uint8_t* Buf);
void stepper_set_en(uint8_t* Buf);

void stepper_get_angle(uint8_t* Buf);
void stepper_get_step(uint8_t* Buf);
void stepper_get_error(uint8_t* Buf);
void stepper_get_en(uint8_t* Buf);
void stepper_get_block(uint8_t* Buf);
void stepper_get_speed(uint8_t* Buf);
void stepper_get_acc(uint8_t* Buf);
void stepper_get_isbusy(uint8_t* Buf);

void stepper_set_speed(uint8_t* Buf);
void stepper_set_acc(uint8_t* Buf);

//robot command
void get_mode(uint8_t* Buf);
void get_position_flag(uint8_t* Buf);
void get_busy_flag(uint8_t* Buf);
void get_home_flag(uint8_t* Buf);
void get_tcp_x(uint8_t* Buf);
void get_tcp_y(uint8_t* Buf);
void get_tcp_z(uint8_t* Buf);
void get_tcp_rx(uint8_t* Buf);
void get_tcp_ry(uint8_t* Buf);
void get_tcp_rz(uint8_t* Buf);
void get_target_tcp_x(uint8_t* Buf);
void get_target_tcp_y(uint8_t* Buf);
void get_target_tcp_z(uint8_t* Buf);
void get_target_tcp_rx(uint8_t* Buf);
void get_target_tcp_ry(uint8_t* Buf);
void get_target_tcp_rz(uint8_t* Buf);
void get_joint_angle(uint8_t* Buf);
void get_target_joint_angle(uint8_t* Buf);

void operate(uint8_t* Buf);
void home(uint8_t* Buf);
void disable(uint8_t* Buf);

void start(uint8_t* Buf);
void stop(uint8_t* Buf);

void set_joint_home_offset(uint8_t* Buf);

void set_joint_angle(uint8_t* Buf);

void set_coor(uint8_t* Buf);
void set_speed(uint8_t* Buf);
void set_acc(uint8_t* Buf);
void set_mov(uint8_t* Buf);
void set_x(uint8_t* Buf);
void set_y(uint8_t* Buf);
void set_z(uint8_t* Buf);
void set_rx(uint8_t* Buf);
void set_ry(uint8_t* Buf);
void set_rz(uint8_t* Buf);

void go_x(uint8_t* Buf);
void go_y(uint8_t* Buf);
void go_z(uint8_t* Buf);
void go_rx(uint8_t* Buf);
void go_ry(uint8_t* Buf);
void go_rz(uint8_t* Buf);

void go_joint(uint8_t* Buf);

void get_msg(uint8_t* Buf);

#endif /* INC_COMMAND_H_ */
