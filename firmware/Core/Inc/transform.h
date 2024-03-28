/*
 * transform.h
 *
 *  Created on: Jan 2, 2024
 *      Author: kylem
 */

#ifndef INC_TRANSFORM_H_
#define INC_TRANSFORM_H_

#include "matmul.h"
#include "stdint.h"
#include "math.h"

float degree_to_radian(float degree);
float radian_to_degree(float radian);

void T_init(float T[4][4]);

void R_and_t_to_T(float R[3][3], float t[3], float T[4][4]);
void T_to_R_and_t(float T[4][4], float R[3][3], float t[3]);

void rotate_x(float T[4][4], float degree);
void rotate_y(float T[4][4], float degree);
void rotate_z(float T[4][4], float degree);
void translate(float T[4][4], float x, float y, float z);
void pose_to_translation_matrix(float pos[3], float rot[3], float T[4][4]);
void translation_matrix_to_pose(float T[4][4], float pos[3], float rot[3]);

void DH_Parser(float d, float theta, float r, float alpha, float T[4][4]);
#endif /* INC_TRANSFORM_H_ */
