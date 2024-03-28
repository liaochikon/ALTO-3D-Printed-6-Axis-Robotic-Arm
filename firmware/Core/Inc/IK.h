/*
 * IK.h
 *
 *  Created on: Jan 5, 2024
 *      Author: kylem
 */

#ifndef INC_IK_H_
#define INC_IK_H_
#include "alto_config.h"
#include "transform.h"

bool IK(float target_tcp_position[3], float target_tcp_rotation[3]);

#endif /* INC_IK_H_ */
