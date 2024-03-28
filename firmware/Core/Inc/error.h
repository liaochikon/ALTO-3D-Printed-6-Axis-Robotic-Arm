/*
 * error.h
 *
 *  Created on: Dec 26, 2023
 *      Author: kylem
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

typedef enum
{
MSG_STEP_ERROR = 1U,
MSG_ANGLE_ERROR,
MSG_ERROR_ERROR,
MSG_BLOCK_ERROR,
MSG_EN_ERROR,

MSG_CANBUS_ERROR,

} _Error_State;

#endif /* INC_ERROR_H_ */
