/*
 * motor.h
 *
 *  Created on: Nov 23, 2022
 *      Author: nicolas
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

struct motor{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	float speed;		// speed in rad.s-1
	int32_t speed64;	// 64x speed in integer
	uint32_t encPos;
	int32_t encSpeed64;	//
	int32_t error64;
	int32_t sumError64;
	int32_t diffError64;
	int32_t commande64;
	int8_t dir;
	uint32_t pwm;
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
};

typedef struct motor Motor_HandleTypeDef;

HAL_StatusTypeDef motorSetSpeed(Motor_HandleTypeDef *hmotor);
void motorLimitSpeed(Motor_HandleTypeDef *hmotor);
void motorLimitPWM(Motor_HandleTypeDef *hmotor);

#endif /* INC_MOTOR_H_ */
