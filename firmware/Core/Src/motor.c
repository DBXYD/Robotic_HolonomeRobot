/*
 * motor.c
 *
 *  Created on: Nov 23, 2022
 *      Author: nicolas
 */

#include "motor.h"
#include "main.h"
#include "tim.h"

// 1632.67 imp par tour

#define MAX_PWM 1024-1
#define MAX_SPEEDx64 2560 // max 40rad.s-1

HAL_StatusTypeDef motorSetSpeed(Motor_HandleTypeDef *hmotor){
	motorLimitPWM(hmotor);
	if(hmotor->commande64==0){
		hmotor->dir = 0;
		hmotor->pwm = 0;
		HAL_TIM_PWM_Stop(hmotor->htim, hmotor->channel);
		return HAL_TIMEx_PWMN_Stop(hmotor->htim, hmotor->channel);
	}
	else if(hmotor->commande64 > 0){
		hmotor->dir = 1;
		hmotor->pwm = hmotor->commande64/64;
		HAL_TIMEx_PWMN_Stop(hmotor->htim,  hmotor->channel);
		__HAL_TIM_SET_COMPARE(hmotor->htim,  hmotor->channel, hmotor->pwm);
		return HAL_TIM_PWM_Start(hmotor->htim,  hmotor->channel);
	}
	else{
		hmotor->dir = -1;
		hmotor->pwm = -hmotor->commande64/64;
		HAL_TIM_PWM_Stop(hmotor->htim,  hmotor->channel);
		__HAL_TIM_SET_COMPARE(hmotor->htim,  hmotor->channel, hmotor->pwm);
		return HAL_TIMEx_PWMN_Start(hmotor->htim, hmotor->channel);
	}

}

void motorLimitSpeed(Motor_HandleTypeDef *hmotor){
	if(hmotor->speed64>MAX_SPEEDx64) hmotor->speed64 = MAX_SPEEDx64;
	if(hmotor->speed64<-MAX_SPEEDx64) hmotor->speed64 = -MAX_SPEEDx64;
}

void motorLimitPWM(Motor_HandleTypeDef *hmotor){
	if(hmotor->pwm>MAX_SPEEDx64) hmotor->pwm = MAX_PWM;
	if(hmotor->pwm<-MAX_SPEEDx64) hmotor->pwm = -MAX_PWM;
}
