#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"


#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

typedef struct MPU_Accel_float {
    float x;
    float y;
    float z;
} MPU_Accel_float;

typedef struct MPU_Gyro_float {
	float x;
	float y;
	float z;
} MPU_Gyro_float;

typedef struct MPU_Accel_Raw {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU_Accel_Raw;

typedef struct MPU_Gyro_Raw {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU_Gyro_Raw;

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Read_Accel(MPU_Accel_float *accel);
HAL_StatusTypeDef MPU6050_Read_Gyro(MPU_Gyro_float *gyro);

#endif /* INC_MPU6050_H_ */
