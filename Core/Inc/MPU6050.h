/*
 * MPU6050.h
 *
 *  Created on: Aug 3, 2022
 *      Author: Prestige
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0xD0

struct MPU6050{

	I2C_HandleTypeDef *hi2c;

	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;
	float Ax;
	float Ay;
	float Az;


	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	float Gx;
	float Gy;
	float Gz;

}typedef MPU6050;


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

void MPU6050_Init (MPU6050* IMU);
void MPU6050_Read_Accel (MPU6050* IMU);
void MPU6050_Read_Gyro (MPU6050* IMU);

#endif /* INC_MPU6050_H_ */
