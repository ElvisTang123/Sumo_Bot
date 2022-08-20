/*
 * MPU6050.c
 *
 *  Created on: Aug 3, 2022
 *      Author: Prestige
 */

#include "MPU6050.h"

void MPU6050_Init (MPU6050* IMU)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (IMU->hi2c, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(IMU->hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(IMU->hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=1 -> ± 4g
		Data = 0x01;
		HAL_I2C_Mem_Write(IMU->hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(IMU->hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}


void MPU6050_Read_Accel (MPU6050* IMU)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (IMU->hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	IMU->Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	IMU->Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	IMU->Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 1. So I am dividing by 8192.0
	     for more details check ACCEL_CONFIG Register              ****/
	IMU->Ax = IMU->Accel_X/8192.0;
	IMU->Ay = IMU->Accel_Y/8192.0;
	IMU->Az = IMU->Accel_Z/8192.0;
}


void MPU6050_Read_Gyro (MPU6050* IMU)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (IMU->hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	IMU->Gyro_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	IMU->Gyro_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	IMU->Gyro_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	IMU->Gx = (float)(IMU->Gyro_X)/131.0;
	IMU->Gy = (float)(IMU->Gyro_Y)/131.0;
	IMU->Gz = (float)(IMU->Gyro_Z)/131.0;
}
