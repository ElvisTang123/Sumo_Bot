/*
 * lidar.c
 *
 *  Created on: Aug 9, 2022
 *      Author: jackson
 */


#include "lidar.h"

void check_ref_reg(VL53L0X_DEV lidar, UART_HandleTypeDef* huart)
{
	uint8_t data1[3] = {0};
	uint16_t data2 = 0;
	uint16_t data3 = 0;
	uint8_t message[200] = {0};
	uint16_t mess_len;

	// Check the reference registers
	VL53L0X_ReadMulti(lidar, LIDAR_REF1, data1, 3);
	VL53L0X_RdWord(lidar, LIDAR_REF4, &data2);
	VL53L0X_RdWord(lidar, LIDAR_REF5, &data3);

	mess_len = (uint16_t) sprintf((char*) message,
				"Ref1 (0xC0): 0x%02X\r\nRef2 (0xC1): 0x%02X\r\nRef3 (0xC2): 0x%02X\r\nRef4 (0x51): 0x%04X\r\nRef5 (0x61): 0x%04X\r\n\r\n",
				data1[0], data1[1],	data1[2], data2, data3);

	HAL_UART_Transmit(&huart1, message, mess_len, 1000);
}
void init_lidar(VL53L0X_DEV lidar, I2C_HandleTypeDef* i2c)
{
	lidar->I2cDevAddr = LIDAR_ADDR;
	lidar->comms_speed_khz = 100;
	lidar->comms_type = 1; // i2c
	lidar->i2c = i2c;
}

VL53L0X_Error setup_lidar_single(VL53L0X_DEV lidar)
{
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	VL53L0X_DeviceInfo_t lidar1_info = {0};

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_DataInit(lidar);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_GetDeviceInfo(lidar, &lidar1_info);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_StaticInit(lidar);
	if (Status == VL53L0X_ERROR_NONE)
	// from https://github.com/STMicroelectronics/STM32CubeL4/blob/master/Projects/B-L475E-IOT01A/Applications/Proximity/Src/vl53l0x/vl53l0x_tof.c
	  Status = VL53L0X_PerformRefCalibration(lidar, &VhvSettings, &PhaseCal);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_PerformRefSpadManagement(lidar, &refSpadCount, &isApertureSpads);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetDeviceMode(lidar, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetLimitCheckEnable(lidar, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetLimitCheckEnable(lidar, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetLimitCheckValue(lidar, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetLimitCheckValue(lidar, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(lidar, 33000);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetVcselPulsePeriod(lidar, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	if (Status == VL53L0X_ERROR_NONE)
	  Status = VL53L0X_SetVcselPulsePeriod(lidar, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	return Status;
}

VL53L0X_Error setup_lidar_continuous(VL53L0X_DEV lidar)
{
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	VL53L0X_DeviceInfo_t lidar_info = {0};

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_DataInit(lidar);
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_GetDeviceInfo(lidar, &lidar_info);
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_StaticInit(lidar);
	if (Status == VL53L0X_ERROR_NONE)
	// from https://github.com/STMicroelectronics/STM32CubeL4/blob/master/Projects/B-L475E-IOT01A/Applications/Proximity/Src/vl53l0x/vl53l0x_tof.c
		Status = VL53L0X_PerformRefCalibration(lidar, &VhvSettings, &PhaseCal);
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_PerformRefSpadManagement(lidar, &refSpadCount, &isApertureSpads);
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetDeviceMode(lidar, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_StartMeasurement(lidar);
}
