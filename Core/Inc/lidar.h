/*
 * lidar.h
 *
 *  Created on: Aug 9, 2022
 *      Author: jackson
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#define LIDAR_ADDR (0b0101001 << 1)
#define LIDAR_REF1 (0xC0)
#define LIDAR_REF2 (0xC1)
#define LIDAR_REF3 (0xC2)
#define LIDAR_REF4 (0x51)
#define LIDAR_REF5 (0x61)

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

extern UART_HandleTypeDef huart1;

void check_ref_reg(VL53L0X_DEV lidar, UART_HandleTypeDef* huart);
void init_lidar(VL53L0X_DEV lidar, I2C_HandleTypeDef* i2c);
VL53L0X_Error setup_lidar_single(VL53L0X_DEV lidar);
VL53L0X_Error setup_lidar_continuous(VL53L0X_DEV lidar);

#endif /* INC_LIDAR_H_ */
