#include "../../vl53l0x_def.h"
#include "../../vl53l0x_i2c_platform.h"
#include "stm32f4xx_hal_def.h"
#include "string.h"


#define MY_I2C_TIMEOUT HAL_MAX_DELAY // probably change delay

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata,
                        uint32_t count, I2C_HandleTypeDef *i2c)
{
	uint8_t buff[65]; // 1 more than VL53L0X_MAX_I2C_XFER_SIZE for address
	buff[0] = index; // write peripheral address first
	memcpy(buff + 1, pdata, count); // copy rest of data
	return HAL_I2C_Master_Transmit(i2c, deviceAddress, buff, count + 1, HAL_MAX_DELAY);
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata,
                       uint32_t count, I2C_HandleTypeDef *i2c)
{
	int32_t status;
	// write index to read from to peripheral
	status = HAL_I2C_Master_Transmit(i2c, deviceAddress, &index, 1, MY_I2C_TIMEOUT);
	if (status != HAL_OK) {
		return status;
	}
	else
	{
		// send device address with read bit set, then receive data
		status = HAL_I2C_Master_Receive(i2c, deviceAddress | 1, pdata, count, MY_I2C_TIMEOUT);
		return status;
	}
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data,
					   I2C_HandleTypeDef *i2c)
{
	uint8_t buff[2];
	buff[0] = index;
	buff[1] = data;
	return HAL_I2C_Master_Transmit(i2c, deviceAddress, buff, 2, MY_I2C_TIMEOUT);
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data,
		               I2C_HandleTypeDef *i2c) {
	uint8_t buff[3];
	buff[0] = index;
	buff[1] = (data & 0xFF00) >> 8;
	buff[2] = (data & 0x00FF);
	return HAL_I2C_Master_Transmit(i2c, deviceAddress, buff, 3, MY_I2C_TIMEOUT);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data,
					    I2C_HandleTypeDef *i2c) {
	uint8_t buff[5];

	buff[0] = index;
	buff[1] = (data & 0xFF000000) >> 24;
	buff[2] = (data & 0x00FF0000) >> 16;
	buff[3] = (data & 0x0000FF00) >> 8;
	buff[4] = (data & 0x000000FF);

	return HAL_I2C_Master_Transmit(i2c, deviceAddress, buff, 5, MY_I2C_TIMEOUT);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data,
					  I2C_HandleTypeDef *i2c) {
	int32_t status;
	// write index to read from to peripheral
	status = HAL_I2C_Master_Transmit(i2c, deviceAddress, &index, 1, MY_I2C_TIMEOUT);
	if (status != HAL_OK) {
		return status;
	}
	else
	{
		// send device address with read bit set, then receive data
		status = HAL_I2C_Master_Receive(i2c, deviceAddress | 1, data, 1, MY_I2C_TIMEOUT);
		return status;
	}
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data,
					  I2C_HandleTypeDef *i2c) {
	uint16_t word;
	uint8_t buff[2];
	int32_t status;
	// write index to read from to peripheral
	status = HAL_I2C_Master_Transmit(i2c, deviceAddress, &index, 1, MY_I2C_TIMEOUT);
	if (status != HAL_OK) {
		return status;
	}
	// send device address with read bit set, then receive data
	status = HAL_I2C_Master_Receive(i2c, deviceAddress | 1, buff, 2, MY_I2C_TIMEOUT);
	if (status != HAL_OK) {
		return status;
	}
	word = buff[0];
	word <<= 8;
	word |= buff[1];
	*data = word;
	return status;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data,
					   I2C_HandleTypeDef *i2c) {
	uint32_t dword;
	uint8_t buff[4];
	int32_t status;
	// write index to read from to peripheral
	status = HAL_I2C_Master_Transmit(i2c, deviceAddress, &index, 1, MY_I2C_TIMEOUT);
	if (status != HAL_OK) {
		return status;
	}
	// send device address with read bit set, then receive data
	status = HAL_I2C_Master_Receive(i2c, deviceAddress | 1, buff, 2, MY_I2C_TIMEOUT);
	if (status != HAL_OK) {
		return status;
	}
	dword = buff[0];
	dword <<= 8;
	dword |= buff[1];
	dword <<= 8;
	dword |= buff[2];
	dword <<= 8;
	dword |= buff[3];
	*data = dword;
	return status;
}
