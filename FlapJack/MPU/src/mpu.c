#include "mpu.h"


void mpu_whoami(uint16_t dev_addr, uint8_t *data){
	// Accel-Gyro who am i, check the whoami reg to verify we are the device
	HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_WHO, 1, data, 1, HAL_MAX_DELAY);
}

int mpu_set_gyro_sample_rate(uint16_t dev_addr, uint8_t *div){
	// divide the Gyro sample rate
	return HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_GSR, 1, div, 1, HAL_MAX_DELAY);
}

int mpu_get_gyro_sample_rate(uint16_t dev_addr, uint8_t *data){
	// divide the Gyro sample rate
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_GSR, 1, data, 1, HAL_MAX_DELAY);
}

int mpu_set_gyro_config(uint16_t dev_addr, uint8_t *config){
	// Set the Gyro Full Scale Range,
	// Conduct Self Test bool based on
	// bit 7:x Axis Gyro
	// bit 6:y Axis Gyro
	// bit 5:z Axis Gyro
	// Self test ensures that the device provided a meaning full output within
	// expected factory range, requires additional setup to calculate FS, see Registermap
	// for more info

	// Set scale on bits 4 and bits 3
	// 0:250, 1:500, 2:1000, 3:2000 [deg/s] See data sheet for more info
	// Bits 2-0 Don't matter
	return HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_GCF, 1, config, 1, HAL_MAX_DELAY);
}

int mpu_get_gyro_config(uint16_t dev_addr, uint8_t *config){
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_GCF, 1, config, 1, HAL_MAX_DELAY);
}

int mpu_set_acel_config(uint16_t dev_addr, uint8_t *config){
	// Set the Acel Full Scale Range,
	// Conduct Self Test bool based on
	// bit 7:x Axis
	// bit 6:y Axis
	// bit 5:z Axis
	// Self test ensures that the device provided a meaning full output within
	// expected factory range, requires additional setup to calculate FS, see Registermap
	// for more info

	// Set scale on bits 4 and bits 3
	// 0:2g, 1:4g, 2:8g, 3:16g[m/(s^2)] See data sheet for more info
	// Bits 2-0 Don't matter
	return HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_ACF, 1, config, 1, HAL_MAX_DELAY);
}

int mpu_get_acel_config(uint16_t dev_addr, uint8_t *config){
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_ACF, 1, config, 1, HAL_MAX_DELAY);
}


int mpu_get_data(uint16_t dev_addr, int16_t *x, int16_t *y, int16_t *z, int16_t *t, int16_t *a, int16_t *b, int16_t *g){
	uint8_t status = 0;
	uint8_t d[14] = {0} ;
	status = HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_DAT, 1, d, 14, HAL_MAX_DELAY);
	*x = (uint16_t)d[0]<<8 | (uint16_t)d[1];
	*y = (uint16_t)d[2]<<8 | (uint16_t)d[3];
	*z = (uint16_t)d[4]<<8 | (uint16_t)d[5];

	*t = (uint16_t)d[6]<<8 | (uint16_t)d[7];

	*a = (uint16_t)d[8]<<8 | (uint16_t)d[9];
	*b = (uint16_t)d[10]<<8 | (uint16_t)d[11];
	*g = (uint16_t)d[12]<<8 | (uint16_t)d[13];

	return status;
}

int mpu_pwr_mgmt(uint16_t dev_addr, uint8_t config1, uint8_t config2){
	// Power Management configuration
	// Register 1
	// Bit7 - set to 1 to reset device
	// Bit6 - set to 1 to enter sleep mode (no updates), by default device is in sleep mode
	// Bit5 - set to 1 while sleep mode 0 to cycle between sleep mode and single measurement (save power, low update)
	// bit4 - na
	// bit3 - set to 1 disable temperature sensor
	// bit2-0 - set clock, just have this set to 0
	// Register 2
	// Just set this byte to zero, or it will put everything in standby mode
	int status = 0;
	status = HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_PW1, 1, &config1, 1, HAL_MAX_DELAY);
	return status | HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_PW2, 1, &config2, 1, HAL_MAX_DELAY);
	return status;
}
