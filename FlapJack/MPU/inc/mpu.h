#include <stdint.h>
#include "stm32f4xx_hal.h"


I2C_HandleTypeDef hi2c1;


#define MPU_DEV 0x68 // Device Address
#define MPU_PW1 0x6B // Device Power Management Register 1
#define MPU_PW2 0x6C // Device Power Management Register 2
#define MPU_WHO 0x75 // Device Whoami register

#define MPU_GSR 0x19 // Gyro Sample Rate Divider Register
#define MPU_GCF 0x1B // Gyro Config Register

#define MPU_ACF 0x1C // Accelerometer Configure Register

#define MPU_DAT 0x3B // Accelerometer X sensor High Byte Starting data address
#define MPU_AXL 0x3C // Accelerometer X sensor Low  Byte

#define MPU_AYH 0x3D
#define MPU_AYL 0x3E

#define MPU_AZH 0x3F
#define MPU_AZL 0x40


void mpu_whoami(uint16_t dev_addr, uint8_t *data);

int mpu_set_gyro_sample_rate(uint16_t dev_addr, uint8_t *div);
int mpu_get_gyro_sample_rate(uint16_t dev_addr, uint8_t *data);
int mpu_set_gyro_config(uint16_t dev_addr, uint8_t *config);

int mpu_get_gyro_config(uint16_t dev_addr, uint8_t *config);
int mpu_set_acel_config(uint16_t dev_addr, uint8_t *config);
int mpu_get_acel_config(uint16_t dev_addr, uint8_t *config);

int mpu_get_data(uint16_t dev_addr, int16_t *x, int16_t *y, int16_t *z, int16_t *t, int16_t *a, int16_t *b, int16_t *g);
int mpu_pwr_mgmt(uint16_t dev_addr, uint8_t config1, uint8_t config2);
