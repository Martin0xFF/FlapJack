/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t msg[1024];

void cdc_printf(const char * fmt, ...){
	// Make a simple function to print to usb
	va_list args;
	va_start(args, fmt);
    vsprintf((char *)msg, fmt, args);
	va_end(args);
	CDC_Transmit_FS(msg, strlen((char *)msg));HAL_Delay(100); // We need to add a little delay or else the databuffer gets overwritten
}

void ag_whoami(uint16_t dev_addr, uint8_t *data){
	// Accel-Gyro who am i, check the whoami reg to verify we are the device
	HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_WHO, 1, data, 1, HAL_MAX_DELAY);
}

int ag_set_gyro_sample_rate(uint16_t dev_addr, uint8_t *div){
	// divide the Gyro sample rate
	return HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_GSR, 1, div, 1, HAL_MAX_DELAY);
}

int ag_get_gyro_sample_rate(uint16_t dev_addr, uint8_t *data){
	// divide the Gyro sample rate
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_GSR, 1, data, 1, HAL_MAX_DELAY);
}

int ag_set_gyro_config(uint16_t dev_addr, uint8_t *config){
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

int ag_get_gyro_config(uint16_t dev_addr, uint8_t *config){
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_GCF, 1, config, 1, HAL_MAX_DELAY);
}

int ag_set_acel_config(uint16_t dev_addr, uint8_t *config){
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

int ag_get_acel_config(uint16_t dev_addr, uint8_t *config){
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, MPU_ACF, 1, config, 1, HAL_MAX_DELAY);
}


int ag_get_data(uint16_t dev_addr, int16_t *x, int16_t *y, int16_t *z, int16_t *t, int16_t *a, int16_t *b, int16_t *g){
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

int ag_pwr_mgmt(uint16_t dev_addr, uint8_t *config1, uint8_t *config2){
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
	status = HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_PW1, 1, config1, 1, HAL_MAX_DELAY);
	return status | HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, MPU_PW2, 1, config2, 1, HAL_MAX_DELAY);
	return status;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  Status = -1;
  uint8_t data;
  uint8_t gyro_scale = 7; // set to 7 so has same sample speed as accelerometer
  uint8_t pwr_config = 0;
  uint8_t accel_config = 0x08;
  int16_t xreading = 0;
  int16_t yreading = 0;
  int16_t zreading = 0;

  int16_t treading = 0;

  int16_t areading = 0;
  int16_t breading = 0;
  int16_t greading = 0;


  ag_pwr_mgmt(MPU_DEV, &pwr_config, &pwr_config);
  ag_set_acel_config(MPU_DEV, &accel_config);
  // Initialize Comms
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while(Status){ // Hold here until You press 'n'
	  }

	ag_get_data(MPU_DEV, &xreading, &yreading, &zreading, &treading, &areading, &breading, &greading);
	cdc_printf("x:%d y:%d z:%d\r\n", xreading, yreading, zreading);//, areading, breading, greading, treading);
	cdc_printf("a:%d b:%d g:%d\r\n", areading, breading, greading);


    Status = -1; // Set the status to -1 so we wait until user issues new read command
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
