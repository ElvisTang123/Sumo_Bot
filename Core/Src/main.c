/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "vl53l0x_api.h"
//#include "vl53l0x_platform.h"

#include "MPU6050.h"
#include "Line_Detect.h"
#include "USART.h"
#include "HCSR04.h"
#include "motor_driver.h"
#include "Remote_Control.h"
#include "lidar.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIDAR_ADDR (0b0101001 << 1)
#define LIDAR_REF1 (0xC0)
#define LIDAR_REF2 (0xC1)
#define LIDAR_REF3 (0xC2)
#define LIDAR_REF4 (0x51)
#define LIDAR_REF5 (0x61)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

MPU6050 IMU_1;
IR_Sensor IR1;
IR_Sensor IR2;
HCSR04 US;
Motor_t  motor_1;
Motor_t  motor_3;
RC_t throttle;
RC_t steering;
uint16_t duty;
uint16_t ADCValue;
VL53L0X_Dev_t lidar1_dev;
VL53L0X_DEV lidar1 = &lidar1_dev;

int8_t state = 0;
int16_t Auto_CNT = 0;
int16_t Auto_Target = 5;
int16_t Man_CNT = 0;
int16_t Man_Target = 5;
uint16_t LongDistTarget = 90;
uint16_t ShortDistTarget = 50;
uint32_t timeout;
uint8_t setLevel = 5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	// MPU6050 struct attribute config
	IMU_1.hi2c = &hi2c1;
	IMU_1.Accel_X = 0;
	IMU_1.Accel_Y = 0;
	IMU_1.Accel_Z = 0;
	IMU_1.Gyro_X = 0;
	IMU_1.Gyro_Y = 0;
	IMU_1.Gyro_Z = 0;

	// IR Sensor struct attribute config
	IR1.DI_GPIO = GPIOA;
	IR1.IDR_Mask = GPIO_IDR_ID11;
	IR1.IDR_Pos = GPIO_IDR_ID11_Pos;

	IR2.DI_GPIO = GPIOC;
	IR2.IDR_Mask = GPIO_IDR_ID13;
	IR2.IDR_Pos = GPIO_IDR_ID13_Pos;

	// HCSR04 struct attribute config
	US.Trig_GPIO = GPIOC;
	US.Trig_Pin = GPIO_PIN_14;
	US.Echo_GPIO = GPIOC;
	US.Echo_IDR_Mask = GPIO_IDR_ID15;
	US.Echo_IDR_Pos = GPIO_IDR_ID15_Pos;

	// Motor struct attribute config
	motor_1.htim = &htim2;
	motor_1.IN1_GPIO = GPIOA;
	motor_1.IN1_pin = TIM_CHANNEL_2;
	motor_1.IN2_GPIO = GPIOA;
	motor_1.IN2_pin = TIM_CHANNEL_3;
	motor_1.EN_GPIO = GPIOA;
	motor_1.EN_pin = GPIO_PIN_4;

	motor_3.htim = &htim3;
	motor_3.IN1_GPIO = GPIOB;
	motor_3.IN1_pin = TIM_CHANNEL_3;
	motor_3.IN2_GPIO = GPIOB;
	motor_3.IN2_pin = TIM_CHANNEL_4;
	motor_3.EN_GPIO = GPIOB;
	motor_3.EN_pin = GPIO_PIN_10;

	// Remote Control struct attribute config
	throttle.htim = &htim4;
	throttle.TIM_CH = TIM_CHANNEL_3;
	throttle.GPIO = GPIOB;
	throttle.IDR_Mask = GPIO_IDR_ID8;
	throttle.IDR_Pos = GPIO_IDR_ID8_Pos;


	steering.htim = &htim4;
	steering.TIM_CH = TIM_CHANNEL_4;
	steering.GPIO = GPIOB;
	steering.IDR_Mask = GPIO_IDR_ID9;
	steering.IDR_Pos = GPIO_IDR_ID9_Pos;

	// ToF Sensor variables
	uint8_t mdr1;
	VL53L0X_RangingMeasurementData_t range_data1;
	uint16_t tof_data;
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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	HAL_ADC_Start(&hadc1);

	MPU6050_Init(&IMU_1);

	HAL_TIM_Base_Start(&htim1);		// Start TIM1 for delay function
	HAL_TIM_Base_Start(&htim4);		// Start TIM4 for external interrupt (PC15) counter reading

	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);	// Capturing RC throttle PWM with TIM4
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);	// Capturing RC steering PWM with TIM4

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	// PWM signals to motor 1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// PWM signals to motor 1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	// PWM signals to motor 3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	// PWM signals to motor 3

	// Set both nSleep Pin and Enable Pin high to enable the motor driver
	enable(&motor_1);
	enable(&motor_3);

	// ToF
	init_lidar(lidar1, &hi2c1); 				// Initialize the device with the i2c handle it is connected to
	check_ref_reg(lidar1, &huart1);				// Check the reference registers, to verify that i2c communication is working
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;	// Initialize the sensor, I reccomend starting to check for errors at this point
	if (Status == VL53L0X_ERROR_NONE)
		Status = setup_lidar_continuous(lidar1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{


		// MPU6050
		// Acceleration
		//		HAL_Delay(1000);
		//		UART_Print_String("-------------------------------------\r\n");
		//		UART_Print_String("MPU6050\r\n");
		//		UART_Print_String("Acceleration\r\n");
		//MPU6050_Read_Accel(&IMU_1);
		//		UART_MPU6050_ACC(IMU_1.Ax,'x');
		//		UART_MPU6050_ACC(IMU_1.Ay,'y');
		//		UART_MPU6050_ACC(IMU_1.Az,'z');
		//		UART_Print_String("\r\n");
		// Angular Velocity
		//		UART_Print_String("Angular Velocity\r\n");
		//MPU6050_Read_Gyro(&IMU_1);
		//		UART_MPU6050_Gyro(IMU_1.Gx,'x');
		//		UART_MPU6050_Gyro(IMU_1.Gy,'y');
		//		UART_MPU6050_Gyro(IMU_1.Gz,'z');
		//		UART_Print_String("-------------------------------------\r\n");

		// Finite State Machine
		switch(state){
		case 0:									// mode select state
			if(steering.usWidth > 1800){
				Auto_CNT++;
				Man_CNT = 0;
				if(Auto_CNT == Auto_Target){
					Auto_CNT = 0;
					state = 5;
				}
				else{
					state = 0;
				}

			}
			else if(steering.usWidth < 1200){
				Man_CNT++;
				Auto_CNT = 0;
				if(Man_CNT == Man_Target){
					Man_CNT = 0;
					state = 4;
				}
				else{
					state = 0;
				}
			}
			else{
				Auto_CNT = 0;
				Man_CNT = 0;
				state = 0;
			}
			break;
		case 1:									//	attack state
			while(throttle.usWidth < 1850){
				set_level(&motor_1, 0);
				set_level(&motor_3, 0);
			}
			// Sensor Read
			// HC-SRO4
			HAL_GPIO_WritePin(US.Trig_GPIO, US.Trig_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			delay_us (10);  // wait for 10 us
			HAL_GPIO_WritePin(US.Trig_GPIO, US.Trig_Pin, GPIO_PIN_RESET);

			// VL53L0X_ToF Sensor
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_GetMeasurementDataReady(lidar1, &mdr1);
			if (mdr1 && Status == VL53L0X_ERROR_NONE)
			{
				Status = VL53L0X_GetRangingMeasurementData(lidar1, &range_data1);
				tof_data = (range_data1.RangeMilliMeter)/10; //converted to cm for simplicity
				UART_Print_Val("Range (cm): %d cm\r\n" , tof_data);
				VL53L0X_ClearInterruptMask(lidar1, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			}

			// Strategy Starts From Here

			UART_Print_Val("US Stuff (cm): %d\r\n cm", US.EXTI_Diff);



			if ((tof_data < ShortDistTarget) || (US.EXTI_Diff < ShortDistTarget)){
				set_level(&motor_1, -2048);
				set_level(&motor_3, 2048);
			}

			else {

				set_level(&motor_1, -2048);
				set_level(&motor_3, -2048);
			}

			state = 2;
			break;

		case 2:									//	Line Detect State
			// Line Detect
			//UART_Print_String("Line Detect 1\r\n");
			LineDetect(&IR1);
			//UART_Print_String("Line Detect 2\r\n");
			LineDetect(&IR2);
			//		UART_Print_String("-------------------------------------\r\n");
			//			set_level(&motor_1, 0);
			//			set_level(&motor_3, 0);
			// Strategy Starts From Here

			if ((IR2.DI_GPIO->IDR & IR2.IDR_Mask) <100){
				//uint32_t start = HAL_GetTick();
				//timeout = HAL_GetTick() + setLevel;
				//UART_Print_Val("time %d \r\n", HAL_GetTick());
				set_level(&motor_1, 2048);
				set_level(&motor_3, -2048);
				HAL_Delay(70);
				set_level(&motor_1, -2048);
				set_level(&motor_3, -2048);
				HAL_Delay(350);


			}

			else if ((IR1.DI_GPIO->IDR & IR1.IDR_Mask)<100){
				set_level(&motor_1, 2048);
				set_level(&motor_3, -2048);
				HAL_Delay(70);
				set_level(&motor_1, 2048);
				set_level(&motor_3, 2048);
				HAL_Delay(350);
			}


			//UART_Print_Val("color %d \r\n", IR1.DI_GPIO->IDR & IR1.IDR_Mask);
			//UART_Print_Val("color %d \r\n", IR2.DI_GPIO->IDR & IR2.IDR_Mask);
			//UART_Print_Val("color %d \r\n", IR1.IDR_Mask);
			//			if(){
			//
			//			}

			state = 3;
			break;
		case 3:
			//Battery Monitoring
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1,1000);
			ADCValue = HAL_ADC_GetValue(&hadc1);
			//UART_Print_Val("ADC Value: %d\r\n", ADCValue);

			HAL_ADC_Stop(&hadc1);
			if (ADCValue < 2880)
			{
				disable(&motor_1);
				disable(&motor_3);
				state = 3;
			}
			else if (ADCValue >= 2880)
			{
				enable(&motor_1);
				enable(&motor_3);
				state = 1;
			}
			break;
		case 4:									// Manual Mode

			UART_Print_Val("Throttle: %d\r\n", throttle.usWidth);
			UART_Print_Val("Steering: %d\r\n", steering.usWidth);

			if(throttle.usWidth > 2000){
				throttle.usWidth = 2000;
			}
			else if(throttle.usWidth < 1000){
				throttle.usWidth = 1000;
			}
			if(steering.usWidth > 2000){
				steering.usWidth = 2000;
			}
			else if(steering.usWidth < 1000){
				steering.usWidth = 1000;
			}

			if (throttle.usWidth>1550){
				duty = (-2048/500)*(throttle.usWidth -1500);
				set_level(&motor_1, -duty);
				set_level(&motor_3, +duty);
			}
			else if(throttle.usWidth<1450){
				duty = (2048/500)*(1500-throttle.usWidth);
				set_level(&motor_1, duty);
				set_level(&motor_3, -duty);
			}
			else if(throttle.usWidth>1450 && throttle.usWidth<1550){
				duty = 0;
				set_level(&motor_1, duty);
				set_level(&motor_3, duty);
			}

			if (steering.usWidth>1550 && throttle.usWidth >1550){
				duty = (2048/500)*(throttle.usWidth-1500);
				set_level(&motor_1, -duty);
				set_level(&motor_3, -duty);
			}
			else if  (steering.usWidth>1550 && throttle.usWidth < 1450){
				duty = (2048/500)*(1500-throttle.usWidth);
				set_level(&motor_1, -duty);
				set_level(&motor_3, -duty);
			}

			else if (steering.usWidth <1450 && throttle.usWidth >1550){
				duty = (2048/500)*(throttle.usWidth-1500);
				set_level(&motor_1, +duty);
				set_level(&motor_3, +duty);
			}
			else if  (steering.usWidth <1450 && throttle.usWidth < 1450){
				duty = (2048/500)*(1500-throttle.usWidth);
				set_level(&motor_1, +duty);
				set_level(&motor_3, +duty);
			}

			break;
		case 5:
			while(throttle.usWidth < 1850){
				set_level(&motor_1, 0);
				set_level(&motor_3, 0);
			}
			set_level(&motor_1, 2048);
			set_level(&motor_3, 2048);
			HAL_Delay(220);
			set_level(&motor_1, 2048);
			set_level(&motor_3, -2048);
			HAL_Delay(300);
			state = 1;
		default:
			UART_Print_String("BAD CODE LOLOLOL\r\n");
		}




		//		HAL_Delay(1000);

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
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	hi2c1.Init.ClockSpeed = 100000;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 95;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2048;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2048;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 95;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, M1_EN_Pin|M2_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, M3_EN_Pin|M4_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Line_Detect2_Pin */
	GPIO_InitStruct.Pin = Line_Detect2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Line_Detect2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : M1_EN_Pin M2_EN_Pin */
	GPIO_InitStruct.Pin = M1_EN_Pin|M2_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : M1_nFAULT_Pin M2_nFAULT_Pin Line_Detect1_Pin */
	GPIO_InitStruct.Pin = M1_nFAULT_Pin|M2_nFAULT_Pin|Line_Detect1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : M3_EN_Pin M4_EN_Pin */
	GPIO_InitStruct.Pin = M3_EN_Pin|M4_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : M3_nFAULT_Pin M4_nFAULT_Pin */
	GPIO_InitStruct.Pin = M3_nFAULT_Pin|M4_nFAULT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	// Throttle
	if(throttle.GPIO->IDR & throttle.IDR_Mask){ //throttle.Edge_Flag == 0
		throttle.IC_Val1 = HAL_TIM_ReadCapturedValue(throttle.htim, throttle.TIM_CH); // read the first value
	}
	else{
		throttle.IC_Val2 = HAL_TIM_ReadCapturedValue(throttle.htim, throttle.TIM_CH);
		getWidth(&throttle);
	}


	// Steering
	if(steering.GPIO->IDR & steering.IDR_Mask){  //steering.Edge_Flag == 0
		steering.IC_Val1 = HAL_TIM_ReadCapturedValue(steering.htim, steering.TIM_CH); // read the first value
	}
	else{
		steering.IC_Val2 = HAL_TIM_ReadCapturedValue(steering.htim, steering.TIM_CH);
		getWidth(&steering);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if((US.Echo_GPIO->IDR & GPIO_IDR_ID15)){
		US.EXTI_Val1 = TIM4->CNT;
	}
	else{
		US.EXTI_Val2 = TIM4->CNT;
		HCSR04_DistCalc(&US);
	}
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
