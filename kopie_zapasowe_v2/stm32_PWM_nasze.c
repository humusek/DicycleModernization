/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "math.h"
#include "stdbool.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.14159265359
#define SMOOTHING_FACTOR 0.2f //HUM FILTR

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//CAN
CAN_TxHeaderTypeDef Tx1Header;
CAN_TxHeaderTypeDef Tx2Header;

uint8_t tx1Data[4];
uint8_t tx2Data[4];
uint32_t Tx1Mailbox;
uint32_t Tx2Mailbox;

CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];

//MPU6050
float angleX, angleY, angleZ;
float Ax, Ay, Az;

//----------------------------------------------------------------
//                IMPORTANT!!! - MOTOR 1 = RIGHT
// 								 MOTOR 2 = LEFT
//----------------------------------------------------------------


float motorCurrentPrimaryStep=0.005; //HUM same as currentstep
float motorCurrentExtendedStep=0.010; //HUM 2xCurrentStep

//Motor 1 params
float motor1Current;
float motor1CurrentStep=0.005;
float motor1CurrentTarget;
int32_t motor1CurrentValue;
float motor1DisablingValue;
//Real values read from vescs
float motor1RealCurrent;
float inputRealCurrentSamples[21];
int32_t motor1RealDutyCycle;

//Motor 2 params
float motor2Current;
float motor2CurrentStep=0.005;
float motor2CurrentTarget;
int32_t motor2CurrentValue;
float motor2DisablingValue;
//Real values
float motor2RealCurrent;
int32_t motor2RealDutyCycle;

int16_t absoluteAxisValue;

//Global variables for control
int16_t axisXValue;
uint16_t throttleValue;
uint16_t brakeValue;
bool buttonAValue = false;
bool buttonBValue = false;
bool buttonXValue = false;
bool buttonYValue = false;
bool Xpressed = false;
float motorCurrentSteps[] = {0.001, 0.003, 0.005, 0.01};
uint8_t stepCount = sizeof(motorCurrentSteps) / sizeof(motorCurrentSteps[0]);
uint8_t stepIndex = 0; //index to motorStep

int throttleActive, brakeActive = 0;
float MAX_CURRENT = 1.0;

//Arrays for Moving Average Filter (MAF)
uint16_t VESC1_ID = 0x0000000A;
uint16_t VESC2_ID = 0x0000000B;
uint32_t COMMAND_ID = 0x00 << 8;

//UART CODE
char uartData[30];
uint16_t indx;

//Encoders
uint16_t Encoder1Counter;
float Velocity1;
uint16_t Encoder2Counter;
float Velocity2;

uint16_t Encoder1; //HUM zmienna pomocnicza
//Logging data
char logString[70];
volatile uint16_t frameNumber = 0;

// Zmienne dla filtru wygładzającego (EMA)
static float filteredAxis = 0.0f;
static float filteredThrottle = 0.0f;
static float filteredBrake = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	int32_t frNum;
	int32_t m1C;
	int32_t m2C;
	int32_t ax;
	int32_t ay;
	int32_t angX;
	int32_t velo1;
	int32_t velo2;
}LogFrame;

typedef struct {
	int32_t velo1;
	int32_t velo2;
	int32_t duty1;
	int32_t duty2;
}kPhiMeasureFrame;

typedef struct {
	int32_t frNum;
	int32_t m1C;
	int32_t m2C;
	int32_t velo1;
	int32_t velo2;
	int32_t accX;
	int32_t accY;
	int32_t angX;
}staticFrictionMeasureFrame;

LogFrame logFr = {0,0,0,0,0,0,0,0};
kPhiMeasureFrame kPhiMeasFr = {0,0,0,0};
staticFrictionMeasureFrame testingFrame = {0,0,0,0,0,0,0};

void CANStart() {
	Tx1Header.DLC = 4;
	Tx1Header.IDE = CAN_ID_EXT;
	Tx1Header.RTR = CAN_RTR_DATA;
	Tx1Header.ExtId = VESC1_ID | COMMAND_ID;

	Tx2Header.DLC = 4;
	Tx2Header.IDE = CAN_ID_EXT;
	Tx2Header.RTR = CAN_RTR_DATA;
	Tx2Header.ExtId = VESC2_ID | COMMAND_ID;

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0x000;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}

void LogData() {
	kPhiMeasFr.velo1 = (int32_t)(Velocity1 * 1000);
	kPhiMeasFr.velo2 = (int32_t)(Velocity2 * 1000); //do zastanowienia sie
	kPhiMeasFr.duty1 = motor1RealDutyCycle;
	kPhiMeasFr.duty2 = motor2RealDutyCycle;

	testingFrame.frNum = frameNumber; // <- ZMIANA
	testingFrame.m1C = (int32_t)(motor1Current * 1000); // <- ZMIANA
	testingFrame.m2C = (int32_t)(motor2Current * 1000); // <- ZMIANA
	testingFrame.velo1 = (int32_t)(Velocity1 * 1000);
	testingFrame.velo2 = (int32_t)(Velocity2 * 1000);
	testingFrame.accX = (int32_t)(Ax * 1000);
	testingFrame.accY = (int32_t)(Ay * 1000);
	testingFrame.angX = (int32_t)(angleX * 1000);

	HAL_UART_Transmit_IT(&huart3, &testingFrame, sizeof(staticFrictionMeasureFrame));
}


void getVelocity(){
	//Velocity of MOTOR 1
	Encoder1Counter = htim3.Instance->CNT;
	Velocity1 = ((int16_t) Encoder1Counter / 1600.0) * 50.0 * 2 * PI * -1.0;
	Encoder1=65535-Encoder1Counter;
	//Velocity of MOTOR 2
	Encoder2Counter = htim4.Instance->CNT;
	Velocity2 = -(((int16_t) Encoder2Counter / 1600.0) * 50.0 * 2 * PI * -1.0);

	htim3.Instance->CNT = 0;
	htim4.Instance->CNT = 0;
}

int START_FLAG = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_Btn_Pin) {
		START_FLAG = 1;
	}
}

float filteredRealCurrent;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		Error_Handler();
	}
	if (RxHeader.ExtId == (VESC1_ID | 0x00000900)) {
		motor1RealCurrent =   (float)(((int16_t)(RxData[4]<<8) | RxData[5]) / 10.0);
		motor1RealDutyCycle = (RxData[6]<<8) | RxData[7];
		if((int16_t)motor1RealDutyCycle < 0){
			motor1RealCurrent *= -1;
		}
	}
	else if (RxHeader.ExtId == (VESC2_ID | 0x00000900)) {
		motor2RealCurrent =   (float)(((int16_t)(RxData[4]<<8) | RxData[5]) / 10.0);
		motor2RealDutyCycle = (RxData[6]<<8) | RxData[7];
		if((int16_t)motor2RealDutyCycle < 0){
			motor2RealCurrent *= -1;
		}
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//Timer 1

	if (htim == &htim1) {
		absoluteAxisValue = abs(axisXValue);
		motor1DisablingValue = 0.0;
		motor2DisablingValue = 0.0;

		getVelocity();

		if (axisXValue > 0) {
			motor1DisablingValue = absoluteAxisValue / 511.0;
		} else if (axisXValue < 0) {
			motor2DisablingValue = absoluteAxisValue / 512.0;
		}

		if (buttonXValue && (Velocity1==0 && Velocity2==0)) {
		    if (Xpressed) {
		        stepIndex++;
		        if (stepIndex >= stepCount) stepIndex = 0;

		        motorCurrentPrimaryStep = motorCurrentSteps[stepIndex];
		        Xpressed = 0;
		    }
		}
		else{Xpressed = 1;}
		if(buttonBValue){
			/*
			if(Velocity1 > 2){
				motor1Current = -0.2;
				motor2Current = -0.2;
			}
			else if(Velocity1 < -2){
				motor1Current = 0.2;
				motor2Current = 0.2;
			}
			else if (Velocity1 > -2 && Velocity1 < 2) {
			motor1Current = 0;
			motor2Current = 0;
			motor1CurrentTarget = 0;
			motor2CurrentTarget = 0;
			}
			*/
		if(Velocity1 > 2){
			motor1Current = -0.2;
		}
		else if(Velocity1 < -2){
			motor1Current = 0.2;

		}
		else if (Velocity1 > -2 && Velocity1 < 2) {
		motor1Current = 0;
		motor1CurrentTarget = 0;
		}

		if(Velocity2 > 2){
			motor2Current = -0.2;
		}
		else if(Velocity2 < -2){
			motor2Current = 0.2;
		}
		else if (Velocity2 > -2 && Velocity2 < 2) {
		motor2Current = 0;
		motor2CurrentTarget = 0;
		}
		}
		else{
						if (throttleValue > 0 && brakeActive == 0) { //poprawic zaokraglanie
							            motor1CurrentTarget = (throttleValue / 1023.0 * (1 - motor1DisablingValue));
							            motor2CurrentTarget = (throttleValue / 1023.0 * (1 - motor2DisablingValue));
							            if (motor1CurrentTarget <0.001 && motor1CurrentTarget >-0.001 && motor2CurrentTarget <0.001 && motor2CurrentTarget >-0.001){motor1CurrentTarget=0; motor2CurrentTarget=0;}
							            throttleActive = 1;
							        } else if(throttleValue <= 0 && throttleActive == 1) {
							            throttleActive = 0;
							        }
				        if (fabs(motor1CurrentTarget-motor1Current)<motor1CurrentStep){motor1Current = motor1CurrentTarget;}
				        else if ((motor1CurrentTarget-motor1Current)>0 && fabs(motor1CurrentTarget-motor1Current)>motor1CurrentStep){motor1Current += motor1CurrentStep;}
				        else if ((motor1CurrentTarget-motor1Current)<0 && fabs(motor1CurrentTarget-motor1Current)>motor1CurrentStep){motor1Current -= motor1CurrentStep;}

				        if (fabs(motor2CurrentTarget-motor2Current)<motor2CurrentStep){motor2Current = motor2CurrentTarget;}
				        else if ((motor2CurrentTarget-motor2Current)>0 && fabs(motor2CurrentTarget-motor2Current)>motor2CurrentStep){motor2Current += motor2CurrentStep;}
				        else if ((motor2CurrentTarget-motor2Current)<0 && fabs(motor2CurrentTarget-motor2Current)>motor2CurrentStep){motor2Current -= motor2CurrentStep;}

				        if (brakeValue > 0 && throttleActive == 0) {
				            motor1CurrentTarget = (-brakeValue / 1023.0 * (1 - motor1DisablingValue));
				            motor2CurrentTarget = (-brakeValue / 1023.0 * (1 - motor2DisablingValue));
				            brakeActive = 1;
				        } else if(brakeValue <= 0 && brakeActive == 1) {
				            brakeActive = 0;
				        }
				        if(motor1DisablingValue>0.01 || motor2DisablingValue>0.01){
				            motor1CurrentStep=motorCurrentExtendedStep;
				            motor2CurrentStep=motorCurrentExtendedStep;
				        }
				        else{
				            motor1CurrentStep=motorCurrentPrimaryStep;
				            motor2CurrentStep=motorCurrentPrimaryStep;
				        }

//HUM hybryda
				        if (motor1Current * motor1CurrentTarget < 0 && motor2Current * motor2CurrentTarget < 0)
				        		{
				        			motor1CurrentStep = 0.03;
				        			motor2CurrentStep = 0.03;
				        		}
				        		else if (motor1DisablingValue == 0 && motor2DisablingValue == 0)
				        		{
				        			if((Velocity1-Velocity2) > 5) {
				        				motor1CurrentStep = motorCurrentPrimaryStep;
				        				motor2CurrentStep = motorCurrentPrimaryStep * 3;
				        			}
				        			else if((Velocity1-Velocity2) < -5) {
				        				motor1CurrentStep = motorCurrentPrimaryStep * 3;
				        				motor2CurrentStep = motorCurrentPrimaryStep;
				        			}
				        			else if((Velocity1-Velocity2) > 2 && (Velocity1-Velocity2) < 5) {
				        				motor1CurrentStep = motorCurrentPrimaryStep;
				        				motor2CurrentStep = motorCurrentPrimaryStep * 2;
				        			}
				        			else if((Velocity1-Velocity2) < -2 && (Velocity1-Velocity2) > -5) {
				        				motor1CurrentStep = motorCurrentPrimaryStep * 2;
				        				motor2CurrentStep = motorCurrentPrimaryStep;
				        			}
				        			else {
				        				motor1CurrentStep = motorCurrentPrimaryStep;
				        				motor2CurrentStep = motorCurrentPrimaryStep;
				        			}
				        		}
				        		else
				        		{
				        			motor1CurrentStep = motorCurrentPrimaryStep;
				        			motor2CurrentStep = motorCurrentPrimaryStep;
				        		}
		//This is normalized number. Multiply it by however many amps you want
//		motor1Current *= 20.0;
//		motor2Current *= 20.0;

		//Hard coded protection from overcurrent

		if(motor1Current > MAX_CURRENT){
			motor1Current = MAX_CURRENT;
		}
		if(motor1Current < -MAX_CURRENT){
			motor1Current = -MAX_CURRENT;
		}
		if(motor2Current > MAX_CURRENT){
			motor2Current = MAX_CURRENT;
		}
		if(motor2Current < -MAX_CURRENT){
			motor2Current = -MAX_CURRENT;
		}



		//Mapping values to match CAN frame
		motor1CurrentValue = motor1Current * 100000;
		motor2CurrentValue = motor2Current * 100000;

		//Mapping MOTOR 1 values to 4 bytes
		tx1Data[0] = (motor1CurrentValue >> 24) & 0xFF;
		tx1Data[1] = (motor1CurrentValue >> 16) & 0xFF;
		tx1Data[2] = (motor1CurrentValue >> 8) & 0xFF;
		tx1Data[3] = motor1CurrentValue & 0xFF;

		//Mapping MOTOR 2 values to 4 bytes
		tx2Data[0] = (motor2CurrentValue >> 24) & 0xFF;
		tx2Data[1] = (motor2CurrentValue >> 16) & 0xFF;
		tx2Data[2] = (motor2CurrentValue >> 8) & 0xFF;
		tx2Data[3] = motor2CurrentValue & 0xFF;

		HAL_CAN_AddTxMessage(&hcan1, &Tx1Header, tx1Data, &Tx1Mailbox);
		HAL_CAN_AddTxMessage(&hcan1, &Tx2Header, tx2Data, &Tx2Mailbox);

		//Read MPU data and log all data
		MPU6050_Read_Accel(&Ax, &Ay, &Az);
		MPU6050_ReadKalmanAngles(&angleX, &angleY);

		LogData();

		if(motor1Current > 0){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}

		frameNumber++;

	}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	indx = Size;

    // Nowy format: XXXX:YYYY:ZZZZ:A:B:X:Y;
    // Sprawdzamy pozycje wszystkich sześciu dwukropków
    if(uartData[4] == ':' && uartData[9] == ':' && uartData[14] == ':' &&
       uartData[16] == ':' && uartData[18] == ':' && uartData[20] == ':'){

		// Poprawiony bufor: 5 znaków (4 cyfry + znak null '\0')
		static char value[5];
		int32_t temp_val;
		float raw_val;

		// --- Oś X --- (pozostaje bez zmian)
		strncpy(value, uartData, 4);
		value[4] = '\0'; // Ręczne dodanie terminatora null
		temp_val = atoi(value);

		// Ograniczenie wartości surowej
		if (temp_val < -512) raw_val = -512.0f;
		else if (temp_val > 511) raw_val = 511.0f;
		else raw_val = (float)temp_val;

		// Zastosuj filtr EMA
		filteredAxis = (SMOOTHING_FACTOR * raw_val) + ((1.0f - SMOOTHING_FACTOR) * filteredAxis);
		// Zaktualizuj ostateczną wartość (zaokrągloną)
		axisXValue = (int16_t)filteredAxis; // Proste rzutowanie jest szybsze niż zaokrąglanie


		// --- Wartość gazu --- (pozostaje bez zmian)
		strncpy(value, uartData + 5, 4);
		value[4] = '\0'; // Ręczne dodanie terminatora null
		temp_val = atoi(value);

		// Ograniczenie wartości surowej
		if (temp_val < 0) raw_val = 0.0f;
		else if (temp_val > 1023) raw_val = 1023.0f;
		else raw_val = (float)temp_val;

		// Zastosuj filtr EMA
		filteredThrottle = (SMOOTHING_FACTOR * raw_val) + ((1.0f - SMOOTHING_FACTOR) * filteredThrottle);
		// Zaktualizuj ostateczną wartość
		throttleValue = (uint16_t)filteredThrottle;


		// --- Wartość hamulca --- (pozostaje bez zmian)
		strncpy(value, uartData + 10, 4);
		value[4] = '\0'; // Ręczne dodanie terminatora null
		temp_val = atoi(value);

		// Ograniczenie wartości surowej
		if (temp_val < 0) raw_val = 0.0f;
		else if (temp_val > 1023) raw_val = 1023.0f;
		else raw_val = (float)temp_val;

		// Zastosuj filtr EMA
		filteredBrake = (SMOOTHING_FACTOR * raw_val) + ((1.0f - SMOOTHING_FACTOR) * filteredBrake);
		// Zaktualizuj ostateczną wartość
		brakeValue = (uint16_t)filteredBrake;


		// --- PARSOWANIE PRZYCISKÓW ---

		// --- Przycisk A (pozycja 15) ---
		if (uartData[15] == '1') {
			buttonAValue = true;
		} else {
			buttonAValue = false;
		}

		// --- Przycisk B (nowa pozycja 17) ---
		if (uartData[17] == '1') {
			buttonBValue = true;
		} else {
			buttonBValue = false;
		}

		// --- Przycisk X (nowa pozycja 19) ---
		if (uartData[19] == '1') {
			buttonXValue = true;
		} else {
			buttonXValue = false;
		}

		// --- Przycisk Y (nowa pozycja 21) ---
		if (uartData[21] == '1') {
			buttonYValue = true;
		} else {
			buttonYValue = false;
		}

	}

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 30);
}

//Activating certain channel of ADC
void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = AdcChannel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan1);
	CANStart();
	//Timers
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 30);

	//CAN Recieve
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
		//Error_Handler();
	}

	//char buf[4];
	MPU6050_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
	while (1) {
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
