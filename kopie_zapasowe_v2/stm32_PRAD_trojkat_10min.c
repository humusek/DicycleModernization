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
#define TIMER_FREQ_HZ 1000

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
//                IMPORTANT!!!
// 								 MOTOR 1 = RIGHT
// 								 MOTOR 2 = LEFT
//----------------------------------------------------------------

//Motor 1 params
float motor1Current;
int32_t motor1CurrentValue;
float motor1DisablingValue;
//Real values read from vescs
float motor1RealCurrent;
float inputRealCurrentSamples[21];
int32_t motor1RealDutyCycle;

//Motor 2 params
float motor2Current;
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

uint8_t btnAValue;
uint8_t btnBValue;
uint8_t btnXValue;
uint8_t btnYValue;

int throttleActive, brakeActive = 0;

// ZMIANA: Ustawiono na 3.0 zgodnie z wzorcem sygnału rampy
float MAX_CURRENT = 3.0;

// Automation variables
int automationActive = 0;
uint32_t automationCounter = 0;

//Arrays for Moving Average Filter (MAF)
#define SAMPLES_COUNT 21
uint16_t inputThrottleSamples[SAMPLES_COUNT];
uint16_t inputAxisSamples[SAMPLES_COUNT];
uint16_t inputBrakeSamples[SAMPLES_COUNT];
uint16_t VESC1_ID = 0x0000000A;
uint16_t VESC2_ID = 0x0000000B;

uint32_t COMMAND_ID = 0x01 << 8;
//UART CODE
char uartData[32];
uint16_t indx;

// UART DMA Flag
// Dodaj flagę, aby nie nadpisywać bufora w trakcie wysyłania
volatile uint8_t uartTxBusy = 0;

//Encoders
uint16_t Encoder1Counter;
float Velocity1;
uint16_t Encoder2Counter;
float Velocity2;

//Logging data
char logString[70];
volatile uint16_t frameNumber = 0;

//Measurements
int32_t INTmotor1RealCurrent;
int32_t INTmotor2RealCurrent;
int32_t INTmotor1Current;
int32_t INTmotor2Current;
int32_t INTangleX;
int32_t INTVelocity1;
int32_t INTVelocity2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct __attribute__((packed)) {
	uint16_t header;
	int32_t val_m1RealCurrent;
	int32_t val_m2RealCurrent;
	int32_t val_m1Current;
	int32_t val_m2Current;
	int32_t val_angleX;
	int32_t val_Velocity1;
	int32_t val_Velocity2;
} SerialDataPacket;

SerialDataPacket txPacket;
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
	canfilterconfig.FilterBank = 18;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0x000;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 20;

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}

// Callback wywoływany, gdy DMA zakończy wysyłanie
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        uartTxBusy = 0; // Transmisja zakończona, można wysyłać ponownie
    }
}

void LogData() {
    // Jeśli poprzednia transmisja jeszcze trwa, pomiń tę ramkę (lepiej zgubić ramkę niż zablokować silniki)
    if (uartTxBusy == 1) {
        return;
    }

    txPacket.header = 0xABCD;
    txPacket.val_m1RealCurrent = INTmotor1RealCurrent;
    txPacket.val_m2RealCurrent = INTmotor2RealCurrent;
    txPacket.val_m1Current = INTmotor1Current;
    txPacket.val_m2Current = INTmotor2Current;

    // UWAGA: Tu wciąż wysyłasz X zamiast Y, jeśli chcesz Y zmień na INTangleY (po dodaniu zmiennej)
    txPacket.val_angleX = INTangleX;

    txPacket.val_Velocity1 = INTVelocity1;
    txPacket.val_Velocity2 = INTVelocity2;

    uartTxBusy = 1;
    // Używamy wersji DMA - nie blokuje procesora!
    if(HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&txPacket, sizeof(SerialDataPacket)) != HAL_OK) {
        uartTxBusy = 0; // Reset flagi w razie błędu
    }
}


void getVelocity(){
	Encoder1Counter = htim3.Instance->CNT;
	Velocity1 = ((int16_t) Encoder1Counter / 1600.0) * 50.0 * 2 * PI * -1.0;
	Encoder2Counter = htim4.Instance->CNT;
	Velocity2 = ((int16_t) Encoder2Counter / 1600.0) * 50.0 * 2 * PI;

	htim3.Instance->CNT = 0;
	htim4.Instance->CNT = 0;

	INTVelocity1=Velocity1*1000;
	INTVelocity2=Velocity2*1000;
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
		INTmotor1RealCurrent=motor1RealCurrent*1000;
		motor1RealDutyCycle = (RxData[6]<<8) |
		RxData[7];
		if((int16_t)motor1RealDutyCycle < 0){
			motor1RealCurrent *= -1;
			INTmotor1RealCurrent=motor1RealCurrent*1000;
		}
	}
	else if (RxHeader.ExtId == (VESC2_ID | 0x00000900)) {
		motor2RealCurrent =   (float)(((int16_t)(RxData[4]<<8) | RxData[5]) / 10.0);
		INTmotor2RealCurrent=motor2RealCurrent*1000;
		motor2RealDutyCycle = (RxData[6]<<8) | RxData[7];
		if((int16_t)motor2RealDutyCycle < 0){
			motor2RealCurrent *= -1;
			INTmotor2RealCurrent=motor2RealCurrent*1000;
		}
	}
}


int valueIndex = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	// -------------------------------------------------
	// TIM7: CONTROL LOOP (1000 Hz)
	// -------------------------------------------------
	if (htim == &htim7) {
		motor1DisablingValue = 0.0;
		motor2DisablingValue = 0.0;

		if(btnXValue == 1) {
			automationActive = 1;
			automationCounter = 0; // ZMIANA: Odkomentowane, żeby resetować sekwencję
		}

		if(btnBValue == 1) {
			automationActive = 0;
			automationCounter = 0;
			// Safety Stop
			motor1Current = 0.0;
			motor2Current = 0.0;
			INTmotor1Current = 0;
			INTmotor2Current = 0;
		}

		// 3. Logika skręcania
		if (axisXValue > 0) {
			motor1DisablingValue = absoluteAxisValue / 511.0;
		} else if (axisXValue < 0) {
			motor2DisablingValue = absoluteAxisValue / 512.0;
		}

		// 4. Glowna logika sterowania (AUTOMAT vs MANUAL)
		if (automationActive) {
			automationCounter++;

			// ZMIANA: Implementacja logiki RAMP (trójkąt)
			uint32_t ticksPhase = 150 * TIMER_FREQ_HZ;
			float currentRampValue = 0.0;

			if (automationCounter <= ticksPhase) {
				currentRampValue = (float)automationCounter / (float)ticksPhase;
			}
			else if (automationCounter <= (2 * ticksPhase)) {
				uint32_t relativeTime = automationCounter - ticksPhase;
				currentRampValue = 1.0f - ((float)relativeTime / (float)ticksPhase);
			}
			else if (automationCounter <= (3 * ticksPhase)) {
				uint32_t relativeTime = automationCounter - (2 * ticksPhase);
				currentRampValue = 0.0f - ((float)relativeTime / (float)ticksPhase);
			}
			else if (automationCounter <= (4 * ticksPhase)) {
				uint32_t relativeTime = automationCounter - (3 * ticksPhase);
				currentRampValue = -1.0f + ((float)relativeTime / (float)ticksPhase);
			}
			else {
				automationActive = 0;
				automationCounter = 0;
				currentRampValue = 0.0;
			}

			motor1Current = currentRampValue * MAX_CURRENT;
			motor2Current = currentRampValue * MAX_CURRENT;

			INTmotor1Current = motor1Current * 1000;
			INTmotor2Current = motor2Current * 1000;

			throttleActive = 0;
			brakeActive = 0;

		} else {
			// MANUAL MODE (Standard Throttle/Brake)
			if (throttleValue > 0 && brakeActive == 0) {
				float throttleFraction = throttleValue / 1023.0;
				motor1Current = (throttleFraction * MAX_CURRENT * (1 - motor1DisablingValue));
				motor2Current = (throttleFraction * MAX_CURRENT * (1 - motor2DisablingValue));

				INTmotor1Current=motor1Current*1000;
				INTmotor2Current=motor2Current*1000;
				throttleActive = 1;
			} else if(throttleValue <= 0 && throttleActive == 1) {
				motor1Current = 0;
				motor2Current = 0;
				INTmotor1Current=motor1Current*1000;
				INTmotor2Current=motor2Current*1000;
				throttleActive = 0;
			}

			if (brakeValue > 0 && throttleActive == 0) {
				float brakeFraction = brakeValue / 1023.0;
				motor1Current = (-brakeFraction * MAX_CURRENT * (1 - motor1DisablingValue));
				motor2Current = (-brakeFraction * MAX_CURRENT * (1 - motor2DisablingValue));

				INTmotor1Current=motor1Current*1000;
				INTmotor2Current=motor2Current*1000;
				brakeActive = 1;
			} else if(brakeValue <= 0 && brakeActive == 1) {
				motor1Current = 0;
				motor2Current = 0;
				INTmotor1Current=motor1Current*1000;
				INTmotor2Current=motor2Current*1000;
				brakeActive = 0;
			}
		}

		// 5. Zabezpieczenia prądowe
		if(motor1Current > MAX_CURRENT) motor1Current = MAX_CURRENT;
		if(motor1Current < -MAX_CURRENT) motor1Current = -MAX_CURRENT;
		if(motor2Current > MAX_CURRENT) motor2Current = MAX_CURRENT;
		if(motor2Current < -MAX_CURRENT) motor2Current = -MAX_CURRENT;

		// 6. Wysyłanie CAN (SZYBKO)
		motor1CurrentValue = (int32_t)(motor1Current * 1000.0);
		motor2CurrentValue = (int32_t)(motor2Current * 1000.0);
		tx1Data[0] = (motor1CurrentValue >> 24) & 0xFF;
		tx1Data[1] = (motor1CurrentValue >> 16) & 0xFF;
		tx1Data[2] = (motor1CurrentValue >> 8) & 0xFF;
		tx1Data[3] = motor1CurrentValue & 0xFF;

		tx2Data[0] = (motor2CurrentValue >> 24) & 0xFF;
		tx2Data[1] = (motor2CurrentValue >> 16) & 0xFF;
		tx2Data[2] = (motor2CurrentValue >> 8) & 0xFF;
		tx2Data[3] = motor2CurrentValue & 0xFF;
		HAL_CAN_AddTxMessage(&hcan1, &Tx1Header, tx1Data, &Tx1Mailbox);
		HAL_CAN_AddTxMessage(&hcan1, &Tx2Header, tx2Data, &Tx2Mailbox);
	}

	// -------------------------------------------------
	// TIM1: LOGGING LOOP (50 Hz)
	// -------------------------------------------------
	if (htim == &htim1) {
		MPU6050_Read_Accel(&Ax, &Ay, &Az);
		MPU6050_ReadKalmanAngles(&angleX, &angleY);
		INTangleX = angleX * 1000;

		absoluteAxisValue = abs(axisXValue);

		getVelocity();
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
	if (huart->Instance == USART2) {
		indx = Size;

		// Sprawdzamy pierwszy separator
		if(uartData[4] == ':'){
			static char value[4];
			strncpy(value, uartData, 4);
			axisXValue = atoi(value);

			strncpy(value, uartData + 5, 4);
			throttleValue = atoi(value);

			strncpy(value, uartData + 10, 4);
			brakeValue = atoi(value);

			if(uartData[15] == '1') btnAValue = 1; else btnAValue = 0;
			if(uartData[17] == '1') btnBValue = 1; else btnBValue = 0;
			if(uartData[19] == '1') btnXValue = 1; else btnXValue = 0;
			if(uartData[21] == '1') btnYValue = 1; else btnYValue = 0;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 32);
	}
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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan1);
	CANStart();

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 32);

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){

	}

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
  * where the assert_param error has occurred.
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
