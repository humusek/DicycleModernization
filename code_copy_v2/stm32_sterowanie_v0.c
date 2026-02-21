/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#define DT 0.001f // Czas próbkowania (1/1000Hz)
#define SMOOTHING_FACTOR 0.2f // Filtr dla ADC

// Zmienne dla filtru wygładzającego (EMA)
static float filteredAxis = 0.0f;
static float filteredThrottle = 0.0f;
static float filteredBrake = 0.0f;
float raw_val=0.0f;

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
// 								 MOTOR 1 = RIGHT (PRAWY)
// 								 MOTOR 2 = LEFT  (LEWY)
//----------------------------------------------------------------

// --- PID CONSTANTS (From User) ---
// MOTOR 1 (RIGHT / PRAWY)
float M1_Kp = 2.7292f;
float M1_Ki = 0.8148f;
float M1_Kd = 1.6846f;

// MOTOR 2 (LEFT / LEWY)
float M2_Kp = 4.3721f;
float M2_Ki = 1.4326f;
float M2_Kd = 0.7763f;

// --- PID STATE VARIABLES ---
float m1_integral = 0.0f;
float m1_prev_error = 0.0f;
float m2_integral = 0.0f;
float m2_prev_error = 0.0f;
float targetVelRaw = 0.0f;

// Target Velocity Settings
float MAX_VELOCITY_RAD_S = 30.0f;

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
//Current limit
float MAX_CURRENT = 7.0; // Limit for Current Mode (Output of PI)
float MAX_DUTY = 0.95;    // Limit for PWM Mode

// --- MODE CONTROL VARIABLES ---
// 1 = PWM (Duty Cycle), 2 = Current (Speed reg), 3 = Position
uint8_t controlMode = 1;
bool btnYLastState = false;
uint32_t ledCounter = 0;
// --- RAMPING VARIABLES FOR MODE 1 (From stm3_newest) ---
float motor1CurrentTarget = 0.0;
float motor2CurrentTarget = 0.0;
float motor1CurrentStep = 0.0002;
float motor2CurrentStep = 0.0002;
float motorCurrentPrimaryStep = 0.0002;
float motorCurrentExtendedStep = 0.0004;

// --- Speed regulator
float error1=0.0f;
float error2=0.0f;
float targetVel1=0.0f;
float targetVel2=0.0f;
float brakeFraction=0.0f;
float P1=0.0f;
float P2=0.0f;
float I1=0.0f;
float I2=0.0f;
float D1=0.0f;
float D2=0.0f;

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
int32_t temp_val;

//Encoders
uint16_t Encoder1Counter;
float Velocity1;
uint16_t Encoder2Counter;
float Velocity2;
//Logging data
char logString[70];
volatile uint16_t frameNumber = 0;

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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Empty callback as USART3 TX is removed
}

void getVelocity(){
	Encoder1Counter = htim3.Instance->CNT;
	Velocity1 = ((int16_t) Encoder1Counter / 1600.0) * 50.0 * 2 * PI * -1.0;
	Encoder2Counter = htim4.Instance->CNT;
	Velocity2 = ((int16_t) Encoder2Counter / 1600.0) * 50.0 * 2 * PI;

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
		motor1RealDutyCycle = (RxData[6]<<8) |
		RxData[7];
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


int valueIndex = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	// -------------------------------------------------
	// TIM7: CONTROL LOOP (1000 Hz)
	// -------------------------------------------------
	if (htim == &htim7) {
		motor1DisablingValue = 0.0;
		motor2DisablingValue = 0.0;

		// --- ZMIANA TRYBÓW (Button Y) ---
		if (btnYValue == 1 && !btnYLastState) {
			if (fabs(Velocity1) < 0.2 && fabs(Velocity2) < 0.2) {
				controlMode++;
				if (controlMode > 3) controlMode = 1;

				// Reset
				motor1Current = 0; motor2Current = 0;
				motor1CurrentTarget = 0; motor2CurrentTarget = 0;
				m1_integral = 0; m2_integral = 0;
				m1_prev_error = 0; m2_prev_error = 0;
				throttleActive = 0; brakeActive = 0;
			}
		}
		btnYLastState = btnYValue;

		// --- SYGNALIZACJA LED TRYBU ---
		ledCounter++;
		uint32_t blinkPeriod;
		if (controlMode == 1) blinkPeriod = 100;
		else if (controlMode == 2) blinkPeriod = 500;
		else blinkPeriod = 1000;

		if (ledCounter >= blinkPeriod) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			ledCounter = 0;
		}

		// --- LOGIKA SKRECANIA ---
		if (axisXValue < 0) {
			motor1DisablingValue = absoluteAxisValue / 511.0;
		} else if (axisXValue > 0) {
			motor2DisablingValue = absoluteAxisValue / 512.0;
		}

		if(btnBValue == 1) {
			// Safety Stop
			motor1Current = 0.0; motor2Current = 0.0;
			motor1CurrentTarget = 0.0; motor2CurrentTarget = 0.0;
			m1_integral = 0; m2_integral = 0;
		}
		else {
			// =========================================================
			// TRYB 1: PWM (Duty Cycle)
			// =========================================================
			if (controlMode == 1) {
				COMMAND_ID = 0x00 << 8;

				if (throttleValue > 0 && brakeActive == 0) {
					motor1CurrentTarget = (throttleValue / 1023.0 * (1 - motor1DisablingValue));
					motor2CurrentTarget = (throttleValue / 1023.0 * (1 - motor2DisablingValue));

					// Deadband
					if (motor1CurrentTarget < 0.001 && motor1CurrentTarget > -0.001) motor1CurrentTarget = 0;
					if (motor2CurrentTarget < 0.001 && motor2CurrentTarget > -0.001) motor2CurrentTarget = 0;
					throttleActive = 1;
				} else if(throttleValue <= 0 && throttleActive == 1) {
					throttleActive = 0;
					motor1CurrentTarget = 0;
					motor2CurrentTarget = 0;
				}

				// --- LOGIKA RAMPY

				// 1. Wybór kroku podstawowego vs rozszerzonego przy skręcaniu
				if(motor1DisablingValue > 0.01 || motor2DisablingValue > 0.01){
				    motor1CurrentStep = motorCurrentExtendedStep;
				    motor2CurrentStep = motorCurrentExtendedStep;
				} else {
				    motor1CurrentStep = motorCurrentPrimaryStep;
				    motor2CurrentStep = motorCurrentPrimaryStep;
				}

				// 2. Zaawansowana logika
				// Jeśli zmieniamy kierunek (prąd * cel < 0), używamy dużego kroku
				if (motor1Current * motor1CurrentTarget < 0 && motor2Current * motor2CurrentTarget < 0)
				{
				    motor1CurrentStep = 0.003;
				    motor2CurrentStep = 0.003;
				}
				// Jeśli jedziemy prosto (brak DisablingValue), sprawdzamy różnicę prędkości
				else if (motor1DisablingValue == 0 && motor2DisablingValue == 0) // Jeśli skręcamy
				{
				    if((Velocity1 - Velocity2) > 5) {
				        motor1CurrentStep = motorCurrentPrimaryStep;
				        motor2CurrentStep = motorCurrentPrimaryStep * 3;
				    }
				    else if((Velocity1 - Velocity2) < -5) {
				        motor1CurrentStep = motorCurrentPrimaryStep * 3;
				        motor2CurrentStep = motorCurrentPrimaryStep;
				    }
				    else if((Velocity1 - Velocity2) > 2 && (Velocity1 - Velocity2) < 5) {
				        motor1CurrentStep = motorCurrentPrimaryStep;
				        motor2CurrentStep = motorCurrentPrimaryStep * 2;
				    }
				    else if((Velocity1 - Velocity2) < -2 && (Velocity1 - Velocity2) > -5) {
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

				// --- APLIKACJA RAMPY ---
				if (fabs(motor1CurrentTarget - motor1Current) < motor1CurrentStep) {
					motor1Current = motor1CurrentTarget;
				} else if ((motor1CurrentTarget - motor1Current) > 0) {
					motor1Current += motor1CurrentStep;
				} else {
					motor1Current -= motor1CurrentStep;
				}

				if (fabs(motor2CurrentTarget - motor2Current) < motor2CurrentStep) {
					motor2Current = motor2CurrentTarget;
				} else if ((motor2CurrentTarget - motor2Current) > 0) {
					motor2Current += motor2CurrentStep;
				} else {
					motor2Current -= motor2CurrentStep;
				}

				// Hamulec (PWM)
				if (brakeValue > 0 && throttleActive == 0) {
					motor1CurrentTarget = (-brakeValue / 1023.0 * (1 - motor1DisablingValue));
					motor2CurrentTarget = (-brakeValue / 1023.0 * (1 - motor2DisablingValue));
					brakeActive = 1;
				} else if(brakeValue <= 0 && brakeActive == 1) {
					brakeActive = 0;
				}

				// Zabezpieczenia Duty
				if(motor1Current > MAX_DUTY) motor1Current = MAX_DUTY;
				if(motor1Current < -MAX_DUTY) motor1Current = -MAX_DUTY;
				if(motor2Current > MAX_DUTY) motor2Current = MAX_DUTY;
				if(motor2Current < -MAX_DUTY) motor2Current = -MAX_DUTY;
			}
			// =========================================================
			// TRYB 2: Speed Regulator (Output: Current)
			// =========================================================
			else if (controlMode == 2) {
				COMMAND_ID = 0x01 << 8;

				if (brakeValue > 0) {
					brakeActive = 1;
					throttleActive = 0;
					m1_integral = 0;
					m2_integral = 0;
					brakeFraction = brakeValue / 1023.0;
					motor1Current = (-brakeFraction * MAX_CURRENT * (1 - motor1DisablingValue));
					motor2Current = (-brakeFraction * MAX_CURRENT * (1 - motor2DisablingValue));
				}
				else {
					brakeActive = 0;
					if (btnXValue==true)
						targetVelRaw=5;
					else
					targetVelRaw = (throttleValue / 1023.0f) * MAX_VELOCITY_RAD_S;
					targetVel1 = targetVelRaw * (1.0f - motor1DisablingValue);
					targetVel2 = targetVelRaw * (1.0f - motor2DisablingValue);

					// PID M1
					error1 = targetVel1 - Velocity1;
					P1 = M1_Kp * error1;
					m1_integral += (error1 * DT);
					if (m1_integral > MAX_CURRENT/M1_Ki) m1_integral = MAX_CURRENT/M1_Ki;
					if (m1_integral < -MAX_CURRENT/M1_Ki) m1_integral = -MAX_CURRENT/M1_Ki;
					I1 = M1_Ki * m1_integral;
					D1 = M1_Kd * ((error1 - m1_prev_error) / DT);
					m1_prev_error = error1;
					if(Velocity1 * Velocity2<=0)
					motor1Current = P1 + I1 + D1;

					// PID M2
					error2 = targetVel2 - Velocity2;
					P2 = M2_Kp * error2;
					m2_integral += (error2 * DT);
					if (m2_integral > MAX_CURRENT/M2_Ki) m2_integral = MAX_CURRENT/M2_Ki;
					if (m2_integral < -MAX_CURRENT/M2_Ki) m2_integral = -MAX_CURRENT/M2_Ki;
					I2 = M2_Ki * m2_integral;
					D2 = M2_Kd * ((error2 - m2_prev_error) / DT);
					m2_prev_error = error2;
					if(Velocity1 * Velocity2 <=0)
					motor2Current = P2 + I2 + D2;
				}

				if(motor1Current > MAX_CURRENT) motor1Current = MAX_CURRENT;
				if(motor1Current < -MAX_CURRENT) motor1Current = -MAX_CURRENT;
				if(motor2Current > MAX_CURRENT) motor2Current = MAX_CURRENT;
				if(motor2Current < -MAX_CURRENT) motor2Current = -MAX_CURRENT;
			}
			// =========================================================
			// TRYB 3: Position Regulator (Placeholder)
			// =========================================================
			else if (controlMode == 3) {
				COMMAND_ID = 0x01 << 8;
				motor1Current = 0;
				motor2Current = 0;
			}
		}

		// --- CAN SENDING ---
		if (controlMode == 1) {
			motor1CurrentValue = (int32_t)(motor1Current * 100000.0);
			motor2CurrentValue = (int32_t)(motor2Current * 100000.0);
		} else {
			motor1CurrentValue = (int32_t)(motor1Current * 1000.0);
			motor2CurrentValue = (int32_t)(motor2Current * 1000.0);
		}

		Tx1Header.ExtId = VESC1_ID | COMMAND_ID;
		Tx2Header.ExtId = VESC2_ID | COMMAND_ID;

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
		absoluteAxisValue = abs(axisXValue);
		getVelocity();
		frameNumber++;
	}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART2) {
		indx = Size;

		// Sprawdzamy, czy format jest poprawny (czy są dwukropki na odpowiednim miejscu)
		// Format zakładany: "XXXX:YYYY:ZZZZ:A:B:X:Y" lub podobny zgodny z parsowaniem
		if(uartData[4] == ':'){
			static char value[5]; // Zwiększono rozmiar na 5 (4 cyfry + null)
			float raw_val_local;
			int32_t current_val_int;

			// --- OŚ X ---
			strncpy(value, uartData, 4);
			value[4] = '\0'; // Dodanie terminatora null, bezpieczne dla atoi
			current_val_int = atoi(value);

			// Limitowanie
			if (current_val_int < -512) raw_val_local = -512.0f;
			else if (current_val_int > 511) raw_val_local = 511.0f;
			else raw_val_local = (float)current_val_int;

			// Filtrowanie
			filteredAxis = (SMOOTHING_FACTOR * raw_val_local) + ((1.0f - SMOOTHING_FACTOR) * filteredAxis);
			axisXValue = (int16_t)filteredAxis;

			// --- THROTTLE (GAZ) ---
			strncpy(value, uartData + 5, 4);
			value[4] = '\0';
			current_val_int = atoi(value);

			// Limitowanie
			if (current_val_int < 0) raw_val_local = 0.0f;
			else if (current_val_int > 1023) raw_val_local = 1023.0f;
			else raw_val_local = (float)current_val_int;

			// Filtrowanie
			filteredThrottle = (SMOOTHING_FACTOR * raw_val_local) + ((1.0f - SMOOTHING_FACTOR) * filteredThrottle);
			throttleValue = (uint16_t)filteredThrottle;

			// --- BRAKE (HAMULEC) ---
			strncpy(value, uartData + 10, 4);
			value[4] = '\0';
			current_val_int = atoi(value);

			// Limitowanie
			if (current_val_int < 0) raw_val_local = 0.0f;
			else if (current_val_int > 1023) raw_val_local = 1023.0f;
			else raw_val_local = (float)current_val_int;

			// Filtrowanie
			filteredBrake = (SMOOTHING_FACTOR * raw_val_local) + ((1.0f - SMOOTHING_FACTOR) * filteredBrake);
			brakeValue = (uint16_t)filteredBrake;

			// --- BUTTONS ---
			if(uartData[15] == '1') btnAValue = 1; else btnAValue = 0;

			if(uartData[17] == '1') btnBValue = 1; else btnBValue = 0;

			if(uartData[19] == '1') btnXValue = 1; else btnXValue = 0;

			if(uartData[21] == '1') btnYValue = 1; else btnYValue = 0;
		}

		// Ponowne uruchomienie nasłuchiwania DMA
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 32);
	}
}

//ADC
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

	// Inicjalizacja MPU6050 PRZED startem timerów, aby uniknąć błędów I2C w przerwaniach
	MPU6050_init();

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 32);

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){

	}

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
