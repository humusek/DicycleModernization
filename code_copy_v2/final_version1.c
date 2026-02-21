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
#include "spi.h"
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
#define DT 0.02f
#define SMOOTHING_FACTOR 0.2f

static float filteredAxis = 0.0f;
static float filteredThrottle = 0.0f;
static float filteredBrake = 0.0f;
float raw_val=0.0f;

#define LCD_CS_PORT   GPIOE
#define LCD_CS_PIN    GPIO_PIN_9
#define LCD_DC_PORT   GPIOF
#define LCD_DC_PIN    GPIO_PIN_13
#define LCD_RES_PORT  GPIOD
#define LCD_RES_PIN   GPIO_PIN_15
#define LCD_BLK_PORT  GPIOD
#define LCD_BLK_PIN   GPIO_PIN_14

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LCD_CS_LOW()   HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()   HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()  HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_RES_LOW()  HAL_GPIO_WritePin(LCD_RES_PORT, LCD_RES_PIN, GPIO_PIN_RESET)
#define LCD_RES_HIGH() HAL_GPIO_WritePin(LCD_RES_PORT, LCD_RES_PIN, GPIO_PIN_SET)
#define LCD_BLK_ON()   HAL_GPIO_WritePin(LCD_BLK_PORT, LCD_BLK_PIN, GPIO_PIN_SET)
#define LCD_BLK_OFF()  HAL_GPIO_WritePin(LCD_BLK_PORT, LCD_BLK_PIN, GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t lcdUpdateNeeded = 0;

// --- LOGGING VARIABLES ---
volatile uint8_t uartTxBusy = 0;
volatile uint8_t isLoggingActive = 0; // Flaga stanu zapisu (0 - stop, 1 - nagrywanie)
bool btnXLastState = false;           // Do wykrywania zbocza przycisku X

typedef struct __attribute__((packed)) {
	uint16_t header;
	int32_t val_m1RealCurrent;
	int32_t val_m2RealCurrent;
	int32_t val_m1Current;
	int32_t val_m2Current;
	int32_t val_angleX;
	int32_t val_rawTargetAngle;
	int32_t val_Velocity1;
	int32_t val_Velocity2;
	int32_t val_targetVel1;
	int32_t val_targetVel2;
} SerialDataPacket;

SerialDataPacket txPacket;
// ---------------------------------

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

// --- PID CONSTANTS ---
// MOTOR 1 (RIGHT / PRAWY)
float M1_Kp = 0.84788f;
float M1_Ki = 0.76984f;
// MOTOR 2 (LEFT / LEWY)
float M2_Kp = 1.01028f;
float M2_Ki = 1.04346f;
// --- PID STATE VARIABLES ---
float m1_integral = 0.0f;
float m1_prev_error = 0.0f;
float m2_integral = 0.0f;
float m2_prev_error = 0.0f;
float targetVelRaw = 0.0f;

// Target Velocity MAX
float MAX_VELOCITY_RAD_S = 30.0f;
//Motor 1 params
float motor1Current;
int32_t motor1CurrentValue;
float motor1DisablingValue;
float motor1RealCurrent;
float inputRealCurrentSamples[21];
int32_t motor1RealDutyCycle;
//Motor 2 params
float motor2Current;
int32_t motor2CurrentValue;
float motor2DisablingValue;
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
float MAX_CURRENT = 20.0;
float MAX_DUTY = 0.95;
// --- MODE CONTROL VARIABLES ---
// 1 = PWM (Duty Cycle), 2 = Current (Speed reg), 3 = Position
uint8_t controlMode = 1;
bool btnYLastState = false;
uint32_t ledCounter = 0;

// --- PWM control ---
float motor1CurrentTarget = 0.0;
float motor2CurrentTarget = 0.0;
float motor1CurrentStep = 0.005;
float motor2CurrentStep = 0.005;
float motorCurrentPrimaryStep = 0.005;
float motorCurrentExtendedStep = 0.01;
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
float M2_STARTUP_BOOST = 2.0f;

// --- Position regulator
#define IS_DELAY_SAMPLES 35
#define IS_A1 0.540f
#define IS_A2 0.460f

float shaperBuffer[IS_DELAY_SAMPLES];
uint8_t shaperIdx = 0;
float targetAngleShaped = 0.0f;
// Regulator PID dla kąta
float ANG_Kp = 2.237f;
float ANG_Ki = 0.0f;
float ANG_Kd = 1.451f;
float ang_integral = 0.0f;
float ang_prev_error = 0.0f;
// --- ZMIENNE DO STEROWANIA POZYCJĄ (TRYB 3) ---
float initialAngleOffset = 0.0f;
float angle_error = 0.0f;
int16_t initialAxisOffset = 0;
static float filtered_derivative = 0.0f;
float rawTargetAngle=0.0f;

// --- KONFIGURACJA KĄTA ---
#define MAX_TILT_ANGLE 25.0f
#define D_TERM_FILTER_ALPHA 0.6f

// --- KOREKCJA KIERUNKU (SYNC) ---
float sync_Kp = 0.5f;
float sync_error = 0.0f;
float sync_output = 0.0f;

#define SAMPLES_COUNT 21
uint16_t inputThrottleSamples[SAMPLES_COUNT];
uint16_t inputAxisSamples[SAMPLES_COUNT];
uint16_t inputBrakeSamples[SAMPLES_COUNT];
uint16_t VESC1_ID = 0x0000000A;
uint16_t VESC2_ID = 0x0000000B;
uint32_t COMMAND_ID = 0x01 << 8;

char uartData[32];
uint16_t indx;
int32_t temp_val;

uint16_t Encoder1Counter;
float Velocity1;
uint16_t Encoder2Counter;
float Velocity2;
char logString[70];
volatile uint16_t frameNumber = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void LCD_Reset(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t *data, uint16_t size);
void LCD_SetBrightness(uint8_t percent);
void LCD_Init(void);
void LCD_Update(void);
void LCD_DrawString(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t bg, uint8_t size);
void LCD_Fill(uint16_t color);
void LogData(void); // Prototype for logging

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t Font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // SPACE
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // \ //
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4c, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x10, 0x08, 0x08, 0x10, 0x08}  // ~
};

void LCD_Reset(void)
{
    LCD_RES_LOW();
    HAL_Delay(20);
    LCD_RES_HIGH();
    HAL_Delay(120);
}

void LCD_SendCommand(uint8_t cmd)
{
    LCD_CS_LOW();
    LCD_DC_LOW();
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
    LCD_CS_HIGH();
}

void LCD_SendData(uint8_t *data, uint16_t size)
{
    LCD_CS_LOW();
    LCD_DC_HIGH();
    HAL_SPI_Transmit(&hspi4, data, size, 100);
    LCD_CS_HIGH();
}

void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];
    LCD_SendCommand(0x2A);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    LCD_SendData(data, 4);

    LCD_SendCommand(0x2B);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    LCD_SendData(data, 4);

    LCD_SendCommand(0x2C);
}

void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if (c < 32 || c > 126) c = 32;
    uint8_t index = c - 32;

    uint16_t width = 5 * size;
    uint16_t height = 7 * size;

    LCD_SetAddressWindow(x, y, x + width - 1, y + height - 1);

    uint8_t buffer[1200];
    uint32_t bufIdx = 0;

    for (int row = 0; row < 7; row++) {
        for (int sy = 0; sy < size; sy++) {
            for (int col = 0; col < 5; col++) {
                uint8_t pixelOn = (Font5x7[index][col] >> row) & 0x01;
                uint16_t pixelColor = pixelOn ? color : bg;

                for (int sx = 0; sx < size; sx++) {
                    buffer[bufIdx++] = (pixelColor >> 8) & 0xFF;
                    buffer[bufIdx++] = pixelColor & 0xFF;
                }
            }
        }
    }
    LCD_SendData(buffer, bufIdx);
}

void LCD_DrawString(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t bg, uint8_t size) {
    while (*str) {
        LCD_DrawChar(x, y, *str, color, bg, size);
        x += (5 * size) + size; // Szerokość znaku + odstęp
        str++;
    }
}

void LCD_SetBrightness(uint8_t percent)
{
    if (percent > 0) LCD_BLK_ON();
    else LCD_BLK_OFF();
}

void LCD_Init(void)
{
    LCD_CS_HIGH();
    LCD_DC_HIGH();
    LCD_RES_HIGH();
    LCD_BLK_ON();

    LCD_Reset();

    LCD_SendCommand(0x11);
    HAL_Delay(120);

    uint8_t colorMode = 0x55;
    LCD_SendCommand(0x3A);
    LCD_SendData(&colorMode, 1);

    uint8_t rotation = 0x00;
    LCD_SendCommand(0x36);
    LCD_SendData(&rotation, 1);

    LCD_SendCommand(0x21);

    LCD_SendCommand(0x29);
    HAL_Delay(10);
}

void LCD_Update(void) {
    char lcd_buffer[40];
    float avgSpeed = (Velocity1 + Velocity2) / 2.0f;

    // 1. Tryb Pracy (Opisy słowne)
    char *modeStr;
    switch(controlMode) {
        case 1: modeStr = "Tryb 1: PWM"; break;
        case 2: modeStr = "Tryb 2: Speed"; break;
        case 3: modeStr = "Tryb 3: Pos"; break;
        default: modeStr = "Tryb: ???"; break;
    }

    // %-16s wyrównuje do lewej i dodaje spacje, czyszcząc stare znaki
    sprintf(lcd_buffer, "%-16s", modeStr);
    LCD_DrawString(20, 40, lcd_buffer, 0xF800, 0x0000, 2);

    // 2. Prędkość (Odsunięte niżej do Y=100)
    sprintf(lcd_buffer, "Predkosc: %.1f  ", avgSpeed);
    LCD_DrawString(20, 100, lcd_buffer, 0xFFE0, 0x0000, 2);

    // 3. Kąt (NOWE! Y=160, Kolor Zielony)
    sprintf(lcd_buffer, "Kat: %.1f      ", angleX);
    LCD_DrawString(20, 160, lcd_buffer, 0x07E0, 0x0000, 2);

    // 4. Stopka (Zalezy od stanu logowania)
    if (isLoggingActive) {
        sprintf(lcd_buffer, "Wcisnij X aby    ");
        LCD_DrawString(20, 260, lcd_buffer, 0xFFFF, 0x0000, 1);
        sprintf(lcd_buffer, "zakonczyc zapis  ");
        LCD_DrawString(20, 280, lcd_buffer, 0x07E0, 0x0000, 1); // Zielony
    } else {
        sprintf(lcd_buffer, "Wcisnij X aby    ");
        LCD_DrawString(20, 260, lcd_buffer, 0xFFFF, 0x0000, 1);
        sprintf(lcd_buffer, "rozpoczac zapis  ");
        LCD_DrawString(20, 280, lcd_buffer, 0xFFFF, 0x0000, 1); // Bialy
    }
}

void LCD_Fill(uint16_t color) {
    LCD_SetAddressWindow(0, 0, 240, 320);
    uint8_t buffer[240 * 2];
    for(int i=0; i<240; i++) {
        buffer[i*2] = (color >> 8) & 0xFF;
        buffer[i*2+1] = color & 0xFF;
    }
    LCD_CS_LOW();
    LCD_DC_HIGH();
    for(int i=0; i<320; i++) {
        HAL_SPI_Transmit(&hspi4, buffer, 480, 100);
    }
    LCD_CS_HIGH();
}

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

// --- LOGGING CALLBACK & FUNCTION (ADDED) ---
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        uartTxBusy = 0; // Transmisja zakończona, można wysyłać ponownie
    }
}

void LogData() {
    // Jeśli poprzednia transmisja jeszcze trwa, pomiń tę ramkę
    if (uartTxBusy == 1) {
        return;
    }

    txPacket.header = 0xABCD;
    // W starym kodzie wartości były w int32 (float * 1000). Konwertujemy w locie.
    txPacket.val_m1RealCurrent = (int32_t)(motor1RealCurrent * 1000.0f);
    txPacket.val_m2RealCurrent = (int32_t)(motor2RealCurrent * 1000.0f);
    txPacket.val_m1Current = (int32_t)(motor1Current * 1000.0f);
    txPacket.val_m2Current = (int32_t)(motor2Current * 1000.0f);
    txPacket.val_angleX = (int32_t)(angleX * 1000.0f);
    txPacket.val_rawTargetAngle = (int32_t)(rawTargetAngle * 1000.0f);
    txPacket.val_Velocity1 = (int32_t)(Velocity1 * 1000.0f);
    txPacket.val_Velocity2 = (int32_t)(Velocity2 * 1000.0f);
    txPacket.val_targetVel1 = (int32_t)(targetVel1 * 1000.0f);
    txPacket.val_targetVel2 = (int32_t)(targetVel2 * 1000.0f);

    uartTxBusy = 1;
    // Używamy wersji DMA - nie blokuje procesora!
    if(HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&txPacket, sizeof(SerialDataPacket)) != HAL_OK) {
        uartTxBusy = 0; // Reset flagi w razie błędu
    }
}
// -------------------------------------------

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

    if (htim == &htim7) {
        motor1DisablingValue = 0.0;
        motor2DisablingValue = 0.0;

        // --- OBSLUGA LOGOWANIA (BTN X) ---
        if (btnXValue == 1 && !btnXLastState) {
            isLoggingActive = !isLoggingActive; // Zmiana stanu
            lcdUpdateNeeded = 1; // Odswiez LCD natychmiast
        }
        btnXLastState = btnXValue;

        // --- ZMIANA TRYBU PRACY (Przycisk Y) ---
        if (btnYValue == 1 && !btnYLastState) {
            if (fabs(Velocity1) < 0.2 && fabs(Velocity2) < 0.2) {
                controlMode++;
                if (controlMode > 3) controlMode = 1;

                // Reset zmiennych sterowania
                motor1Current = 0; motor2Current = 0;
                motor1CurrentTarget = 0; motor2CurrentTarget = 0;
                m1_integral = 0; m2_integral = 0;
                m1_prev_error = 0; m2_prev_error = 0;
                throttleActive = 0; brakeActive = 0;

                // --- KONFIGURACJA DLA TRYBU 3 (POZYCJA) ---
                if (controlMode == 3) {
                    // 1. Ustaw obecny kat jako zero
                    initialAngleOffset = angleX;

                    // 2. Ustaw obecna pozycje joysticka jako zero
                    initialAxisOffset = axisXValue;

                    // 3. Zresetuj PID i filtry
                    ang_integral = 0.0f;
                    ang_prev_error = 0.0f;
                    filtered_derivative = 0.0f;

                    // 4. Wypelnij bufor shapera nowym katem, aby uniknac szarpniecia
                    //    gdyz shaper pamieta stare wartosci (bliskie 0.0), a nowy cel to np. 5.0
                    for(int i=0; i<IS_DELAY_SAMPLES; i++) {
                        shaperBuffer[i] = initialAngleOffset;
                    }
                    shaperIdx = 0;
                }
                else {
                    // Opcjonalnie reset offsetow dla innych trybow
                    initialAngleOffset = 0.0f;
                    initialAxisOffset = 0;
                }
            }
        }
        btnYLastState = btnYValue;

        ledCounter++;
        uint32_t blinkPeriod;
        if (controlMode == 1) blinkPeriod = 5;
        else if (controlMode == 2) blinkPeriod = 25;
        else blinkPeriod = 50;

        if (ledCounter >= blinkPeriod) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            ledCounter = 0;
        }

        if (axisXValue < 0) {
            motor1DisablingValue = absoluteAxisValue / 511.0;
        } else if (axisXValue > 0) {
            motor2DisablingValue = absoluteAxisValue / 512.0;
        }

        if(btnBValue == 1) {
            motor1Current = 0.0; motor2Current = 0.0;
            motor1CurrentTarget = 0.0; motor2CurrentTarget = 0.0;
            m1_integral = 0; m2_integral = 0;
        }
        else {
            //PWM
            if (controlMode == 1) {
                COMMAND_ID = 0x00 << 8;

                if (throttleValue > 0 && brakeActive == 0) {
                    motor1CurrentTarget = (throttleValue / 1023.0 * (1 - motor1DisablingValue));
                    motor2CurrentTarget = (throttleValue / 1023.0 * (1 - motor2DisablingValue));

                    if (motor1CurrentTarget < 0.001 && motor1CurrentTarget > -0.001) motor1CurrentTarget = 0;
                    if (motor2CurrentTarget < 0.001 && motor2CurrentTarget > -0.001) motor2CurrentTarget = 0;
                    throttleActive = 1;
                } else if(throttleValue <= 0 && throttleActive == 1) {
                    throttleActive = 0;
                    motor1CurrentTarget = 0;
                    motor2CurrentTarget = 0;
                }

                if(motor1DisablingValue > 0.01 || motor2DisablingValue > 0.01){
                    motor1CurrentStep = motorCurrentExtendedStep;
                    motor2CurrentStep = motorCurrentExtendedStep;
                } else {
                    motor1CurrentStep = motorCurrentPrimaryStep;
                    motor2CurrentStep = motorCurrentPrimaryStep;
                }

                if (motor1Current * motor1CurrentTarget < 0 && motor2Current * motor2CurrentTarget < 0)
                {
                    motor1CurrentStep = 0.03;
                    motor2CurrentStep = 0.03;
                }
                else if (motor1DisablingValue == 0 && motor2DisablingValue == 0)
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

                if (brakeValue > 0 && throttleActive == 0) {
                    motor1CurrentTarget = (-brakeValue / 1023.0 * (1 - motor1DisablingValue));
                    motor2CurrentTarget = (-brakeValue / 1023.0 * (1 - motor2DisablingValue));
                    brakeActive = 1;
                } else if(brakeValue <= 0 && brakeActive == 1) {
                    brakeActive = 0;
                }

                if(motor1Current > MAX_DUTY) motor1Current = MAX_DUTY;
                if(motor1Current < -MAX_DUTY) motor1Current = -MAX_DUTY;
                if(motor2Current > MAX_DUTY) motor2Current = MAX_DUTY;
                if(motor2Current < -MAX_DUTY) motor2Current = -MAX_DUTY;
            }
            //Speed regulator
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
                    // UWAGA: Usunieto automatyczne ustawianie predkosci na 5 przy wcisnieciu X
                    targetVelRaw = (throttleValue / 1023.0f) * MAX_VELOCITY_RAD_S;

                    targetVel1 = targetVelRaw * (1.0f - motor1DisablingValue);
                    targetVel2 = targetVelRaw * (1.0f - motor2DisablingValue);

                    error1 = targetVel1 - Velocity2;
                    P1 = M1_Kp * error1;
                    m1_integral += (error1 * DT);
                    if (m1_integral > MAX_CURRENT/M1_Ki) m1_integral = MAX_CURRENT/M1_Ki;
                    if (m1_integral < -MAX_CURRENT/M1_Ki) m1_integral = -MAX_CURRENT/M1_Ki;
                    I1 = M1_Ki * m1_integral;
                    m1_prev_error = error1;
                    motor1Current = P1 + I1;

                    error2 = targetVel2 - Velocity1;
                    P2 = M2_Kp * error2;
                    m2_integral += (error2 * DT);
                    if (m2_integral > MAX_CURRENT/M2_Ki) m2_integral = MAX_CURRENT/M2_Ki;
                    if (m2_integral < -MAX_CURRENT/M2_Ki) m2_integral = -MAX_CURRENT/M2_Ki;
                    I2 = M2_Ki * m2_integral;
                    m2_prev_error = error2;
                    motor2Current = P2 + I2;
                }

                if(motor1Current > MAX_CURRENT) motor1Current = MAX_CURRENT;
                if(motor1Current < -MAX_CURRENT) motor1Current = -MAX_CURRENT;
                if(motor2Current > MAX_CURRENT) motor2Current = MAX_CURRENT;
                if(motor2Current < -MAX_CURRENT) motor2Current = -MAX_CURRENT;
            }
            //Position control (TRYB 3)
                        else if (controlMode == 3) {
                            COMMAND_ID = 0x01 << 8;

                            // 1. Oblicz cel (Target)
                            float effectiveAxis = (float)(axisXValue - initialAxisOffset);
                            float joystickDelta = (effectiveAxis / 512.0f) * MAX_TILT_ANGLE;

                            if(fabs(joystickDelta) < 0.5f) joystickDelta = 0.0f;

                            rawTargetAngle = initialAngleOffset + joystickDelta;

                            // 2. Input Shaper (Wygładzanie celu)
                            float delayedInput = shaperBuffer[shaperIdx];
                            shaperBuffer[shaperIdx] = rawTargetAngle;

                            shaperIdx++;
                            if (shaperIdx >= IS_DELAY_SAMPLES) shaperIdx = 0;

                            targetAngleShaped = (IS_A1 * rawTargetAngle) + (IS_A2 * delayedInput);

                            // 3. Oblicz błąd
                            angle_error = targetAngleShaped - angleX;

                            // 4. PID (Bez zbędnego Tolerace/Deadzone)

                            // --- P (Proporcjonalny) ---
                            float P_out = ANG_Kp * angle_error;

                            // --- I (Całkujący) ---
                            ang_integral += (angle_error * DT);

                            // Limit całki (anti-windup)
                            float integral_limit = MAX_CURRENT / 2.0f;
                            // Zabezpieczenie przed dzieleniem przez zero, gdybyś ustawił Ki=0
                            if (ANG_Ki > 0.0001f) integral_limit = MAX_CURRENT / ANG_Ki;

                            if (ang_integral > integral_limit) ang_integral = integral_limit;
                            if (ang_integral < -integral_limit) ang_integral = -integral_limit;

                            float I_out = ANG_Ki * ang_integral;

                            // --- D (Różniczkujący z filtrem) ---
                            // Liczymy pochodną zawsze, żeby filtr pracował ciągle
                            float raw_derivative = (angle_error - ang_prev_error) / DT;

                            // Filtr dolnoprzepustowy (Twoja stała 0.6f jest już OK)
                            filtered_derivative = (D_TERM_FILTER_ALPHA * raw_derivative) + ((1.0f - D_TERM_FILTER_ALPHA) * filtered_derivative);

                            float D_out = ANG_Kd * filtered_derivative;

                            // Zapisz błąd do następnego kroku
                            ang_prev_error = angle_error;

                            // 5. Wyjście na silniki
                            float pid_output = P_out + I_out + D_out;

                            motor1Current = pid_output;
                            motor2Current = pid_output;

                            // Limit prądu
                            if(motor1Current > MAX_CURRENT) motor1Current = MAX_CURRENT;
                            if(motor1Current < -MAX_CURRENT) motor1Current = -MAX_CURRENT;
                            if(motor2Current > MAX_CURRENT) motor2Current = MAX_CURRENT;
                            if(motor2Current < -MAX_CURRENT) motor2Current = -MAX_CURRENT;
                        }
        }

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

    if (htim == &htim1) {
        MPU6050_Read_Accel(&Ax, &Ay, &Az);
        MPU6050_ReadKalmanAngles(&angleX, &angleY);
        absoluteAxisValue = abs(axisXValue);
        getVelocity();

        lcdUpdateNeeded = 1;
        frameNumber++;
    }

    if (htim == &htim5){
        if (isLoggingActive) {
            LogData();
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART2) {
		indx = Size;

		if(uartData[4] == ':'){
			static char value[5];
			float raw_val_local;
			int32_t current_val_int;

			strncpy(value, uartData, 4);
			value[4] = '\0';
			current_val_int = atoi(value);

			if (current_val_int < -512) raw_val_local = -512.0f;
			else if (current_val_int > 511) raw_val_local = 511.0f;
			else raw_val_local = (float)current_val_int;

			filteredAxis = (SMOOTHING_FACTOR * raw_val_local) + ((1.0f - SMOOTHING_FACTOR) * filteredAxis);
			axisXValue = (int16_t)filteredAxis;

			strncpy(value, uartData + 5, 4);
			value[4] = '\0';
			current_val_int = atoi(value);

			if (current_val_int < 0) raw_val_local = 0.0f;
			else if (current_val_int > 1023) raw_val_local = 1023.0f;
			else raw_val_local = (float)current_val_int;

			filteredThrottle = (SMOOTHING_FACTOR * raw_val_local) + ((1.0f - SMOOTHING_FACTOR) * filteredThrottle);
			throttleValue = (uint16_t)filteredThrottle;

			strncpy(value, uartData + 10, 4);
			value[4] = '\0';
			current_val_int = atoi(value);

			if (current_val_int < 0) raw_val_local = 0.0f;
			else if (current_val_int > 1023) raw_val_local = 1023.0f;
			else raw_val_local = (float)current_val_int;

			filteredBrake = (SMOOTHING_FACTOR * raw_val_local) + ((1.0f - SMOOTHING_FACTOR) * filteredBrake);
			brakeValue = (uint16_t)filteredBrake;

			if(uartData[15] == '1') btnAValue = 1; else btnAValue = 0;
			if(uartData[17] == '1') btnBValue = 1; else btnBValue = 0;
			if(uartData[19] == '1') btnXValue = 1; else btnXValue = 0;
			if(uartData[21] == '1') btnYValue = 1; else btnYValue = 0;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &uartData, 32);
	}
}

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
  MX_SPI4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan1);
	CANStart();

	MPU6050_init();

	LCD_Init();
	LCD_SetBrightness(100);
    LCD_Fill(0x0000);
    LCD_Update();

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim5);

	__HAL_UART_CLEAR_OREFLAG(&huart2);
	__HAL_UART_CLEAR_NEFLAG(&huart2);
	__HAL_UART_CLEAR_FEFLAG(&huart2);
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)uartData, 32);

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){

	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (lcdUpdateNeeded) {
			LCD_Update();
			lcdUpdateNeeded = 0;
		}
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
