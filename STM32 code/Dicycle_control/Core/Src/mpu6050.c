#include "MPU6050.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

uint32_t timer;

Kalman_t KalmanStructX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanStructY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

float KalmanAngleX, KalmanAngleY;

extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly
void MPU6050_init(void)
{
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);
	if (check == 104)
	{
		//Power management register write all 0's to wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		//Set data rate of 1KHz by writing SMPRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//Writing both register with 0 to set full scale range
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}

}

//Function with multiple return using pointer

void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	//Adding 2 BYTES into 16 bit integer 
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	*Ax = Accel_X_RAW/16384.0;
	*Ay = Accel_Y_RAW/16384.0;
	*Az = Accel_Z_RAW/14418.0;
}

void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // Correctly assign raw data values for each axis
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *Gx = Gyro_X_RAW / 131.0;
    *Gy = Gyro_Y_RAW / 131.0;
    *Gz = Gyro_Z_RAW / 131.0;
}

void MPU6050_ReadKalmanAngles(float* KalmanX, float* KalmanY){
	float Ax, Ay, Az;
	MPU6050_Read_Accel(&Ax, &Ay, &Az);

	float Gx, Gy, Gz;
	MPU6050_Read_Gyro(&Gx, &Gy, &Gz);

	// Kalman angle solve
	double dt = (double)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	double roll;
	double roll_sqrt = sqrt(Accel_X_RAW * Accel_X_RAW + Accel_Z_RAW * Accel_Z_RAW);
	if (roll_sqrt != 0.0)
	{
		roll = atan(Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
	}
	else
	{
		roll = 0.0;
	}
	double pitch = atan2(-Accel_X_RAW, Accel_Z_RAW) * RAD_TO_DEG;
	if ((pitch < -90 && KalmanAngleY > 90) || (pitch > 90 && KalmanAngleY < -90))
	{
		KalmanStructY.angle = pitch;
		KalmanAngleY = pitch;
	}
	else
	{
		KalmanAngleY = Kalman_getAngle(&KalmanStructY, pitch, Gy, dt);
	}
	if (fabs(KalmanAngleY) > 90)
		Gx = -Gx;
	KalmanAngleX = Kalman_getAngle(&KalmanStructX, roll, Gx, dt);

	*KalmanX = KalmanAngleX;
	*KalmanY = KalmanAngleY;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00 = Kalman->P[0][0];
    double P01 = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00;
    Kalman->P[0][1] -= K[0] * P01;
    Kalman->P[1][0] -= K[1] * P00;
    Kalman->P[1][1] -= K[1] * P01;

    return Kalman->angle;
}
