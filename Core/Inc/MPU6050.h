/*
 * MPU6050.h
 *
 *  Created on: Feb 24, 2026
 *      Author: HunterCHCL
 */
//顺时针是负数，逆时针是正数
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <math.h>
#include "OLED.h"
#include "i2c.h"
#include "cmsis_os.h"

#define MPU6050Addr 0xD0

#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

#define MPU6050_I2C hi2c1
#define MPU6050_CYCLE_TIME 10 //ms

#define MPU6050_Accel_Sensitivity_16G 2048.0f
#define MPU6050_Gyro_Sensitivity_2000DPS 16.4f

void MPU6050_Init(void);
void MPU6050_ReadAccel(int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ);
void MPU6050_ReadGyro(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_ProcessYaw(float GyroZ, float *yaw,float GyroZ_Offset);
float MPU6050_Accel_To_G_16G(int16_t AccelRaw);
float MPU6050_Gyro_To_DegPerSec_2000(int16_t GyroRaw);

#endif /* INC_MPU6050_H_ */
