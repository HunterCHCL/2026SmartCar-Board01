/*
 * MPU6050.h
 *
 *  Created on: Feb 24, 2026
 *      Author: HunterCHCL
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <math.h>

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

typedef struct {
    int16_t AccelX_Raw;
    int16_t AccelY_Raw;
    int16_t AccelZ_Raw;
    
    int16_t GyroX_Raw;
    int16_t GyroY_Raw;
    int16_t GyroZ_Raw;

    float AccelX;
    float AccelY;
    float AccelZ;

    float GyroX;
    float GyroY;
    float GyroZ;

    float Pitch;
    float Roll;
    float Yaw;

    // 校准偏移量
    int32_t AccelX_Offset;
    int32_t AccelY_Offset;
    int32_t AccelZ_Offset;
    int32_t GyroX_Offset;
    int32_t GyroY_Offset;
    int32_t GyroZ_Offset;
} MPU6050_Data_t;

void MPU6050_WriteReg(uint8_t RegAdress, uint8_t Data);
void MPU6050_ReadReg(uint8_t RegAdress, uint8_t *Data);
void MPU6050_ReadMultiReg(uint8_t RegAdress, uint8_t *Data, uint16_t Length);
void MPU6050_Init(void);
void MPU6050_ReadAccel(int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ);
void MPU6050_ReadGyro(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_ReadAll(MPU6050_Data_t *DataStruct);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Calibrate(MPU6050_Data_t *DataStruct);
void MPU6050_ProcessData(MPU6050_Data_t *DataStruct);

float MPU6050_Accel_To_G_16G(int16_t AccelRaw);
float MPU6050_Gyro_To_DegPerSec_2000(int16_t GyroRaw);

#endif /* INC_MPU6050_H_ */
