/*
 * MPU6050.h
 *
 *  Created on: Feb 24, 2026
 *      Author: HunterCHCL
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "cmsis_os.h"

extern osThreadId_t MPU6050Handle;

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

    int16_t GyroZ_Raw;

    float AccelBodyX;//cm/s^2（车体坐标系）
    float AccelBodyY;//cm/s^2

    float AccelX;//cm/s^2（世界坐标系）
    float AccelY;//cm/s^2

    float VelX;//cm/s
    float VelY;//cm/s

    float PosX;//cm
    float PosY;//cm

    float GyroZ;//度/秒
    float Yaw;//度

    // Sensor offsets from static calibration
    int32_t AccelX_Offset;
    int32_t AccelY_Offset;
    int32_t AccelZ_Offset;
    int32_t GyroZ_Offset;

    uint32_t LastTick;
    uint16_t StationaryCount;
    uint8_t IsStationary;
} MPU6050_Data_t;

void MPU6050_WriteReg(uint8_t RegAdress, uint8_t Data);
void MPU6050_ReadReg(uint8_t RegAdress, uint8_t *Data);
void MPU6050_ReadMultiReg(uint8_t RegAdress, uint8_t *Data, uint16_t Length);

void MPU6050_Init(void);

void MPU6050_ReadAll(MPU6050_Data_t *DataStruct);
void MPU6050_Calibrate(MPU6050_Data_t *DataStruct);
void MPU6050_Update(MPU6050_Data_t *DataStruct);
void MPU6050_ResetOdometry(MPU6050_Data_t *DataStruct);

void MPU6050Task(void *argument);

extern MPU6050_Data_t mpu_data;

#endif /* INC_MPU6050_H_ */
