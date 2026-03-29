/*
 * MPU6050.c
 *
 *  Created on: Feb 24, 2026
 *      Author: HunterCHCL
 */

#include "i2c.h"
#include "MPU6050.h"
#include <math.h>

#define MPU6050_I2C hi2c1

#define MPU6050_ACCEL_LSB_PER_G        2048.0f
#define MPU6050_GYRO_LSB_PER_DPS_2000  16.4f
#define GRAVITY_CM_S2                  980.665f
#define DEG_TO_RAD                     0.01745329252f
#define MPU6050_DT_FALLBACK            0.01f
#define MPU6050_ACCEL_DEADBAND_CM_S2   3.0f
#define MPU6050_STILL_GYRO_DPS_TH      1.5f
#define MPU6050_STILL_ACC_XY_CM_S2_TH  12.0f
#define MPU6050_STILL_ACC_NORM_G_TH    0.08f
#define MPU6050_STILL_CONFIRM_SAMPLES  10U
#define MPU6050_BIAS_LEARN_ALPHA       0.01f

static float MPU6050_GetDeltaTime(MPU6050_Data_t *DataStruct)
{
    uint32_t currentTick = osKernelGetTickCount();
    float dt = (currentTick - DataStruct->LastTick) / (float)osKernelGetTickFreq();

    if (dt <= 0.0f || dt > 1.0f) {
        dt = MPU6050_DT_FALLBACK;
    }

    DataStruct->LastTick = currentTick;
    return dt;
}

void MPU6050_WriteReg(uint8_t RegAdress, uint8_t Data)
{
    uint8_t pBuffer[2] = {RegAdress, Data};
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, pBuffer, 2, HAL_MAX_DELAY);
}

void MPU6050_ReadReg(uint8_t RegAdress, uint8_t *Data)
{
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, &RegAdress, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050Addr, Data, 1, HAL_MAX_DELAY);
}

void MPU6050_ReadMultiReg(uint8_t RegAdress, uint8_t *Data, uint16_t Length)
{
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, &RegAdress, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050Addr, Data, Length, HAL_MAX_DELAY);
}

void MPU6050_Init(void)
{
    osDelay(100);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

void MPU6050_ReadAll(MPU6050_Data_t *DataStruct)
{
    uint8_t data[14];

    MPU6050_ReadMultiReg(MPU6050_ACCEL_XOUT_H, data, 14);

    DataStruct->AccelX_Raw = (int16_t)((data[0] << 8) | data[1]);
    DataStruct->AccelY_Raw = (int16_t)((data[2] << 8) | data[3]);
    DataStruct->AccelZ_Raw = (int16_t)((data[4] << 8) | data[5]);
    DataStruct->GyroZ_Raw = (int16_t)((data[12] << 8) | data[13]);
}

void MPU6050_ResetOdometry(MPU6050_Data_t *DataStruct)
{
    DataStruct->VelX = 0.0f;
    DataStruct->VelY = 0.0f;
    DataStruct->PosX = 0.0f;
    DataStruct->PosY = 0.0f;
}

void MPU6050_Calibrate(MPU6050_Data_t *DataStruct)
{
    int32_t ax = 0;
    int32_t ay = 0;
    int32_t az = 0;
    int32_t gz = 0;
    const int numSamples = 200;

    for (int i = 0; i < numSamples; i++) {
        MPU6050_ReadAll(DataStruct);
        ax += DataStruct->AccelX_Raw;
        ay += DataStruct->AccelY_Raw;
        az += DataStruct->AccelZ_Raw;
        gz += DataStruct->GyroZ_Raw;
        osDelay(5);
    }

    DataStruct->AccelX_Offset = ax / numSamples;
    DataStruct->AccelY_Offset = ay / numSamples;
    DataStruct->AccelZ_Offset = (az / numSamples) - (int32_t)MPU6050_ACCEL_LSB_PER_G;
    DataStruct->GyroZ_Offset = gz / numSamples;

    DataStruct->Yaw = 0.0f;
    DataStruct->GyroZ = 0.0f;
    DataStruct->AccelBodyX = 0.0f;
    DataStruct->AccelBodyY = 0.0f;
    DataStruct->AccelX = 0.0f;
    DataStruct->AccelY = 0.0f;
    DataStruct->StationaryCount = 0U;
    DataStruct->IsStationary = 1U;
    MPU6050_ResetOdometry(DataStruct);
}

void MPU6050_Update(MPU6050_Data_t *DataStruct)
{
    float dt;
    int16_t axRaw;
    int16_t ayRaw;
    int16_t azRaw;
    int16_t gzRaw;
    float accelXBody;
    float accelYBody;
    float yawRad;
    float accelXWorld;
    float accelYWorld;
    float accelNormG;
    float accelXYNorm;
    uint8_t stillCandidate;

    dt = MPU6050_GetDeltaTime(DataStruct);

    axRaw = DataStruct->AccelX_Raw - (int16_t)DataStruct->AccelX_Offset;
    ayRaw = DataStruct->AccelY_Raw - (int16_t)DataStruct->AccelY_Offset;
    azRaw = DataStruct->AccelZ_Raw - (int16_t)DataStruct->AccelZ_Offset;
    gzRaw = DataStruct->GyroZ_Raw - (int16_t)DataStruct->GyroZ_Offset;

    DataStruct->GyroZ = gzRaw / MPU6050_GYRO_LSB_PER_DPS_2000;
    DataStruct->Yaw += DataStruct->GyroZ * dt;

    accelXBody = (axRaw / MPU6050_ACCEL_LSB_PER_G) * GRAVITY_CM_S2;
    accelYBody = (ayRaw / MPU6050_ACCEL_LSB_PER_G) * GRAVITY_CM_S2;
    DataStruct->AccelBodyX = accelXBody;
    DataStruct->AccelBodyY = accelYBody;

    accelNormG = sqrtf(((float)axRaw * (float)axRaw) +
                       ((float)ayRaw * (float)ayRaw) +
                       ((float)azRaw * (float)azRaw)) / MPU6050_ACCEL_LSB_PER_G;
    accelXYNorm = sqrtf(accelXBody * accelXBody + accelYBody * accelYBody);
    stillCandidate = (uint8_t)((fabsf(DataStruct->GyroZ) < MPU6050_STILL_GYRO_DPS_TH) &&
                               (accelXYNorm < MPU6050_STILL_ACC_XY_CM_S2_TH) &&
                               (fabsf(accelNormG - 1.0f) < MPU6050_STILL_ACC_NORM_G_TH));

    if (stillCandidate != 0U) {
        if (DataStruct->StationaryCount < MPU6050_STILL_CONFIRM_SAMPLES) {
            DataStruct->StationaryCount++;
        }
    } else {
        DataStruct->StationaryCount = 0U;
    }

    DataStruct->IsStationary = (uint8_t)(DataStruct->StationaryCount >= MPU6050_STILL_CONFIRM_SAMPLES);

    yawRad = DataStruct->Yaw * DEG_TO_RAD;
    accelXWorld = accelXBody * cosf(yawRad) - accelYBody * sinf(yawRad);
    accelYWorld = accelXBody * sinf(yawRad) + accelYBody * cosf(yawRad);

    if (fabsf(accelXWorld) < MPU6050_ACCEL_DEADBAND_CM_S2) {
        accelXWorld = 0.0f;
    }
    if (fabsf(accelYWorld) < MPU6050_ACCEL_DEADBAND_CM_S2) {
        accelYWorld = 0.0f;
    }

    DataStruct->AccelX = accelXWorld;
    DataStruct->AccelY = accelYWorld;

    if (DataStruct->IsStationary != 0U) {
        DataStruct->VelX = 0.0f;
        DataStruct->VelY = 0.0f;

        DataStruct->GyroZ_Offset = (int32_t)((1.0f - MPU6050_BIAS_LEARN_ALPHA) * DataStruct->GyroZ_Offset +
                                             MPU6050_BIAS_LEARN_ALPHA * DataStruct->GyroZ_Raw);
        DataStruct->AccelX_Offset = (int32_t)((1.0f - MPU6050_BIAS_LEARN_ALPHA) * DataStruct->AccelX_Offset +
                                              MPU6050_BIAS_LEARN_ALPHA * DataStruct->AccelX_Raw);
        DataStruct->AccelY_Offset = (int32_t)((1.0f - MPU6050_BIAS_LEARN_ALPHA) * DataStruct->AccelY_Offset +
                                              MPU6050_BIAS_LEARN_ALPHA * DataStruct->AccelY_Raw);
        DataStruct->AccelZ_Offset = (int32_t)((1.0f - MPU6050_BIAS_LEARN_ALPHA) * DataStruct->AccelZ_Offset +
                                              MPU6050_BIAS_LEARN_ALPHA * (DataStruct->AccelZ_Raw - (int16_t)MPU6050_ACCEL_LSB_PER_G));
    } else {
        DataStruct->PosX += DataStruct->VelX * dt + 0.5f * accelXWorld * dt * dt;
        DataStruct->PosY += DataStruct->VelY * dt + 0.5f * accelYWorld * dt * dt;
        DataStruct->VelX += accelXWorld * dt;
        DataStruct->VelY += accelYWorld * dt;
    }
}

MPU6050_Data_t mpu_data = {0};

void MPU6050Task(void *argument)
{
    (void)argument;

    MPU6050_Init();
    osDelay(50);

    MPU6050_Calibrate(&mpu_data);
    mpu_data.LastTick = osKernelGetTickCount();

    for (;;) {
        MPU6050_ReadAll(&mpu_data);
        MPU6050_Update(&mpu_data);
        osDelay(10);
    }
}
