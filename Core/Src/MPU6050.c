/*
 * MPU6050.c
 *
 *  Created on: Feb 24, 2026
 *      Author: HunterCHCL
 */
#define MPU6050Addr 0xD0
#include "i2c.h"
#define MPU6050_I2C hi2c1
#include "MPU6050.h"

uint8_t sendBuffer[2];

/*
@brief  MPU6050写寄存器
@param  RegAdress 寄存器地址
@param  Data 要写入寄存器的数据
*/
void MPU6050_WriteReg(uint8_t RegAdress, uint8_t Data)
{
    uint8_t pBuffer[2] = {RegAdress, Data};
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, pBuffer, 2, HAL_MAX_DELAY);
}

/*
@brief  MPU6050读寄存器
@param  RegAdress 寄存器地址
@param  Data 读取寄存器的数据指针
*/
void MPU6050_ReadReg(uint8_t RegAdress, uint8_t *Data)
{
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, &RegAdress, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050Addr, Data, 1, HAL_MAX_DELAY);
}

/*
@brief  MPU6050连续读寄存器
@param  RegAdress 寄存器起始地址
@param  Data 数据存储指针
@param  Length 读取长度
*/
void MPU6050_ReadMultiReg(uint8_t RegAdress, uint8_t *Data, uint16_t Length)
{
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, &RegAdress, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050Addr, Data, Length, HAL_MAX_DELAY);
}

/*
@brief  MPU6050初始化
@return 无
*/
void MPU6050_Init(void)
{
    HAL_Delay(100);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);     // 电源管理寄存器1，取消睡眠模式，时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		// 电源管理寄存器2，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		// 采样率分频，100Hz
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			// 配置寄存器，DLPF 5Hz
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	// 陀螺仪满量程 ±2000dps
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	// 加速度计满量程 ±16g
}

/*
@brief  读取MPU6050原始数据
@param  DataStruct MPU6050的数据结构体指针
@return 无
*/
void MPU6050_ReadAll(MPU6050_Data_t *DataStruct)
{
    uint8_t data[14];
    MPU6050_ReadMultiReg(MPU6050_ACCEL_XOUT_H, data, 14);

    DataStruct->AccelX_Raw = (int16_t)(data[0] << 8 | data[1]);
    DataStruct->AccelY_Raw = (int16_t)(data[2] << 8 | data[3]);
    DataStruct->AccelZ_Raw = (int16_t)(data[4] << 8 | data[5]);
    DataStruct->GyroX_Raw = (int16_t)(data[8] << 8 | data[9]);
    DataStruct->GyroY_Raw = (int16_t)(data[10] << 8 | data[11]);
    DataStruct->GyroZ_Raw = (int16_t)(data[12] << 8 | data[13]);
}

/*
@brief  直接获取加速度和陀螺仪原始数据
@param  AccX 加速度X轴原始数据指针
@param  AccY 加速度Y轴原始数据指针
@param  AccZ 加速度Z轴原始数据指针
@param  GyroX 陀螺仪X轴原始数据指针
@param  GyroY 陀螺仪Y轴原始数据指针
@param  GyroZ 陀螺仪Z轴原始数据指针
@return 无
*/
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t data[14];
    MPU6050_ReadMultiReg(MPU6050_ACCEL_XOUT_H, data, 14);
    *AccX = (int16_t)(data[0] << 8 | data[1]);
    *AccY = (int16_t)(data[2] << 8 | data[3]);
    *AccZ = (int16_t)(data[4] << 8 | data[5]);
    *GyroX = (int16_t)(data[8] << 8 | data[9]);
    *GyroY = (int16_t)(data[10] << 8 | data[11]);
    *GyroZ = (int16_t)(data[12] << 8 | data[13]);
}

/*
@brief  转换加速度原始值为g (±16g范围)
@param  AccelRaw 加速度原始值
@return 转换后的g值
*/
float MPU6050_Accel_To_G_16G(int16_t AccelRaw)
{
    return AccelRaw / 2048.0f;
}

/*
@brief  转换陀螺仪原始值为dps (±2000dps范围)
@param  GyroRaw 陀螺仪原始值
@return 转换后的dps值
*/
float MPU6050_Gyro_To_DegPerSec_2000(int16_t GyroRaw)
{
    return GyroRaw / 16.4f;
}


/*
@brief  校准函数，静止放置时调用以更新MPU6050数据结构体中的偏移量
@param  DataStruct MPU6050的数据结构体指针
@return 无
*/
void MPU6050_Calibrate(MPU6050_Data_t *DataStruct)
{
    int32_t ax = 0, ay = 0, az = 0;
    int32_t gx = 0, gy = 0, gz = 0;
    const int num_samples = 100;

    for(int i = 0; i < num_samples; i++)
    {
        MPU6050_ReadAll(DataStruct);
        ax += DataStruct->AccelX_Raw;
        ay += DataStruct->AccelY_Raw;
        az += DataStruct->AccelZ_Raw;
        gx += DataStruct->GyroX_Raw;
        gy += DataStruct->GyroY_Raw;
        gz += DataStruct->GyroZ_Raw;
        HAL_Delay(10);
    }

    DataStruct->AccelX_Offset = ax / num_samples;
    DataStruct->AccelY_Offset = ay / num_samples;
    DataStruct->AccelZ_Offset = (az / num_samples) - 2048; // ±16g模式下，1g = 2048 LSB
    DataStruct->GyroX_Offset = gx / num_samples;
    DataStruct->GyroY_Offset = gy / num_samples;
    DataStruct->GyroZ_Offset = gz / num_samples;
}

/*
@brief  处理数据并解算姿态角
@param  DataStruct MPU6050的数据结构体指针，包含原始数据、偏移量和计算结果
@return 无
*/
void MPU6050_ProcessData(MPU6050_Data_t *DataStruct)
{
    // 1. 减去零偏后的原始数据
    int16_t ax = DataStruct->AccelX_Raw - DataStruct->AccelX_Offset;
    int16_t ay = DataStruct->AccelY_Raw - DataStruct->AccelY_Offset;
    int16_t az = DataStruct->AccelZ_Raw - DataStruct->AccelZ_Offset;
    int16_t gx = DataStruct->GyroX_Raw - DataStruct->GyroX_Offset;
    int16_t gy = DataStruct->GyroY_Raw - DataStruct->GyroY_Offset;
    int16_t gz = DataStruct->GyroZ_Raw - DataStruct->GyroZ_Offset;

    // 2. 转换为物理单位
    // ±16g -> 2048 LSB/g
    DataStruct->AccelX = ax / 2048.0f;
    DataStruct->AccelY = ay / 2048.0f;
    DataStruct->AccelZ = az / 2048.0f;

    // ±2000dps -> 16.4 LSB/dps
    DataStruct->GyroX = gx / 16.4f;
    DataStruct->GyroY = gy / 16.4f;
    DataStruct->GyroZ = gz / 16.4f;

    // 3. 姿态解算 (互补滤波)
    float dt = 0.01f; // 100Hz采样率
    float alpha = 0.98f; // 互补系数

    // 加速度计计算的角度项
    float accelPitch = atan2f(DataStruct->AccelY, DataStruct->AccelZ) * 57.29578f; // 180/PI
    float accelRoll = atan2f(-DataStruct->AccelX, sqrtf(DataStruct->AccelY * DataStruct->AccelY + DataStruct->AccelZ * DataStruct->AccelZ)) * 57.29578f;

    // 记录上一时刻的角度用于计算变化量 (用于累计)
    float prevPitch = DataStruct->Pitch;
    float prevRoll = DataStruct->Roll;

    // 互补滤波融合 (标准角度解算，通常在 ±180° 范围内)
    DataStruct->Pitch = alpha * (DataStruct->Pitch + DataStruct->GyroX * dt) + (1.0f - alpha) * accelPitch;
    DataStruct->Roll = alpha * (DataStruct->Roll + DataStruct->GyroY * dt) + (1.0f - alpha) * accelRoll;
    
    // 处理角度突变 (跨越 ±180°)，并累加到总角度中
    float dPitch = DataStruct->Pitch - prevPitch;
    float dRoll = DataStruct->Roll - prevRoll;

    // 如果变化量超过 180 度，说明发生了回绕
    if (dPitch > 180.0f) dPitch -= 360.0f;
    else if (dPitch < -180.0f) dPitch += 360.0f;

    if (dRoll > 180.0f) dRoll -= 360.0f;
    else if (dRoll < -180.0f) dRoll += 360.0f;

    // Yaw轴仅靠积分，由于没有磁力计，随着时间会有漂移
    DataStruct->Yaw += DataStruct->GyroZ * dt;
}