/*
 * MPU6050.c
 *
 *  Created on: Feb 24, 2026
 *      Author: HunterCHCL
 */
#include "MPU6050.h"

uint8_t sendBuffer[2];

static void MPU6050_WriteReg(uint8_t RegAdress, uint8_t Data)
{
    uint8_t pBuffer[2] = {RegAdress, Data};
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, pBuffer, 2, HAL_MAX_DELAY);
}

static void MPU6050_ReadReg(uint8_t RegAdress, uint8_t *Data)
{
    HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050Addr, &RegAdress, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050Addr, Data, 1, HAL_MAX_DELAY);
}

static void MPU6050_ReadMultiReg(uint8_t RegAdress, uint8_t *Data, uint16_t Length)
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
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		// 100Hz
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			// 配置寄存器，DLPF 5Hz
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	// 陀螺仪量程 ±2000dps
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	// 加速度计量程 ±16g
}

void MPU6050_ReadYaw(float *gyroZ)
{
    uint8_t data[2];
    int16_t gyroZ_raw;
    
    // 连续读取 Z 轴的高 8 位和低 8 位数据
    MPU6050_ReadMultiReg(MPU6050_GYRO_ZOUT_H, data, 2);
    gyroZ_raw = (int16_t)(data[0] << 8 | data[1]);
    
    // 转换为实际的角速度(度/秒)。在此前配置量程为 ±2000dps 时，灵敏度为 16.4 LSB/(°/s)
    *gyroZ = gyroZ_raw / MPU6050_Gyro_Sensitivity_2000DPS; 
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
float MPU6050_Accel_To_G(int16_t AccelRaw, float sensitivity)
{
    return AccelRaw / sensitivity;
}

/*
@brief  转换陀螺仪原始值为dps (±2000dps范围)
@param  GyroRaw 陀螺仪原始值
@return 转换后的dps值
*/
float MPU6050_Gyro_To_DegPerSec(int16_t GyroRaw, float sensitivity)
{
    return GyroRaw / sensitivity;
}

void MPU6050_CalibrateGyroZ(float *GyroZ_Offset)
{
    int32_t gz = 0;
    const int num_samples = 100;

    for(int i = 0; i < num_samples; i++)
    {
        uint8_t data[2];
        int16_t gyroZ_raw;
        MPU6050_ReadMultiReg(MPU6050_GYRO_ZOUT_H, data, 2);
        gyroZ_raw = (int16_t)(data[0] << 8 | data[1]);
        gz += gyroZ_raw;
        HAL_Delay(10);
    }

    *GyroZ_Offset = (gz / (float)num_samples) / MPU6050_Gyro_Sensitivity_2000DPS;
}

void MPU6050_ProcessYaw(float GyroZ, float *yaw, float GyroZ_Offset)
{
    static uint32_t last_time = 0;
    
    if (last_time == 0) {
        last_time = HAL_GetTick();
        return;
    }
    
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f; // 获取真实的系统差值时间(秒)
    last_time = now;
    
    // 获取扣除零偏后的真实角速度
    float true_rate = GyroZ - GyroZ_Offset;
    
    // 设置一个死区（例如 ±0.5 度/秒），消除由于微小噪声引起的静止漂移
    if (true_rate > -0.5f && true_rate < 1.0f) {
        true_rate = 0;
    }
    
    *yaw += true_rate * dt;
}
float MPU6050_yaw = 0.0f;
float MPU6050_GyroZ_Offset = 0.0f;

void MPU6050Task(void *argument)
{
    MPU6050_Init();
    MPU6050_CalibrateGyroZ(&MPU6050_GyroZ_Offset);
    OLED_Init();
    float GyroZ;
    while(1)
    {
        MPU6050_ReadYaw(&GyroZ);
        MPU6050_ProcessYaw(GyroZ, &MPU6050_yaw,MPU6050_GyroZ_Offset);
        OLED_ShowFloat(1, 1, MPU6050_yaw, 2);
        osDelay(MPU6050_CYCLE_TIME); // 100Hz
    }
}
