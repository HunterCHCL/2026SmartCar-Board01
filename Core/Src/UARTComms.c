/*
 * UARTComms.c
 *
 *  Created on: Mar 19, 2026
 *      Author: HunterCHCL
 */

#include "UARTComms.h"


extern osThreadId_t CommsHandle;

#define COMMS_SIGNAL_RECEIVED 0x01

uint8_t receiveBuffer[50];
uint8_t globalBuffer[50];
 uint8_t receivedData[48];
 uint8_t receivedCMD;
uint8_t BT24receiveBuffer[50];
uint8_t BT24globalBuffer[50];
uint8_t BT24receivedData[48];
uint8_t BT24receivedCMD;

void UARTComms_Transmit_Data(UART_HandleTypeDef *UARTPort,uint8_t cmd,uint8_t *data,uint8_t len)
{
	globalBuffer[0]=cmd;
	memcpy(&globalBuffer[1], data, len);
	Verification_AddXOR(globalBuffer, len+1);
	memmove(globalBuffer+2, globalBuffer, len + 2);
	globalBuffer[0] = PackageHead1;
	globalBuffer[1] = PackageHead2;
	HAL_UART_Transmit_DMA(UARTPort,globalBuffer,len+4);
}

 void UARTComms_Receive_Data(UART_HandleTypeDef *UARTPort,uint8_t *received,uint8_t len)
 {
     if(received[0]==PackageHead1&&received[1]==PackageHead2)
     {
         // 剥离包头
         memmove(received,received+2,len-2);
         len-=2;
         if(Verification_CheckXOR(received, len))
         {
             receivedCMD=received[0];
             // 复制数据
             memcpy(receivedData, received+1, len-2);
         }
         else{
             uint8_t errorMsg[] = "Verification Error\r\n";
             HAL_UART_Transmit(UARTPort, errorMsg, sizeof(errorMsg) - 1, HAL_MAX_DELAY);
             return;}
     }
 }

void UARTComms_BT24Receive_Data(UART_HandleTypeDef *UARTPort,uint8_t *received,uint8_t len)
{
    if(received[0]==PackageHead1&&received[1]==PackageHead2)
    {
        // 剥离包头
        memmove(received,received+2,len-2);
        len-=2;
        if(Verification_CheckXOR(received, len))
        {
            BT24receivedCMD=received[0];
            // 复制数据
            memcpy(BT24receivedData, received+1, len-2);
        }
        else{
            uint8_t errorMsg[] = "Verification Error\r\n";
            HAL_UART_Transmit(UARTPort, errorMsg, sizeof(errorMsg) - 1, HAL_MAX_DELAY);
            return;}
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&UARTComms_Port)
	{
		UARTComms_Receive_Data(&UARTComms_Port, receiveBuffer, Size);
        if (CommsHandle != NULL) {
            osThreadFlagsSet(CommsHandle, COMMS_SIGNAL_RECEIVED);
        }
		HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
	}
    else if(huart==&UARTComms_BT24_Port)
    {
        UARTComms_BT24Receive_Data(&UARTComms_BT24_Port,BT24receiveBuffer,Size);
        if (CommsHandle != NULL) {
        	osThreadFlagsSet(CommsHandle, COMMS_SIGNAL_RECEIVED);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_BT24_Port, BT24receiveBuffer, sizeof(BT24receiveBuffer));
    }
}

void UARTComms_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_BT24_Port, BT24receiveBuffer, sizeof(BT24receiveBuffer));
}

void CommsTask(void *argument)
{
    UARTComms_Init();

    float alpha=0, beta=0;
    uint32_t flags;
    uint8_t buffer[]={'h','e','l','l','o'};
    UARTComms_Transmit_Data(&UARTComms_BT24_Port, 0x01, buffer, sizeof(buffer));
    while(1)
    {
        flags = osThreadFlagsWait(COMMS_SIGNAL_RECEIVED, osFlagsWaitAny, osWaitForever);

        if (flags & COMMS_SIGNAL_RECEIVED)
        {
            if(BT24receivedCMD == 0x01)//转向
            {
                memcpy(&alpha, &BT24receivedData[0], 4);//角度（单位：度）
                memcpy(&beta, &BT24receivedData[4], 4);//旋转速度（单位：度/s）
                Car_Control.target_yawAngle = alpha;
                Car_Control.target_rotationSpeed = beta;
                Car_Control.mode = CAR_CONTROL_TURN;
                MPU6050_yaw = 0.0f;//重置陀螺仪角度
            }
            else if(BT24receivedCMD == 0x02)//平移
            {
                memcpy(&alpha, &BT24receivedData[0], 4);//x距离（单位：cm）
                memcpy(&beta, &BT24receivedData[4], 4);//y距离（单位：cm）
                // Encoder_FL = 0;
                // Encoder_FR = 0;
                Motor_FL.current_position = 0.0f;
                Motor_FR.current_position = 0.0f;
                Car_Control.target_position.x = alpha;
                Car_Control.target_position.y = beta;
                Car_Control.mode = CAR_CONTROL_MOVE;
                // UARTComms_Transmit_Data(&UARTComms_Port, 0x2A, 0x00, 1);//重置编码器
            }
            else if(BT24receivedCMD == 0xB1)//遥控模式
            {
                memcpy(&Remote_Control_Input.Forward, &BT24receivedData[0], 1);
                memcpy(&Remote_Control_Input.Left, &BT24receivedData[1], 1);
                memcpy(&Remote_Control_Input.Backward, &BT24receivedData[2], 1);
                memcpy(&Remote_Control_Input.Right, &BT24receivedData[3], 1);
                memcpy(&Remote_Control_Input.RotateLeft, &BT24receivedData[4], 1);
                memcpy(&Remote_Control_Input.RotateRight, &BT24receivedData[5], 1);
                Car_Control.mode = CAR_CONTROL_COAST;
                Car_RemoteControl(&Remote_Control_Input);
            }
            else if(BT24receivedCMD == 0xB2)//遥控终止
            {
                Remote_Control_Input.Forward=0;
                Remote_Control_Input.Left=0;
                Remote_Control_Input.Backward=0;
                Remote_Control_Input.Right=0;
                Remote_Control_Input.RotateLeft=0;
                Remote_Control_Input.RotateRight=0;
                Car_Control.mode = CAR_CONTROL_COAST;
                Car_RemoteControl(&Remote_Control_Input);
            }
            else if(BT24receivedCMD == 0xB3)
            {
                memcpy(&Remote_Control_Input.Forward, &BT24receivedData[0], 1);
                memcpy(&Remote_Control_Input.Left, &BT24receivedData[1], 1);
                memcpy(&Remote_Control_Input.Backward, &BT24receivedData[2], 1);
                memcpy(&Remote_Control_Input.Right, &BT24receivedData[3], 1);
                memcpy(&Car_Control.target_rotationSpeed, &BT24receivedData[4], 4);
                Car_Control.mode = CAR_CONTROL_ROTATING_COAST;
                Car_RemoteControl(&Remote_Control_Input);
            }
            else if(BT24receivedCMD == 0xC1)
            {
                memcpy(&alpha, &BT24receivedData[0], 4);
                memcpy(&beta, &BT24receivedData[4], 4);
                Motor_FL.target_velocity = WHELL_FL_DIR*alpha;
                Motor_FR.target_velocity = WHELL_FR_DIR*beta;
                Motor_FL.mode = MOTOR_CONTROL_VELOCITY;
                Motor_FR.mode = MOTOR_CONTROL_VELOCITY;
            }
            else if(BT24receivedCMD == 0xC2)
            {
                memcpy(&alpha, &BT24receivedData[0], 4);
                memcpy(&beta, &BT24receivedData[4], 4);
                Motor_FL.current_position = 0.0f;
                Motor_FR.current_position = 0.0f;
                Motor_FL.target_position = WHELL_FL_DIR*alpha;
                Motor_FR.target_position = WHELL_FR_DIR*beta;
                Motor_FL.mode = MOTOR_CONTROL_POSITION;
                Motor_FR.mode = MOTOR_CONTROL_POSITION;
            }
            else if(BT24receivedCMD == 0xD1)
            {
                memcpy(&alpha, &BT24receivedData[0], 4);
                memcpy(&beta, &BT24receivedData[4], 4);
                Motor_FL.target_velocity = WHELL_FL_DIR*alpha;
                Motor_FR.target_velocity = WHELL_FR_DIR*alpha;
                Motor_RL.target_velocity = WHELL_RL_DIR*beta;
                Motor_RR.target_velocity = WHELL_RR_DIR*beta;
                Motor_FL.mode = MOTOR_CONTROL_VELOCITY;
                Motor_FR.mode = MOTOR_CONTROL_VELOCITY;
                Motor_RL.mode = MOTOR_CONTROL_VELOCITY;
                Motor_RR.mode = MOTOR_CONTROL_VELOCITY;
            }
            else if(BT24receivedCMD == 0xD2)
            {
                memcpy(&alpha, &BT24receivedData[0], 4);
                memcpy(&beta, &BT24receivedData[4], 4);
                Motor_FL.current_position = 0.0f;
                Motor_FR.current_position = 0.0f;
                Motor_RL.current_position = 0.0f;
                Motor_RR.current_position = 0.0f;
                Motor_FL.target_position = WHELL_FL_DIR*alpha;
                Motor_FR.target_position = WHELL_FR_DIR*alpha;
                Motor_RL.target_position = WHELL_RL_DIR*beta;
                Motor_RR.target_position = WHELL_RR_DIR*beta;
                Motor_FL.mode = MOTOR_CONTROL_POSITION;
                Motor_FR.mode = MOTOR_CONTROL_POSITION;
                Motor_RL.mode = MOTOR_CONTROL_POSITION;
                Motor_RR.mode = MOTOR_CONTROL_POSITION;
            }
            else if(BT24receivedCMD == 0xE1)
            {
                UARTComms_Transmit_Data(&UARTComms_Port, 0x1A,BT24receivedData, 8);
            }
            else if(BT24receivedCMD == 0xE2)
            {
                UARTComms_Transmit_Data(&UARTComms_Port, 0x1B, BT24receivedData, 8);
            }
            BT24receivedCMD = 0;
            if(receivedCMD == 0xF1)
            {
                UARTComms_Transmit_Data(&UARTComms_BT24_Port, 0xF1, receivedData, 8);
            }
            
        }
    }
}
