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

void UARTComms_Transmmit_Data(UART_HandleTypeDef *UARTPort,uint8_t cmd,uint8_t *data,uint8_t len)
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
        UARTComms_BT24Receive_Data(&UARTComms_BT24_Port,BT24receiveBuffer,sizeof(BT24receiveBuffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_BT24_Port, BT24receiveBuffer, sizeof(BT24receiveBuffer));
    }
}

void UARTComms_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_BT24_Port, BT24receiveBuffer, sizeof(BT24receiveBuffer));
}

void UARTComms_Task(void *argument)
{
    UARTComms_Init();

    float alpha=0, beta=0;
    uint32_t flags;

    while(1)
    {
        flags = osThreadFlagsWait(COMMS_SIGNAL_RECEIVED, osFlagsWaitAny, osWaitForever);

        if (flags & COMMS_SIGNAL_RECEIVED)
        {
            if(receivedCMD == 0x01)//转向
            {
                memcpy(&alpha, &receivedData[0], 4);//角度（单位：度）
                memcpy(&beta, &receivedData[4], 4);//旋转速度（单位：度/s）
                
            }
            else if(receivedCMD == 0x02)//平移
            {
                memcpy(&alpha, &receivedData[0], 4);//x距离（单位：cm）
                memcpy(&beta, &receivedData[4], 4);//y距离（单位：cm）
                Encoder_FL = 0;
                Encoder_FR = 0;
                UARTComms_Transmmit_Data(&UARTComms_Port, 0x2A, 0x00, 1);//重置编码器
            }
            else if(receivedCMD == 0x03)//设置巡航速度
            {
                memcpy(&alpha, &receivedData[0], 4);//x速度（单位：cm/s）
                memcpy(&beta, &receivedData[4], 4);//y速度（单位：cm/s）
            }
            else if(receivedCMD == 0x04)//设置旋转速度
            {
                memcpy(&alpha, &receivedData[0], 4);//旋转速度（单位：度/s）
            }
            else if(receivedCMD == 0xB1)//遥控模式
            {
                memcpy(&Remote_Control_Input->Forward, &receivedData[0], 1);
                memcpy(&Remote_Control_Input->Backward, &receivedData[1], 1);
                memcpy(&Remote_Control_Input->Left, &receivedData[2], 1);
                memcpy(&Remote_Control_Input->Right, &receivedData[3], 1);
                memcpy(&Remote_Control_Input->RotateLeft, &receivedData[4], 1);
                memcpy(&Remote_Control_Input->RotateRight, &receivedData[5], 1);
                Car_Control.mode = CAR_CONTROL_COAST;
            }
            //Car_UpdateTarget(receivedCMD, alpha, beta);
            receivedCMD = 0;
        }
    }
}
