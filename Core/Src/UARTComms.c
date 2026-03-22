/*
 * UARTComms.c
 *
 *  Created on: Mar 19, 2026
 *      Author: HunterCHCL
 */
#include "verification.h"

uint8_t receivebuffer[50];
uint8_t globaBuffer[50];
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
	CRC08_Append(globalBuffer, len + 2);
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
