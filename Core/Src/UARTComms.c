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

void UARTComms_Transmmit_Data(UART_HandleTypeDef *UARTPort,uint8_t cmd,uint8_t *data,uint8_t len)
{
	globalBuffer[0]=cmd;
	memcpy(&globalBuffer[1], data, len);
	CRC08_Append(globalBuffer, len + 2);
	memmove(globalBuffer+2, globalBuffer, len + 2);
	globalBuffer[0] = PackageHead1;
	globalBuffer[1] = PackageHead2;
	HAL_UART_Transmit_DMA(&UARTPort,globalBuffer,len+4);
}

void UARTComms_Recieve_Data(UART_HandleTypeDef *UARTPort,uint8_t *received,uint8_t len)
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
            char errorMsg[] = "Verification Error\r\n";
            HAL_UART_Transmit(&UARTPort, (uint8_t*)errorMsg, sizeof(errorMsg) - 1, HAL_MAX_DELAY);
            return;}
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *UARTPort,UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&UARTComms_Port)
	{
		UARTComms_Recieve_Data(receiveBuffer,Size);

		HAL_UARTEx_ReceiveToIdle_DMA(&UARTPort, receiveBuffer, sizeof(receiveBuffer));
	}
}


void UARTComms_Init(UART_HandleTypeDef *UARTPort)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&UARTPort, receiveBuffer, sizeof(receiveBuffer));
}
