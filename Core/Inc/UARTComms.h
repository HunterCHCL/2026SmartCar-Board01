/*
 * UARTComms.h
 *
 *  Created on: Mar 19, 2026
 *      Author: HunterCHCL
 */

#ifndef INC_UARTCOMMS_H_
#define INC_UARTCOMMS_H_

#include "main.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "verification.h"
#include "WheelControls.h"

#define PackageHead1 0xFA
#define PackageHead2 0xAF

#define UARTComms_Port huart2
#define UARTComms_BT24_Port huart1

extern uint8_t receivedData[48];
extern uint8_t receivedCMD;
extern uint8_t BT24receivedData[48];
extern uint8_t BT24receivedCMD;

void UARTComms_Transmmit_Data(UART_HandleTypeDef *UARTPort,uint8_t cmd,uint8_t *data,uint8_t len);
void UARTComms_Init(void);

#endif /* INC_UARTCOMMS_H_ */
