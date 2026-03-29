/*
 * MotorControl.h
 *
 *  Created on: Mar 21, 2026
 *      Author: HunterCHCL
 */
#include "pid.h"

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#define Motor_Transmission_Ratio 34
#define Motor_Encoder_Resolution 13

#define Motor_FL_PWM TIM_CHANNEL_1
#define Motor_FR_PWM TIM_CHANNEL_2

#define Motor_FL_PWM_TIMEBASE htim1
#define Motor_FR_PWM_TIMEBASE htim1

#define Motor_FL_IN1_GPIO_Port GPIOB
#define Motor_FL_IN2_GPIO_Port GPIOB
#define Motor_FR_IN1_GPIO_Port GPIOB
#define Motor_FR_IN2_GPIO_Port GPIOB

#define Motor_FL_IN1_Pin GPIO_PIN_15
#define Motor_FL_IN2_Pin GPIO_PIN_14
#define Motor_FR_IN1_Pin GPIO_PIN_13
#define Motor_FR_IN2_Pin GPIO_PIN_12

#define Motor_FL_Encoder_Timebase htim2
#define Motor_FR_Encoder_Timebase htim3

#define Motor_FL_Encoder_A_Pin GPIO_PIN_0
#define Motor_FL_Encoder_B_Pin GPIO_PIN_1
#define Motor_FR_Encoder_A_Pin GPIO_PIN_2
#define Motor_FR_Encoder_B_Pin GPIO_PIN_3

/*
#define Motor_RL_PWM TIM_CHANNEL_3
#define Motor_RR_PWM TIM_CHANNEL_4

#define Motor_RL_PWM_TIMEBASE htim3
#define Motor_RR_PWM_TIMEBASE htim3

#define Motor_RL_IN1_GPIO_Port GPIOB
#define Motor_RL_IN2_GPIO_Port GPIOB
#define Motor_RR_IN1_GPIO_Port GPIOB
#define Motor_RR_IN2_GPIO_Port GPIOA

#define Motor_RL_IN1_Pin GPIO_PIN_2
#define Motor_RL_IN2_Pin GPIO_PIN_1
#define Motor_RR_IN1_Pin GPIO_PIN_0
#define Motor_RR_IN2_Pin GPIO_PIN_7

#define Motor_RL_Encoder_A_Port GPIOA
#define Motor_RL_Encoder_B_Port GPIOA
#define Motor_RR_Encoder_A_Port GPIOA
#define Motor_RR_Encoder_B_Port GPIOA

#define Motor_RL_Encoder_A_Pin GPIO_PIN_4
#define Motor_RL_Encoder_B_Pin GPIO_PIN_5
#define Motor_RR_Encoder_A_Pin GPIO_PIN_6
#define Motor_RR_Encoder_B_Pin GPIO_PIN_7
*/
typedef enum
{
    Motor_Forward,
    Motor_Backward,
    Motor_Brake,
    Motor_Coast
} Motor_State;

typedef struct
{
    pid_type_def velocity_pid;
    fp32 target_velocity;
    fp32 current_velocity;
} Motor_HandleTypeDef;

extern Motor_HandleTypeDef Motor_FL;
extern Motor_HandleTypeDef Motor_FR;
// extern Motor_HandleTypeDef Motor_RL;
// extern Motor_HandleTypeDef Motor_RR;

void MotorControl_Init(void);
void Motor_FL_SetState(Motor_State state);
void Motor_FR_SetState(Motor_State state);
void Motor_FL_SetVelocity(float target_velocity);
void Motor_FR_SetVelocity(float target_velocity);
void Motor_FL_Update(float current_velocity);
void Motor_FR_Update(float current_velocity);
float Motor_FL_ReadEncoder(void);
float Motor_FR_ReadEncoder(void);

// void Motor_RL_SetState(Motor_State state);
// void Motor_RR_SetState(Motor_State state);
// void Motor_RL_SetVelocity(float target_velocity);
// void Motor_RR_SetVelocity(float target_velocity);
// void Motor_RL_Update(float current_velocity);
// void Motor_RR_Update(float current_velocity);

#endif /* INC_MOTORCONTROL_H_ */
