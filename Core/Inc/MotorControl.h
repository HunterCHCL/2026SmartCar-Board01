/*
 * MotorControl.h
 *
 *  Created on: Mar 21, 2026
 *      Author: HunterCHCL
 */
#include "pid.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#define Motor_Speed_Limit 30.0f // cm/s
#define Motor_Position_I_Limit 15.0f
#define Motor_Power_Limit 144.0f// 100% PWM
#define Motor_Velocity_I_Limit 40.0f

#define Motor_Transmission_Ratio 34
#define Motor_Encoder_Resolution 13
#define Motor_Encoder_Mode 4 //TI1&TI2
#define Motor_Pulses_Per_Revolution (Motor_Transmission_Ratio * Motor_Encoder_Resolution * Motor_Encoder_Mode)
#define Motor_Wheel_Diameter 0.06f // 60mm
#define Motor_Pulse_To_Distance (314.15926f * Motor_Wheel_Diameter / Motor_Pulses_Per_Revolution)//cm/pulse

#define CycleTime 0.01f // 10ms

#define Motor_FL_Encoder_DIR_FACTOR 1
#define Motor_FR_Encoder_DIR_FACTOR 1

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



typedef enum
{
    Motor_Forward,
    Motor_Backward,
    Motor_Brake,
    Motor_Coast
} Motor_State;

typedef enum
{
    Motor_FL_ID,
    Motor_FR_ID,
    Motor_RL_ID,
    Motor_RR_ID
}Motor_ID;

typedef struct
{
    pid_type_def position_pid;
    pid_type_def velocity_pid;
    float target_position;//cm
    float current_position;
    float target_velocity;//cm/s
    float current_velocity;
    MotorControlMode mode;
    uint32_t last_time;
} Motor_HandleTypeDef;

typedef enum
{
    MOTOR_CONTROL_POSITION,
    MOTOR_CONTROL_VELOCITY,
} MotorControlMode;

extern Motor_HandleTypeDef Motor_FL;
extern Motor_HandleTypeDef Motor_FR;
extern Motor_HandleTypeDef Motor_RL;
extern Motor_HandleTypeDef Motor_RR;

extern int32_t Encoder_FL;
extern int32_t Encoder_FR;
/*
extern int32_t Encoder_RL;
extern int32_t Encoder_RR;
*/

void MotorControl_Init(void);
void Motor_SetState(Motor_ID motor_id, Motor_State state);
void Motor_SetTargetPosition(Motor_ID motor_id, float target_position);
void Motor_FL_Update(void);
void Motor_FR_Update(void);
/*
void Motor_RL_Update(void);
void Motor_RR_Update(void);
*/
float Motor_FL_ReadEncoder(void);
float Motor_FR_ReadEncoder(void);
/*
float Motor_RL_ReadEncoder(void);
float Motor_RR_ReadEncoder(void);
*/

void MotorTask(void const * argument);

#endif /* INC_MOTORCONTROL_H_ */
