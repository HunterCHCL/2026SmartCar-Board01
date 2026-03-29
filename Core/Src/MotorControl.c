/*
 * MotorControl.c
 *
 *  Created on: Mar 21, 2026
 *      Author: HunterCHCL
 */

#include "MotorControl.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>

Motor_HandleTypeDef Motor_FL;
Motor_HandleTypeDef Motor_FR;
// Motor_HandleTypeDef Motor_RL;
// Motor_HandleTypeDef Motor_RR;

void MotorControl_Init(void)
{
    const fp32 pid_params[3] = {10.0f, 0.5f, 0.0f}; // Adjust these placeholder values later
    PID_init(&Motor_FL.velocity_pid, PID_POSITION, pid_params, 1000.0f, 500.0f);
    PID_init(&Motor_FR.velocity_pid, PID_POSITION, pid_params, 1000.0f, 500.0f);
    // PID_init(&Motor_RL.velocity_pid, PID_POSITION, pid_params, 1000.0f, 500.0f);
    // PID_init(&Motor_RR.velocity_pid, PID_POSITION, pid_params, 1000.0f, 500.0f);

    HAL_TIM_PWM_Start(&Motor_FL_PWM_TIMEBASE, Motor_FL_PWM);
    HAL_TIM_PWM_Start(&Motor_FR_PWM_TIMEBASE, Motor_FR_PWM);
    HAL_TIM_Encoder_Start(&Motor_FL_Encoder_Timebase, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Motor_FR_Encoder_Timebase, TIM_CHANNEL_ALL);
    // HAL_TIM_PWM_Start(&Motor_RL_PWM_TIMEBASE, Motor_RL_PWM);
    // HAL_TIM_PWM_Start(&Motor_RR_PWM_TIMEBASE, Motor_RR_PWM);
}
void Motor_FL_SetState(Motor_State state)
{
    switch(state)
    {
        case Motor_Forward:
            HAL_GPIO_WritePin(Motor_FL_IN1_GPIO_Port, Motor_FL_IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_FL_IN2_GPIO_Port, Motor_FL_IN2_Pin, GPIO_PIN_RESET);
            break;
        case Motor_Backward:
            HAL_GPIO_WritePin(Motor_FL_IN1_GPIO_Port, Motor_FL_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_FL_IN2_GPIO_Port, Motor_FL_IN2_Pin, GPIO_PIN_SET);
            break;
        case Motor_Brake:
            HAL_GPIO_WritePin(Motor_FL_IN1_GPIO_Port, Motor_FL_IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_FL_IN2_GPIO_Port, Motor_FL_IN2_Pin, GPIO_PIN_SET);
            break;
        case Motor_Coast:
            HAL_GPIO_WritePin(Motor_FL_IN1_GPIO_Port, Motor_FL_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_FL_IN2_GPIO_Port, Motor_FL_IN2_Pin, GPIO_PIN_RESET);
            break;
    }
}
void Motor_FR_SetState(Motor_State state)
{
    switch(state)
    {
        case Motor_Forward:
            HAL_GPIO_WritePin(Motor_FR_IN1_GPIO_Port, Motor_FR_IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_FR_IN2_GPIO_Port, Motor_FR_IN2_Pin, GPIO_PIN_RESET);
            break;
        case Motor_Backward:
            HAL_GPIO_WritePin(Motor_FR_IN1_GPIO_Port, Motor_FR_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_FR_IN2_GPIO_Port, Motor_FR_IN2_Pin, GPIO_PIN_SET);
            break;
        case Motor_Brake:
            HAL_GPIO_WritePin(Motor_FR_IN1_GPIO_Port, Motor_FR_IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_FR_IN2_GPIO_Port, Motor_FR_IN2_Pin, GPIO_PIN_SET);
            break;
        case Motor_Coast:
            HAL_GPIO_WritePin(Motor_FR_IN1_GPIO_Port, Motor_FR_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_FR_IN2_GPIO_Port, Motor_FR_IN2_Pin, GPIO_PIN_RESET);
            break;
    }
}
// void Motor_RL_SetState(Motor_State state)
// {
//     switch(state)
//     {
//         case Motor_Forward:
//             HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_RESET);
//             break;
//         case Motor_Backward:
//             HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_SET);
//             break;
//         case Motor_Brake:
//             HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_SET);
//             break;
//         case Motor_Coast:
//             HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_RESET);
//             break;
//     }
// }
// void Motor_RR_SetState(Motor_State state)
// {
//     switch(state)
//     {
//         case Motor_Forward:
//             HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_RESET);
//             break;
//         case Motor_Backward:
//             HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_SET);
//             break;
//         case Motor_Brake:
//             HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_SET);
//             break;
//         case Motor_Coast:
//             HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_RESET);
//             break;
//     }
// }
void Motor_FL_SetPWM(uint16_t duty)
{
    uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&Motor_FL_PWM_TIMEBASE);
    if (duty > max_duty) {
        duty = max_duty;
    }
    __HAL_TIM_SET_COMPARE(&Motor_FL_PWM_TIMEBASE, Motor_FL_PWM, duty);
}
void Motor_FR_SetPWM(uint16_t duty)
{
    uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&Motor_FR_PWM_TIMEBASE);
    if (duty > max_duty) {
        duty = max_duty;
    }
    __HAL_TIM_SET_COMPARE(&Motor_FR_PWM_TIMEBASE, Motor_FR_PWM, duty);
}
// void Motor_RL_SetPWM(uint16_t duty)
// {
//     uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&Motor_RL_PWM_TIMEBASE);
//     if (duty > max_duty) {
//         duty = max_duty;
//     }
//     __HAL_TIM_SET_COMPARE(&Motor_RL_PWM_TIMEBASE, Motor_RL_PWM, duty);
// }
// void Motor_RR_SetPWM(uint16_t duty)
// {
//     uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&Motor_RR_PWM_TIMEBASE);
//     if (duty > max_duty) {
//         duty = max_duty;
//     }
//     __HAL_TIM_SET_COMPARE(&Motor_RR_PWM_TIMEBASE, Motor_RR_PWM, duty);
// }
void Motor_FL_Drive(float output_pwm)
{
    if (output_pwm > 0)
    {
        Motor_FL_SetState(Motor_Forward);
        Motor_FL_SetPWM((uint16_t)output_pwm);
    }
    else if (output_pwm < 0)
    {
        Motor_FL_SetState(Motor_Backward);
        Motor_FL_SetPWM((uint16_t)(-output_pwm));
    }
    else
    {
        Motor_FL_SetState(Motor_Brake);
        Motor_FL_SetPWM(0);
    }
}

void Motor_FR_Drive(float output_pwm)
{
    if (output_pwm > 0)
    {
        Motor_FR_SetState(Motor_Forward);
        Motor_FR_SetPWM((uint16_t)output_pwm);
    }
    else if (output_pwm < 0)
    {
        Motor_FR_SetState(Motor_Backward);
        Motor_FR_SetPWM((uint16_t)(-output_pwm));
    }
    else
    {
        Motor_FR_SetState(Motor_Brake);
        Motor_FR_SetPWM(0);
    }
}
// void Motor_RL_Drive(float output_pwm)
// {
//     if (output_pwm > 0)
//     {
//         Motor_RL_SetState(Motor_Forward);
//         Motor_RL_SetPWM((uint16_t)output_pwm);
//     }
//     else if (output_pwm < 0)
//     {
//         Motor_RL_SetState(Motor_Backward);
//         Motor_RL_SetPWM((uint16_t)(-output_pwm));
//     }
//     else
//     {
//         Motor_RL_SetState(Motor_Brake);
//         Motor_RL_SetPWM(0);
//     }
// }
// void Motor_RR_Drive(float output_pwm)
// {
//     if (output_pwm > 0)
//     {
//         Motor_RR_SetState(Motor_Forward);
//         Motor_RR_SetPWM((uint16_t)output_pwm);
//     }
//     else if (output_pwm < 0)
//     {
//         Motor_RR_SetState(Motor_Backward);
//         Motor_RR_SetPWM((uint16_t)(-output_pwm));
//     }
//     else
//     {
//         Motor_RR_SetState(Motor_Brake);
//         Motor_RR_SetPWM(0);
//     }
// }

void Motor_FL_SetVelocity(float target_velocity)
{
    Motor_FL.target_velocity = target_velocity;
}

void Motor_FR_SetVelocity(float target_velocity)
{
    Motor_FR.target_velocity = target_velocity;
}

// void Motor_RL_SetVelocity(float target_velocity)
// {
//     Motor_RL.target_velocity = target_velocity;
// }
// 
// void Motor_RR_SetVelocity(float target_velocity)
// {
//     Motor_RR.target_velocity = target_velocity;
// }

void Motor_FL_Update(float current_velocity)
{
    Motor_FL.current_velocity = current_velocity;
    fp32 pwm = PID_calc(&Motor_FL.velocity_pid, current_velocity, Motor_FL.target_velocity);
    Motor_FL_Drive(pwm);
}

void Motor_FR_Update(float current_velocity)
{
    Motor_FR.current_velocity = current_velocity;
    fp32 pwm = PID_calc(&Motor_FR.velocity_pid, current_velocity, Motor_FR.target_velocity);
    Motor_FR_Drive(pwm);
}

// void Motor_RL_Update(float current_velocity)
// {
//     Motor_RL.current_velocity = current_velocity;
//     fp32 pwm = PID_calc(&Motor_RL.velocity_pid, current_velocity, Motor_RL.target_velocity);
//     Motor_RL_Drive(pwm);
// }
// 
// void Motor_RR_Update(float current_velocity)
// {
//     Motor_RR.current_velocity = current_velocity;
//     fp32 pwm = PID_calc(&Motor_RR.velocity_pid, current_velocity, Motor_RR.target_velocity);
//     Motor_RR_Drive(pwm);
// }

float Motor_FL_ReadEncoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&Motor_FL_Encoder_Timebase);
    __HAL_TIM_SET_COUNTER(&Motor_FL_Encoder_Timebase, 0);
    
    // Calculate RPM
    // Total counts per wheel revolution = Motor_Encoder_Resolution * 4 * Motor_Transmission_Ratio
    // dt = 10ms (100Hz), so RPM = (count / total_counts) * 100 * 60
    // Note: Assuming TIM2 and TIM3 are configured to Encoder Mode TI1 and TI2 (4x resolution)
    float rpm = (float)count * 6000.0f / (Motor_Encoder_Resolution * 4.0f * Motor_Transmission_Ratio);
    return rpm;
}

float Motor_FR_ReadEncoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&Motor_FR_Encoder_Timebase);
    __HAL_TIM_SET_COUNTER(&Motor_FR_Encoder_Timebase, 0);
    
    // Reverse count if motor orientation is mirrored physically
    float rpm = (float)count * 6000.0f / (Motor_Encoder_Resolution * 4.0f * Motor_Transmission_Ratio);
    return rpm;
}

void MotorTask(void *argument)
{
    MotorControl_Init();
    
    while(1)
    {
        float current_velocity_FL = Motor_FL_ReadEncoder();
        float current_velocity_FR = Motor_FR_ReadEncoder();
        
        Motor_FL_Update(current_velocity_FL);
        Motor_FR_Update(current_velocity_FR);
        
        osDelay(10);
    }
}