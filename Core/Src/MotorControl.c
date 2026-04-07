/*
 * MotorControl.c
 *
 *  Created on: Mar 21, 2026
 *      Author: HunterCHCL
 */

#include "MotorControl.h"

Motor_HandleTypeDef Motor_FL;
Motor_HandleTypeDef Motor_FR;
Motor_HandleTypeDef Motor_RL;
Motor_HandleTypeDef Motor_RR;

// int32_t Encoder_FL = 0;
// int32_t Encoder_FR = 0;

void MotorControl_Init(void)
{
    const float position_pid_params[3] = {2.0f, 0.0f, 0.2f};
    const float speed_pid_params[3] = {2.8f, 0.1f, 0.0f};
    
    Motor_FL.target_position = 0.0f;
    Motor_FL.current_position = 0.0f;
    Motor_FL.target_velocity = 0.0f;
    Motor_FL.current_velocity = 0.0f;
    Motor_FR.target_position = 0.0f;
    Motor_FR.current_position = 0.0f;
    Motor_FR.target_velocity = 0.0f;
    Motor_FR.current_velocity = 0.0f;
    Motor_FL.mode = MOTOR_CONTROL_VELOCITY;
    Motor_FR.mode = MOTOR_CONTROL_VELOCITY;
    Motor_FL.last_time = HAL_GetTick();
    Motor_FR.last_time = HAL_GetTick();

    PID_init(&Motor_FL.position_pid, PID_POSITION, position_pid_params, Motor_Speed_Limit, Motor_Position_I_Limit);//输出最大值，I最大值
    PID_init(&Motor_FL.velocity_pid, PID_POSITION, speed_pid_params, Motor_Power_Limit, Motor_Velocity_I_Limit);
    PID_init(&Motor_FR.position_pid, PID_POSITION, position_pid_params, Motor_Speed_Limit, Motor_Position_I_Limit);
    PID_init(&Motor_FR.velocity_pid, PID_POSITION, speed_pid_params, Motor_Power_Limit, Motor_Velocity_I_Limit);

    HAL_TIM_PWM_Start(&Motor_FL_PWM_TIMEBASE, Motor_FL_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&Motor_FR_PWM_TIMEBASE, Motor_FR_PWM_CHANNEL);

    HAL_TIM_Encoder_Start(&Motor_FL_Encoder_Timebase, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Motor_FR_Encoder_Timebase, TIM_CHANNEL_ALL);
}

void Motor_SetState(Motor_ID motor_id, Motor_State state)
{
    switch(motor_id)
    {
        case Motor_FL_ID:
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
            break;
        case Motor_FR_ID:
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
            break;
        /*
        case Motor_RL_ID:
            switch(state)
            {
                case Motor_Forward:
                    HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_RESET);
                    break;
                case Motor_Backward:
                    HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_SET);
                    break;
                case Motor_Brake:
                    HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_SET);
                    break;
                case Motor_Coast:
                    HAL_GPIO_WritePin(Motor_RL_IN1_GPIO_Port, Motor_RL_IN1_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(Motor_RL_IN2_GPIO_Port, Motor_RL_IN2_Pin, GPIO_PIN_RESET);
                    break;
            }
            break;
        case Motor_RR_ID:
            switch(state)
            {
                case Motor_Forward:
                    HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_RESET);
                    break;
                case Motor_Backward:
                    HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_SET);
                    break;
                case Motor_Brake:
                    HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_SET);
                    break;
                case Motor_Coast:
                    HAL_GPIO_WritePin(Motor_RR_IN1_GPIO_Port, Motor_RR_IN1_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(Motor_RR_IN2_GPIO_Port, Motor_RR_IN2_Pin, GPIO_PIN_RESET);
                    break;
            }
            break;
        */
    }
}

void Motor_SetPWM(Motor_ID motor_id, uint16_t duty)
{
    uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&Motor_FL_PWM_TIMEBASE);
    if (duty > max_duty) {
        duty = max_duty;
    }
    switch(motor_id)
    {
        case Motor_FL_ID:
            __HAL_TIM_SET_COMPARE(&Motor_FL_PWM_TIMEBASE, Motor_FL_PWM_CHANNEL, duty);
            break;
        case Motor_FR_ID:
            __HAL_TIM_SET_COMPARE(&Motor_FR_PWM_TIMEBASE, Motor_FR_PWM_CHANNEL, duty);
            break;
        /*
        case Motor_RL_ID:
            __HAL_TIM_SET_COMPARE(&Motor_RL_PWM_TIMEBASE, Motor_RL_PWM_CHANNEL, duty);
            break;
        case Motor_RR_ID:
            __HAL_TIM_SET_COMPARE(&Motor_RR_PWM_TIMEBASE, Motor_RR_PWM_CHANNEL, duty);
            break;
        */
    }
}

void Motor_Drive(Motor_ID motor_id, float output_pwm)
{
     if (output_pwm > 0)
    {
        Motor_SetState(motor_id, Motor_Forward);
        Motor_SetPWM(motor_id, (uint16_t)output_pwm);
    }
    else if (output_pwm < 0)
    {
        Motor_SetState(motor_id, Motor_Backward);
        Motor_SetPWM(motor_id, (uint16_t)(-output_pwm));
    }
    else
    {
        Motor_SetState(motor_id, Motor_Brake);
        Motor_SetPWM(motor_id, 0);
    }
}

void Motor_SetTargetPosition(Motor_ID motor_id, float target_position)
{
    switch(motor_id)
    {
        case Motor_FL_ID: Motor_FL.mode = MOTOR_CONTROL_POSITION;Motor_FL.target_position = WHELL_FL_DIR * target_position; break;
        case Motor_FR_ID: Motor_FR.mode = MOTOR_CONTROL_POSITION;Motor_FR.target_position = WHELL_FR_DIR * target_position; break;
        case Motor_RL_ID: Motor_RL.mode = MOTOR_CONTROL_POSITION;Motor_RL.target_position = WHELL_RL_DIR * target_position; break;
        case Motor_RR_ID: Motor_RR.mode = MOTOR_CONTROL_POSITION;Motor_RR.target_position = WHELL_RR_DIR * target_position; break;
    }
}

void Motor_SetTargetVelocity(Motor_ID motor_id, float target_velocity)
{
    switch(motor_id)
    {
        case Motor_FL_ID: Motor_FL.mode = MOTOR_CONTROL_VELOCITY;Motor_FL.target_velocity = WHELL_FL_DIR * target_velocity; break;
        case Motor_FR_ID: Motor_FR.mode = MOTOR_CONTROL_VELOCITY;Motor_FR.target_velocity = WHELL_FR_DIR * target_velocity; break;
        case Motor_RL_ID: Motor_RL.mode = MOTOR_CONTROL_VELOCITY;Motor_RL.target_velocity = WHELL_RL_DIR * target_velocity; break;
        case Motor_RR_ID: Motor_RR.mode = MOTOR_CONTROL_VELOCITY;Motor_RR.target_velocity = WHELL_RR_DIR * target_velocity; break;
    }
}
int16_t Motor_FL_ReadEncoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&Motor_FL_Encoder_Timebase);
    __HAL_TIM_SET_COUNTER(&Motor_FL_Encoder_Timebase, 0);
    return count;
}

int16_t Motor_FR_ReadEncoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&Motor_FR_Encoder_Timebase);
    __HAL_TIM_SET_COUNTER(&Motor_FR_Encoder_Timebase, 0);
    return count;
}
/*
int16_t Motor_RL_ReadEncoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&Motor_RL_Encoder_Timebase);
    __HAL_TIM_SET_COUNTER(&Motor_RL_Encoder_Timebase, 0);
    return count;
}

int16_t Motor_RR_ReadEncoder(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&Motor_RR_Encoder_Timebase);
    __HAL_TIM_SET_COUNTER(&Motor_RR_Encoder_Timebase, 0);
    return count;
}
*/
int8_t Motor_DirFactor(Motor_ID motor_id)
{
    switch(motor_id)
    {
        case Motor_FL_ID: return Motor_FL_Encoder_DIR_FACTOR;
        case Motor_FR_ID: return Motor_FR_Encoder_DIR_FACTOR;
        // case Motor_RL_ID: return Motor_RL_Encoder_DIR_FACTOR;
        // case Motor_RR_ID: return Motor_RR_Encoder_DIR_FACTOR;
    }
    return 1;
}

static void Motor_UpdateMotor(Motor_HandleTypeDef* motor, Motor_ID id, int16_t encoder_delta)
{
    uint32_t current_time = HAL_GetTick();
    if (motor->last_time == 0) 
    {
        motor->last_time = current_time;
        return;
    }
    float dt = (current_time - motor->last_time) / 1000.0f; // s
    if (dt <= 0.0f) dt = 0.001f;
    motor->current_velocity = encoder_delta * Motor_Pulse_To_Distance / dt; // cm/s
    motor->current_position += encoder_delta * Motor_Pulse_To_Distance; // cm
    if(motor->mode == MOTOR_CONTROL_POSITION)
    {
        motor->target_velocity = PID_calc(&motor->position_pid, motor->current_position, motor->target_position);
    }
    float pwm = PID_calc(&motor->velocity_pid, motor->current_velocity, motor->target_velocity);
    Motor_Drive(id, pwm);
    motor->last_time = current_time;
}


void Motor_FL_Update(void)
{
    int16_t delta = Motor_DirFactor(Motor_FL_ID)*Motor_FL_ReadEncoder();
    // Encoder_FL += delta;
    Motor_UpdateMotor(&Motor_FL, Motor_FL_ID, delta);
}

void Motor_FR_Update(void)
{
    int16_t delta = Motor_DirFactor(Motor_FR_ID)*Motor_FR_ReadEncoder();
    // Encoder_FR += delta;
    Motor_UpdateMotor(&Motor_FR, Motor_FR_ID, delta);
}

/*
void Motor_RL_Update(void)
{
    int16_t delta = Motor_RL_ReadEncoder();
    Global_EncoderPulse_RL += delta;
    Motor_UpdateSingle(&Motor_RL, Motor_RL_ID, delta);
}

void Motor_RR_Update(void)
{
    int16_t delta = Motor_RR_ReadEncoder();
    Global_EncoderPulse_RR += delta;
    Motor_UpdateSingle(&Motor_RR, Motor_RR_ID, delta);
}
*/

void MotorTask(void const * argument)
{
    MotorControl_Init();
    //HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET); // 使能电机驱动
  while(1)
  {
     Motor_FL_Update();
     Motor_FR_Update();
    /*
    Motor_RL_Update();
    Motor_RR_Update();
    */
//    int16_t delta_fl = Motor_DirFactor(Motor_FL_ID)*Motor_FL_ReadEncoder();
//    int16_t delta_fr = Motor_DirFactor(Motor_FR_ID)*Motor_FR_ReadEncoder();
//    UARTComms_Transmmit_Data(&UARTComms_BT24_Port,0x01, (uint8_t*)&delta_fl, sizeof(delta_fl));
//    UARTComms_Transmmit_Data(&UARTComms_BT24_Port,0x01, (uint8_t*)&delta_fr, sizeof(delta_fr));
//    Motor_Drive(Motor_FL_ID, 25.0f);
//    Motor_Drive(Motor_FR_ID, 25.0f);
    osDelay(CycleTime*1000);
  }
}
