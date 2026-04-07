/*
 * WheelControls.c
 */

#include "WheelControls.h"

CarControl Car_Control={0};
//static int32_t last_pulse_fl = 0;
//static int32_t last_pulse_fr = 0;
//static int32_t last_pulse_rl = 0;
//static int32_t last_pulse_rr = 0;

//pid_type_def pid_turn;

static Coordinates TransformCoordinates(Coordinates input, float Angle,uint8_t clockwise)//顺时针1，逆时针0
{
    Coordinates coord;
    if(clockwise)
    {
        coord.x = input.x * cosf(Angle) + input.y * sinf(Angle);
        coord.y = -input.x * sinf(Angle) + input.y * cosf(Angle);
    }
    else
    {
        coord.x = input.x * cosf(Angle) - input.y * sinf(Angle);
        coord.y = input.x * sinf(Angle) + input.y * cosf(Angle);
    }
    return coord;
}

//void Car_PID_Init(void)
//{
//    const float turn_pid_params[3] = {5.0f, 0.0f, 0.5f};
//    PID_init(&pid_turn, PID_POSITION, turn_pid_params, 100.0f, 50.0f);
//}

void Car_UpdateTarget(uint8_t cmd, float param1, float param2)
{
    if(cmd==0x01)//转向
    {
        Car_Control.target_yawAngle = param1;
        Car_Control.target_rotationSpeed = param2;
        Car_Control.mode = CAR_CONTROL_TURN;
    }
    else if(cmd==0x02)//平移
    {
        Car_Control.target_position.x = param1;
        Car_Control.target_position.y = param2;
        Car_Control.mode = CAR_CONTROL_MOVE;
    }
    else if(cmd==0x03)//设置巡航速度
    {
        Car_Control.target_velocity.x = param1;
        Car_Control.target_velocity.y = param2;
        Car_Control.mode = CAR_CONTROL_COAST;
    }
    else if(cmd==0x04)//设置旋转速度
    {
        Car_Control.target_rotationSpeed = param1;
    }
}

RemoteControlInput Remote_Control_Input={0};

void Car_RemoteControl(RemoteControlInput *input)
{
    if(input->Forward)
    {
        Car_Control.target_velocity.x = CAR_REMOTE_CONTROL_FORWARD_SPEED;
    }
    else if(input->Backward)
    {
        Car_Control.target_velocity.x = -CAR_REMOTE_CONTROL_FORWARD_SPEED;
    }
    else
    {
        Car_Control.target_velocity.x = 0.0f;
    }

    if(input->Left)
    {
        Car_Control.target_velocity.y = -CAR_REMOTE_CONTROL_SIDEWAYS_SPEED;
    }
    else if(input->Right)
    {
        Car_Control.target_velocity.y = CAR_REMOTE_CONTROL_SIDEWAYS_SPEED;
    }
    else
    {
        Car_Control.target_velocity.y = 0.0f;
    }

    if(input->RotateLeft)
    {
        Car_Control.target_rotationSpeed = CAR_REMOTE_CONTROL_ROTATION_SPEED;
    }
    else if(input->RotateRight)
    {
        Car_Control.target_rotationSpeed = -CAR_REMOTE_CONTROL_ROTATION_SPEED;
    }
    else
    {
        Car_Control.target_rotationSpeed = 0.0f;
    }

    Car_Control.mode = CAR_CONTROL_COAST;
}

/*
@brief 麦轮解算
@param vx 期望的前向速度，正值表示向前，负值表示向后
@param vy 期望的横向速度，正值表示向右，负值表示向左
@param wz 期望的旋转速度，单位为度/秒，正值表示逆时针旋转，负值表示顺时针旋转
@return 无
*/
// static void RotationControl(float vx, float vy, float wz)
// {
//     // 将角速度 deg/s 转换成 rad/s
//     float wz_rad = wz * (3.14159f / 180.0f);
    
//     // 底盘心到轮子的距离常数
//     float K = (CAR_WIDTH_CM / 2.0f) + (CAR_LENGTH_CM / 2.0f);
    
//     // 麦轮标准解算公式 (线速度 cm/s)
//     float v_fl = vx - vy - wz_rad * K; // 前左
//     float v_fr = vx + vy + wz_rad * K; // 前右
//     float v_rl = vx + vy - wz_rad * K; // 后左
//     float v_rr = vx - vy + wz_rad * K; // 后右
    
//     // 将解算的速度赋予各个电机目标速度(如果用转速RPM，这里还要除以周长)
//     // 目前假定底层的 target_velocity 是线速度(cm/s)
//     Motor_FL.target_velocity = v_fl/100/(3.14159f * WHEEL_DIAMETER_CM); // 转换成转速，单位为转/s
//     Motor_FR.target_velocity = v_fr/100/(3.14159f * WHEEL_DIAMETER_CM);
//     Motor_RL.target_velocity = v_rl/100/(3.14159f * WHEEL_DIAMETER_CM);
//     Motor_RR.target_velocity = v_rr/100/(3.14159f * WHEEL_DIAMETER_CM);

//     // 通过串口发送后轮目标速度(0x1A命令)，协议规定负载为float类型
//     float target_v_rl = v_rl / 100.0f;
//     float target_v_rr = v_rr / 100.0f;
//     uint8_t buffer[8];
//     memcpy(&buffer[0], &target_v_rl, 4);
//     memcpy(&buffer[4], &target_v_rr, 4);
    
//     UARTComms_Transmmit_Data(&UARTComms_Port, 0x1A, buffer, 8);
// }
static void Car_Turn(void)
{
    if(Car_Control.target_yawAngle<DEADZONE_ANGLE&&Car_Control.target_yawAngle>-DEADZONE_ANGLE)
    {
        return;
    }
    if(Car_Control.target_yawAngle<0)
    {
        Motor_SetTargetVelocity(Motor_FL_ID, -Car_Control.target_rotationSpeed);
        Motor_SetTargetVelocity(Motor_FR_ID, Car_Control.target_rotationSpeed);
        Motor_SetTargetVelocity(Motor_RL_ID, -Car_Control.target_rotationSpeed);
        Motor_SetTargetVelocity(Motor_RR_ID, Car_Control.target_rotationSpeed);
    }
    else
    {
        Motor_SetTargetVelocity(Motor_FL_ID, Car_Control.target_rotationSpeed);
        Motor_SetTargetVelocity(Motor_FR_ID, -Car_Control.target_rotationSpeed);
        Motor_SetTargetVelocity(Motor_RL_ID, Car_Control.target_rotationSpeed);
        Motor_SetTargetVelocity(Motor_RR_ID, -Car_Control.target_rotationSpeed);
    }
}

static void Car_Move(Coordinates target_position)
{
    Coordinates transformed_position = TransformCoordinates(target_position,0.78539816f, 0);//转换到轮子坐标系
    Motor_SetTargetPosition(Motor_FL_ID, transformed_position.y);
    Motor_SetTargetPosition(Motor_FR_ID, transformed_position.x);
    Motor_SetTargetPosition(Motor_RL_ID, transformed_position.x);
    Motor_SetTargetPosition(Motor_RR_ID, transformed_position.y);
}

static void Car_Coast(void)
{
    Coordinates transformed_velocity = TransformCoordinates((Coordinates){Car_Control.target_velocity.x, Car_Control.target_velocity.y},0.78539816f, 0);//转换到轮子坐标系
    // Motor_FL.target_velocity = WHELL_FL_DIR* (transformed_velocity.y - Car_Control.target_rotationSpeed);
    // Motor_FR.target_velocity = WHELL_FR_DIR* (transformed_velocity.x + Car_Control.target_rotationSpeed);
    // Motor_RL.target_velocity = WHELL_RL_DIR* (transformed_velocity.x - Car_Control.target_rotationSpeed);
    // Motor_RR.target_velocity = WHELL_RR_DIR* (transformed_velocity.y + Car_Control.target_rotationSpeed);
    Motor_SetTargetVelocity(Motor_FL_ID, transformed_velocity.y - Car_Control.target_rotationSpeed);
    Motor_SetTargetVelocity(Motor_FR_ID, transformed_velocity.x + Car_Control.target_rotationSpeed);
    Motor_SetTargetVelocity(Motor_RL_ID, transformed_velocity.x - Car_Control.target_rotationSpeed);
    Motor_SetTargetVelocity(Motor_RR_ID, transformed_velocity.y + Car_Control.target_rotationSpeed);
}

void CarControlTask(void *argument)
{
//    Car_PID_Init();
    Car_Control.mode = CAR_CONTROL_TEST;
    while(1)
    {
        switch(Car_Control.mode)
        {
            case CAR_CONTROL_TURN:
                Car_Turn();
                break;
            case CAR_CONTROL_MOVE:
                Car_Move(Car_Control.target_position);
                break;
            case CAR_CONTROL_COAST:
                Car_Coast();
                break;
            case CAR_CONTROL_TEST:
                break;
        }
        uint8_t buffer[8];
        if(Motor_RL.mode == MOTOR_CONTROL_POSITION)
        {
            memcpy(&buffer[0],&Motor_RL.target_position,4);
            memcpy(&buffer[4],&Motor_RR.target_position,4);
            UARTComms_Transmmit_Data(&UARTComms_Port, 0x1A, buffer, 8);
//            UARTComms_Transmmit_Data(&UARTComms_BT24_Port, 0x01, buffer, 8);
        }
        else if(Motor_RL.mode == MOTOR_CONTROL_VELOCITY)
        {
            memcpy(&buffer[0],&Motor_RL.target_velocity,4);
            memcpy(&buffer[4],&Motor_RR.target_velocity,4);
            UARTComms_Transmmit_Data(&UARTComms_Port, 0x1B, buffer, 8);
//            UARTComms_Transmmit_Data(&UARTComms_BT24_Port, 0x01, buffer, 8);
        }
        osDelay(DELAY_TIME);
    }
}
