/*
 * WheelControls.c
 */

#include "WheelControls.h"
#include "MotorControl.h"
#include "MPU6050.h"
#include "cmsis_os.h"
#include <math.h>
#include "pid.h"

// --- 麦轮参数宏定义 ---
#define WHEEL_DIAMETER_CM 6.0f     // 轮子直径(cm)
#define CAR_WIDTH_CM      15.0f    // 左右轮距(cm)，需要根据实际测量值修改
#define CAR_LENGTH_CM     15.0f    // 前后轮距(cm)，需要根据实际测量值修改

// --- PID 相关参数与控制变量 ---
#define TURN_KP 1.5f
#define TURN_KI 0.0f
#define TURN_KD 0.1f
#define TURN_MAX_OUT  100.0f
#define TURN_MAX_IOUT 20.0f

#define MOVE_KP 1.0f
#define MOVE_KI 0.0f
#define MOVE_KD 0.2f
#define MOVE_MAX_OUT  100.0f
#define MOVE_MAX_IOUT 20.0f

#define COAST_KP 0.5f  
#define COAST_KI 0.0f
#define COAST_KD 0.0f
#define COAST_MAX_OUT  100.0f
#define COAST_MAX_IOUT 30.0f

pid_type_def pid_turn;
pid_type_def pid_move_x;
pid_type_def pid_move_y;
pid_type_def pid_coast_x;
pid_type_def pid_coast_y;

CarControl Car_Control={0};

static void Car_PID_Init(void)
{
    const fp32 turn_pid_param[3] = {TURN_KP, TURN_KI, TURN_KD};
    const fp32 move_pid_param[3] = {MOVE_KP, MOVE_KI, MOVE_KD};
    const fp32 coast_pid_param[3] = {COAST_KP, COAST_KI, COAST_KD};

    PID_init(&pid_turn, PID_POSITION, turn_pid_param, TURN_MAX_OUT, TURN_MAX_IOUT);
    PID_init(&pid_move_x, PID_POSITION, move_pid_param, MOVE_MAX_OUT, MOVE_MAX_IOUT);
    PID_init(&pid_move_y, PID_POSITION, move_pid_param, MOVE_MAX_OUT, MOVE_MAX_IOUT);
    PID_init(&pid_coast_x, PID_POSITION, coast_pid_param, COAST_MAX_OUT, COAST_MAX_IOUT);
    PID_init(&pid_coast_y, PID_POSITION, coast_pid_param, COAST_MAX_OUT, COAST_MAX_IOUT);
}

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
        Car_Control.target_x = param1;
        Car_Control.target_y = param2;
        Car_Control.mode = CAR_CONTROL_MOVE;
    }
    else if(cmd==0x03)//设置巡航速度
    {
        Car_Control.target_vx = param1;
        Car_Control.target_vy = param2;
        Car_Control.mode = CAR_CONTROL_COAST;
    }
    else if(cmd==0x04)//设置旋转速度
    {
        Car_Control.target_rotationSpeed = param1;
    }
}

// --- 麦轮速度解算通用函数 ---
// 输入: vx(cm/s, 前方为负/正视情况), vy(cm/s, 右方正), wz(deg/s, 逆时针为正)
static void WheelControl(float vx, float vy, float wz)
{
    // 将角速度 deg/s 转换成 rad/s
    float wz_rad = wz * (3.14159f / 180.0f);
    
    // 底盘心到轮子的距离常数
    float K = (CAR_WIDTH_CM / 2.0f) + (CAR_LENGTH_CM / 2.0f);
    
    // 麦轮标准解算公式 (以轮边线速度 cm/s 为单位)
    // 依据电机的安装和极性，这里的加减号可能需要微调
    float v_fl = vx - vy - wz_rad * K; // 前左
    float v_fr = vx + vy + wz_rad * K; // 前右
    float v_rl = vx + vy - wz_rad * K; // 后左
    float v_rr = vx - vy + wz_rad * K; // 后右
    
    // 将解算的速度赋予各个电机目标速度(如果用转速RPM，这里还要除以周长)
    // 目前假定底层的 target_velocity 是线速度(cm/s)
    Motor_FL.target_velocity = v_fl/100;
    Motor_FR.target_velocity = v_fr/100;
    // 如果后方电机结构体解除了注释，请取消下面的注释：
    // Motor_RL.target_velocity = v_rl;
    // Motor_RR.target_velocity = v_rr;
}

static void Car_Turn(void)
{
    float current_yaw = mpu_data.Yaw; 
    float target_wz = PID_calc(&pid_turn, current_yaw, Car_Control.target_yawAngle);
    
    // 仅转动：vx=0, vy=0, wz=输出
    WheelControl(0.0f, 0.0f, target_wz);
}

static void Car_Move(void)
{
    float current_x = mpu_data.PosX;
    float current_y = mpu_data.PosY;
    float target_vx = PID_calc(&pid_move_x, current_x, Car_Control.target_x);
    float target_vy = PID_calc(&pid_move_y, current_y, Car_Control.target_y);
    
    // 仅平移：vx，vy为输出计算值，wz=0。或保持车头稳定可以再加一个Yaw的PID串进去
    WheelControl(target_vx, target_vy, 0.0f);
}

static void Car_Coast(void)
{
    // 巡航定速: 期望速度 + PID修正补偿。若底层已有电机闭环，也可直接下发Car_Control.target_vx
    float current_vx = mpu_data.VelX;
    float current_vy = mpu_data.VelY;
    float out_vx = Car_Control.target_vx + PID_calc(&pid_coast_x, current_vx, Car_Control.target_vx);
    float out_vy = Car_Control.target_vy + PID_calc(&pid_coast_y, current_vy, Car_Control.target_vy);
    
    WheelControl(out_vx, out_vy, 0.0f);
}

void CarControlTask(void *argument)
{
    Car_PID_Init(); // 启动前初始化 PID 实例

    while(1)
    {
        switch(Car_Control.mode)
        {
            case CAR_CONTROL_TURN:
                Car_Turn();
                break;
            case CAR_CONTROL_MOVE:
                Car_Move();
                break;
            case CAR_CONTROL_COAST:
                Car_Coast();
                break;
        }
        osDelay(100);
    }
}
