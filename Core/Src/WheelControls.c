/*
 * WheelControls.c
 */

#include "WheelControls.h"
#include "MotorControl.h"

void WheelControls_Translation(float vx, float vy)
{
    float v_fl = vy + vx;
    float v_fr = vy - vx;
    // float v_rl = vy - vx;
    // float v_rr = vy + vx;

    Motor_FL_SetVelocity(v_fl);
    Motor_FR_SetVelocity(v_fr);
    // Motor_RL_SetVelocity(v_rl);
    // Motor_RR_SetVelocity(v_rr);
}

void WheelControls_Spin(float vx, float vy, float wz)
{
    float v_fl = vy + vx + wz;
    float v_fr = vy - vx - wz;
    // float v_rl = vy - vx + wz;
    // float v_rr = vy + vx - wz;

    Motor_FL_SetVelocity(v_fl);
    Motor_FR_SetVelocity(v_fr);
    // Motor_RL_SetVelocity(v_rl);
    // Motor_RR_SetVelocity(v_rr);
}