/*
 * WheelControls.h
 */

#ifndef INC_WHEELCONTROLS_H_
#define INC_WHEELCONTROLS_H_

#include "main.h"
#include "MotorControl.h"
#include "MPU6050.h"
#include "pid.h"
#include "cmsis_os.h"

typedef enum{
    CAR_CONTROL_TURN,
    CAR_CONTROL_MOVE,
    CAR_CONTROL_COAST
} CarControlMode;

typedef struct {
    float x;
    float y;
    float vx;//cm/s
    float vy;
    float yawAngle;//度
    float rotationSpeed;//度/秒

    float target_x;
    float target_y;
    float target_vx;
    float target_vy;
    float target_yawAngle;
    float target_rotationSpeed;

    CarControlMode mode;
} CarControl;

extern CarControl Car_Control;

void Car_UpdateState(void);
void Car_UpdateTarget(uint8_t cmd, float param1, float param2);
void Car_ControlTask(void *argument);

#endif /* INC_WHEELCONTROLS_H_ */