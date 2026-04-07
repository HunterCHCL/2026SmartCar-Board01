/*
 * WheelControls.h
 */

#ifndef INC_WHEELCONTROLS_H_
#define INC_WHEELCONTROLS_H_

#include "main.h"
#include "MotorControl.h"
#include "cmsis_os.h"
#include <math.h>
#include "UARTComms.h"
#include "MPU6050.h"

#define WHEEL_DIAMETER_CM 6.0f     // 轮子直径(cm)

#define CAR_REMOTE_CONTROL_FORWARD_SPEED 18.0f // cm/s
#define CAR_REMOTE_CONTROL_SIDEWAYS_SPEED 18.0f // deg/s
#define CAR_REMOTE_CONTROL_ROTATION_SPEED 10.0f // deg/s

#define WHELL_FL_DIR 1 //电机方向系数(以y方向为基准)
#define WHELL_FR_DIR 1 
#define WHELL_RL_DIR -1 
#define WHELL_RR_DIR -1 

#define DEADZONE_ANGLE 5.0f //deg，死区

#define DELAY_TIME 200//ms

typedef enum{
    CAR_CONTROL_TURN,
    CAR_CONTROL_MOVE,
    CAR_CONTROL_COAST,
    CAR_CONTROL_TEST
} CarControlMode;

typedef struct{
    float x;
    float y;
}Coordinates;

typedef struct {
    Coordinates velocity;//cm/s
    float yawAngle;//rad
    float rotationSpeed;//rad/s

    Coordinates target_position;//cm
    Coordinates target_velocity;//速度矢量 cm/s
    float target_yawAngle;
    float target_rotationSpeed;

    CarControlMode mode;
} CarControl;

typedef struct{
    uint8_t Forward;
    uint8_t Left;
    uint8_t Backward;
    uint8_t Right;
    uint8_t RotateLeft;
    uint8_t RotateRight;
} RemoteControlInput;

extern CarControl Car_Control;
extern RemoteControlInput Remote_Control_Input;

void Car_UpdateState(void);
void Car_UpdateTarget(uint8_t cmd, float param1, float param2);
void Car_ControlTask(void *argument);
void Car_RemoteControl(RemoteControlInput *input);

#endif /* INC_WHEELCONTROLS_H_ */