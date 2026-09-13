#ifndef __MOTORCTRL_H
#define __MOTORCTRL_H

#define SPEED_SOLL  1000

#define Kp      0.03f
#define Ki      0.000005f

typedef enum
{
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 2
} MotorDirection;

uint8_t MotorCtrl_GetHall(void);
MotorDirection MotorCtrl_GetDirection(void);
void MotorCtrl_PWMCallback(MotorDirection direction);

#endif