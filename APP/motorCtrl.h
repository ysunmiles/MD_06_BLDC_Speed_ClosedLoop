#ifndef __MOTORCTRL_H
#define __MOTORCTRL_H

#define SPEED_AVERAGE_WINDOW 6U

typedef enum
{
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 2
} MotorDirection;



void MotorCtrl_PWMCallback(void);

MotorDirection MotorCtrl_GetDirection(void);
uint8_t MotorCtrl_GetHall(void);
uint16_t MotorCtrl_GetSpeed(void);
void MotorCtrl_SetDuty(uint16_t uartDuty);
uint16_t MotorCtrl_GetDuty(void);
void MotorCtrl_SetSpeedZero(void);

void MotorCtrl_SetSpeedAim(uint16_t cmdSpeed);

#endif