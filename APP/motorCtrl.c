#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "motorCtrl.h"
#include "monitor.h"

static volatile uint8_t motorState;
static uint8_t lastHallSignal = 0xFF;
static volatile MotorDirection motorDirection = MOTOR_DIR_FORWARD;
uint8_t duty_int = 0;

// 测试用
// float duty_set = 4 * 10;

static float intgr = 0;

MotorDirection MotorCtrl_GetDirection(void)
{
    return motorDirection;
}

void MotorCtrl_SetShutdown(GPIO_PinState State)
{
    HAL_GPIO_WritePin(CTRL_SD_GPIO_Port, CTRL_SD_Pin, State);
}

void MotorCtrl_Reset(void)
{
    HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop(&htim1);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

void MotorCtrl_DriveMotor(uint8_t rotateDirection, uint8_t duty,uint8_t hallSignal)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);

    if (hallSignal == lastHallSignal){
        return;
    }

    MotorCtrl_Reset();
    if (rotateDirection == 1){
        if (hallSignal == 5 || lastHallSignal == 4) { // U+ V-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 1 || lastHallSignal == 5) { // U+ W-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 3 || lastHallSignal == 1) { // V+ W-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
            HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 2 || lastHallSignal == 3) { // V+ U-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
            HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 6 || lastHallSignal == 2) { // W+ U-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 4 || lastHallSignal == 6) { // W+ V-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
        }
    }
    else if (rotateDirection == 2) {
        if (hallSignal == 5 || lastHallSignal == 1) {   // V+ U-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
            HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 4 || lastHallSignal == 5) { // V+ W-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
            HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 6 || lastHallSignal == 4) { // U+ W-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 2 || lastHallSignal == 6) { // U+ V-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 3 || lastHallSignal == 2) { // W+ V-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
        }
        else if (hallSignal == 1 || lastHallSignal == 3) { // W+ U-
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
        }
    }
    lastHallSignal = hallSignal;
    HAL_TIM_Base_Start_IT(&htim1);
}

void StartMotorCtrlTask(void *argument)
{
    for(;;)
    {
        // 按钮检测、开始驱动
        uint8_t keyValue = ulTaskNotifyTake(pdTRUE, osWaitForever);
        lastHallSignal = 0xFF;
        intgr = 0;
        if (keyValue == 1)
        {
            if (motorState==0 || motorState==2)
            {
                motorDirection = MOTOR_DIR_FORWARD;
                MotorCtrl_SetShutdown(GPIO_PIN_SET);
                MotorCtrl_PWMCallback(motorDirection);
                motorState = 1;
            }
            else if (motorState == 1)
            {
                MotorCtrl_Reset();
                MotorCtrl_SetShutdown(GPIO_PIN_RESET);
                motorState = 0;
            }
        }
        else if (keyValue == 2)
        {
            if (motorState==0 || motorState==1)
            {
                motorDirection = MOTOR_DIR_REVERSE;
                MotorCtrl_SetShutdown(GPIO_PIN_SET);
                MotorCtrl_PWMCallback(motorDirection);
                motorState = 2;
            }
            else if (motorState == 2)
            {
                MotorCtrl_Reset();
                MotorCtrl_SetShutdown(GPIO_PIN_RESET);
                motorState = 0;
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY1_Pin)
    {
        xTaskNotifyFromISR(MotorCtrlTaskHandle, 0x01, eSetValueWithOverwrite, pdFALSE);
    }
    else if (GPIO_Pin == KEY2_Pin)
    {
        xTaskNotifyFromISR(MotorCtrlTaskHandle, 0x02, eSetValueWithOverwrite, pdFALSE);
    }

    else if (GPIO_Pin & (HALLU_Pin|HALLV_Pin|HALLW_Pin))
    {
        static uint32_t tickus = 0;
        HAL_TIM_Base_Stop(&htim5);
        tickus = __HAL_TIM_GET_COUNTER(&htim5);
        if (tickus == 0) {   
            HAL_TIM_Base_Start(&htim5);
            return;}

        xTaskNotifyFromISR(MonitorTaskHandle, (uint32_t)tickus, eSetValueWithOverwrite, pdFALSE);
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        HAL_TIM_Base_Start(&htim5);
    }
}

uint8_t MotorCtrl_GetHall(void)
{
    uint8_t hallu, hallv, hallw;
    hallu = HAL_GPIO_ReadPin(HALLU_GPIO_Port, HALLU_Pin);
    hallv = HAL_GPIO_ReadPin(HALLV_GPIO_Port, HALLV_Pin);
    hallw = HAL_GPIO_ReadPin(HALLW_GPIO_Port, HALLW_Pin);

    uint8_t hallSignal = (hallu<<2)|(hallv<<1)|(hallw);
    return hallSignal;
}

float MotorCtrl_CalcDuty(float speed)
{
    float diff = SPEED_SOLL - speed;
    float duty_p = Kp * diff;
    intgr += diff;
    float duty_i = Ki * intgr;

    float duty = duty_p + duty_i;
    if (duty > 100) {duty = 100;}
    else if (duty < 0) {duty = 0;}
    return duty;
}

void MotorCtrl_PWMCallback(MotorDirection direction)
{
    // 获取当前速度
    float speed = Monitor_GetSpeed();
    // 根据当前速度使用PI计算占空比
    float duty = MotorCtrl_CalcDuty(speed);
    // 测试用
    // static float duty_set = 4 * 10;
    // duty = duty_set;
    duty_int = (uint8_t)duty;
    // 获取当前转子位置（hall信号）
    uint8_t hallSignal = MotorCtrl_GetHall();
    // 基于正反转、hall信号、占空比应值设定磁矢量
    MotorCtrl_DriveMotor((uint8_t)direction, (uint8_t)duty, hallSignal);
}