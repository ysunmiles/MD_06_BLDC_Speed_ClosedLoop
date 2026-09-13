#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "OLED.h"
#include "motorCtrl.h"

static uint8_t lastHallSignal = 0xFF;

static volatile uint8_t motorState;
static MotorDirection motorDirection = MOTOR_DIR_FORWARD;
static uint16_t duty = 10;
static float speed, speedSample = 0;
static float speedAim = 500;

static float Kp = 0.002;
static float Ki = 0.00005;
static float intgr = 0;

void MotorCtrl_SetShutdown(GPIO_PinState State)
{
    HAL_GPIO_WritePin(CTRL_SD_GPIO_Port, CTRL_SD_Pin, State);
}

void MotorCtrl_SetSpeedAim(uint16_t cmdSpeed)
{
    speedAim = (float)cmdSpeed;
}

void MotorCtrl_SetDuty(uint16_t uartDuty)
{
    duty = uartDuty;
}
uint16_t MotorCtrl_GetDuty(void)
{
    return duty;
}
uint16_t MotorCtrl_GetSpeed(void)
{
    return (uint16_t)speed;
}


static float updateSpeedAverage(float sample)
{
    static float samples[SPEED_AVERAGE_WINDOW] = {0.0f};
    static float sum = 0.0f;
    static uint32_t nextSample = 0U;
    static uint32_t sampleCount = 0U;

    sum -= samples[nextSample];
    samples[nextSample] = sample;
    sum += sample;

    nextSample = (nextSample + 1U) % SPEED_AVERAGE_WINDOW;
    if (sampleCount < SPEED_AVERAGE_WINDOW)
    {
        sampleCount++;
    }

    return sum / (float)sampleCount;
}

void MotorCtrl_SetSpeedZero(void)
{
    speedSample = 0;
    speed = updateSpeedAverage(speedSample);
    HAL_TIM_Base_Start_IT(&htim5);
}

void MotorCtrl_Reset(void)
{
    HAL_TIM_Base_Stop(&htim1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_RESET);
}

void MotorCtrl_DriveMotor(uint8_t hallSignal)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);

    if (hallSignal == lastHallSignal){
        return;
    }else{
        lastHallSignal = hallSignal;
    }

    MotorCtrl_Reset();
    if (motorDirection == 1)
    {
        switch (hallSignal) {
            case 5: // U+ V-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
                break;
            case 1: // U+ W-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
                break;
            case 3: // V+ W-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
                break;
            case 2: // V+ U-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
                break;
            case 6: // W+ U-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
                HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
                break;
            case 4: // W+ V-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
                HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
                break;
            default:
                break;
        }
    }
    else if (motorDirection == 2)
    {
        switch (hallSignal) {
            case 5: // V+ U-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
                break;
            case 1: // W+ U-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
                HAL_GPIO_WritePin(PWM_UL_GPIO_Port, PWM_UL_Pin, GPIO_PIN_SET);
                break;
            case 3: // W+ V-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
                HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
                break;
            case 2: // U+ V-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(PWM_VL_GPIO_Port, PWM_VL_Pin, GPIO_PIN_SET);
                break;
            case 6: // U+ W-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
                break;
            case 4: // V+ W-
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_GPIO_WritePin(PWM_WL_GPIO_Port, PWM_WL_Pin, GPIO_PIN_SET);
                break;
            default:
                break;
        }
    }
    HAL_TIM_Base_Start_IT(&htim1);
}

void StartMotorCtrlTask(void *argument)
{
    for(;;)
    {
        uint8_t keyValue = ulTaskNotifyTake(pdTRUE, osWaitForever);
        lastHallSignal = 0xFF;
        intgr = 0;
        if (keyValue == 1)
        {
            if (motorState==0 || motorState==2)
            {
                motorDirection = MOTOR_DIR_FORWARD;
                MotorCtrl_SetShutdown(GPIO_PIN_SET);
                MotorCtrl_PWMCallback();
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
                MotorCtrl_PWMCallback();
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
        HAL_TIM_Base_Stop_IT(&htim5);
        tickus = __HAL_TIM_GET_COUNTER(&htim5);
        if (tickus == 0) {   
            HAL_TIM_Base_Start_IT(&htim5);
            return;
        }
        speedSample = (float)60e6/(tickus*6*2);
        speed = updateSpeedAverage(speedSample);
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        HAL_TIM_Base_Start_IT(&htim5);
    }
}

uint8_t MotorCtrl_GetHall(void)
{
    uint8_t hallu, hallv, hallw;
    hallu = HAL_GPIO_ReadPin(HALLU_GPIO_Port, HALLU_Pin);
    hallv = HAL_GPIO_ReadPin(HALLV_GPIO_Port, HALLV_Pin);
    hallw = HAL_GPIO_ReadPin(HALLW_GPIO_Port, HALLW_Pin);

    uint8_t hallSignal = (hallw<<2)|(hallv<<1)|(hallu);
    return hallSignal;
}

void MotorCtrl_CalcDuty(void)
{
    float speed_diff = speedAim - speed;
    float duty_p = Kp * speed_diff;
    intgr += speed_diff;
    float duty_i = Ki * intgr;

    duty = duty_p + duty_i;
}

void MotorCtrl_PWMCallback(void)
{
    // 获取hall信号
    uint8_t hallSignal = MotorCtrl_GetHall();
    // 计算duty
    MotorCtrl_CalcDuty();
    // 通过hall信号设定磁矢量
    MotorCtrl_DriveMotor(hallSignal);
}