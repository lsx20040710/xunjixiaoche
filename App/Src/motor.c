/* motor.c */
#include "motor.h"

/**
 * @brief 设置左右轮速度和方向
 * @param LeftSpeed 左轮速度 (-9999 到 9999)。
 * @param RightSpeed 右轮速度 (-9999 到 9999)。
 */
void Motor_Set_Speed(int16_t LeftSpeed, int16_t RightSpeed)
{
    uint16_t Left_PWM_Duty = 0;
    uint16_t Right_PWM_Duty = 0;

    // --- 左轮控制 (IN1, IN2, ENA) ---
    // (IN1=PB1, IN2=PB2, ENA=PA8/TIM1_CH1)
    if (LeftSpeed >= 0) // 前进
    {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
        Left_PWM_Duty = (uint16_t)LeftSpeed;
    }
    else // 后退
    {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
        Left_PWM_Duty = (uint16_t)(-LeftSpeed);
    }
    
    if (Left_PWM_Duty > PWM_MAX_DUTY) 
    {
        Left_PWM_Duty = PWM_MAX_DUTY;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Left_PWM_Duty);


    // --- 右轮控制 (IN3, IN4, ENB) ---
    // (IN3=PB3, IN4=PB4, ENB=PA9/TIM1_CH2)
    if (RightSpeed >= 0) // 前进
    {
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
        Right_PWM_Duty = (uint16_t)RightSpeed;
    }
    else // 后退
    {
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);
        Right_PWM_Duty = (uint16_t)(-RightSpeed);
    }

    if (Right_PWM_Duty > PWM_MAX_DUTY) 
    {
        Right_PWM_Duty = PWM_MAX_DUTY;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Right_PWM_Duty);
}