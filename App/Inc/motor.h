/* motor.h */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h" // 包含 CubeMX 生成的引脚定义和 HAL 库

// 从 main.c 中导入 TIM1 句柄
// 确保在 main.c 中，TIM_HandleTypeDef htim1; 是一个全局变量
extern TIM_HandleTypeDef htim1;

// PWM 周期 (ARR 寄存器的值)。
// 假设 CubeMX 中 TIM1 的 Prescaler=71, Period=9999
// (72MHz / (71+1) = 1MHz. 1MHz / 10000 = 100Hz)
#define PWM_PERIOD      (9999) // 您的 CubeMX ARR 值
#define PWM_MAX_DUTY    PWM_PERIOD

/**
 * @brief 设置左右轮速度和方向
 * @param LeftSpeed 左轮速度 (-9999 到 9999)。正为前进，负为后退。
 * @param RightSpeed 右轮速度 (-9999 到 9999)。正为前进，负为后退。
 */
void Motor_Set_Speed(int16_t LeftSpeed, int16_t RightSpeed);

#endif /* __MOTOR_H */