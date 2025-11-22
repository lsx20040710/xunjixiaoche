/* track_sensor.h */
#ifndef __TRACK_SENSOR_H
#define __TRACK_SENSOR_H

#include "main.h" // 包含 CubeMX 生成的引脚定义

// 循迹状态结构体
typedef struct {
    uint8_t Out1;   // 连接到 PA1 - 物理最右侧
    uint8_t Out2;   // 连接到 PA2 - 物理右中
    uint8_t Out3;   // 连接到 PA3 - 物理正中
    uint8_t Out4;   // 连接到 PA4 - 物理左中
    uint8_t Out5;   // 连接到 PA5 - 物理最左侧
} Track_Sensor_State_t;

/**
 * @brief 读取所有传感器状态
 * @retval Track_Sensor_State_t 包含5个传感器状态的结构体
 * @note 假设: 黑线=0 (GPIO_PIN_RESET), 白线=1 (GPIO_PIN_SET)
 */
Track_Sensor_State_t Track_Sensor_Read(void);

#endif /* __TRACK_SENSOR_H */