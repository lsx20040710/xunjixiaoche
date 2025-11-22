/* track_sensor.c */
#include "track_sensor.h"

/**
 * @brief 读取所有传感器状态
 * @retval Track_Sensor_State_t 包含5个传感器状态的结构体
 */
Track_Sensor_State_t Track_Sensor_Read(void)
{
    Track_Sensor_State_t state;

    // 读取IO口状态 (CubeMX 在 main.h 中定义了这些宏)
    state.Out1 = HAL_GPIO_ReadPin(SENSOR_OUT1_GPIO_Port, SENSOR_OUT1_Pin); // PA1
    state.Out2 = HAL_GPIO_ReadPin(SENSOR_OUT2_GPIO_Port, SENSOR_OUT2_Pin); // PA2
    state.Out3 = HAL_GPIO_ReadPin(SENSOR_OUT3_GPIO_Port, SENSOR_OUT3_Pin); // PA3
    state.Out4 = HAL_GPIO_ReadPin(SENSOR_OUT4_GPIO_Port, SENSOR_OUT4_Pin); // PA4
    state.Out5 = HAL_GPIO_ReadPin(SENSOR_OUT5_GPIO_Port, SENSOR_OUT5_Pin); // PA5

    return state;
}