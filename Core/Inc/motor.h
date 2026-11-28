#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

//创建电机结构体
typedef struct {
    int motor_id; // 电机编号
    double speed; // motor_run使用PWM百分比；motor_pid_run的输入目标为毫米/秒
    int dir;     // 电机方向：1前进，0后退
    float p;
    float i;
    float d;
} Motor;

typedef enum {
    TASK_PARKING_SUCCESS = 0U,
    TASK_PARKING_IMU_ERROR,
    TASK_PARKING_ROTATE_TIMEOUT,
    TASK_PARKING_MOVE_ERROR
} TaskParkingStatus_t;

void motor_init(void);
uint64_t speed2pluse(float speed, TIM_HandleTypeDef *timer);
void motor_run(Motor motor);
/* Call periodically; sampling and PID calculations are handled in motor.c. */
void motor_pid_run(Motor motor);
TaskParkingStatus_t Task_ReverseParking(void);

#endif
