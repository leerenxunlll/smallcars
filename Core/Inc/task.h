#ifndef TASK_H
#define TASK_H

#include "main.h"
//创建任务状态枚举
typedef enum {
    TASK_BENGIN = 0U, //任务初始化状态
    TASK_RUNNING=1U,   //任务运行状态
    LINE_STATE =2U,     //循迹状态
    LINE_STOP=3U,      //循迹停止状态
    TASK_STOP=4U,      //任务停止状态
    TASK_ERROR=5U      //任务错误状态
} TaskState_t;

typedef enum {
    TASK_ROTATE_LEFT = 0U,
    TASK_ROTATE_RIGHT = 1U
} TaskRotateDirection_t;

typedef enum {
    TASK_ROTATE_SUCCESS = 0U,
    TASK_ROTATE_INVALID_ARGUMENT,
    TASK_ROTATE_IMU_ERROR
} TaskRotateStatus_t;

typedef enum {
    TASK_MOVE_FORWARD = 0U,
    TASK_MOVE_BACKWARD
} TaskMoveDirection_t;

typedef enum {
    TASK_MOVE_SUCCESS = 0U,
    TASK_MOVE_INVALID_ARGUMENT,
    TASK_MOVE_TIMEOUT
} TaskMoveStatus_t;

HAL_StatusTypeDef Task_Init(void);
TaskRotateStatus_t Task_Rotate(TaskRotateDirection_t direction,
                               float speed_mm_s,
                               float angle_deg);
TaskMoveStatus_t Task_MoveDistance(TaskMoveDirection_t direction,
                                   float distance_mm,
                                   uint32_t timeout_ms);
TaskState_t Task1_Run(void);
void Task1_UartIdleCallback(void);
void Task1_UartError(void);
TaskState_t Task2_Run(void);
TaskState_t Task3_Run(void);
TaskState_t Task4_Run(void);
TaskState_t Task5_Run(void);
TaskState_t Task6_Run(void);


#endif
