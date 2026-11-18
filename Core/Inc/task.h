#ifndef TASK_H
#define TASK_H

#include "main.h"

//创建任务状态枚举
typedef enum {
    TASK_BENGIN = 0U, //任务初始化状态
    TASK_RUNNING=1U,   //任务运行状态
    TASK_STOP=3U,      //任务停止状态
    TASK_ERROR=2U      //任务错误状态
} TaskState_t;

HAL_StatusTypeDef Task_Init(void);
TaskState_t Task1_Run(void);
void Task1_UartIdleCallback(void);
void Task1_UartError(void);
TaskState_t Task2_Run(void);
TaskState_t Task3_Run(void);
TaskState_t Task4_Run(void);
TaskState_t Task5_Run(void);
TaskState_t Task6_Run(void);


#endif
