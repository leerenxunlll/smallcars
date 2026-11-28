#include "motor.h"
#include "encoder.h"
#include "tim.h"
#include "main.h"
#include "task.h"
#include "imu.h"

#define MOTOR_PWM_FULL_SCALE_PERCENT 100.0f
#define MOTOR_SPEED_SAMPLE_PERIOD_MS 10U
#define MOTOR_COUNT 4U
#define PARKING_BASE_SPEED 120
#define PARKING_INITIAL_TURN_DEG 45.0f
#define PARKING_TARGET_TURN_DEG 90.0f
#define PARKING_APPROACH_DISTANCE_MM 100.0f
#define PARKING_FINAL_DISTANCE_MM 250.0f
#define PARKING_MOVE_TIMEOUT_MS 10000U
#define PARKING_ROTATE_TIMEOUT_MS 10000U

typedef enum {
    PARKING_ROTATE_CLOCKWISE = 1,
    PARKING_ROTATE_COUNTERCLOCKWISE = -1
} ParkingRotationDirection_t;

typedef enum {
    PARKING_TRAVEL_FORWARD = 0,
    PARKING_TRAVEL_BACKWARD
} ParkingTravelDirection_t;

extern Motor motorRL;
extern Motor motorRR;
extern Motor motorFL;
extern Motor motorFR;

typedef struct {
    float integral;
    float previous_measurement;
    uint32_t previous_tick;
    uint8_t initialized;
} MotorPidState;

static MotorPidState motor_pid_states[MOTOR_COUNT];

uint64_t speed2pluse(float speed, TIM_HandleTypeDef *timer)
{
    uint64_t period = (uint64_t)__HAL_TIM_GET_AUTORELOAD(timer) + 1U;
    float drive_counts;

    if (!(speed > 0.0f)) {
        speed = 0.0f;
    } else if (speed > MOTOR_PWM_FULL_SCALE_PERCENT) {
        speed = MOTOR_PWM_FULL_SCALE_PERCENT;
    }

    drive_counts = ((float)period * speed / MOTOR_PWM_FULL_SCALE_PERCENT) + 0.5f;
    return period - (uint64_t)drive_counts;
}

static uint32_t motor_get_oc_mode(TIM_HandleTypeDef *timer, uint32_t channel)//
{
    switch (channel) {
    case TIM_CHANNEL_1:
        return (timer->Instance->CCMR1 & TIM_CCMR1_OC1M) >> TIM_CCMR1_OC1M_Pos;
    case TIM_CHANNEL_2:
        return (timer->Instance->CCMR1 & TIM_CCMR1_OC2M) >> TIM_CCMR1_OC2M_Pos;
    case TIM_CHANNEL_3:
        return (timer->Instance->CCMR2 & TIM_CCMR2_OC3M) >> TIM_CCMR2_OC3M_Pos;
    case TIM_CHANNEL_4:
        return (timer->Instance->CCMR2 & TIM_CCMR2_OC4M) >> TIM_CCMR2_OC4M_Pos;
    default:
        Error_Handler();
        return 0U;
    }
}

static void motor_configure_oc_mode(TIM_HandleTypeDef *timer,
                                    uint32_t channel,
                                    uint32_t mode)
{
    TIM_OC_InitTypeDef config = {0};
    HAL_StatusTypeDef status;

    config.OCMode = mode;
    config.Pulse = 0U;
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;
    config.OCIdleState = TIM_OCIDLESTATE_RESET;
    config.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (mode == TIM_OCMODE_PWM1) {
        status = HAL_TIM_PWM_ConfigChannel(timer, &config, channel);
    } else {
        status = HAL_TIM_OC_ConfigChannel(timer, &config, channel);
    }

    if (status != HAL_OK) {
        Error_Handler();
    }
}

static void motor_set_compare(TIM_HandleTypeDef *timer,
                              uint32_t channel,
                              uint64_t compare,
                              uint64_t period)
{
    if (compare > UINT16_MAX) {
        if (compare != period) {
            Error_Handler();
            return;
        }
        if (motor_get_oc_mode(timer, channel) !=
            (TIM_OCMODE_FORCED_ACTIVE >> TIM_CCMR1_OC1M_Pos)) {
            motor_configure_oc_mode(timer, channel, TIM_OCMODE_FORCED_ACTIVE);
        }
        return;
    }

    if (motor_get_oc_mode(timer, channel) !=
        (TIM_OCMODE_PWM1 >> TIM_CCMR1_OC1M_Pos)) {
        motor_configure_oc_mode(timer, channel, TIM_OCMODE_PWM1);
    }
    __HAL_TIM_SET_COMPARE(timer, channel, (uint32_t)compare);
}

void motor_init(void)
{
    uint32_t index;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    for (index = 0U; index < MOTOR_COUNT; ++index) {
        motor_pid_states[index].integral = 0.0f;
        motor_pid_states[index].previous_measurement = 0.0f;
        motor_pid_states[index].previous_tick = HAL_GetTick();
        motor_pid_states[index].initialized = 0U;
    }
}

void motor_run(Motor motor)
{
    TIM_HandleTypeDef *timer;
    uint32_t in1_channel;
    uint32_t in2_channel;
    int motor_direction;
    uint64_t period;
    uint64_t pwm_compare;
    uint64_t fixed_high_compare;

    if (motor.dir != 0 && motor.dir != 1) {
        Error_Handler();
        return;
    }

    switch (motor.motor_id) {
    case 1:
        timer = &htim1;
        in1_channel = TIM_CHANNEL_1;
        in2_channel = TIM_CHANNEL_2;
        break;
    case 2:
        timer = &htim1;
        in1_channel = TIM_CHANNEL_3;
        in2_channel = TIM_CHANNEL_4;
        break;
    case 3:
        timer = &htim9;
        in1_channel = TIM_CHANNEL_1;
        in2_channel = TIM_CHANNEL_2;
        break;
    case 4:
        timer = &htim12;
        in1_channel = TIM_CHANNEL_1;
        in2_channel = TIM_CHANNEL_2;
        break;
    default:
        Error_Handler();
        return;
    }

    motor_direction = motor.dir;
    /* Motor 1 is correctly oriented; motors 2-4 need the opposite drive polarity. */
    if (motor.motor_id != 1) {
        motor_direction = 1 - motor_direction;
    }

    period = (uint64_t)__HAL_TIM_GET_AUTORELOAD(timer) + 1U;
    pwm_compare = speed2pluse((float)motor.speed, timer);
    fixed_high_compare = period;

    if (motor.speed <= 0.0) {
        motor_set_compare(timer, in1_channel, fixed_high_compare, period);
        motor_set_compare(timer, in2_channel, fixed_high_compare, period);
    } else if (motor_direction == 1) {
        motor_set_compare(timer, in1_channel, fixed_high_compare, period);
        motor_set_compare(timer, in2_channel, pwm_compare, period);
    } else {
        motor_set_compare(timer, in1_channel, pwm_compare, period);
        motor_set_compare(timer, in2_channel, fixed_high_compare, period);
    }
}

void motor_pid_run(Motor motor)
{
    uint32_t index;
    uint32_t current_tick;
    uint32_t elapsed_ms;
    float target_speed;
    float dt_seconds;
    float measured_speed;
    float error;
    float derivative;
    float candidate_integral;
    float output;
    float raw_output;
    MotorPidState *state;

    if (motor.motor_id < 1 || motor.motor_id > (int)MOTOR_COUNT ||
        (motor.dir != 0 && motor.dir != 1) ||
        !(motor.speed >= 0.0) ||
        !(motor.p >= 0.0f) ||
        !(motor.i >= 0.0f) ||
        !(motor.d >= 0.0f)) {
        Error_Handler();
        return;
    }

    index = (uint32_t)(motor.motor_id - 1);
    state = &motor_pid_states[index];
    current_tick = HAL_GetTick();
    elapsed_ms = (uint32_t)(current_tick - state->previous_tick);
    if (elapsed_ms < MOTOR_SPEED_SAMPLE_PERIOD_MS) {
        return;
    }

    dt_seconds = (float)elapsed_ms / 1000.0f;
    state->previous_tick = current_tick;

    target_speed = (float)motor.speed;
    measured_speed = encoder_get_speed(motor.motor_id);
    if (motor.dir == 0) {
        measured_speed = -measured_speed;
    }

    if (state->initialized == 0U) {
        state->previous_measurement = measured_speed;
        state->initialized = 1U;
    }

    if (target_speed == 0.0f) {
        state->integral = 0.0f;
        state->previous_measurement = measured_speed;
        motor.speed = 0.0;
        motor_run(motor);
        return;
    }

    error = target_speed - measured_speed;
    derivative = -(measured_speed - state->previous_measurement) / dt_seconds;
    candidate_integral = state->integral + error * dt_seconds;
    raw_output = motor.p * error +
                 motor.i * candidate_integral +
                 motor.d * derivative;

    if ((raw_output >= 0.0f && raw_output <= MOTOR_PWM_FULL_SCALE_PERCENT) ||
        (raw_output > MOTOR_PWM_FULL_SCALE_PERCENT && error < 0.0f) ||
        (raw_output < 0.0f && error > 0.0f)) {
        state->integral = candidate_integral;
    }

    output = motor.p * error +
             motor.i * state->integral +
             motor.d * derivative;
    if (output < 0.0f) {
        output = 0.0f;
    } else if (output > MOTOR_PWM_FULL_SCALE_PERCENT) {
        output = MOTOR_PWM_FULL_SCALE_PERCENT;
    }

    state->previous_measurement = measured_speed;
    motor.speed = output;
    motor_run(motor);
}

static void Parking_StopMotors(void)
{
    motorRL.speed = 0.0;
    motorRR.speed = 0.0;
    motorFL.speed = 0.0;
    motorFR.speed = 0.0;
    motor_run(motorRL);
    motor_run(motorRR);
    motor_run(motorFL);
    motor_run(motorFR);
    HAL_Delay(10U);
    motor_pid_run(motorRL);
    motor_pid_run(motorRR);
    motor_pid_run(motorFL);
    motor_pid_run(motorFR);
}

static void Parking_ApplyTaskMotorSettings(void)
{
    motorRL = (Motor){1, 0.0, 1, 0.15f, 0.05f, 0.0f};
    motorRR = (Motor){2, 0.0, 1, 0.15f, 0.0615f, 0.0f};
    motorFL = (Motor){3, 0.0, 1, 0.132f, 0.04f, 0.0f};
    motorFR = (Motor){4, 0.0, 1, 0.145f, 0.051f, 0.0f};
}

static void Parking_RestoreTaskMotorSettings(void)
{
    Parking_ApplyTaskMotorSettings();
    Parking_StopMotors();
}
//顺时针向前转动
static void Parking_ApplyClockwiseForwardSettings(void)
{
    motorRL = (Motor){1, PARKING_BASE_SPEED + 50, 1, 0.15f, 0.05f, 0.0f};
    motorRR = (Motor){2, PARKING_BASE_SPEED + 100, 0, 1.5f, 0.0615f, 0.0f};
    motorFL = (Motor){3, PARKING_BASE_SPEED + 50, 1, 0.132f, 0.04f, 0.0f};
    motorFR = (Motor){4, 0.0, 0, 0.0f, 0.0f, 0.0f};
}

static void Parking_ApplyCounterclockwiseBackwardSettings(void)
{
    motorRL = (Motor){1, PARKING_BASE_SPEED + 50, 0, 0.15f, 0.05f, 0.0f};
    motorRR = (Motor){2, 0, 0, 0.0f, 0.0f, 0.0f};
    motorFL = (Motor){3, PARKING_BASE_SPEED + 50, 0, 0.132f, 0.04f, 0.0f};
    motorFR = (Motor){4, PARKING_BASE_SPEED + 100, 1, 1.45f, 0.051f, 0.0f};
}

static void Parking_ApplyClockwiseBackwardSettings(void)
{
    motorRL = (Motor){1, 0, 0, 0.0f, 0.0f, 0.0f};
    motorRR = (Motor){2, PARKING_BASE_SPEED + 50, 0, 0.15f, 0.0615f, 0.0f};
    motorFL = (Motor){3, PARKING_BASE_SPEED + 100, 1, 1.32f, 0.04f, 0.0f};
    motorFR = (Motor){4, PARKING_BASE_SPEED + 50, 0, 0.145f, 0.051f, 0.0f};
}

static void Parking_ApplyCounterclockwiseForwardSettings(void)
{
    motorRL = (Motor){1, PARKING_BASE_SPEED + 100, 0, 1.5f, 0.05f, 0.0f};
    motorRR = (Motor){2, PARKING_BASE_SPEED + 50, 1, 0.15f, 0.0615f, 0.0f};
    motorFL = (Motor){3, 0, 0, 0.0f, 0.0f, 0.0f};
    motorFR = (Motor){4, PARKING_BASE_SPEED + 50, 1, 0.145f, 0.051f, 0.0f};
}

static void Parking_ApplyRotationSettings(
    ParkingRotationDirection_t rotation_direction,
    ParkingTravelDirection_t travel_direction)
{
    if (rotation_direction == PARKING_ROTATE_CLOCKWISE) {
        if (travel_direction == PARKING_TRAVEL_FORWARD) {
            Parking_ApplyClockwiseForwardSettings();
        } else {
            Parking_ApplyClockwiseBackwardSettings();
        }
    } else if (travel_direction == PARKING_TRAVEL_FORWARD) {
        Parking_ApplyCounterclockwiseForwardSettings();
    } else {
        Parking_ApplyCounterclockwiseBackwardSettings();
    }
}

static float Parking_GetDirectedAngleDifference(float start_yaw,
                                                float current_yaw,
                                                ParkingRotationDirection_t
                                                    rotation_direction)
{
    float angle_difference = current_yaw - start_yaw;

    return (rotation_direction == PARKING_ROTATE_CLOCKWISE)
               ? angle_difference
               : -angle_difference;
}

static TaskParkingStatus_t Parking_RotateToAngle(
    float start_yaw,
    float target_angle_deg,
    ParkingRotationDirection_t rotation_direction)
{
    IMU_Data_t imu_data;
    uint32_t start_tick = HAL_GetTick();
    float angle_diff;

    while ((uint32_t)(HAL_GetTick() - start_tick) <
           PARKING_ROTATE_TIMEOUT_MS) {
        IMU_Update();
        if (IMU_IsReady() == 0U) {
            Parking_StopMotors();
            return TASK_PARKING_IMU_ERROR;
        }

        IMU_GetData(&imu_data);
        angle_diff = Parking_GetDirectedAngleDifference(
            start_yaw, imu_data.yaw_cont_deg, rotation_direction);
        if (angle_diff >= target_angle_deg) {
            Parking_StopMotors();
            return TASK_PARKING_SUCCESS;
        }

        motor_pid_run(motorRL);
        motor_pid_run(motorRR);
        motor_pid_run(motorFL);
        motor_pid_run(motorFR);
        HAL_Delay(1U);
    }

    Parking_StopMotors();
    return TASK_PARKING_ROTATE_TIMEOUT;
}

TaskParkingStatus_t Task_ReverseParking(void)
{
    IMU_Data_t imu_data;
    TaskParkingStatus_t status;
    float start_yaw;
    float angle_diff;
    const ParkingRotationDirection_t rotation_direction =
        PARKING_ROTATE_COUNTERCLOCKWISE;
    const ParkingTravelDirection_t rotation_travel_direction =
        PARKING_TRAVEL_BACKWARD;

    if (IMU_IsReady() == 0U) {
        Parking_RestoreTaskMotorSettings();
        return TASK_PARKING_IMU_ERROR;
    }

    IMU_Update();
    IMU_GetData(&imu_data);
    start_yaw = imu_data.yaw_cont_deg;

    Parking_ApplyRotationSettings(rotation_direction,
                                  rotation_travel_direction);
    status = Parking_RotateToAngle(start_yaw,
                                   PARKING_INITIAL_TURN_DEG,
                                   rotation_direction);
    if (status != TASK_PARKING_SUCCESS) {
        Parking_RestoreTaskMotorSettings();
        return status;
    }

    Parking_ApplyTaskMotorSettings();
    if (Task_MoveDistance(TASK_MOVE_BACKWARD,
                          PARKING_APPROACH_DISTANCE_MM,
                          PARKING_MOVE_TIMEOUT_MS) != TASK_MOVE_SUCCESS) {
        Parking_RestoreTaskMotorSettings();
        return TASK_PARKING_MOVE_ERROR;
    }

    IMU_Update();
    if (IMU_IsReady() == 0U) {
        Parking_RestoreTaskMotorSettings();
        return TASK_PARKING_IMU_ERROR;
    }
    IMU_GetData(&imu_data);
    angle_diff = Parking_GetDirectedAngleDifference(
        start_yaw, imu_data.yaw_cont_deg, rotation_direction);
    if (angle_diff < PARKING_TARGET_TURN_DEG) {
        Parking_ApplyRotationSettings(rotation_direction,
                                      rotation_travel_direction);
        status = Parking_RotateToAngle(start_yaw,
                                       PARKING_TARGET_TURN_DEG,
                                       rotation_direction);
        if (status != TASK_PARKING_SUCCESS) {
            Parking_RestoreTaskMotorSettings();
            return status;
        }
    }

    Parking_ApplyTaskMotorSettings();
    if (Task_MoveDistance(TASK_MOVE_BACKWARD,
                          PARKING_FINAL_DISTANCE_MM,
                          PARKING_MOVE_TIMEOUT_MS) != TASK_MOVE_SUCCESS) {
        Parking_RestoreTaskMotorSettings();
        return TASK_PARKING_MOVE_ERROR;
    }

    Parking_RestoreTaskMotorSettings();
    return TASK_PARKING_SUCCESS;
}

