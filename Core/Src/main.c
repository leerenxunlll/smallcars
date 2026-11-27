/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "imu.h"
#include "motor.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
all_task_state_t current_task_state=SELECT_TASK;//创建全局变量，表示当前任务状态
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_DIR_FORWARD 1
#define MOTOR_DIR_BACKWARD 0
#define BASE_SPEED 120

#define FIRST_ROTATE_TARGET_ANGLE_DEG 45.0f
#define TOTAL_ROTATE_TARGET_ANGLE_DEG 90.0f
#define MOVE_DISTANCE_MM 100.0f
#define FINAL_FORWARD_DISTANCE_MM 250.0f
#define MOVE_TIMEOUT_MS 10000U
#define ROTATE_STAGE_TIMEOUT_MS 10000U
#define KEY_DEBOUNCE_MS 20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const Motor motorRL_phase1 = {1, BASE_SPEED+50, 1, 0.15f, 0.05f, 0.0f};
static const Motor motorRR_phase1 = {2, BASE_SPEED+100, 0, 1.5f, 0.0615f, 0.0f};
static const Motor motorFL_phase1 = {3, BASE_SPEED+50, 1, 0.132f, 0.04f, 0.0f};
static const Motor motorFR_phase1 = {4, 0, 0, 0.0f, 0.0f, 0.0f};

static const Motor motorRL_phase2 = {1, BASE_SPEED+50, 1, 0.15f, 0.05f, 0.0f};
static const Motor motorRR_phase2 = {2, BASE_SPEED+100, 0, 1.5f, 0.0615f, 0.0f};
static const Motor motorFL_phase2 = {3, BASE_SPEED+100, 1, 0.132f, 0.04f, 0.0f};
static const Motor motorFR_phase2 = {4, 0, 0, 0.0f, 0.0f, 0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern Motor motorFL;
extern Motor motorFR;
extern Motor motorRL;
extern Motor motorRR;

static void Apply_Motor_Settings(Motor *motor, const Motor *settings)
{
  motor->speed = settings->speed;
  motor->dir = settings->dir;
  motor->p = settings->p;
  motor->i = settings->i;
  motor->d = settings->d;
}

static void Apply_Rotation_Settings(const Motor *rl,
                                   const Motor *rr,
                                   const Motor *fl,
                                   const Motor *fr)
{
  Apply_Motor_Settings(&motorRL, rl);
  Apply_Motor_Settings(&motorRR, rr);
  Apply_Motor_Settings(&motorFL, fl);
  Apply_Motor_Settings(&motorFR, fr);
}

static void Stop_Test_Motors(void)
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

static uint8_t Rotate_To_Angle(float start_yaw,
                               float target_angle_deg,
                               const Motor *rl,
                               const Motor *rr,
                               const Motor *fl,
                               const Motor *fr)
{
  IMU_Data_t imu_data;
  float angle_diff;
  uint8_t completed = 0U;
  uint32_t start_tick = HAL_GetTick();

  if ((IMU_IsReady() == 0U) || (target_angle_deg == 0.0f))
  {
    return 0U;
  }

  Apply_Rotation_Settings(rl, rr, fl, fr);

  while ((uint32_t)(HAL_GetTick() - start_tick) < ROTATE_STAGE_TIMEOUT_MS)
  {
    IMU_Update();
    if (IMU_IsReady() == 0U)
    {
      break;
    }

    IMU_GetData(&imu_data);
    angle_diff = imu_data.yaw_cont_deg - start_yaw;

    if ((target_angle_deg > 0.0f && angle_diff >= target_angle_deg) ||
        (target_angle_deg < 0.0f && angle_diff <= target_angle_deg))
    {
      completed = 1U;
      break;
    }

    motor_pid_run(motorRL);
    motor_pid_run(motorRR);
    motor_pid_run(motorFL);
    motor_pid_run(motorFR);
    HAL_Delay(1U);
  }

  Stop_Test_Motors();

  return completed;
}

static uint8_t Motor_Angle_Sequence(void)
{
  IMU_Data_t imu_data;
  float start_yaw;
  float angle_diff;

  if (IMU_IsReady() == 0U)
  {
    return 0U;
  }

  IMU_Update();
  IMU_GetData(&imu_data);
  start_yaw = imu_data.yaw_cont_deg;

  if (Rotate_To_Angle(start_yaw, FIRST_ROTATE_TARGET_ANGLE_DEG,
                      &motorRL_phase1, &motorRR_phase1,
                      &motorFL_phase1, &motorFR_phase1) == 0U)
  {
    return 0U;
  }

  if (Task_MoveDistance(TASK_MOVE_FORWARD, MOVE_DISTANCE_MM,
                        MOVE_TIMEOUT_MS) != TASK_MOVE_SUCCESS)
  {
    Stop_Test_Motors();
    return 0U;
  }

  IMU_Update();
  IMU_GetData(&imu_data);
  angle_diff = imu_data.yaw_cont_deg - start_yaw;
  if (angle_diff < TOTAL_ROTATE_TARGET_ANGLE_DEG &&
      Rotate_To_Angle(start_yaw, TOTAL_ROTATE_TARGET_ANGLE_DEG,
                      &motorRL_phase2, &motorRR_phase2,
                      &motorFL_phase2, &motorFR_phase2) == 0U)
  {
    return 0U;
  }

  if (Task_MoveDistance(TASK_MOVE_FORWARD, FINAL_FORWARD_DISTANCE_MM,
                        MOVE_TIMEOUT_MS) != TASK_MOVE_SUCCESS)
  {
    Stop_Test_Motors();
    return 0U;
  }

  return 1U;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  if (Task_Init() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IMU_Update();
    if (key1_read == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DEBOUNCE_MS);
      if (key1_read == GPIO_PIN_RESET)
      {
        if (Motor_Angle_Sequence() == 0U)
        {
          Error_Handler();
        }
        while (key1_read == GPIO_PIN_RESET)
        {
          IMU_Update();
          HAL_Delay(10U);
        }
      }
    }
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
