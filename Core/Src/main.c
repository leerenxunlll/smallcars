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
#include "encoder.h"
#include "imu.h"
#include <stdint.h>
#include <stdio.h>
#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
all_task_state_t current_task_state=SELECT_TASK;//创建全局变量，表示当前任务状态
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern Motor motorFL;
extern Motor motorFR;
extern Motor motorRL;
extern Motor motorRR;
#define ROTATE_ANGLE_DEG 45.0f
#define ROTATE_SPEED_MM_S 250.0
#define KEY_DEBOUNCE_MS 20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t last_encoder_print_tick;
  uint32_t key_debounce_tick;
  IMU_Data_t imu_data;
  float start_yaw;
  float angle_diff;

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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//Motor motorRL = {1, 200, 1, 0.15f, 0.05f, 0.0f};
//Motor motorRR = {2, 200, 0, 0.15f, 0.0615f, 0.0f};
//Motor motorFL = {3, 200, 1, 0.132f, 0.04f, 0.0f};
//Motor motorFR = {4, 200, 0, 0.145f, 0.051f, 0.0f};
printf("successful initialization\n");
  motorFL.speed = 0.0;
  motorFR.speed = 0.0;
  motorRL.speed = 0.0;
  motorRR.speed = 0.0;
  motor_run(motorFL);
  motor_run(motorFR);
  motor_run(motorRL);
  motor_run(motorRR);

  last_encoder_print_tick = HAL_GetTick();
  while (1)
  {
    uint32_t now;

    //motor_pid_run(motorFL);
    //motor_pid_run(motorFR);
    //motor_pid_run(motorRL);
    //motor_pid_run(motorRR);

   
  


    //if (Task1_Run() == TASK_ERROR)
    //{ 
    //  printf("Task1_Run() returned TASK_ERROR\n");
      //Error_Handler();
    //}

    IMU_Update();
    if (key1_read == GPIO_PIN_RESET)
    {
      key_debounce_tick = HAL_GetTick();
      while ((uint32_t)(HAL_GetTick() - key_debounce_tick) < KEY_DEBOUNCE_MS)
      {
        IMU_Update();
        HAL_Delay(1U);
      }

      if (key1_read == GPIO_PIN_RESET)
      {
        IMU_GetData(&imu_data);
        start_yaw = imu_data.yaw_cont_deg;

        motorFL.speed = ROTATE_SPEED_MM_S;
        motorRL.speed = ROTATE_SPEED_MM_S;
        motorFR.speed = ROTATE_SPEED_MM_S;
        motorRR.speed = ROTATE_SPEED_MM_S;
        motorFL.dir = 1;
        motorRL.dir = 1;
        motorFR.dir = 0;
        motorRR.dir = 0;

        while (1)
        {
          IMU_Update();
          IMU_GetData(&imu_data);
          angle_diff = imu_data.yaw_cont_deg - start_yaw;
          if (angle_diff >= ROTATE_ANGLE_DEG ||
              angle_diff <= -ROTATE_ANGLE_DEG)
          {
            break;
          }

          motor_pid_run(motorFL);
          motor_pid_run(motorRL);
          motor_pid_run(motorFR);
          motor_pid_run(motorRR);
        }

        motorFL.speed = 0.0;
        motorFR.speed = 0.0;
        motorRL.speed = 0.0;
        motorRR.speed = 0.0;
        motor_run(motorFL);
        motor_run(motorFR);
        motor_run(motorRL);
        motor_run(motorRR);

        do
        {
          while (key1_read == GPIO_PIN_RESET)
          {
            IMU_Update();
            HAL_Delay(1U);
          }
          key_debounce_tick = HAL_GetTick();
          while ((uint32_t)(HAL_GetTick() - key_debounce_tick) <
                 KEY_DEBOUNCE_MS)
          {
            IMU_Update();
            HAL_Delay(1U);
          }
        } while (key1_read == GPIO_PIN_RESET);
      }
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - last_encoder_print_tick) >= 1000U)
    {
      last_encoder_print_tick = now;
      printf("motorRL:%+ld mm/s\n",
             (long)encoder_get_speed(1));
            
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   
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
