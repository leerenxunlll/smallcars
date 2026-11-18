#include "task.h"

#include "encoder.h"
#include "i2c.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>

#define OBSTACLE_MARK  0x4EU
#define FRAME_END      0x5AU
#define GARAGE_COUNT   6U
#define UART_DMA_BUFFER_SIZE 64U
#define UART_FIFO_CAPACITY   256U

typedef enum
{
  FRAME_HEADER_SLOT = 0xCCU
} FrameHeader_t;

typedef enum
{
  FRAME_WAIT_HEADER = 0U,//解释：等待帧头状态，表示当前正在等待接收帧的头部信息
  FRAME_READ_STATUS,//解释：读取状态状态，表示当前正在读取帧的状态信息
  FRAME_READ_SLOT,//解释：读取车位状态，表示当前正在读取帧的车位信息
  FRAME_WAIT_END//解释：等待帧尾状态，表示当前正在等待接收帧的尾部信息
} FrameParserState_t;

static uint8_t uart_dma_buffer[UART_DMA_BUFFER_SIZE];//解释：用于存储从串口接收的数据的DMA缓冲区，大小为UART_DMA_BUFFER_SIZE字节
static volatile uint8_t uart_fifo[UART_FIFO_CAPACITY];//解释：用于存储从串口接收的数据的FIFO缓冲区，大小为UART_FIFO_CAPACITY字节
static volatile uint16_t uart_fifo_head;//解释：用于指示FIFO缓冲区中下一个可写入数据的位置的索引
static volatile uint16_t uart_fifo_tail;//解释：用于指示FIFO缓冲区中下一个可读取数据的位置的索引
static uint16_t uart_dma_position;//解释：用于指示DMA缓冲区中下一个可读取数据的位置的索引
static volatile uint8_t frame_ready;//解释：帧就绪标志，用于指示是否接收到完整的帧
static uint8_t garage_status[GARAGE_COUNT];//解释：车位状态数组，用于存储每个车位的状态信息
static uint8_t garage_known_mask;//解释：车位已知状态掩码，用于标记哪些车位的状态已知
static uint8_t display_pending;//解释：显示待处理标志，用于指示是否需要更新OLED显示
static volatile uint8_t uart_error;//解释：串口错误标志，用于指示是否发生了串口通信错误
static FrameParserState_t frame_parser_state = FRAME_WAIT_HEADER;
static FrameHeader_t current_frame_header;//解释：当前帧头信息，用于存储接收到的帧的头部信息
static uint8_t current_frame_status;//解释：当前帧状态，用于存储接收到的帧的状态信息
static uint8_t current_frame_slot;//解释：当前帧车位，用于存储接收到的帧的车位信息

static uint16_t UartFifo_Next(uint16_t index)
{
  ++index;//解释：将索引加1，指向FIFO缓冲区中的下一个位置
  return (index >= UART_FIFO_CAPACITY) ? 0U : index;
}

static void GarageStatus_Process(uint8_t status, uint8_t slot)//解释：处理车位状态的函数，根据接收到的状态和车位编号更新车位状态数组和已知状态掩码
{
  if ((slot < 1U) || (slot > GARAGE_COUNT))
  {
    return;
  }

  garage_status[slot - 1U] = status;//解释：将接收到的状态值存储到车位状态数组中，索引为slot - 1，因为车位编号从1开始，而数组索引从0开始
  garage_known_mask |= (uint8_t)(1U << (slot - 1U));//解释：将已知状态掩码的对应位设置为1，表示该车位的状态已知
  display_pending = 1U;//解释：将显示待处理标志设置为1，表示需要更新OLED显示
}

static void FrameParser_ProcessByte(uint8_t byte)
{
  switch (frame_parser_state)//解释：根据当前的帧解析状态进行处理
  {
    case FRAME_WAIT_HEADER:
      if (byte == (uint8_t)FRAME_HEADER_SLOT)
      {
        current_frame_header = FRAME_HEADER_SLOT;
        frame_parser_state = FRAME_READ_STATUS;
      }
      break;

    case FRAME_READ_STATUS:
      if (byte == (uint8_t)FRAME_HEADER_SLOT)
      {
        current_frame_header = FRAME_HEADER_SLOT;
      }
      else
      {
        current_frame_status = byte;
        frame_parser_state = FRAME_READ_SLOT;
      }
      break;

    case FRAME_READ_SLOT:
      if (byte == (uint8_t)FRAME_HEADER_SLOT)
      {
        current_frame_header = FRAME_HEADER_SLOT;
        frame_parser_state = FRAME_READ_STATUS;
      }
      else
      {
        current_frame_slot = byte;
        frame_parser_state = FRAME_WAIT_END;
      }
      break;

    case FRAME_WAIT_END:
      if (byte == FRAME_END)
      {
        switch (current_frame_header)
        {
          case FRAME_HEADER_SLOT:
            GarageStatus_Process(current_frame_status, current_frame_slot);
            break;
          default:
            break;
        }
        frame_parser_state = FRAME_WAIT_HEADER;
      }
      else if (byte == (uint8_t)FRAME_HEADER_SLOT)
      {
        current_frame_header = FRAME_HEADER_SLOT;
        frame_parser_state = FRAME_READ_STATUS;
      }
      else
      {
        frame_parser_state = FRAME_WAIT_HEADER;
      }
      break;

    default:
      frame_parser_state = FRAME_WAIT_HEADER;
      break;
  }
}

void Task1_UartIdleCallback(void)
//解释：串口空闲中断回调函数，用于处理串口接收数据的空闲中断事件
{
  uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);//解释：获取DMA接收缓冲区中剩余未接收的数据字节数
  uint16_t current_position;//解释：用于存储当前DMA缓冲区中已接收的数据字节数
  uint16_t new_bytes;//解释：用于存储新接收到的数据字节数
  uint16_t i;

  if (remaining == 0U)
  {
    current_position = 0U;//解释：如果DMA缓冲区中没有剩余未接收的数据字节数，则当前接收位置为0
    new_bytes = (uint16_t)(UART_DMA_BUFFER_SIZE - uart_dma_position);//解释：新接收到的数据字节数为DMA缓冲区大小减去当前接收位置
  }
  else
  {
    current_position = (uint16_t)(UART_DMA_BUFFER_SIZE - remaining);//解释：当前接收位置为DMA缓冲区大小减去剩余未接收的数据字节数
    new_bytes = (current_position >= uart_dma_position)
                    ? (uint16_t)(current_position - uart_dma_position)
                    : (uint16_t)(UART_DMA_BUFFER_SIZE - uart_dma_position +
                                 current_position);//解释：新接收到的数据字节数为当前接收位置减去上次接收位置，如果当前接收位置小于上次接收位置，则需要考虑DMA缓冲区的循环情况
  }

  for (i = 0U; i < new_bytes; ++i)
  {
    uint16_t next = UartFifo_Next(uart_fifo_head);//解释：计算FIFO缓冲区中下一个可写入数据的位置的索引

    if (next == uart_fifo_tail)//解释：如果下一个可写入数据的位置与尾部索引相同，表示FIFO缓冲区已满，无法再写入数据
    {
      uart_error = 1U;
      break;
    }

    uart_fifo[uart_fifo_head] = uart_dma_buffer[uart_dma_position];//解释：将DMA缓冲区中当前接收位置的数据字节写入FIFO缓冲区的头部索引位置
    uart_fifo_head = next;//解释：更新FIFO缓冲区的头部索引，指向下一个可写入数据的位置
    ++uart_dma_position;//解释：更新DMA缓冲区的接收位置索引，指向下一个可接收数据的位置
    if (uart_dma_position >= UART_DMA_BUFFER_SIZE)//解释：如果DMA缓冲区的接收位置索引超过了缓冲区大小，则将其重置为0，表示循环回到缓冲区的起始位置
    {
      uart_dma_position = 0U;
    }
  }

  if (i > 0U)//解释：如果有新接收到的数据字节，则将帧就绪标志设置为1，表示有完整的帧可供处理
  {
    frame_ready = 1U;
  }
}

HAL_StatusTypeDef Task_Init(void)
{
  uint8_t i;
  HAL_StatusTypeDef status;

  for (i = 0U; i < GARAGE_COUNT; ++i)
  {
    garage_status[i] = 0U;
  }
  uart_fifo_head = 0U;//解释：初始化FIFO缓冲区的头部索引为0
  uart_fifo_tail = 0U;//解释：初始化FIFO缓冲区的尾部索引为0
  uart_dma_position = 0U;//解释：初始化DMA缓冲区的接收位置索引为0
  frame_ready = 0U;//解释：初始化帧就绪标志为0，表示没有接收到完整的帧
  garage_known_mask = 0U;//解释：初始化车位状态为未知
  display_pending = 1U;//解释：初始化显示待处理标志
  uart_error = 0U;//解释：初始化串口错误标志
  frame_parser_state = FRAME_WAIT_HEADER;

  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  status = HAL_UART_Receive_DMA(&huart1, uart_dma_buffer,
                                sizeof(uart_dma_buffer));
  if (status != HAL_OK)
  {
    return status;
  }
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  encoder_init();
  motor_init();
  if (IMU_Init() != 0U)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  { 
    return HAL_ERROR;
  }
  printf("Task_Init: all initialized successfully.\n");
  return OLED_Init(&hi2c2);
}

static void Garage_Display(const uint8_t status[GARAGE_COUNT], uint8_t known_mask)
{
  static const char garage_names[GARAGE_COUNT] = {'A', 'B', 'C', 'D', 'E', 'F'};
  char label[3];//解释：用于存储车位标签的字符串，长度为3，包括车位名称、冒号和空字符
  uint8_t i;

  OLED_Clear();
  for (i = 0U; i < GARAGE_COUNT; ++i)
  {
    int16_t y = (int16_t)(i * 8U);
    label[0] = garage_names[i];
    label[1] = ':';
    label[2] = '\0';
    OLED_ShowString(0, y, label, OLED_6X8);

    if ((known_mask & (uint8_t)(1U << i)) == 0U)//解释：如果车位状态未知，则显示问号
    {
      OLED_ShowString(12, y, "?", OLED_6X8);
    }
    else if (status[i] == OBSTACLE_MARK)//解释：如果车位状态为障碍物标记，则显示"no stop"
    {
      OLED_ShowString(12, y, "no stop", OLED_6X8);
    }
    else
    {
      uint8_t digits = (status[i] >= 100U) ? 3U :
                       ((status[i] >= 10U) ? 2U : 1U);
      OLED_ShowNum(12, y, status[i], digits, OLED_6X8);//解释：否则显示车位状态的数字，使用OLED_ShowNum函数显示数字，长度根据状态值的大小确定
    }
  }
}

 

TaskState_t Task1_Run(void)
{
  uint8_t snapshot[GARAGE_COUNT];//解释：用于存储车位状态的快照数组，长度为车位数量
  uint8_t known_mask;//解释：用于存储车位已知状态的掩码
  uint8_t i;//解释：用于循环迭代的索引变量

  if (uart_error != 0U)
  {
    return TASK_ERROR;
  }

  if (frame_ready != 0U)
  {
    while (uart_fifo_tail != uart_fifo_head)//解释：当FIFO缓冲区中有数据时，循环处理接收到的数据
    {
      uint8_t byte = uart_fifo[uart_fifo_tail];//解释：从FIFO缓冲区中获取当前尾部索引位置的数据字节

      uart_fifo_tail = UartFifo_Next(uart_fifo_tail);//解释：更新尾部索引，指向下一个可读取数据的位置
      FrameParser_ProcessByte(byte);//解释：将接收到的数据字节传递给帧解析器进行处理，最后得到车位状态信息，存放在garage_status数组中
    }

    frame_ready = 0U;
    if (uart_fifo_tail != uart_fifo_head)
    {
      frame_ready = 1U;
    }
  }

  if (uart_error != 0U)
  {
    return TASK_ERROR;
  }
  if (display_pending == 0U)
  {
    return TASK_RUNNING;
  }

  for (i = 0U; i < GARAGE_COUNT; ++i)
  {
    snapshot[i] = garage_status[i];
  }
  known_mask = garage_known_mask;
  display_pending = 0U;

  Garage_Display(snapshot, known_mask);//解释：调用Garage_Display函数，将车位状态的快照和已知状态掩码传递给OLED显示函数，更新OLED显示屏上的车位状态信息
  if (OLED_Update() != HAL_OK)
  {
    return TASK_ERROR;
  }
  return TASK_RUNNING;
}

void Task1_UartError(void)
{
  uart_error = 1U;
}
