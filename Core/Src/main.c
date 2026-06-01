/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 빔 밸런싱 애플리케이션의 메인 프로그램 본체.
  *
  * 프로젝트 목적:
  * - 빔 밸런싱 실험을 위해 거리 관련 센서 데이터를 100Hz로 취득한다.
  * - 광학 거리 센서를 ADC1 정규 변환으로 측정한다.
  * - HC-SR04 초음파 센서를 TIM1 입력 캡처로 측정한다.
  * - SG-90 서보모터를 TIM4 PWM 출력으로 구동한다.
  * - UART 기반 런타임 모니터링 및 명령 입력 기능을 제공한다.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint16_t channel[3U];
} AdcFrame_t;

typedef struct
{
  AdcFrame_t frame[256U];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
  uint32_t overflow_count;
} AdcFrameQueue_t;

typedef enum
{
  APP_MODE_NONE  = 0,
  APP_MODE_ADC   = 1,
  APP_MODE_SERVO = 2
} AppMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_CHANNEL_COUNT       3U
#define ADC_BUFFER_LENGTH       256U
#define ADC_SAMPLE_RATE_HZ      100.0f
/* ADC 입력 용도:
 * - CH0 (PA0, ADC_CHANNEL_0): 광학 거리 센서 아날로그 출력(주 측정)
 * - CH1 (PA1), CH4 (PA4): 보조/예비 입력
 */
/* L432-SG90 기준 캘리브레이션 범위/기본값 */
#define SERVO_CALIB_MIN_US       500U
#define SERVO_CALIB_MAX_US      3000U
#define SERVO_DEFAULT_MIN_US     500U
#define SERVO_DEFAULT_MAX_US    2500U
#define UART_RX_RING_SIZE         64U

/* HC-SR04 -------------------------------------------------------------------*/
#define HCSR04_TRIG_PORT        GPIOA
#define HCSR04_TRIG_PIN         GPIO_PIN_6
/* TIM1: PSC=83 → 1MHz tick, ARR=9999 → 10ms 주기(100Hz) */
/* ECHO 유효 최대 거리: 10ms/2 × 34300 cm/s ≈ 171 cm */
#define HCSR04_TIM_ARR          9999U
#define HCSR04_TRIG_PULSE_US    10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t g_led_toggle_count = 0U;
AdcFrameQueue_t g_adc_queue;
AdcFrame_t g_adc_current_frame;
/* 최신 ADC 3채널 값 스냅샷(항상 마지막 완성 프레임) */
volatile uint16_t g_adc_latest_raw[ADC_CHANNEL_COUNT] = {0U, 0U, 0U};
volatile uint32_t g_adc_total_frames = 0U;
volatile uint8_t g_adc_rank_index = 0U;
uint8_t g_adc_stream_enabled = 1U;
uint8_t g_servo_current_angle = 90U;
volatile uint32_t g_servo_min_pulse_us = SERVO_DEFAULT_MIN_US;
volatile uint32_t g_servo_max_pulse_us = SERVO_DEFAULT_MAX_US;
AppMode_t g_app_mode = APP_MODE_NONE;
volatile uint8_t g_uart2_rx_ring[UART_RX_RING_SIZE];
volatile uint8_t g_uart2_rx_head = 0U;
volatile uint8_t g_uart2_rx_tail = 0U;
volatile uint8_t g_uart2_rx_it_byte = 0U;

/* HC-SR04 측정 변수 */
typedef enum { ECHO_IDLE = 0, ECHO_RISING = 1 } EchoState_t;
volatile EchoState_t g_echo_state     = ECHO_IDLE;
volatile uint32_t    g_echo_start_us  = 0U;
volatile uint32_t    g_echo_width_us  = 0U;
volatile float       g_hcsr04_dist_cm = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
static void AdcQueue_Init(AdcFrameQueue_t* queue);
static uint8_t AdcQueue_Push(AdcFrameQueue_t* queue, const AdcFrame_t* frame);
static uint8_t AdcQueue_Pop(AdcFrameQueue_t* queue, AdcFrame_t* frame);
static uint8_t AdcQueue_IsFull(const AdcFrameQueue_t* queue);
static uint8_t AdcQueue_IsEmpty(const AdcFrameQueue_t* queue);
static uint16_t __attribute__((unused)) AdcQueue_Count(const AdcFrameQueue_t* queue);
static uint32_t __attribute__((unused)) AdcQueue_OverflowCount(const AdcFrameQueue_t* queue);
static void SetServoAngle(uint8_t angle);
static void PrintServoHelp(uint8_t invalid);
static void PrintServoStatus(void);
static void PrintServoPrompt(void);
static void ProcessServoInput(void);
static void PrintAdcHelp(uint8_t invalid);
static void PrintAdcStatus(void);
static void PrintAdcPrompt(void);
static void ProcessAdcInput(void);
static AppMode_t SelectMode(void);
static uint8_t Uart2_ReadByte(uint8_t* out);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  /* 줄바꿈 문자를 CRLF로 변환 */
  if (c == '\n')
  {
    uint8_t cr = '\r';
    HAL_UART_Transmit(&huart2, &cr, 1U, HAL_MAX_DELAY);
  }

  HAL_UART_Transmit(&huart2, &c, 1U, HAL_MAX_DELAY);
  return ch;
}

static AppMode_t SelectMode(void)
{
  uint8_t rx_char;

  printf("\n=== Mode Select ===\n");
  printf("1: ADC Mode   - 3ch raw data CSV output\n");
  printf("2: Servo Mode - command interface (A, CMIN, CMAX, STATUS, HELP)\n");
  printf("Press 1 or 2: ");

  while (1)
  {
    if (Uart2_ReadByte(&rx_char) != 0U)
    {
      if (rx_char == '1')
      {
        printf("1\n");
        printf("[ADC Mode] Sampling: 3ch, 100Hz\n");
        printf("Output format: raw0,raw1,raw2\n");
        PrintAdcHelp(0U);
        PrintAdcPrompt();
        return APP_MODE_ADC;
      }
      else if (rx_char == '2')
      {
        printf("2\n");
        printf("[Servo Mode] Angle + calibration command interface enabled\n");
        PrintServoHelp(0U);
        PrintServoPrompt();
        return APP_MODE_SERVO;
      }
    }
  }
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 큐/서보 상태 초기화 순서: 1) 큐 초기화 -> 2) 서보 초기 듀티 적용 */
  AdcQueue_Init(&g_adc_queue);
  SetServoAngle(g_servo_current_angle);

  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UART_Receive_IT(&huart2, (uint8_t*)&g_uart2_rx_it_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* HC-SR04: TRIG 초기 LOW, ECHO 입력 캡처 + Update/CC2 인터럽트 시작 */
  HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE | TIM_IT_CC2);

  /* 부팅 완료 후 사용자에게 모드 선택 요청 */
  g_app_mode = SelectMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* uint32_t last_report_tick = HAL_GetTick(); */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t popped;
    AdcFrame_t frame;

    if (g_app_mode == APP_MODE_SERVO)
    {
      /* Servo 모드: 명령 입력 기반 각도/캘리브레이션 제어 */
      ProcessServoInput();
    }
    else
    {
      ProcessAdcInput();

      /* ADC 모드: ADC 큐에서 프레임을 꺼내 CSV 출력 */
      /*
       * ISR(ADC callback)에서 push 중일 수 있으므로
       * pop 시점만 짧게 임계구역으로 보호한다.
       */
      __disable_irq();
      popped = AdcQueue_Pop(&g_adc_queue, &frame);
      __enable_irq();

      if (popped != 0U)
      {
        if (g_adc_stream_enabled != 0U)
        {
          printf("%u,%u,%u\n",
                 (unsigned int)frame.channel[0],
                 (unsigned int)frame.channel[1],
                 (unsigned int)frame.channel[2]);
        }
      }
    }

    /*
    now_tick = HAL_GetTick();
    if ((now_tick - last_report_tick) >= 1000U)
    {
      last_report_tick += 1000U;

      __disable_irq();
      queue_count_snapshot = AdcQueue_Count(&g_adc_queue);
      total_frames_snapshot = g_adc_total_frames;
      overflow_snapshot = AdcQueue_OverflowCount(&g_adc_queue);
      __enable_irq();

      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      printf("ADC frames: %lu, queue: %u/256, ovf: %lu, Servo: %u%%\n",
             (unsigned long)total_frames_snapshot,
             (unsigned int)queue_count_snapshot,
             (unsigned long)overflow_snapshot,
             (unsigned int)g_pwm_duty_percent);
      g_led_toggle_count++;
    }
    */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  *        - TIM3 TRGO(100Hz) 외부 트리거 기반 3채널 스캔
  *        - Rank1 CH0(PA0): 광학 거리 센서
  *        - Rank2 CH1(PA1), Rank3 CH4(PA4): 보조/예비 입력
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  /* Rank1: CH0(PA0) = 광학 거리 센서 */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  /* Rank2: CH1(PA1) = 보조/예비 입력 */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  /* Rank3: CH4(PA4) = 보조/예비 입력 */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 초기화 함수 (HC-SR04 ECHO 입력 캡처 + 100Hz TRIG 생성)
  *        PSC=83 -> 1MHz tick, ARR=9999 -> 10ms 주기(100Hz)
  *        CH1: 입력 캡처 (PA8, ECHO)
  *        CH2: Output Compare TIMING (인터럽트만, 핀 없음, 10us 후 TRIG LOW)
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig     = {0};
  TIM_IC_InitTypeDef sConfigIC              = {0};
  TIM_OC_InitTypeDef sConfigOC              = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 83U;           /* 84MHz / 84 = 1MHz */
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = HCSR04_TIM_ARR; /* 9999 = 10ms */
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* CH1: ECHO 상승 엣지 입력 캡처 */
  sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter    = 0U;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* CH2: TRIG 펄스 종료용 OC TIMING (10us = CCR2=10) */
  sConfigOC.OCMode       = TIM_OCMODE_TIMING;
  sConfigOC.Pulse        = HCSR04_TRIG_PULSE_US;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 초기화 함수 (SG-90 서보 PWM 출력)
  *        - CH1(PB6), 50Hz(20ms), 1us tick 기반 펄스폭 제어
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void AdcQueue_Init(AdcFrameQueue_t* queue)
{
  /* 링버퍼 인덱스/카운터를 모두 초기 상태로 설정 */
  queue->head = 0U;
  queue->tail = 0U;
  queue->count = 0U;
  queue->overflow_count = 0U;
}

static uint8_t AdcQueue_IsFull(const AdcFrameQueue_t* queue)
{
  return (queue->count >= ADC_BUFFER_LENGTH) ? 1U : 0U;
}

static uint8_t AdcQueue_IsEmpty(const AdcFrameQueue_t* queue)
{
  return (queue->count == 0U) ? 1U : 0U;
}

static uint16_t AdcQueue_Count(const AdcFrameQueue_t* queue)
{
  return queue->count;
}

static uint32_t AdcQueue_OverflowCount(const AdcFrameQueue_t* queue)
{
  return queue->overflow_count;
}

static uint8_t AdcQueue_Push(AdcFrameQueue_t* queue, const AdcFrame_t* frame)
{
  uint8_t pushed_without_overflow = 1U;

  if (AdcQueue_IsFull(queue) != 0U)
  {
    /* 실시간 스트리밍 우선 정책: 가장 오래된 프레임 폐기 후 최신 프레임 보존 */
    queue->head++;
    if (queue->head >= ADC_BUFFER_LENGTH)
    {
      queue->head = 0U;
    }
    queue->count--;
    queue->overflow_count++;
    pushed_without_overflow = 0U;
  }

  queue->frame[queue->tail] = *frame;
  queue->tail++;
  if (queue->tail >= ADC_BUFFER_LENGTH)
  {
    queue->tail = 0U;
  }
  queue->count++;

  return pushed_without_overflow;
}

static uint8_t AdcQueue_Pop(AdcFrameQueue_t* queue, AdcFrame_t* frame)
{
  if (AdcQueue_IsEmpty(queue) != 0U)
  {
    /* 소비할 데이터가 없으면 즉시 실패 반환 */
    return 0U;
  }

  *frame = queue->frame[queue->head];
  queue->head++;
  if (queue->head >= ADC_BUFFER_LENGTH)
  {
    queue->head = 0U;
  }
  queue->count--;

  return 1U;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t adc_value;
  uint8_t rank_index;

  /* 다른 ADC 인스턴스 인터럽트는 무시 */
  if (hadc->Instance != ADC1)
  {
    return;
  }

  /* 변환 완료된 현재 rank의 값 1개를 읽는다 */
  adc_value = (uint16_t)HAL_ADC_GetValue(hadc);
  rank_index = g_adc_rank_index;

  /* rank 순서대로 임시 프레임 버퍼에 저장 */
  if (rank_index < ADC_CHANNEL_COUNT)
  {
    g_adc_current_frame.channel[rank_index] = adc_value;
  }

  rank_index++;
  if (rank_index >= ADC_CHANNEL_COUNT)
  {
    /* 3채널이 모두 채워지면 1프레임 완성으로 큐에 적재 */
    rank_index = 0U;

    /* 메인 루프/다른 코드에서 즉시 참조할 최신 스냅샷 갱신 */
    g_adc_latest_raw[0] = g_adc_current_frame.channel[0];
    g_adc_latest_raw[1] = g_adc_current_frame.channel[1];
    g_adc_latest_raw[2] = g_adc_current_frame.channel[2];

    (void)AdcQueue_Push(&g_adc_queue, &g_adc_current_frame);
    g_adc_total_frames++;
  }

  g_adc_rank_index = rank_index;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t next_head = (uint8_t)(g_uart2_rx_head + 1U);

    if (next_head >= UART_RX_RING_SIZE)
    {
      next_head = 0U;
    }

    if (next_head != g_uart2_rx_tail)
    {
      g_uart2_rx_ring[g_uart2_rx_head] = g_uart2_rx_it_byte;
      g_uart2_rx_head = next_head;
    }

    (void)HAL_UART_Receive_IT(&huart2, (uint8_t*)&g_uart2_rx_it_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart2, (uint8_t*)&g_uart2_rx_it_byte, 1U);
  }
}

/* ---------------------------------------------------------------------------
 * HC-SR04 TIM1 콜백
 * ---------------------------------------------------------------------------*/

/**
 * @brief Update 인터럽트 - TRIG 펄스 시작, 이전 측정 미완료 시 리셋
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM1)
  {
    /* 이전 에코가 끝나지 않은 경우 - 측정 무효 처리 */
    if (g_echo_state == ECHO_RISING)
    {
      g_echo_state = ECHO_IDLE;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }

    /* TRIG 상승 - 10us 후 CC2 인터럽트에서 하강 */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
  }
}

/**
 * @brief CC2 OC 인터럽트 - TRIG 하강 (10us 펄스 완료)
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
  }
}

/**
 * @brief CH1 입력 캡처 - 상승 엣지에서 시작 시각 저장, 하강 엣지에서 거리 계산
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
  if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    if (g_echo_state == ECHO_IDLE)
    {
      /* 상승 엣지 - 시작값 저장, 하강 엣지로 전환 */
      g_echo_start_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      g_echo_state    = ECHO_RISING;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (g_echo_state == ECHO_RISING)
    {
      /* 하강 엣지 - 폭 계산, 카운터 랩오버 보정 포함 */
      uint32_t echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uint32_t width_us;

      if (echo_end >= g_echo_start_us)
      {
        width_us = echo_end - g_echo_start_us;
      }
      else
      {
        /* 카운터가 ARR에서 낙하한 경우 보정 */
        width_us = (HCSR04_TIM_ARR - g_echo_start_us) + echo_end + 1U;
      }

      g_echo_width_us  = width_us;
      g_hcsr04_dist_cm = (float)width_us / 58.0f;

      g_echo_state = ECHO_IDLE;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

static uint8_t Uart2_ReadByte(uint8_t* out)
{
  uint8_t tail;
  uint8_t value;

  if (out == NULL)
  {
    return 0U;
  }

  __disable_irq();
  if (g_uart2_rx_head == g_uart2_rx_tail)
  {
    __enable_irq();
    return 0U;
  }

  tail = g_uart2_rx_tail;
  value = g_uart2_rx_ring[tail];
  tail++;
  if (tail >= UART_RX_RING_SIZE)
  {
    tail = 0U;
  }
  g_uart2_rx_tail = tail;
  __enable_irq();

  *out = value;
  return 1U;
}

static void SetServoAngle(uint8_t angle)
{
  uint32_t pulse;

  if (angle > 180U)
  {
    angle = 180U;
  }

  pulse = g_servo_min_pulse_us
      + ((uint32_t)angle * (g_servo_max_pulse_us - g_servo_min_pulse_us)) / 180U;
  /* CCR 갱신으로 PWM 듀티 반영 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);

  g_servo_current_angle = angle;
  printf("Angle set: %u deg (%luus)\n",
         (unsigned int)g_servo_current_angle,
         (unsigned long)pulse);
}

static void PrintServoHelp(uint8_t invalid)
{
  if (invalid != 0U)
  {
    printf("\nInvalid command\n");
  }
  else
  {
    printf("\n[HELP]\n");
  }

  printf("  A<deg>    : set angle (A0~A180)\n");
  printf("  CMIN:<us> : set 0deg pulse (500~3000us)\n");
  printf("  CMAX:<us> : set 180deg pulse (500~3000us)\n");
  printf("  STATUS    : show current settings\n");
  printf("  HELP      : show this help\n");
}

static void PrintServoStatus(void)
{
  uint32_t pulse;

  pulse = g_servo_min_pulse_us
      + ((uint32_t)g_servo_current_angle * (g_servo_max_pulse_us - g_servo_min_pulse_us)) / 180U;

  printf("\n[STATUS]\n");
  printf("  angle      : %u deg\n", (unsigned int)g_servo_current_angle);
  printf("  CMIN       : %lu us\n", (unsigned long)g_servo_min_pulse_us);
  printf("  CMAX       : %lu us\n", (unsigned long)g_servo_max_pulse_us);
  printf("  pulse(now) : %lu us\n", (unsigned long)pulse);
}

static void PrintServoPrompt(void)
{
  printf("Input> ");
}

static void ProcessServoInput(void)
{
  uint8_t rx_char;
  static char input_buf[20];
  static uint8_t input_len = 0U;

  /* non-blocking 1-byte read from IRQ-backed UART RX ring */
  if (Uart2_ReadByte(&rx_char) == 0U)
  {
    return;
  }

  /* 엔터 입력 시 명령어 실행 */
  if ((rx_char == '\r') || (rx_char == '\n'))
  {
    uint8_t i;

    /* 빈 라인 엔터는 무시 */
    if (input_len == 0U)
    {
      PrintServoPrompt();
      return;
    }

    input_buf[input_len] = '\0';

    for (i = 0U; i < input_len; i++)
    {
      if ((input_buf[i] >= 'a') && (input_buf[i] <= 'z'))
      {
        input_buf[i] = (char)(input_buf[i] - 'a' + 'A');
      }
    }

    if (input_buf[0] == 'A')
    {
      char* endptr;
      long angle = strtol(&input_buf[1], &endptr, 10);

      if ((*endptr == '\0') && (angle >= 0L) && (angle <= 180L))
      {
        SetServoAngle((uint8_t)angle);
      }
      else
      {
        printf("Invalid. Use A0~A180\n");
      }
    }
    else if (strncmp(input_buf, "CMIN:", 5U) == 0)
    {
      char* endptr;
      long value = strtol(&input_buf[5], &endptr, 10);

      if ((*endptr == '\0') && (value >= (long)SERVO_CALIB_MIN_US) && (value <= (long)SERVO_CALIB_MAX_US))
      {
        if ((uint32_t)value < g_servo_max_pulse_us)
        {
          g_servo_min_pulse_us = (uint32_t)value;
          SetServoAngle(g_servo_current_angle);
          printf("CMIN = %lu us (CMAX = %lu us)\n",
                 (unsigned long)g_servo_min_pulse_us,
                 (unsigned long)g_servo_max_pulse_us);
        }
        else
        {
          printf("Error: CMIN must be < CMAX (%lu us)\n",
                 (unsigned long)g_servo_max_pulse_us);
        }
      }
      else
      {
        printf("Invalid value (range: 500~3000 us)\n");
      }
    }
    else if (strncmp(input_buf, "CMAX:", 5U) == 0)
    {
      char* endptr;
      long value = strtol(&input_buf[5], &endptr, 10);

      if ((*endptr == '\0') && (value >= (long)SERVO_CALIB_MIN_US) && (value <= (long)SERVO_CALIB_MAX_US))
      {
        if ((uint32_t)value > g_servo_min_pulse_us)
        {
          g_servo_max_pulse_us = (uint32_t)value;
          SetServoAngle(g_servo_current_angle);
          printf("CMAX = %lu us (CMIN = %lu us)\n",
                 (unsigned long)g_servo_max_pulse_us,
                 (unsigned long)g_servo_min_pulse_us);
        }
        else
        {
          printf("Error: CMAX must be > CMIN (%lu us)\n",
                 (unsigned long)g_servo_min_pulse_us);
        }
      }
      else
      {
        printf("Invalid value (range: 500~3000 us)\n");
      }
    }
    else if (strcmp(input_buf, "STATUS") == 0)
    {
      PrintServoStatus();
    }
    else if (strcmp(input_buf, "HELP") == 0)
    {
      PrintServoHelp(0U);
    }
    else
    {
      PrintServoHelp(1U);
    }

    /* 다음 입력을 위해 버퍼 초기화 */
    input_len = 0U;
    PrintServoPrompt();
    return;
  }

  if ((rx_char >= ' ') && (rx_char <= '~'))
  {
    if (input_len < (sizeof(input_buf) - 1U))
    {
      input_buf[input_len++] = (char)rx_char;
    }
  }
}

static void PrintAdcHelp(uint8_t invalid)
{
  if (invalid != 0U)
  {
    printf("\nInvalid command\n");
  }
  else
  {
    printf("\n[ADC HELP]\n");
  }

  printf("  START   : enable ADC CSV streaming\n");
  printf("  STOP    : disable ADC CSV streaming\n");
  printf("  STATUS  : show ADC stream/queue status\n");
  printf("  HELP    : show this help\n");
}

static void PrintAdcStatus(void)
{
  uint16_t queue_count_snapshot;
  uint32_t total_frames_snapshot;
  uint32_t overflow_snapshot;

  __disable_irq();
  queue_count_snapshot = AdcQueue_Count(&g_adc_queue);
  total_frames_snapshot = g_adc_total_frames;
  overflow_snapshot = AdcQueue_OverflowCount(&g_adc_queue);
  __enable_irq();

  printf("\n[ADC STATUS]\n");
  printf("  stream    : %s\n", (g_adc_stream_enabled != 0U) ? "ON" : "OFF");
  printf("  sample    : %.0f Hz\n", ADC_SAMPLE_RATE_HZ);
  printf("  frames    : %lu\n", (unsigned long)total_frames_snapshot);
  printf("  queue     : %u/256\n", (unsigned int)queue_count_snapshot);
  printf("  overflow  : %lu\n", (unsigned long)overflow_snapshot);
}

static void PrintAdcPrompt(void)
{
  printf("Input> ");
}

static void ProcessAdcInput(void)
{
  uint8_t rx_char;
  static char input_buf[20];
  static uint8_t input_len = 0U;

  if (Uart2_ReadByte(&rx_char) == 0U)
  {
    return;
  }

  if ((rx_char == '\r') || (rx_char == '\n'))
  {
    uint8_t i;

    if (input_len == 0U)
    {
      PrintAdcPrompt();
      return;
    }

    input_buf[input_len] = '\0';

    for (i = 0U; i < input_len; i++)
    {
      if ((input_buf[i] >= 'a') && (input_buf[i] <= 'z'))
      {
        input_buf[i] = (char)(input_buf[i] - 'a' + 'A');
      }
    }

    if (strcmp(input_buf, "START") == 0)
    {
      g_adc_stream_enabled = 1U;
      printf("ADC stream ON\n");
    }
    else if (strcmp(input_buf, "STOP") == 0)
    {
      g_adc_stream_enabled = 0U;
      printf("ADC stream OFF\n");
    }
    else if (strcmp(input_buf, "STATUS") == 0)
    {
      PrintAdcStatus();
    }
    else if (strcmp(input_buf, "HELP") == 0)
    {
      PrintAdcHelp(0U);
    }
    else
    {
      PrintAdcHelp(1U);
    }

    input_len = 0U;
    PrintAdcPrompt();
    return;
  }

  if ((rx_char >= ' ') && (rx_char <= '~'))
  {
    if (input_len < (sizeof(input_buf) - 1U))
    {
      input_buf[input_len++] = (char)rx_char;
    }
  }
}

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
#ifdef USE_FULL_ASSERT
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
