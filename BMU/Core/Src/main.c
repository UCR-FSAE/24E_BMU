/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "bq79616.h"
#include "bq79600.h"
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int Max_Channels = 16;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t adcValue[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  void select_adc_channel(int channel)
  {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    switch (channel)
    {
    case 1:
      sConfig.Channel = ADC_CHANNEL_0;
      sConfig.Rank = 1;

      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;

    case 2:
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
      sConfig.Channel = ADC_CHANNEL_1;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 3:
      sConfig.Channel = ADC_CHANNEL_2;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 4:
      sConfig.Channel = ADC_CHANNEL_3;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 5:
      sConfig.Channel = ADC_CHANNEL_4;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 6:
      sConfig.Channel = ADC_CHANNEL_5;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 7:
      sConfig.Channel = ADC_CHANNEL_6;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 8:
      sConfig.Channel = ADC_CHANNEL_7;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 9:
      sConfig.Channel = ADC_CHANNEL_8;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 10:
      sConfig.Channel = ADC_CHANNEL_9;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 11:
      sConfig.Channel = ADC_CHANNEL_10;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 12:
      sConfig.Channel = ADC_CHANNEL_11;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 13:
      sConfig.Channel = ADC_CHANNEL_12;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 14:
      sConfig.Channel = ADC_CHANNEL_13;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;
      /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
       */
    case 15:
      sConfig.Channel = ADC_CHANNEL_14;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;

    case 16:
      sConfig.Channel = ADC_CHANNEL_15;
      sConfig.Rank = 1;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      break;

    default:
      break;
    }
  }

  void CollectADCData()
  {
    for (int i = 0; i < Max_Channels; i++)
    {
      select_adc_channel(i + 1);
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      adcValue[i] = HAL_ADC_GetValue(&hadc1) % 4;
      HAL_ADC_Stop(&hadc1);
    }
  }

  // Starting timer for delayus in bq79616.c
  HAL_TIM_Base_Start(&htim1);

  // FSM Creation

  // Creating variables
  bool precharge_flag;
  bool idle_flag;
  bool charge_flag;
  bool discharge_flag;
  bool fault_flag;

  char uart_buf[50];
  int uart_buf_len;

  enum States
  {
    INIT,
    PRECHARGE,
    IDLE,
    CHARGE,
    DISCHARGE,
    FAULT
  } state = INIT;

  void BMS_Tick()
  {
    // Transistions
    switch (state)
    {
    case INIT:
      if (precharge_flag)
      {
        state = PRECHARGE;
      }
      break;

    case PRECHARGE:
      if (idle_flag)
      {
        state = IDLE;
      }
      else if (fault_flag)
      {
        state = FAULT;
      }
      break;

    case IDLE:
      if (charge_flag)
      {
        state = CHARGE;
      }
      else if (discharge_flag)
      {
        state = DISCHARGE;
      }
      else if (fault_flag)
      {
        state = FAULT;
      }
      break;

    case CHARGE:
      if (idle_flag)
      {
        state = IDLE;
      }
      else if (fault_flag)
      {
        state = FAULT;
      }
      break;

    case DISCHARGE:
      if (idle_flag)
      {
        state = IDLE;
      }
      else if (fault_flag)
      {
        state = FAULT;
      }
      break;

    case FAULT:
      strcpy((char *)uart_buf, "FAULT\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;

    default:
      strcpy((char *)uart_buf, "DEFAULT\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;
    }

    // Actions
    switch (state)
    {
    case INIT:
      strcpy((char *)uart_buf, "INIT\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;

    case PRECHARGE:
      strcpy((char *)uart_buf, "PRECHARGE\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;

    case IDLE:
      strcpy((char *)uart_buf, "IDLE\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;

    case CHARGE:
      strcpy((char *)uart_buf, "CHARGE\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;

    case DISCHARGE:
      strcpy((char *)uart_buf, "DISCHARGE\r\n");
      uart_buf_len = strlen((char *)uart_buf);
      HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
      break;

    case FAULT:
      break;

    default:
      break;
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    BMS_Tick();
    HAL_Delay(1000);
    // Testing
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_RDY_Pin USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = SPI_RDY_Pin | USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
