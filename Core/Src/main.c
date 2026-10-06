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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
static uint8_t i2c_bus_idle_scl;
static uint8_t i2c_bus_idle_sda;
static uint8_t i2c_bus_pullup_scl;
static uint8_t i2c_bus_pullup_sda;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void MPU6050_RawI2cDiagnostic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void I2C1_ClearBus(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  for (volatile uint32_t delay = 0U; delay < 1000U; ++delay) { __NOP(); }
  for (uint32_t pulse = 0U; pulse < 16U; ++pulse)
  {
    if ((pulse >= 9U) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)) break;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    for (volatile uint32_t delay = 0U; delay < 1000U; ++delay) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile uint32_t delay = 0U; delay < 1000U; ++delay) { __NOP(); }
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  for (volatile uint32_t delay = 0U; delay < 1000U; ++delay) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  for (volatile uint32_t delay = 0U; delay < 1000U; ++delay) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  for (volatile uint32_t delay = 0U; delay < 1000U; ++delay) { __NOP(); }
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  App_CaptureResetDiagnostics();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* Capture PB8/PB9 electrical levels before I2C1 claims the pins. */
  GPIO_InitTypeDef i2c_bus_probe = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  i2c_bus_probe.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  i2c_bus_probe.Mode = GPIO_MODE_INPUT;
  i2c_bus_probe.Pull = GPIO_NOPULL;
  i2c_bus_probe.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &i2c_bus_probe);
  i2c_bus_idle_scl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;
  i2c_bus_idle_sda = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;
  i2c_bus_probe.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &i2c_bus_probe);
  i2c_bus_pullup_scl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;
  i2c_bus_pullup_sda = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;
  I2C1_ClearBus();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Independent of FreeRTOS, mutexes, sensor access, and the watchdog. */
  static const uint8_t uart_startup_banner[] =
      "\r\n[BOOT] USART1 PB14 TX READY @ 115200 8N1\r\n";
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart_startup_banner,
                          sizeof(uart_startup_banner) - 1U, 100U);
  static const char *const i2c_level_name[] = { "LOW", "HIGH" };
  char i2c_bus_message[56];
  int i2c_bus_length = snprintf(i2c_bus_message, sizeof(i2c_bus_message),
                                "[I2C BUS] PB8/SCL=%s PB9/SDA=%s\r\n",
                                i2c_level_name[i2c_bus_idle_scl],
                                i2c_level_name[i2c_bus_idle_sda]);
  if (i2c_bus_length > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)i2c_bus_message,
                            (uint16_t)i2c_bus_length, 100U);
  }
  i2c_bus_length = snprintf(i2c_bus_message, sizeof(i2c_bus_message),
                            "[I2C PULLUP TEST] PB8/SCL=%s PB9/SDA=%s\r\n",
                            i2c_level_name[i2c_bus_pullup_scl],
                            i2c_level_name[i2c_bus_pullup_sda]);
  if (i2c_bus_length > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)i2c_bus_message,
                            (uint16_t)i2c_bus_length, 100U);
  }
  MPU6050_RawI2cDiagnostic();
  App_Initialize(&hi2c1, &huart1);
  static const uint8_t app_initialized_message[] = "[APP] App_Initialize OK\r\n";
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)app_initialized_message,
                          sizeof(app_initialized_message) - 1U, 100U);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (App_CreateTasks() != pdPASS)
  {
    static const uint8_t tasks_failed_message[] = "[APP] Task/IWDG setup FAILED\r\n";
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tasks_failed_message,
                            sizeof(tasks_failed_message) - 1U, 100U);
    Error_Handler();
  }
  static const uint8_t tasks_created_message[] = "[RTOS] Tasks created; IWDG started\r\n";
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)tasks_created_message,
                          sizeof(tasks_created_message) - 1U, 100U);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 110;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0xD0922E33;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Direct, pre-RTOS I2C probe: address 0x68 is shifted exactly once for HAL. */
static void MPU6050_RawI2cDiagnostic(void)
{
  char message[160];
  uint8_t who_am_i = 0U, pwr_before = 0U, pwr_after = 0U, raw[14] = {0};
  HAL_StatusTypeDef normal, normal_initial, pullup = HAL_ERROR, who = HAL_ERROR, pwr = HAL_ERROR, data = HAL_ERROR;
  uint32_t error, isr_before, isr_after;
  uint8_t pullup_ran = 0U, who_ran = 0U, pwr_ran = 0U, data_ran = 0U;
  GPIO_InitTypeDef gpio = {0};
  int length;

#define I2C_DIAG_PRINT(...) do { \
  length = snprintf(message, sizeof(message), __VA_ARGS__); \
  if (length > 0) (void)HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)length, 200U); \
} while (0)

  I2C_DIAG_PRINT("[I2C CFG] source=D2PCLK1 PCLK1=%lu TIMING=0x%08lX CR1=0x%08lX CR2=0x%08lX ISR=0x%08lX PE=%lu\r\n",
                 (unsigned long)HAL_RCC_GetPCLK1Freq(),
                 (unsigned long)hi2c1.Init.Timing, (unsigned long)I2C1->CR1,
                 (unsigned long)I2C1->CR2, (unsigned long)I2C1->ISR,
                 (unsigned long)((I2C1->CR1 & I2C_CR1_PE) != 0U));
  I2C_DIAG_PRINT("[I2C CFG] addrmode=%lu own=0x%lX dual=%lu general=%lu nostretch=%lu\r\n",
                 (unsigned long)hi2c1.Init.AddressingMode, (unsigned long)hi2c1.Init.OwnAddress1,
                 (unsigned long)hi2c1.Init.DualAddressMode, (unsigned long)hi2c1.Init.GeneralCallMode,
                 (unsigned long)hi2c1.Init.NoStretchMode);
  isr_before = I2C1->ISR;
  I2C_DIAG_PRINT("[I2C BEFORE] ISR=0x%08lX state=%lu error=0x%08lX SCL=%u SDA=%u\r\n",
                 (unsigned long)isr_before, (unsigned long)HAL_I2C_GetState(&hi2c1),
                 (unsigned long)HAL_I2C_GetError(&hi2c1),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8), (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));

  normal = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x68U << 1), 3U, 100U);
  normal_initial = normal;
  error = HAL_I2C_GetError(&hi2c1); isr_after = I2C1->ISR;
  I2C_DIAG_PRINT("[I2C TEST] addr=0x68 status=%d error=0x%08lX state=%lu ISR=0x%08lX SCL=%u SDA=%u\r\n",
                 (int)normal, (unsigned long)error, (unsigned long)HAL_I2C_GetState(&hi2c1),
                 (unsigned long)isr_after, (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));

  if (normal != HAL_OK) {
    /* Diagnostic variant only: AF4 I2C open-drain with internal pull-ups. */
    (void)HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9; gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP; gpio.Speed = GPIO_SPEED_FREQ_LOW; gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);
    pullup_ran = 1U;
    pullup = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x68U << 1), 3U, 100U);
    error = HAL_I2C_GetError(&hi2c1);
    I2C_DIAG_PRINT("[I2C INTERNAL PU TEST] status=%d error=0x%08lX state=%lu ISR=0x%08lX SCL=%u SDA=%u\r\n",
                   (int)pullup, (unsigned long)error, (unsigned long)HAL_I2C_GetState(&hi2c1),
                   (unsigned long)I2C1->ISR, (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
                   (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
    (void)HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init(); /* Final normal CubeMX AF4, open-drain, GPIO_NOPULL restoration. */
  }

  if (normal == HAL_OK) {
    who_ran = 1U;
    who = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68U << 1), 0x75U, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, 100U);
    I2C_DIAG_PRINT("[I2C TEST] WHO_AM_I status=%d value=0x%02X error=0x%08lX\r\n", (int)who, who_am_i, (unsigned long)HAL_I2C_GetError(&hi2c1));
    if (who == HAL_OK) {
      pwr_ran = 1U;
      pwr = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68U << 1), 0x6BU, I2C_MEMADD_SIZE_8BIT, &pwr_before, 1U, 100U);
      if (pwr == HAL_OK) pwr = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x68U << 1), 0x6BU, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0U}, 1U, 100U);
      HAL_Delay(100U);
      if (pwr == HAL_OK) pwr = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68U << 1), 0x6BU, I2C_MEMADD_SIZE_8BIT, &pwr_after, 1U, 100U);
      I2C_DIAG_PRINT("[I2C TEST] PWR_MGMT_1 status=%d before=0x%02X after=0x%02X error=0x%08lX\r\n", (int)pwr, pwr_before, pwr_after, (unsigned long)HAL_I2C_GetError(&hi2c1));
      if (pwr == HAL_OK) {
        data_ran = 1U;
        data = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68U << 1), 0x3BU, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 100U);
      }
      I2C_DIAG_PRINT("[I2C TEST] 14-BYTE status=%d error=0x%08lX %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                     (int)data, (unsigned long)HAL_I2C_GetError(&hi2c1), raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],raw[8],raw[9],raw[10],raw[11],raw[12],raw[13]);
    }
  }

  normal = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x68U << 1), 3U, 100U);
  error = HAL_I2C_GetError(&hi2c1);
  I2C_DIAG_PRINT("========== I2C DIAGNOSTIC SUMMARY ==========\r\n");
  I2C_DIAG_PRINT("GPIO NOPULL:       SCL=%s SDA=%s\r\n", i2c_bus_idle_scl ? "HIGH" : "LOW", i2c_bus_idle_sda ? "HIGH" : "LOW");
  I2C_DIAG_PRINT("GPIO PULLUP:       SCL=%s SDA=%s\r\n", i2c_bus_pullup_scl ? "HIGH" : "LOW", i2c_bus_pullup_sda ? "HIGH" : "LOW");
  I2C_DIAG_PRINT("I2C TIMINGR:       0x%08lX; ISR BEFORE: 0x%08lX\r\n", (unsigned long)I2C1->TIMINGR, (unsigned long)isr_before);
  I2C_DIAG_PRINT("I2C READY NORMAL:  %s; PULLUP: %s\r\n", normal_initial == HAL_OK ? "PASS" : "FAIL", pullup_ran ? (pullup == HAL_OK ? "PASS" : "FAIL") : "NOT_RUN");
  I2C_DIAG_PRINT("WHO_AM_I:          %s; PWR_MGMT_1: %s; 14-BYTE: %s\r\n", who_ran ? (who == HAL_OK ? "PASS" : "FAIL") : "NOT_RUN", pwr_ran ? (pwr == HAL_OK ? "PASS" : "FAIL") : "NOT_RUN", data_ran ? (data == HAL_OK ? "PASS" : "FAIL") : "NOT_RUN");
  I2C_DIAG_PRINT("FINAL NORMAL TEST: %s; HAL ERROR: 0x%08lX\r\n", normal == HAL_OK ? "PASS" : "FAIL", (unsigned long)error);
  I2C_DIAG_PRINT("LIKELY FAULT DOMAIN: %s\r\n==============================================\r\n", (normal_initial != HAL_OK && pullup_ran && pullup == HAL_OK) ? "PULL-UP" : (normal_initial != HAL_OK) ? "BUS / PERIPHERAL" : (who_ran && who != HAL_OK) ? "MPU REGISTER ACCESS" : "APPLICATION");
#undef I2C_DIAG_PRINT
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
