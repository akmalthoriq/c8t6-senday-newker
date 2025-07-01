/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "usbd_cdc_if.h" // Untuk CDC_Transmit_FS
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
TIM_HandleTypeDef htim1;

/* Definitions for atcTask */
osThreadId_t atcTaskHandle;
const osThreadAttr_t atcTask_attributes = {
    .name = "atcTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ATCRotationQueue */
osMessageQueueId_t ATCRotationQueueHandle;
const osMessageQueueAttr_t ATCRotationQueue_attributes = {
    .name = "ATCRotationQueue"};
/* Definitions for ATCLockSema */
osSemaphoreId_t ATCLockSemaHandle;
const osSemaphoreAttr_t ATCLockSema_attributes = {
    .name = "ATCLockSema"};
/* Definitions for ATCPositionSema */
osSemaphoreId_t ATCPositionSemaHandle;
const osSemaphoreAttr_t ATCPositionSema_attributes = {
    .name = "ATCPositionSema"};
/* Definitions for ToolChangeSema */
osSemaphoreId_t ToolChangeSemaHandle;
const osSemaphoreAttr_t ToolChangeSema_attributes = {
    .name = "ToolChangeSema"};
/* USER CODE BEGIN PV */
// Variabel global untuk status ATC
volatile ATCLockState_t g_atc_lock_status = ATC_LOCKED;
volatile uint8_t g_current_tool = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void StartAtcTask(void *argument);

/* USER CODE BEGIN PFP */
// Fungsi-fungsi untuk ATC
static void DWT_Init(void);
static void delay_us(uint32_t us);
static void Stepper_Pulse(void);
static void ATC_Rotate(ATCRotationDirection_t direction);
static uint8_t Read_Tool_Position(void);
static void Update_Newker_Tool_Output(uint8_t tool_number);
static ATCLockState_t Get_Lock_Status(void);
static void ATC_Unlock(void);
static void ATC_Lock(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  CDC_Transmit_FS((uint8_t *)ptr, len);
  return len;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* creation of ATCLockSema */
  ATCLockSemaHandle = osSemaphoreNew(1, 0, &ATCLockSema_attributes);

  /* creation of ATCPositionSema */
  ATCPositionSemaHandle = osSemaphoreNew(1, 0, &ATCPositionSema_attributes);

  /* creation of ToolChangeSema */
  ToolChangeSemaHandle = osSemaphoreNew(1, 0, &ToolChangeSema_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ATCRotationQueue */
  ATCRotationQueueHandle = osMessageQueueNew(16, sizeof(ATCRotationDirection_t), &ATCRotationQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of atcTask */
  atcTaskHandle = osThreadNew(StartAtcTask, NULL, &atcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NEWKER_TOOL_1_Pin | NEWKER_TOOL_2_Pin | NEWKER_TOOL_3_Pin | NEWKER_TOOL_4_Pin | NEWKER_TOOL_5_Pin | NEWKER_TOOL_6_Pin | NEWKER_TOOL_7_Pin | STEPPER_DIR_Pin | STEPPER_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NEWKER_TOOL_8_Pin | USER_LED_Pin | OUT_ATC_LOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NEWKER_TOOL_1_Pin NEWKER_TOOL_2_Pin NEWKER_TOOL_3_Pin NEWKER_TOOL_4_Pin
                           NEWKER_TOOL_5_Pin NEWKER_TOOL_6_Pin NEWKER_TOOL_7_Pin STEPPER_DIR_Pin
                           STEPPER_ENA_Pin */
  GPIO_InitStruct.Pin = NEWKER_TOOL_1_Pin | NEWKER_TOOL_2_Pin | NEWKER_TOOL_3_Pin | NEWKER_TOOL_4_Pin | NEWKER_TOOL_5_Pin | NEWKER_TOOL_6_Pin | NEWKER_TOOL_7_Pin | STEPPER_DIR_Pin | STEPPER_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NEWKER_TOOL_8_Pin USER_LED_Pin OUT_ATC_LOCK_Pin */
  GPIO_InitStruct.Pin = NEWKER_TOOL_8_Pin | USER_LED_Pin | OUT_ATC_LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_NEWKER_TOK_Pin IN_NEWKER_T_PLUS_Pin IN_NEWKER_T_MIN_Pin */
  GPIO_InitStruct.Pin = IN_NEWKER_TOK_Pin | IN_NEWKER_T_PLUS_Pin | IN_NEWKER_T_MIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PROXY_TOOL_D_Pin */
  GPIO_InitStruct.Pin = PROXY_TOOL_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PROXY_TOOL_D_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PROXY_TOOL_C_Pin PROXY_TOOL_B_Pin PROXY_TOOL_A_Pin PROXY_POSITION_Pin
                           PROXY_LOCK_Pin */
  GPIO_InitStruct.Pin = PROXY_TOOL_C_Pin | PROXY_TOOL_B_Pin | PROXY_TOOL_A_Pin | PROXY_POSITION_Pin | PROXY_LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Inisialisasi DWT (Data Watchpoint and Trace) untuk delay presisi tinggi.
 */
static void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

/**
 * @brief Fungsi delay dalam microsecond menggunakan DWT.
 */
static void delay_us(uint32_t us)
{
  uint32_t start_tick = DWT->CYCCNT;
  uint32_t delay_ticks = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start_tick) < delay_ticks)
    ;
}

/**
 * @brief Fungsi untuk menghasilkan satu pulsa pada motor stepper.
 */
static void Stepper_Pulse(void)
{
  HAL_GPIO_WritePin(STEPPER_PULSE_GPIO_Port, STEPPER_PULSE_Pin, GPIO_PIN_SET);
  delay_us(STEPPER_PULSE_DELAY_US);
  HAL_GPIO_WritePin(STEPPER_PULSE_GPIO_Port, STEPPER_PULSE_Pin, GPIO_PIN_RESET);
  delay_us(STEPPER_PULSE_DELAY_US);
}

/**
 * @brief Membaca status 4 pin proxy dan mengembalikan nomor tool.
 * @retval Nomor tool (1-8), atau 0 jika kombinasi tidak valid.
 */
static uint8_t Read_Tool_Position(void)
{
  // Membaca status pin. Asumsi aktif LOW (sensor NPN), jadi kita balik logikanya.
  bool a = HAL_GPIO_ReadPin(PROXY_TOOL_A_GPIO_Port, PROXY_TOOL_A_Pin) == GPIO_PIN_RESET;
  bool b = HAL_GPIO_ReadPin(PROXY_TOOL_B_GPIO_Port, PROXY_TOOL_B_Pin) == GPIO_PIN_RESET;
  bool c = HAL_GPIO_ReadPin(PROXY_TOOL_C_GPIO_Port, PROXY_TOOL_C_Pin) == GPIO_PIN_RESET;
  bool d = HAL_GPIO_ReadPin(PROXY_TOOL_D_GPIO_Port, PROXY_TOOL_D_Pin) == GPIO_PIN_RESET;

  // Logika berdasarkan tabel yang diberikan
  if (!a && b && !c && !d)
    return 1;
  if (a && !b && !c && !d)
    return 2;
  if (!a && !b && !c && d)
    return 3;
  if (!a && !b && c && !d)
    return 4;
  if (!a && b && c && d)
    return 5;
  if (a && !b && c && d)
    return 6;
  if (a && b && !c && d)
    return 7;
  if (a && b && c && !d)
    return 8;

  return 0; // Kombinasi tidak dikenal
}

/**
 * @brief Mengupdate pin output ke NEWKER sesuai nomor tool.
 * @param tool_number: Nomor tool yang aktif (1-8).
 */
static void Update_Newker_Tool_Output(uint8_t tool_number)
{
  // Reset semua pin output tool
  HAL_GPIO_WritePin(GPIOC, NEWKER_TOOL_1_Pin | NEWKER_TOOL_2_Pin | NEWKER_TOOL_3_Pin | NEWKER_TOOL_4_Pin | NEWKER_TOOL_5_Pin | NEWKER_TOOL_6_Pin | NEWKER_TOOL_7_Pin | NEWKER_TOOL_8_Pin, GPIO_PIN_RESET);

  switch (tool_number)
  {
  case 1:
    HAL_GPIO_WritePin(NEWKER_TOOL_1_GPIO_Port, NEWKER_TOOL_1_Pin, GPIO_PIN_SET);
    break;
  case 2:
    HAL_GPIO_WritePin(NEWKER_TOOL_2_GPIO_Port, NEWKER_TOOL_2_Pin, GPIO_PIN_SET);
    break;
  case 3:
    HAL_GPIO_WritePin(NEWKER_TOOL_3_GPIO_Port, NEWKER_TOOL_3_Pin, GPIO_PIN_SET);
    break;
  case 4:
    HAL_GPIO_WritePin(NEWKER_TOOL_4_GPIO_Port, NEWKER_TOOL_4_Pin, GPIO_PIN_SET);
    break;
  case 5:
    HAL_GPIO_WritePin(NEWKER_TOOL_5_GPIO_Port, NEWKER_TOOL_5_Pin, GPIO_PIN_SET);
    break;
  case 6:
    HAL_GPIO_WritePin(NEWKER_TOOL_6_GPIO_Port, NEWKER_TOOL_6_Pin, GPIO_PIN_SET);
    break;
  case 7:
    HAL_GPIO_WritePin(NEWKER_TOOL_7_GPIO_Port, NEWKER_TOOL_7_Pin, GPIO_PIN_SET);
    break;
  case 8:
    HAL_GPIO_WritePin(NEWKER_TOOL_8_GPIO_Port, NEWKER_TOOL_8_Pin, GPIO_PIN_SET);
    break;
  default:
    break; // Tidak melakukan apa-apa jika tool tidak valid
  }
}

/**
 * @brief Memutar ATC sampai tool berikutnya terdeteksi.
 * @param direction: Arah putaran (CW atau CCW).
 */
static void ATC_Rotate(ATCRotationDirection_t direction)
{
  if (direction == ATC_NO_ROTATION)
    return;

  // Set arah motor
  GPIO_PinState dir_pin_state = (direction == ATC_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, dir_pin_state);

  // Aktifkan motor
  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_RESET); // Asumsi ENA aktif LOW
  osDelay(10);                                                               // Tunggu sebentar setelah enable

  uint8_t initial_tool = Read_Tool_Position();
  uint8_t current_tool = initial_tool;

  printf("ATC Rotating %s from tool %d...\r\n", (direction == ATC_CW) ? "CW" : "CCW", initial_tool);

  // Putar motor sampai nomor tool berubah
  while (current_tool == initial_tool || current_tool == 0)
  {
    Stepper_Pulse();
    current_tool = Read_Tool_Position();
    // Tambahkan timeout di sini jika perlu untuk mencegah loop tak terbatas
  }

  // Matikan motor
  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_SET);

  g_current_tool = current_tool;
  Update_Newker_Tool_Output(g_current_tool);
  printf("ATC Rotation finished. Current tool: %d\r\n", g_current_tool);
}

/**
 * @brief Mendapatkan status lock/unlock dari sensor.
 * @retval Status lock (ATC_LOCKED atau ATC_UNLOCKED).
 */
static ATCLockState_t Get_Lock_Status(void)
{
  // Asumsi PROXY_LOCK aktif (LOW) saat terkunci
  if (HAL_GPIO_ReadPin(PROXY_LOCK_GPIO_Port, PROXY_LOCK_Pin) == GPIO_PIN_RESET)
  {
    return ATC_LOCKED;
  }
  return ATC_UNLOCKED;
}

/**
 * @brief Prosedur untuk membuka kunci ATC.
 */
static void ATC_Unlock(void)
{
  printf("ATC Unlocking...\r\n");
  // Cek apakah ATC sudah pada posisi yang benar untuk di-unlock
  if (HAL_GPIO_ReadPin(PROXY_POSITION_GPIO_Port, PROXY_POSITION_Pin) == GPIO_PIN_SET)
  {
    printf("Error: ATC not in correct position to unlock!\r\n");
    // Mungkin perlu berputar sedikit untuk mencari posisi
    return;
  }

  // Aktifkan solenoid unlock
  HAL_GPIO_WritePin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_SET); // Asumsi SET = UNLOCK

  // Tunggu sinyal dari PROXY_LOCK yang menandakan sudah unlock
  if (osSemaphoreAcquire(ATCLockSemaHandle, 1000) != osOK)
  { // Timeout 1 detik
    printf("Error: ATC Unlock timeout!\r\n");
    // Matikan solenoid jika timeout
    HAL_GPIO_WritePin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_RESET);
  }
  else
  {
    g_atc_lock_status = ATC_UNLOCKED;
    printf("ATC Unlocked successfully.\r\n");
  }
}

/**
 * @brief Prosedur untuk mengunci ATC.
 */
static void ATC_Lock(void)
{
  printf("ATC Locking...\r\n");
  // Matikan solenoid (atau aktifkan solenoid lock jika sistemnya berbeda)
  HAL_GPIO_WritePin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_RESET); // Asumsi RESET = LOCK

  // Tunggu sinyal dari PROXY_LOCK yang menandakan sudah lock
  if (osSemaphoreAcquire(ATCLockSemaHandle, 1000) != osOK)
  { // Timeout 1 detik
    printf("Error: ATC Lock timeout!\r\n");
  }
  else
  {
    g_atc_lock_status = ATC_LOCKED;
    printf("ATC Locked successfully.\r\n");
  }
}

/**
 * @brief GPIO EXTI callback.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  else if (GPIO_Pin == PROXY_LOCK_Pin)
  {
    // Memberi sinyal bahwa status lock telah berubah
    osSemaphoreRelease(ATCLockSemaHandle);
  }
  else if (GPIO_Pin == PROXY_POSITION_Pin)
  {
    // Memberi sinyal bahwa ATC berada di posisi yang benar
    osSemaphoreRelease(ATCPositionSemaHandle);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAtcTask */
/**
 * @brief  Function implementing the atcTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAtcTask */
void StartAtcTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  printf("ATC Task Started.\r\n");

  // Inisialisasi status awal
  g_atc_lock_status = Get_Lock_Status();
  g_current_tool = Read_Tool_Position();
  Update_Newker_Tool_Output(g_current_tool);
  printf("Initial state: Lock=%s, Tool=%d\r\n",
         (g_atc_lock_status == ATC_LOCKED) ? "LOCKED" : "UNLOCKED",
         g_current_tool);

  ATCRotationDirection_t rotation_cmd;
  /* Infinite loop */
  for (;;)
  {
    // Cek apakah ada perintah rotasi dari interrupt
    if (osMessageQueueGet(ATCRotationQueueHandle, &rotation_cmd, NULL, 10) == osOK)
    {
      // 1. Pastikan ATC dalam keadaan terkunci sebelum memulai
      if (g_atc_lock_status != ATC_LOCKED)
      {
        printf("Warning: ATC is not locked. Attempting to lock first.\r\n");
        ATC_Lock();
      }

      // 2. Jika sudah terkunci, buka kuncinya
      if (g_atc_lock_status == ATC_LOCKED)
      {
        ATC_Unlock();
      }

      // 3. Jika sudah tidak terkunci, lakukan rotasi
      if (g_atc_lock_status == ATC_UNLOCKED)
      {
        ATC_Rotate(rotation_cmd);
      }

      // 4. Setelah rotasi selesai, kunci kembali ATC
      ATC_Lock();
    }

    // Logika untuk IN_NEWKER_TOK bisa ditambahkan di sini
    // Contoh: jika IN_NEWKER_TOK aktif, lakukan lock/unlock
    if (HAL_GPIO_ReadPin(IN_NEWKER_TOK_GPIO_Port, IN_NEWKER_TOK_Pin) == GPIO_PIN_RESET)
    {
      osDelay(20); // Debounce
      if (HAL_GPIO_ReadPin(IN_NEWKER_TOK_GPIO_Port, IN_NEWKER_TOK_Pin) == GPIO_PIN_RESET)
      {
        if (g_atc_lock_status == ATC_LOCKED)
        {
          ATC_Unlock();
        }
        else
        {
          ATC_Lock();
        }
        // Tunggu sampai pin dilepas untuk menghindari eksekusi berulang
        while (HAL_GPIO_ReadPin(IN_NEWKER_TOK_GPIO_Port, IN_NEWKER_TOK_Pin) == GPIO_PIN_RESET)
          ;
      }
    }
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
