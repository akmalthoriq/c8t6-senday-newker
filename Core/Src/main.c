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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>
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

/* USER CODE BEGIN PV */
// --- Variabel Global untuk Super-loop dan State Machine ---

// Variabel status global ATC
volatile ATCLockState_t g_atc_lock_status = ATC_LOCKED;
volatile uint8_t g_current_tool = 0;
volatile uint8_t g_target_tool = 0; // Untuk perintah 'Go to Tool'
uint8_t g_new_tool_pos;

// Variabel untuk State Machine
volatile ATC_State_t g_atc_state = ATC_STATE_IDLE;
volatile ATCRotationDirection_t g_rotation_command = ATC_NO_ROTATION;
uint32_t g_timeout_start_tick = 0;

// Event flags dari ISR (menggantikan Semaphores)
volatile bool g_lock_event = false;
volatile bool g_position_event = false;
volatile bool g_tool_change_event = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
// Fungsi-fungsi untuk ATC
static void ATC_Start_Rotation_PWM(uint16_t frequency_hz);
static void ATC_Stop_Rotation_PWM(void);
static GPIO_PinState Read_Input_Pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static void Write_Output_Pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
static uint8_t Read_Tool_Position(void);
static void Poll_Tool_Position(void);
static void Update_Newker_Tool_Output(uint8_t tool_number);
static ATCLockState_t Get_Lock_Status(void);
void ATC_Process_StateMachine(void);
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
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000); // Tunda sejenak agar USB siap

  Settings_Init(); // Inisialisasi pengaturan dari Flash atau gunakan default

  printf("\r\nATC Controller Ready (Super-Loop).\r\n");
  printf("Kirim '$$' untuk melihat pengaturan, '$P' untuk pinout.\r\n");

  g_atc_lock_status = Get_Lock_Status();
  g_current_tool = Read_Tool_Position();
  Update_Newker_Tool_Output(g_current_tool);
  printf("Initial state: Lock=%s, Tool=%d\r\n", (g_atc_lock_status == ATC_LOCKED) ? "LOCKED" : "UNLOCKED", g_current_tool);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ATC_Process_StateMachine();

    // Jalankan polling posisi tool hanya saat sistem IDLE
    if (g_atc_state == ATC_STATE_IDLE)
    {
      Poll_Tool_Position();
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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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

  /*Configure GPIO pin : IN_NEWKER_TOK_Pin */
  GPIO_InitStruct.Pin = IN_NEWKER_TOK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN_NEWKER_TOK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_NEWKER_T_PLUS_Pin IN_NEWKER_T_MIN_Pin PROXY_TOOL_C_Pin PROXY_TOOL_B_Pin
                           PROXY_TOOL_A_Pin PROXY_POSITION_Pin PROXY_LOCK_Pin */
  GPIO_InitStruct.Pin = IN_NEWKER_T_PLUS_Pin | IN_NEWKER_T_MIN_Pin | PROXY_TOOL_C_Pin | PROXY_TOOL_B_Pin | PROXY_TOOL_A_Pin | PROXY_POSITION_Pin | PROXY_LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PROXY_TOOL_D_Pin */
  GPIO_InitStruct.Pin = PROXY_TOOL_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PROXY_TOOL_D_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Process_Serial_Command(uint8_t *buf, uint32_t len)
{
  if (len == 0)
    return;

  // Ubah perintah menjadi huruf besar untuk konsistensi
  for (uint32_t i = 0; i < len; i++)
  {
    buf[i] = toupper(buf[i]);
  }

  char cmd_buffer[32];
  uint32_t copy_len = len < 31 ? len : 31;
  strncpy(cmd_buffer, (char *)buf, copy_len);
  cmd_buffer[copy_len] = '\0';

  // Perintah Pengaturan ($)
  if (cmd_buffer[0] == '$')
  {
    if (strncmp(cmd_buffer, "$$", 2) == 0)
    {
      Print_Settings();
    }
    else if (strncmp(cmd_buffer, "$P", 2) == 0)
    {
      Print_Pins();
    }
    else
    {
      int setting_num = 0, value = 0;
      if (sscanf(cmd_buffer, "$%d=%d", &setting_num, &value) == 2)
      {
        bool changed = true;
        switch (setting_num)
        {
        case 1:
          g_settings.invert_input = (value != 0);
          break;
        case 2:
          g_settings.invert_output = (value != 0);
          break;
        case 3:
          g_settings.invert_direction = (value != 0);
          break;
        case 5:
          g_settings.invert_enable = (value != 0);
          break;
        case 6:
          g_settings.stepper_speed_hz = value;
          break;
        case 7:
          g_settings.timeout_ms = value;
          break;
        case 8:
          g_settings.timeout_enabled = (value != 0);
          break; // Pengaturan baru
        default:
          changed = false;
          printf("Error: Unknown setting $%d\r\n", setting_num);
          break;
        }
        if (changed)
        {
          if (Flash_Write_Settings() == HAL_OK)
          {
            printf("[OK] Setting $%d changed to %d and saved.\r\n", setting_num, value);
          }
          else
          {
            printf("Error: Failed to save settings to Flash!\r\n");
          }
        }
      }
    }
  }
  // Perintah Ganti Tool (T#)
  else if (cmd_buffer[0] == 'T')
  {
    int tool_num = 0;
    if (sscanf(cmd_buffer, "T%d", &tool_num) == 1)
    {
      if (tool_num >= 1 && tool_num <= 8)
      {
        ATC_GoToTool(tool_num);
      }
      else
      {
        printf("Error: Invalid tool number. Must be 1-8.\r\n");
      }
    }
  }
  // Perintah Manual Satu Huruf
  else if (len == 1)
  {
    switch (cmd_buffer[0])
    {
    case 'A':
      ATC_Rotate(ATC_CW, ATC_ROTATION_MANUAL);
      break;
    case 'B':
      ATC_Rotate(ATC_CCW, ATC_ROTATION_MANUAL);
      break;
    case 'C':
      ATC_Stop();
      break;
    case 'L':
      ATC_Manual_Lock();
      break;
    case 'U':
      ATC_Manual_Unlock();
      break;
    default:
      printf("Error: Unknown command '%c'\r\n", cmd_buffer[0]);
      break;
    }
  }
  else
  {
    printf("Error: Unknown command format.\r\n");
  }
}

/**
 * @brief  Secara berkala memeriksa posisi tool dan melaporkan jika ada perubahan.
 */
static void Poll_Tool_Position(void)
{
  static uint8_t last_reported_tool = 0;

  // Untuk mencegah pembacaan yang terlalu cepat, bisa ditambahkan delay kecil
  // atau menggunakan timer jika diperlukan, namun untuk polling sederhana ini cukup.

  g_current_tool = Read_Tool_Position();
  printf("Position Updated -> Tool: %d\r\n", g_current_tool);
  HAL_Delay(500);

  // if (g_new_tool_pos != last_reported_tool)
  // {
  //   g_current_tool = g_new_tool_pos;
  //   Update_Newker_Tool_Output(g_current_tool);
  //   printf("Position Updated -> Tool: %d\r\n", g_current_tool);
  //   last_reported_tool = g_current_tool;
  // }
}
// --- Implementasi Fungsi Kontrol Manual ---
void ATC_Rotate(ATCRotationDirection_t direction, ATCRotationMode_t mode)
{
  if (g_atc_state != ATC_STATE_IDLE && mode == ATC_ROTATION_MANUAL)
  {
    printf("Error: ATC is busy. Cannot start manual rotation.\r\n");
    return;
  }

  if (g_atc_lock_status == ATC_LOCKED)
  {
    printf("Error: ATC is locked. Unlock first with 'U'.\r\n");
    return;
  }
  printf("Manual Rotate: %s\r\n", (direction == ATC_CW) ? "CW" : "CCW");
  GPIO_PinState dir_state = (direction == ATC_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, g_settings.invert_direction ? !dir_state : dir_state);
  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, g_settings.invert_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
  ATC_Start_Rotation_PWM(g_settings.stepper_speed_hz);
}

void ATC_Stop(void)
{
  printf("Manual Stop\r\n");
  ATC_Stop_Rotation_PWM();
  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, g_settings.invert_enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void ATC_Manual_Unlock(void)
{
  if (g_atc_state != ATC_STATE_IDLE)
  {
    printf("Error: ATC is busy.\r\n");
    return;
  }
  printf("Manual Unlock...\r\n");
  Write_Output_Pin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_SET);
  g_lock_event = false; // Set event flag untuk menunggu lock
  g_timeout_start_tick = HAL_GetTick();
  g_atc_state = ATC_STATE_MANUAL_UNLOCK_WAIT;
}

void ATC_Manual_Lock(void)
{
  if (g_atc_state != ATC_STATE_IDLE)
  {
    printf("Error: ATC is busy.\r\n");
    return;
  }
  printf("Manual Lock...\r\n");
  Write_Output_Pin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_RESET);
  g_lock_event = false;
  g_timeout_start_tick = HAL_GetTick();
  g_atc_state = ATC_STATE_MANUAL_LOCK_WAIT;
}

void ATC_GoToTool(uint8_t target_tool)
{
  if (g_atc_state != ATC_STATE_IDLE)
  {
    printf("Error: ATC is busy.\r\n");
    return;
  }
  if (target_tool == g_current_tool)
  {
    printf("Already at tool %d.\r\n", target_tool);
    return;
  }
  g_target_tool = target_tool;
  g_atc_state = ATC_STATE_GOTO_START;
}

/**
 * @brief Fungsi utama yang menjalankan state machine ATC.
 */

void ATC_Process_StateMachine(void)
{
  // Cek timeout untuk state yang menunggu event, jika diaktifkan
  if (g_settings.timeout_enabled &&
      (g_atc_state == ATC_STATE_AUTO_UNLOCK_WAIT || g_atc_state == ATC_STATE_AUTO_ROTATING_WAIT ||
       g_atc_state == ATC_STATE_AUTO_LOCK_WAIT || g_atc_state == ATC_STATE_MANUAL_LOCK_WAIT ||
       g_atc_state == ATC_STATE_MANUAL_UNLOCK_WAIT))
  {

    if ((HAL_GetTick() - g_timeout_start_tick) > g_settings.timeout_ms)
    {
      printf("Error: Timeout in state %d\r\n", g_atc_state);
      ATC_Stop();
      Write_Output_Pin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_RESET);
      g_atc_state = ATC_STATE_TIMEOUT_ERROR;
    }
  }

  switch (g_atc_state)
  {
  case ATC_STATE_IDLE:
    if (g_rotation_command != ATC_NO_ROTATION)
    {
      g_atc_state = ATC_STATE_AUTO_UNLOCK_START;
    }
    break;

  // --- Siklus Otomatis (dari input pin) ---
  case ATC_STATE_AUTO_UNLOCK_START:
    printf("Auto-cycle: Unlocking...\r\n");
    Write_Output_Pin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_SET);
    g_lock_event = false;
    g_timeout_start_tick = HAL_GetTick();
    g_atc_state = ATC_STATE_AUTO_UNLOCK_WAIT;
    break;
  case ATC_STATE_AUTO_UNLOCK_WAIT:
    if (g_lock_event)
    {
      g_lock_event = false;
      g_atc_lock_status = ATC_UNLOCKED;
      g_atc_state = ATC_STATE_AUTO_ROTATING_START;
    }
    break;
  case ATC_STATE_AUTO_ROTATING_START:
    ATC_Rotate(g_rotation_command, ATC_ROTATION_AUTO);
    g_tool_change_event = false; // Reset event flag
    printf("Auto-cycle: Rotating %s...\r\n", (g_rotation_command == ATC_CW) ? "CW" : "CCW");
    g_timeout_start_tick = HAL_GetTick();
    g_atc_state = ATC_STATE_AUTO_ROTATING_WAIT;
    break;
  case ATC_STATE_AUTO_ROTATING_WAIT:
    if (g_tool_change_event)
    {
      // g_tool_change_event = false;
      g_current_tool = Read_Tool_Position();
      Update_Newker_Tool_Output(g_current_tool);
      if (g_current_tool == g_target_tool) // Jika sudah sampai di tool yang dituju
      {
        ATC_Stop(); // Hentikan rotasi
        printf("Auto-cycle: Reached target tool %d.\r\n", g_target_tool);
        g_rotation_command = ATC_NO_ROTATION;    // Reset perintah rotasi
        g_atc_state = ATC_STATE_AUTO_LOCK_START; // Lanjut ke siklus kunci
      }
      else
        printf("Auto-cycle: Did not reach target tool %d, current tool is %d.\r\n", g_target_tool, g_current_tool);
    }
    break;
  case ATC_STATE_AUTO_LOCK_START:
    printf("Auto-cycle: Locking...\r\n");
    Write_Output_Pin(OUT_ATC_LOCK_GPIO_Port, OUT_ATC_LOCK_Pin, GPIO_PIN_RESET);
    g_lock_event = false;
    g_timeout_start_tick = HAL_GetTick();
    g_atc_state = ATC_STATE_AUTO_LOCK_WAIT;
    break;
  case ATC_STATE_AUTO_LOCK_WAIT:
    if (g_lock_event)
    {
      g_lock_event = false;
      g_atc_lock_status = ATC_LOCKED;
      printf("Auto-cycle: Complete.\r\n");
      g_atc_state = ATC_STATE_IDLE;
    }
    break;

  // --- Siklus Go To Tool (dari serial) ---
  case ATC_STATE_GOTO_START:
    if (g_current_tool == g_target_tool)
    {
      printf("GoTo: Already at tool %d.\r\n", g_target_tool);
      g_atc_state = ATC_STATE_IDLE;
      break;
    }
    printf("GoTo: Moving to tool %d\r\n", g_target_tool);
    // Tentukan arah putaran terdekat
    int diff = g_target_tool - g_current_tool;
    if (abs(diff) <= 4)
    { // 8 total tools / 2
      g_rotation_command = (diff > 0) ? ATC_CW : ATC_CCW;
    }
    else
    {
      g_rotation_command = (diff > 0) ? ATC_CCW : ATC_CW;
    }
    g_atc_state = ATC_STATE_AUTO_UNLOCK_START; // Memulai siklus otomatis
    break;

  // --- State Tunggu Manual ---
  case ATC_STATE_MANUAL_UNLOCK_WAIT:
    if (g_lock_event)
    {
      g_lock_event = false;
      g_atc_lock_status = ATC_UNLOCKED;
      printf("Manual Unlock successful.\r\n");
      g_atc_state = ATC_STATE_IDLE;
    }
    break;
  case ATC_STATE_MANUAL_LOCK_WAIT:
    if (g_lock_event)
    {
      g_lock_event = false;
      g_atc_lock_status = ATC_LOCKED;
      printf("Manual Lock successful.\r\n");
      g_atc_state = ATC_STATE_IDLE;
    }
    break;

  case ATC_STATE_TIMEOUT_ERROR:
    printf("Error state. Resetting to Idle in 2 seconds...\r\n");
    g_rotation_command = ATC_NO_ROTATION;
    HAL_Delay(2000);
    g_atc_state = ATC_STATE_IDLE;
    break;
  }
}

static void ATC_Start_Rotation_PWM(uint16_t frequency_hz)
{
  if (frequency_hz == 0)
    return;
  uint32_t tim_clock = HAL_RCC_GetPCLK2Freq();
  uint32_t prescaler = (tim_clock / (frequency_hz * 2000UL)) - 1;
  if (prescaler > 65535)
    prescaler = 65535;
  uint32_t period = (tim_clock / (frequency_hz * (prescaler + 1))) - 1;
  if (period > 65535)
    period = 65535;
  __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
  __HAL_TIM_SET_AUTORELOAD(&htim1, period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

static void ATC_Stop_Rotation_PWM(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

static GPIO_PinState Read_Input_Pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  return g_settings.invert_input ? !state : state;
}

static void Write_Output_Pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, g_settings.invert_output ? !PinState : PinState);
}

static uint8_t Read_Tool_Position(void)
{
  static uint8_t last_tool = 1; // Default ke 1 pada awal boot
  bool a = Read_Input_Pin(PROXY_TOOL_A_GPIO_Port, PROXY_TOOL_A_Pin) == GPIO_PIN_SET;
  bool b = Read_Input_Pin(PROXY_TOOL_B_GPIO_Port, PROXY_TOOL_B_Pin) == GPIO_PIN_SET;
  bool c = Read_Input_Pin(PROXY_TOOL_C_GPIO_Port, PROXY_TOOL_C_Pin) == GPIO_PIN_SET;
  bool d = Read_Input_Pin(PROXY_TOOL_D_GPIO_Port, PROXY_TOOL_D_Pin) == GPIO_PIN_SET;
  uint8_t tool = last_tool;

  if (!a && b && !c && !d)
    tool = 1;
  else if (a && !b && !c && !d)
    tool = 2;
  else if (!a && !b && !c && d)
    tool = 3;
  else if (!a && !b && c && !d)
    tool = 4;
  else if (!a && b && c && d)
    tool = 5;
  else if (a && !b && c && d)
    tool = 6;
  else if (a && b && !c && d)
    tool = 7;
  else if (a && b && c && !d)
    tool = 8;
  // Jika tidak cocok, tool tetap last_tool

  last_tool = tool;
  return tool;
}

static void Update_Newker_Tool_Output(uint8_t tool_number)
{
  Write_Output_Pin(NEWKER_TOOL_1_GPIO_Port, NEWKER_TOOL_1_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_2_GPIO_Port, NEWKER_TOOL_2_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_3_GPIO_Port, NEWKER_TOOL_3_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_4_GPIO_Port, NEWKER_TOOL_4_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_5_GPIO_Port, NEWKER_TOOL_5_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_6_GPIO_Port, NEWKER_TOOL_6_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_7_GPIO_Port, NEWKER_TOOL_7_Pin, GPIO_PIN_RESET);
  Write_Output_Pin(NEWKER_TOOL_8_GPIO_Port, NEWKER_TOOL_8_Pin, GPIO_PIN_RESET);

  switch (tool_number)
  {
  case 1:
    Write_Output_Pin(NEWKER_TOOL_1_GPIO_Port, NEWKER_TOOL_1_Pin, GPIO_PIN_SET);
    break;
  case 2:
    Write_Output_Pin(NEWKER_TOOL_2_GPIO_Port, NEWKER_TOOL_2_Pin, GPIO_PIN_SET);
    break;
  case 3:
    Write_Output_Pin(NEWKER_TOOL_3_GPIO_Port, NEWKER_TOOL_3_Pin, GPIO_PIN_SET);
    break;
  case 4:
    Write_Output_Pin(NEWKER_TOOL_4_GPIO_Port, NEWKER_TOOL_4_Pin, GPIO_PIN_SET);
    break;
  case 5:
    Write_Output_Pin(NEWKER_TOOL_5_GPIO_Port, NEWKER_TOOL_5_Pin, GPIO_PIN_SET);
    break;
  case 6:
    Write_Output_Pin(NEWKER_TOOL_6_GPIO_Port, NEWKER_TOOL_6_Pin, GPIO_PIN_SET);
    break;
  case 7:
    Write_Output_Pin(NEWKER_TOOL_7_GPIO_Port, NEWKER_TOOL_7_Pin, GPIO_PIN_SET);
    break;
  case 8:
    Write_Output_Pin(NEWKER_TOOL_8_GPIO_Port, NEWKER_TOOL_8_Pin, GPIO_PIN_SET);
    break;
  default:
    break;
  }
}

static ATCLockState_t Get_Lock_Status(void)
{
  return (Read_Input_Pin(PROXY_LOCK_GPIO_Port, PROXY_LOCK_Pin) == GPIO_PIN_SET) ? ATC_LOCKED : ATC_UNLOCKED;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case IN_NEWKER_T_MIN_Pin:
    if (HAL_GPIO_ReadPin(IN_NEWKER_T_MIN_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET)
    {
      if (g_atc_state == ATC_STATE_IDLE)
        g_rotation_command = ATC_CW;
    }
    break;
  case IN_NEWKER_T_PLUS_Pin:
    if (HAL_GPIO_ReadPin(IN_NEWKER_T_PLUS_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET)
    {
      if (g_atc_state == ATC_STATE_IDLE)
        g_rotation_command = ATC_CCW;
    }
    break;
  case PROXY_LOCK_Pin:
    if (HAL_GPIO_ReadPin(PROXY_LOCK_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET)
    {
      g_atc_lock_status = ATC_LOCKED;
      g_lock_event = true;
    }
    else
    {
      g_atc_lock_status = ATC_UNLOCKED;
      g_lock_event = true;
    }
    break;
  case PROXY_POSITION_Pin:
    g_position_event = true;
    break;
  case PROXY_TOOL_A_Pin:
  case PROXY_TOOL_B_Pin:
  case PROXY_TOOL_C_Pin:
  case PROXY_TOOL_D_Pin:
    g_tool_change_event = true;
    break;
  }
}

/* USER CODE END 4 */

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
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    HAL_Delay(100);
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
