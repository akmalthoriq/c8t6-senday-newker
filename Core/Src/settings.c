#include "settings.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "usbd_cdc_if.h"
#include <ctype.h>

// Definisikan variabel global pengaturan
Settings_t g_settings;

// Fungsi helper untuk menulis ke flash
static HAL_StatusTypeDef Flash_Write_Data(uint32_t address, uint32_t *data, uint32_t num_words)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = address;
    EraseInitStruct.NbPages = 1;
    uint32_t PageError = 0;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    for (uint32_t i = 0; i < num_words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i * 4), data[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

// Fungsi helper untuk membaca dari flash
static void Flash_Read_Data(uint32_t address, uint32_t *data, uint32_t num_words)
{
    for (uint32_t i = 0; i < num_words; i++)
    {
        data[i] = *(__IO uint32_t *)(address + (i * 4));
    }
}

void Settings_Init(void)
{
    // Baca data dari alamat flash yang ditentukan
    Flash_Read_Data(SETTINGS_FLASH_PAGE_ADDRESS, (uint32_t *)&g_settings, sizeof(Settings_t) / 4);

    // Cek apakah data valid menggunakan magic number
    if (g_settings.magic_number == SETTINGS_MAGIC_NUMBER)
    {
        printf("Settings loaded from Flash.\r\n");
    }
    else
    {
        printf("No valid settings found in Flash. Loading defaults.\r\n");
        // Muat nilai default jika tidak ada pengaturan yang valid
        g_settings.magic_number = SETTINGS_MAGIC_NUMBER;
        g_settings.invert_input = false;
        g_settings.invert_output = false;
        g_settings.invert_direction = false;
        g_settings.invert_enable = false;
        g_settings.stepper_speed_hz = 1000;
        g_settings.timeout_ms = 3000;
        g_settings.timeout_enabled = true; // Timeout aktif secara default
        g_settings.padding = 0;

        // Tulis nilai default ke flash untuk penggunaan berikutnya
        Flash_Write_Settings();
    }
}

HAL_StatusTypeDef Flash_Write_Settings(void)
{
    return Flash_Write_Data(SETTINGS_FLASH_PAGE_ADDRESS, (uint32_t *)&g_settings, sizeof(Settings_t) / 4);
}

void Print_Settings(void)
{
    char buf[1024];
    int len = 0;
    len += sprintf(buf + len, "--- ATC Settings ---\r\n");
    len += sprintf(buf + len, "$1=%d (invert_input)\r\n", g_settings.invert_input);
    len += sprintf(buf + len, "$2=%d (invert_output)\r\n", g_settings.invert_output);
    len += sprintf(buf + len, "$3=%d (invert_direction)\r\n", g_settings.invert_direction);
    len += sprintf(buf + len, "$5=%d (invert_enable)\r\n", g_settings.invert_enable);
    len += sprintf(buf + len, "$6=%d (stepper_speed_hz)\r\n", g_settings.stepper_speed_hz);
    len += sprintf(buf + len, "$7=%d (timeout_ms)\r\n", g_settings.timeout_ms);
    len += sprintf(buf + len, "$8=%d (timeout_enabled)\r\n", g_settings.timeout_enabled);
    CDC_Transmit_FS((uint8_t*)buf, len);
}

void Print_Pins(void)
{
    char buf[1024];
    int len = 0;
    len += sprintf(buf + len, "--- Pinout ---\r\n");
    len += sprintf(buf + len, "IN_NEWKER_T_MIN (CW)  : PB14\r\n");
    len += sprintf(buf + len, "IN_NEWKER_T_PLUS (CCW): PB13\r\n");
    len += sprintf(buf + len, "STEPPER_PULSE (PWM)   : PA8 (TIM1_CH1)\r\n");
    len += sprintf(buf + len, "STEPPER_DIR           : PA9\r\n");
    len += sprintf(buf + len, "STEPPER_ENA           : PA10\r\n");
    len += sprintf(buf + len, "OUT_ATC_LOCK          : PB15\r\n");
    len += sprintf(buf + len, "PROXY_POSITION        : PB6\r\n");
    len += sprintf(buf + len, "PROXY_LOCK            : PB7\r\n");
    len += sprintf(buf + len, "PROXY_TOOL_A-D        : PB5, PB4, PB3, PA15\r\n");
    len += sprintf(buf + len, "NEWKER_TOOL_1-7       : PA1-PA7\r\n");
    len += sprintf(buf + len, "NEWKER_TOOL_8         : PB0\r\n");
    CDC_Transmit_FS((uint8_t*)buf, len);
}
