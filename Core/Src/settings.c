#include "settings.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
    if (Flash_Read_Settings() && g_settings.magic_number == SETTINGS_MAGIC_NUMBER)
    {
        printf("Settings loaded from Flash.\r\n");
    }
    else
    {
        printf("No valid settings found in Flash. Loading defaults.\r\n");
        // Muat nilai default
        g_settings.magic_number = SETTINGS_MAGIC_NUMBER;
        g_settings.invert_input = false;     // $1=0
        g_settings.invert_output = false;    // $2=0
        g_settings.invert_direction = false; // $3=0
        g_settings.invert_enable = false;    // $5=0 (Aktif LOW)
        g_settings.stepper_speed_hz = 1000;  // $6=1000 (1 kHz)

        // Tulis default ke flash
        Flash_Write_Settings();
    }
}

bool Flash_Read_Settings(void)
{
    Flash_Read_Data(SETTINGS_FLASH_PAGE_ADDRESS, (uint32_t *)&g_settings, sizeof(Settings_t) / 4);
    return true;
}

HAL_StatusTypeDef Flash_Write_Settings(void)
{
    return Flash_Write_Data(SETTINGS_FLASH_PAGE_ADDRESS, (uint32_t *)&g_settings, sizeof(Settings_t) / 4);
}

void Print_Settings(void)
{
    printf("--- ATC Settings ---\r\n");
    printf("$1=%d (invert_input)\r\n", g_settings.invert_input);
    printf("$2=%d (invert_output)\r\n", g_settings.invert_output);
    printf("$3=%d (invert_direction)\r\n", g_settings.invert_direction);
    printf("$5=%d (invert_enable)\r\n", g_settings.invert_enable);
    printf("$6=%d (stepper_speed_hz)\r\n", g_settings.stepper_speed_hz);
    printf("--------------------\r\n");
}

void Print_Pins(void)
{
    printf("--- Pinout ---\r\n");
    printf("IN_NEWKER_T_MIN (CW)  : PB0\r\n");
    printf("IN_NEWKER_T_POS (CCW) : PB1\r\n");
    printf("IN_NEWKER_TOK         : PB12\r\n");
    printf("STEPPER_PULSE (PWM)   : PA0 (TIM2_CH1)\r\n");
    printf("STEPPER_DIR           : PA1\r\n");
    printf("STEPPER_ENA           : PA2\r\n");
    printf("OUT_ATC_LOCK          : PB6\r\n");
    printf("PROXY_POSITION        : PB7\r\n");
    printf("PROXY_LOCK            : PB8\r\n");
    printf("PROXY_TOOL_A          : PB5\r\n");
    printf("PROXY_TOOL_B          : PB4\r\n");
    printf("PROXY_TOOL_C          : PB3\r\n");
    printf("PROXY_TOOL_D          : PA15\r\n");
    printf("NEWKER_TOOL_1..8      : PC4..PC11\r\n");
    printf("--------------\r\n");
}

void Process_Serial_Command(uint8_t *buf, uint32_t len)
{
    if (len < 2)
        return;

    // Pastikan buffer adalah null-terminated string
    char cmd_buffer[32];
    strncpy(cmd_buffer, (char *)buf, len < 31 ? len : 31);
    cmd_buffer[len] = '\0';

    if (strncmp(cmd_buffer, "$$", 2) == 0)
    {
        Print_Settings();
    }
    else if (strncmp(cmd_buffer, "$P", 2) == 0)
    {
        Print_Pins();
    }
    else if (cmd_buffer[0] == '$')
    {
        int setting_num = 0;
        int value = 0;
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
            default:
                changed = false;
                printf("Error: Unknown setting $%d\r\n", setting_num);
                break;
            }
            if (changed)
            {
                if (Flash_Write_Settings() == HAL_OK)
                {
                    printf("[OK] Setting $%d changed to %d and saved to Flash.\r\n", setting_num, value);
                }
                else
                {
                    printf("Error: Failed to save settings to Flash!\r\n");
                }
            }
        }
    }
}
