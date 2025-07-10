#ifndef __SETTINGS_H
#define __SETTINGS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// Alamat Flash untuk menyimpan pengaturan.
// Untuk STM32F103C8T6 (64KB Flash), halaman terakhir dimulai di 0x0800FC00.
// Ukuran halaman adalah 1KB.
#define SETTINGS_FLASH_PAGE_ADDRESS 0x0800FC00
#define SETTINGS_MAGIC_NUMBER 0xDEADBEEF

    // Struktur untuk menyimpan semua pengaturan yang dapat dikonfigurasi
    typedef struct
    {
        uint32_t magic_number;     // Untuk validasi data di flash
        bool invert_input;         // $1: Membalik logika semua input (Active HIGH/LOW)
        bool invert_output;        // $2: Membalik logika semua output (Active HIGH/LOW)
        bool invert_direction;     // $3: Membalik putaran motor stepper
        bool invert_enable;        // $5: Membalik sinyal enable stepper (Active HIGH/LOW)
        uint16_t stepper_speed_hz; // $6: Kecepatan stepper dalam Hz (frekuensi pulsa)
        uint16_t timeout_ms;       // $7: Timeout untuk setiap operasi (unlock, rotate, lock)
        bool timeout_enabled;      // 1 byte. $8: Mengaktifkan/menonaktifkan timeout
        uint16_t padding;          // Padding untuk memastikan ukuran kelipatan 4 byte
        uint32_t pause_ms;
    } Settings_t;

    // Deklarasi variabel global untuk pengaturan
    extern Settings_t g_settings;

    /**
     * @brief Menginisialisasi pengaturan.
     * Membaca dari Flash jika ada, jika tidak, muat default dan simpan ke Flash.
     */
    void Settings_Init(void);

    /**
     * @brief Menulis pengaturan saat ini ke Flash Memory.
     * @retval HAL_StatusTypeDef status dari operasi penulisan.
     */
    HAL_StatusTypeDef Flash_Write_Settings(void);

    /**
     * @brief Mencetak semua pengaturan saat ini ke port serial.
     */
    void Print_Settings(void);

    /**
     * @brief Mencetak semua pin yang digunakan dan keterangannya.
     */
    void Print_Pins(void);

#ifdef __cplusplus
}
#endif

#endif /* __SETTINGS_H */
