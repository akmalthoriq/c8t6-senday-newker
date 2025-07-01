# STM32F103C8T6 ATC (Automatic Tool Changer) Firmware

Firmware ini dirancang untuk mengontrol Automatic Tool Changer (ATC) berbasis mikrokontroler STM32F103C8T6. Program menggunakan super-loop (tanpa RTOS), mendukung komunikasi USB CDC (virtual COM port) untuk konfigurasi dan monitoring, serta menyimpan pengaturan ke Flash.

## Fitur Utama
- Kontrol motor stepper untuk rotasi ATC (CW/CCW) dengan output PWM (TIM1).
- Deteksi posisi dan status kunci ATC menggunakan sensor/proxy.
- Pengaturan parameter (seperti arah, enable, kecepatan stepper, timeout, logika input/output) yang dapat disimpan ke Flash dan diubah via perintah serial.
- Output digital untuk memilih tool (NEWKER_TOOL_1..8) dan mengunci/membuka ATC.
- Komunikasi serial USB untuk perintah konfigurasi dan monitoring status.
- Implementasi state machine berbasis super-loop, bukan multitasking RTOS.

## File Penting
- `main.c` : Program utama, inisialisasi hardware, super-loop, state machine, dan logika ATC.
- `settings.c/h` : Manajemen pengaturan yang dapat disimpan ke Flash dan diakses via perintah serial.
- `main.h` : Definisi pinout, tipe data, state machine, dan makro.

## Cara Konfigurasi
- Hubungkan board ke PC, gunakan terminal serial (baudrate tidak relevan, gunakan port USB CDC).
- Perintah `$$` untuk melihat semua pengaturan.
- Perintah `$P` untuk melihat daftar pinout.
- Perintah `$n=val` untuk mengubah pengaturan (misal `$1=1` untuk membalik logika input, `$7=3000` untuk mengatur timeout operasi dalam ms).

### Daftar Pengaturan Serial
- `$1` = invert_input (membalik logika semua input)
- `$2` = invert_output (membalik logika semua output)
- `$3` = invert_direction (membalik arah stepper)
- `$5` = invert_enable (membalik sinyal enable stepper)
- `$6` = stepper_speed_hz (frekuensi PWM stepper)
- `$7` = timeout_ms (timeout operasi ATC dalam ms)

### Daftar Pinout (lihat juga output `$P` di serial)
- IN_NEWKER_T_MIN (CW)  : PB14
- IN_NEWKER_T_PLUS (CCW): PB13
- IN_NEWKER_TOK         : PB12
- STEPPER_PULSE (PWM)   : PA8 (TIM1_CH1)
- STEPPER_DIR           : PA9
- STEPPER_ENA           : PA10
- OUT_ATC_LOCK          : PB15
- PROXY_POSITION        : PB6
- PROXY_LOCK            : PB7
- PROXY_TOOL_A-D        : PB5, PB4, PB3, PA15
- NEWKER_TOOL_1-7       : PA1-PA7
- NEWKER_TOOL_8         : PB0

## Catatan
- Firmware ini dapat dikembangkan lebih lanjut untuk menambah fitur atau menyesuaikan dengan hardware ATC lain.
- Semua pengaturan disimpan di Flash dan otomatis dimuat saat booting.
- Timeout operasi dapat diatur untuk mencegah ATC macet.
