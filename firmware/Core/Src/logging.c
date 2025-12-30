#include "logging.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

/* External HAL handles from CubeMX */
extern I2C_HandleTypeDef hi2c1;     // BMP580
extern SPI_HandleTypeDef hspi1;     // ICM45686
extern UART_HandleTypeDef huart1;   // GPS (PE7/PE8)
extern SD_HandleTypeDef hsd1;

// FATFS
static FATFS fs;
static FIL log_file;
static bool sd_ok = false;

// Timing
static uint32_t last_tick = 0;

// Simple log buffer
static char log_buf[256];

/* ---------- TEMP SENSOR READ STUBS ---------- */

static void read_bmp580(float *p, float *t)
{
    // Placeholder values
    *p = 101325.0f;
    *t = 25.0f;
}

static void read_icm45686(float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz)
{
    *ax = *ay = *az = 0.0f;
    *gx = *gy = *gz = 0.0f;
}

/* -------------------------------------------- */

bool logging_init(void)
{
    // Mount SD card
    if (f_mount(&fs, "", 1) != FR_OK)
        return false;

    // Open log file
    if (f_open(&log_file, "log.csv",
               FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
        return false;

    f_lseek(&log_file, f_size(&log_file));

    // Write CSV header
    const char *header =
        "time_ms,ax,ay,az,gx,gy,gz,pressure,temp\r\n";
    UINT bw;
    f_write(&log_file, header, strlen(header), &bw);

    sd_ok = true;
    return true;
}

void logging_tick(void)
{
    if (HAL_GetTick() - last_tick < 10)
        return;     // 100 Hz

    last_tick = HAL_GetTick();

    float ax, ay, az, gx, gy, gz;
    float pressure, temperature;

    read_icm45686(&ax, &ay, &az, &gx, &gy, &gz);
    read_bmp580(&pressure, &temperature);

    int len = snprintf(log_buf, sizeof(log_buf),
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\r\n",
        HAL_GetTick(),
        ax, ay, az,
        gx, gy, gz,
        pressure, temperature);

    // Write to SD 
    if (sd_ok)
    {
        UINT bw;
        f_write(&log_file, log_buf, len, &bw);
        f_sync(&log_file);
    }

    // Mirror to USB CDC 
    CDC_Transmit_FS((uint8_t *)log_buf, len);
}
