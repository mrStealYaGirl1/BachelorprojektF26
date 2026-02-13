#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "bmi270.h"
#include "esp_rom_sys.h"   // <-- ADD THIS

static const char *TAG = "IMU";

/* ===== Pins ===== */
#define PIN_MOSI  35
#define PIN_SCLK  36
#define PIN_MISO  37
#define PIN_CS    10

static spi_device_handle_t s_bmi_spi = NULL;
static bmi270_dev_t s_bmi;

/* =====================================================
   SPI LOW-LEVEL FUNCTIONS FOR BMI270 DRIVER
===================================================== */

static esp_err_t spi_write(uint8_t reg,
                           const uint8_t *data,
                           uint16_t len,
                           void *intf_ptr)
{
    uint8_t tx[1 + len];
    tx[0] = reg & 0x7F;   // write
    memcpy(&tx[1], data, len);

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;

    return spi_device_transmit(s_bmi_spi, &t);
}

static esp_err_t spi_read(uint8_t reg,
                          uint8_t *data,
                          uint16_t len,
                          void *intf_ptr)
{
    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    tx[0] = reg | 0x80;   // read
    memset(&tx[1], 0, len);

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(s_bmi_spi, &t);
    if (err != ESP_OK)
        return err;

    memcpy(data, &rx[1], len);
    return ESP_OK;
}

static void spi_delay(uint32_t us, void *intf_ptr)
{
    esp_rom_delay_us(us);
}

/* =====================================================
   SPI SETUP
===================================================== */

static esp_err_t bmi270_spi_setup(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_bmi_spi));

    return ESP_OK;
}

/* =====================================================
   IMU INIT
===================================================== */

void imu_init(void)
{
    ESP_LOGI(TAG, "Initializing BMI270...");

    ESP_ERROR_CHECK(bmi270_spi_setup());

    vTaskDelay(pdMS_TO_TICKS(50));

    s_bmi.read = spi_read;
    s_bmi.write = spi_write;
    s_bmi.delay_us = spi_delay;
    s_bmi.intf_ptr = NULL;

    esp_err_t err = bmi270_init(&s_bmi);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BMI270 init failed!");
        return;
    }

    ESP_LOGI(TAG, "BMI270 initialized successfully!");

    bmi270_enable_accel(&s_bmi);
    bmi270_enable_gyro(&s_bmi);
}

/* =====================================================
   TASK
===================================================== */

void imu_task(void *pvParameters)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    while (1)
    {
        if (bmi270_read_accel(&s_bmi, &ax, &ay, &az) == ESP_OK &&
            bmi270_read_gyro(&s_bmi, &gx, &gy, &gz) == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "ACC: %d %d %d | GYRO: %d %d %d",
                     ax, ay, az,
                     gx, gy, gz);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}




/* Kathrines suggestion for imu.c
// imu.c: 
#include "imu.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "bmi270.h"

static const char *TAG = "IMU";
static bmi270_t s_bmi;

// =====================================================
// IMU INIT
// =====================================================
void imu_init(void)
{
    ESP_LOGI(TAG, "IMU init...");
    esp_err_t err = bmi270_init(&s_bmi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 init failed: %s", esp_err_to_name(err));
    }
}
// =====================================================
// TASK
// =====================================================
void imu_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "IMU task started");

    while (1) {
        ESP_LOGI(TAG, "IMU running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/