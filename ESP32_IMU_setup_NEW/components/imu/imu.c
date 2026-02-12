#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"



static const char *TAG = "IMU";

// ====== PIN CONFIGURATION ======
// SparkFun BMI270 SPI mapping:
// SDA  -> MOSI
// SCL  -> SCK
// ADR  -> MISO
// CS   -> CSB

#define PIN_MOSI  35   // -> SDA
#define PIN_SCLK  36   // -> SCL
#define PIN_MISO  37   // -> ADR
#define PIN_CS    10   // -> CS
// =================================

#define BMI270_REG_CHIP_ID  0x00
#define BMI270_CHIP_ID      0x24

static spi_device_handle_t s_bmi_spi = NULL;


// =====================================================
// SPI SETUP
// =====================================================
static esp_err_t bmi270_spi_setup(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 3,                        // BMI270 stable in mode 3
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed");
        return err;
    }

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &s_bmi_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed");
        return err;
    }

    ESP_LOGI(TAG, "SPI initialized");
    return ESP_OK;
}


// =====================================================
// BMI270 REGISTER READ
// =====================================================
static esp_err_t bmi270_read_reg(uint8_t reg, uint8_t *val)
{
    // Read format:
    // Byte0 = 0x80 | reg
    // Byte1 = dummy
    // Byte2 = dummy
    // Data appears in rx[2]

    uint8_t tx[3] = { (uint8_t)(0x80 | reg), 0x00, 0x00 };
    uint8_t rx[3] = {0};

    spi_transaction_t t = {0};
    t.length = 24;  // 3 bytes
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(s_bmi_spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed");
        return err;
    }

    ESP_LOGI(TAG, "RAW RX: %02X %02X %02X", rx[0], rx[1], rx[2]);

    *val = rx[2];
    return ESP_OK;
}


// =====================================================
// IMU INIT
// =====================================================
void imu_init(void)
{
    ESP_LOGI(TAG, "Initializing BMI270 (SPI)...");

    // 🔴 CRITICAL: Force CS LOW before SPI init
    gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_ERROR_CHECK(bmi270_spi_setup());

    // Give sensor time after power-up
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t chip_id = 0;
    esp_err_t err = bmi270_read_reg(BMI270_REG_CHIP_ID, &chip_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID");
        return;
    }

    ESP_LOGI(TAG, "BMI270 CHIP_ID = 0x%02X (expected 0x%02X)",
             chip_id, BMI270_CHIP_ID);

    if (chip_id == BMI270_CHIP_ID) {
        ESP_LOGI(TAG, "✅ BMI270 detected successfully!");
    } else {
        ESP_LOGW(TAG, "❌ CHIP_ID mismatch!");
        ESP_LOGW(TAG, "Check:");
        ESP_LOGW(TAG, "- ADR wired to MISO?");
        ESP_LOGW(TAG, "- CS LOW at boot?");
        ESP_LOGW(TAG, "- SPI mode 3?");
        ESP_LOGW(TAG, "- Common GND?");
    }
}


// =====================================================
// TASK
// =====================================================
void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");

    while (1) {
        ESP_LOGI(TAG, "IMU running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// void imu_init(void)
// {
//     ESP_LOGI(TAG, "IMU init (stub)");
// }

// void imu_task(void *pvParameters)
// {
//     ESP_LOGI(TAG, "IMU task started");

//     while (1)
//     {
//         ESP_LOGI(TAG, "IMU running");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }