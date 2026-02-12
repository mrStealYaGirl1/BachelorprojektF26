#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include "driver/spi_master.h"      // SPI master driver - ESP-IDF provides an API to configure and use the SPI bus in master mode.


static const char *TAG = "IMU";

// ====== PINS ======
#define PIN_MISO  37     //SDA/PICO
#define PIN_SCLK  36    //SCL/SCK
#define PIN_MOSI  35    //ADR/POCI
#define PIN_CS    45   //CS/SS
// ===============================

#define BMI270_REG_CHIP_ID  0x00
#define BMI270_CHIP_ID      0x24


static spi_device_handle_t s_bmi_spi = NULL;

static esp_err_t bmi270_spi_setup(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };
    
    // BMI270 supports SPI mode 0 and 3. Start with mode 0.
    spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 100 * 1000,   // 100 kHz
    .mode = 3,                      // start with mode 0 again
    .spics_io_num = -1, //PIN_CS,
    .queue_size = 1,
    .cs_ena_pretrans = 2,           // keep CS low a bit before clock
    .cs_ena_posttrans = 2,          // keep CS low a bit after
};

    // // BMI270 supports SPI mode 0 and 3. Start with mode 0.
    // spi_device_interface_config_t devcfg = {
    //     .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz safe start
    //     .mode = 3,                   // SPI mode 
    //     .spics_io_num = PIN_CS, 
    //     .queue_size = 1,
    // };

    // Use SPI2_HOST (general purpose SPI host)
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return spi_bus_add_device(SPI2_HOST, &devcfg, &s_bmi_spi);
}

// // BMI270 SPI: first byte is addr with bit7 = 1 for read.
// // Then one dummy byte to clock out the data.
// static esp_err_t bmi270_read_reg(uint8_t reg, uint8_t *val)
// {
//     uint8_t tx[2] = { (uint8_t)(0x80 | (reg & 0x7F)), 0x00 };
//     uint8_t rx[2] = {0};

//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = 8 * sizeof(tx);
//     t.tx_buffer = tx;
//     t.rx_buffer = rx;

//     esp_err_t err = spi_device_transmit(s_bmi_spi, &t);
//     if (err != ESP_OK) return err;

//     *val = rx[1]; // rx[0] is dummy, rx[1] is the register data
//     return ESP_OK;
// }
static esp_err_t bmi270_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = { (uint8_t)(0x80 | (reg & 0x7F)), 0x00, 0x00 };
    uint8_t rx[3] = {0};

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(s_bmi_spi, &t);
    if (err != ESP_OK) return err;

    // Try both positions for debugging:
    ESP_LOGI(TAG, "RX bytes: %02X %02X %02X", rx[0], rx[1], rx[2]);

    // Often data ends up in last byte when there is an extra dummy
    *val = rx[2];
    return ESP_OK;
}



void imu_init(void)
{
    ESP_LOGI(TAG, "IMU init: setting up BMI270 SPI...");

    ESP_ERROR_CHECK(bmi270_spi_setup());

    // give sensor time after power-up
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t chip_id = 0x00;
    esp_err_t err = bmi270_read_reg(BMI270_REG_CHIP_ID, &chip_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "BMI270 CHIP_ID = 0x%02X (expected 0x%02X)", chip_id, BMI270_CHIP_ID);

    if (chip_id != BMI270_CHIP_ID) {
        ESP_LOGW(TAG, "CHIP_ID mismatch. Check: ADR jumper open (SPI), MOSI/MISO swapped, mode=3, CS wiring.");
    }
}

void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");

    while (1) {
        // For now just keep the task alive; next step we’ll read data registers.
        ESP_LOGI(TAG, "IMU running");
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