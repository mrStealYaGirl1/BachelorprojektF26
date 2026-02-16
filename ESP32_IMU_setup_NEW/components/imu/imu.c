#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"

#include "bmi2.h"
#include "bmi270.h"

static const char *TAG = "IMU";

/* ===== SPI PINS ===== */
#define PIN_MOSI  35
#define PIN_MISO  37
#define PIN_SCLK  36
#define PIN_CS    10

static spi_device_handle_t s_spi;
static struct bmi2_dev s_bmi;

/* =====================================================
   SPI WRITE (Bosch expected signature)
===================================================== */

static int8_t spi_write(uint8_t reg_addr,
                        const uint8_t *data,
                        uint32_t len,
                        void *intf_ptr)
{
    spi_device_handle_t dev = (spi_device_handle_t)intf_ptr;

    uint8_t tx[1 + len];
    tx[0] = reg_addr & 0x7F;   // Write bit = 0
    memcpy(&tx[1], data, len);

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;

    return (spi_device_transmit(dev, &t) == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

/* =====================================================
   SPI READ
===================================================== */

static int8_t spi_read(uint8_t reg_addr,
                       uint8_t *data,
                       uint32_t len,
                       void *intf_ptr)
{
    spi_device_handle_t dev = (spi_device_handle_t)intf_ptr;

    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    tx[0] = reg_addr | 0x80;
    memset(&tx[1], 0, len);

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (spi_device_transmit(dev, &t) != ESP_OK)
        return BMI2_E_COM_FAIL;

    memcpy(data, &rx[1], len);

    return BMI2_OK;
}

/* =====================================================
   DELAY
===================================================== */

static void spi_delay(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

/* =====================================================
   SPI SETUP
===================================================== */

static void spi_init_bus(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz for init
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));
}

/* =====================================================
   IMU INIT
===================================================== */

void imu_init(void)
{
    ESP_LOGI(TAG, "Initializing BMI270...");

    spi_init_bus();
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Fill Bosch struct */
    s_bmi.read = spi_read;
    s_bmi.write = spi_write;
    s_bmi.delay_us = spi_delay;
    s_bmi.intf = BMI2_SPI_INTF;
    s_bmi.intf_ptr = s_spi;
    s_bmi.read_write_len = 32;  // recommended
    s_bmi.config_file_ptr = NULL;
    s_bmi.intf = BMI2_SPI_INTF;

    int8_t rslt = bmi270_init(&s_bmi);

    if (rslt != BMI2_OK)
    {
        ESP_LOGE(TAG, "BMI270 init failed: %d", rslt);
        return;
    }

    ESP_LOGI(TAG, "BMI270 initialized");

    /* Enable accel + gyro */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, &s_bmi);

    if (rslt != BMI2_OK)
        ESP_LOGE(TAG, "Sensor enable failed");
}

/* =====================================================
   TASK
===================================================== */

void imu_task(void *pvParameters)
{
    struct bmi2_sens_data sensor_data;

    while (1)
    {
        int8_t rslt = bmi2_get_sensor_data(&sensor_data, &s_bmi);

        if (rslt == BMI2_OK)
        {
            ESP_LOGI(TAG,
                     "ACC: %d %d %d | GYRO: %d %d %d",
                     sensor_data.acc.x,
                     sensor_data.acc.y,
                     sensor_data.acc.z,
                     sensor_data.gyr.x,
                     sensor_data.gyr.y,
                     sensor_data.gyr.z);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// #include "imu.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "bmi270.h"
// #include "esp_rom_sys.h"

// static const char *TAG = "IMU";

// /* ===== Pins ===== */
// #define PIN_MOSI  35
// #define PIN_SCLK  36
// #define PIN_MISO  37
// #define PIN_CS    10

// static spi_device_handle_t s_bmi_spi = NULL;
// static bmi270_dev_t s_bmi;

// /* =====================================================
//    SPI LOW-LEVEL FUNCTIONS FOR BMI270 DRIVER
// ===================================================== */

// static bmi270_status_t spi_write(uint8_t reg,
//                         const uint8_t *data,
//                         uint16_t len,
//                         void *intf_ptr)
// {
//     spi_device_handle_t dev = (spi_device_handle_t)intf_ptr;

//     uint8_t tx[1 + len];
//     tx[0] = reg & 0x7F;   // write
//     memcpy(&tx[1], data, len);

//     spi_transaction_t t = {0};
//     t.length = (1 + len) * 8;
//     t.tx_buffer = tx;

//     if (spi_device_transmit(dev, &t) != ESP_OK)
//         return -1;

//     return 0;
// }

// static bmi270_status_t spi_read(uint8_t reg_addr,
//                                 uint8_t *data,
//                                 uint16_t len,
//                                 void *intf_ptr)
// {
//     spi_device_handle_t dev = (spi_device_handle_t)intf_ptr;

//     uint8_t tx[2 + len];
//     uint8_t rx[2 + len];

//     tx[0] = reg_addr | 0x80;  // READ
//     tx[1] = 0x00;             // Dummy
//     memset(&tx[2], 0, len);

//     spi_transaction_t t = {0};
//     t.length = (2 + len) * 8;
//     t.tx_buffer = tx;
//     t.rx_buffer = rx;

//     if (spi_device_transmit(dev, &t) != ESP_OK)
//         return -1;

//     memcpy(data, &rx[2], len);

//     return 0;
// }

// static void spi_delay(uint32_t us, void *intf_ptr)
// {
//     (void)intf_ptr;
//     esp_rom_delay_us(us);
// }

// /* =====================================================
//    SPI SETUP
// ===================================================== */

// static esp_err_t bmi270_spi_setup(void)
// {
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = PIN_MOSI,
//         .miso_io_num = PIN_MISO,
//         .sclk_io_num = PIN_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//     };

//     spi_device_interface_config_t devcfg = {
//         .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz (sikkert under init)
//         .mode = 0,
//         .spics_io_num = PIN_CS,
//         .queue_size = 1,
//         .cs_ena_pretrans = 10,
//         .cs_ena_posttrans = 10,
//     };

//     ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
//     ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_bmi_spi));

//     return ESP_OK;
// }

// /* =====================================================
//    IMU INIT
// ===================================================== */

// void imu_init(void)
// {
//     ESP_LOGI(TAG, "Initializing BMI270...");

//     ESP_ERROR_CHECK(bmi270_spi_setup());
//     vTaskDelay(pdMS_TO_TICKS(100));

//     /* RAW test read */
//     uint8_t id = 0xAA;
//     spi_read(0x00, &id, 1, s_bmi_spi);
//     ESP_LOGI(TAG, "RAW read chip_id = 0x%02X", id);

//     /* Link driver */
//     s_bmi.read = spi_read;
//     s_bmi.write = spi_write;
//     s_bmi.delay_us = spi_delay;
//     s_bmi.intf_ptr = s_bmi_spi;   // 🔥 MEGET vigtigt

//     esp_err_t err = bmi270_init(&s_bmi);

//     if (err != ESP_OK)
//     {
//         ESP_LOGE(TAG, "BMI270 init failed!");
//         return;
//     }

//     ESP_LOGI(TAG, "BMI270 initialized successfully!");

//     bmi270_enable_accel(&s_bmi);
//     bmi270_enable_gyro(&s_bmi);
// }

// /* =====================================================
//    TASK
// ===================================================== */

// void imu_task(void *pvParameters)
// {
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;

//     while (1)
//     {
//         if (bmi270_read_accel(&s_bmi, &ax, &ay, &az) == BMI270_OK &&
//             bmi270_read_gyro(&s_bmi, &gx, &gy, &gz) == BMI270_OK)
//         {
//             ESP_LOGI(TAG,
//                      "ACC: %d %d %d | GYRO: %d %d %d",
//                      ax, ay, az,
//                      gx, gy, gz);
//         }

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }

// #include "imu.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "bmi270.h"
// #include "esp_rom_sys.h"   // <-- ADD THIS

// static const char *TAG = "IMU";

// /* ===== Pins ===== */
// #define PIN_MOSI  35
// #define PIN_SCLK  36
// #define PIN_MISO  37
// #define PIN_CS    10

// static spi_device_handle_t s_bmi_spi = NULL;
// static bmi270_dev_t s_bmi;

// /* =====================================================
//    SPI LOW-LEVEL FUNCTIONS FOR BMI270 DRIVER
// ===================================================== */

// static esp_err_t spi_write(uint8_t reg,
//                            const uint8_t *data,
//                            uint16_t len,
//                            void *intf_ptr)
// {
//     uint8_t tx[1 + len];
//     tx[0] = reg & 0x7F;   // write
//     memcpy(&tx[1], data, len);

//     spi_transaction_t t = {0};
//     t.length = (1 + len) * 8;
//     t.tx_buffer = tx;
//     t.rxlength = 0;


//     return spi_device_transmit(s_bmi_spi, &t);
// }

// static esp_err_t spi_read(uint8_t reg,
//                           uint8_t *data,
//                           uint16_t len,
//                           void *intf_ptr)
// {
//     (void)intf_ptr;

//     // +2: 1 byte addr + 1 byte dummy + len data
//     uint8_t tx[2 + len];
//     uint8_t rx[2 + len];

//     tx[0] = reg | 0x80;   // read
//     tx[1] = 0x00;         // dummy byte (VIGTIG!)
//     memset(&tx[2], 0, len);

//     spi_transaction_t t = {0};
//     t.length = (2 + len) * 8;
//     t.tx_buffer = tx;
//     t.rx_buffer = rx;

//     esp_err_t err = spi_device_transmit(s_bmi_spi, &t);
//     if (err != ESP_OK) return err;

//     memcpy(data, &rx[2], len); // data starter efter addr + dummy
//     return ESP_OK;
// }


// static void spi_delay(uint32_t us, void *intf_ptr)
// {
//     esp_rom_delay_us(us);
// }

// /* =====================================================
//    SPI SETUP
// ===================================================== */

// static esp_err_t bmi270_spi_setup(void)
// {
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = PIN_MOSI,
//         .miso_io_num = PIN_MISO,
//         .sclk_io_num = PIN_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//     };

//     spi_device_interface_config_t devcfg = {
//         .clock_speed_hz = 200 * 1000, //changed temporarily to 200 kHz - before "1 * 1000 * 1000,"
//         .mode = 0,
//         .spics_io_num = PIN_CS,
//         .queue_size = 1,

//         // VIGTIGT: giv CS lidt tid før/efter
//         .cs_ena_pretrans = 10,
//         .cs_ena_posttrans = 10,

//     };

//     ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
//     ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_bmi_spi));

//     return ESP_OK;
// }

// /* =====================================================
//    IMU INIT
// ===================================================== */

// void imu_init(void)
// {
//     ESP_LOGI(TAG, "Initializing BMI270...");

//     // // 🔴 CRITICAL: Force CS LOW before SPI init
//     // gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
//     // gpio_set_level(PIN_CS, 0);
//     // vTaskDelay(pdMS_TO_TICKS(10));
//     // gpio_set_level(PIN_CS, 1);
//     // vTaskDelay(pdMS_TO_TICKS(1));

//     ESP_ERROR_CHECK(bmi270_spi_setup());

//     vTaskDelay(pdMS_TO_TICKS(100));

//     uint8_t id = 0xAA;
//     esp_err_t e = spi_read(0x00, &id, 1, NULL);
//     ESP_LOGI(TAG, "RAW read chip_id err=%d id=0x%02X", (int)e, id);

//     s_bmi.read = spi_read;
//     s_bmi.write = spi_write;
//     s_bmi.delay_us = spi_delay;
//     s_bmi.intf_ptr = NULL;

//     esp_err_t err = bmi270_init(&s_bmi);

//     if (err != ESP_OK)
//     {
//         ESP_LOGE(TAG, "BMI270 init failed!");
//         return;
//     }

//     ESP_LOGI(TAG, "BMI270 initialized successfully!");

//     bmi270_enable_accel(&s_bmi);
//     bmi270_enable_gyro(&s_bmi);
// }

// /* =====================================================
//    TASK
// ===================================================== */

// void imu_task(void *pvParameters)
// {
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;

//     while (1)
//     {
//         if (bmi270_read_accel(&s_bmi, &ax, &ay, &az) == BMI270_OK &&
//             bmi270_read_gyro(&s_bmi, &gx, &gy, &gz) == BMI270_OK)
//         {
//             ESP_LOGI(TAG,
//                      "ACC: %d %d %d | GYRO: %d %d %d",
//                      ax, ay, az,
//                      gx, gy, gz);
//         }

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }




// /* Kathrines suggestion for imu.c
// // imu.c: 
// #include "imu.h"
// #include "esp_log.h"
// #include "freertos/task.h"
// #include "bmi270.h"

// static const char *TAG = "IMU";
// static bmi270_t s_bmi;

// // =====================================================
// // IMU INIT
// // =====================================================
// void imu_init(void)
// {
//     ESP_LOGI(TAG, "IMU init...");
//     esp_err_t err = bmi270_init(&s_bmi);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "BMI270 init failed: %s", esp_err_to_name(err));
//     }
// }
// // =====================================================
// // TASK
// // =====================================================
// void imu_task(void *pvParameters)
// {
//     (void)pvParameters;
//     ESP_LOGI(TAG, "IMU task started");

//     while (1) {
//         ESP_LOGI(TAG, "IMU running...");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
// */