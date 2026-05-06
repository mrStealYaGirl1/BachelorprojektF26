#include "spi_driver.h"
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define SPI_NODE DT_NODELABEL(spi1)

#define CS_PIN 29
#define CS_NODE DT_NODELABEL(gpio0)

static const struct device *spi_dev;
static const struct device *gpio_dev;

static struct spi_config spi_cfg = {
    .frequency = 125000,  // 👈 lavere hastighed
    // .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,  // 👈 fjern CPHA + CPOL
    .operation = SPI_WORD_SET(8) |
             SPI_TRANSFER_MSB |
             SPI_MODE_CPOL |
             SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,
};

int spi_driver_init(void)
{
    spi_dev = DEVICE_DT_GET(SPI_NODE);
    gpio_dev = DEVICE_DT_GET(CS_NODE);

    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -1;
    }

    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return -1;
    }

    // CS pin som output (HIGH = idle)
    gpio_pin_configure(gpio_dev, CS_PIN, GPIO_OUTPUT_HIGH);

    printk("SPI device ready!\n");
    return 0;
}

int spi_driver_transceive(uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    struct spi_buf tx = {
        .buf = tx_buf,
        .len = len
    };

    struct spi_buf rx = {
        .buf = rx_buf,
        .len = len
    };

    struct spi_buf_set tx_set = {
        .buffers = &tx,
        .count = 1
    };

    struct spi_buf_set rx_set = {
        .buffers = &rx,
        .count = 1
    };

    // CS LOW
    gpio_pin_set(gpio_dev, CS_PIN, 0);
    k_usleep(5);   // 👈 HER indsættes delay

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);

    // CS HIGH
    gpio_pin_set(gpio_dev, CS_PIN, 1);

    if (ret != 0) {
        printk("SPI ERROR: %d\n", ret);
    }

    return ret;
}