#include "spi_drv.h"
#include "esp_log.h"

static const char *TAG = "SPI_DRV";

esp_err_t spi_drv_bus_init(const spi_bus_cfg_t *cfg)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = cfg->mosi_io,
        .miso_io_num = cfg->miso_io,
        .sclk_io_num = cfg->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = cfg->max_transfer_sz,
    };

    esp_err_t err = spi_bus_initialize(cfg->host, &buscfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized (ok)");
        return ESP_OK;
    }
    return err;
}

esp_err_t spi_drv_add_device(const spi_bus_cfg_t *bus,
                             const spi_dev_cfg_t *dev,
                             spi_device_handle_t *out_handle)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = dev->clock_hz,
        .mode = dev->mode,
        .spics_io_num = dev->cs_io,
        .queue_size = dev->queue_size,
    };

    return spi_bus_add_device(bus->host, &devcfg, out_handle);
}

esp_err_t spi_drv_transmit(spi_device_handle_t dev,
                           const void *tx, void *rx,
                           int n_bytes)
{
    spi_transaction_t t = {0};
    t.length = n_bytes * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_transmit(dev, &t);
}
