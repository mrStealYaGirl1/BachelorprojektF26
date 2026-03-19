#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"

typedef struct {
    spi_host_device_t host;
    int mosi_io;
    int miso_io;
    int sclk_io;
    int max_transfer_sz;
} spi_bus_cfg_t;

typedef struct {
    int cs_io;
    int clock_hz;
    int mode;          // 0..3
    int queue_size;
} spi_dev_cfg_t;

esp_err_t spi_drv_bus_init(const spi_bus_cfg_t *cfg);
esp_err_t spi_drv_add_device(const spi_bus_cfg_t *bus,
                             const spi_dev_cfg_t *dev,
                             spi_device_handle_t *out_handle);

esp_err_t spi_drv_transmit(spi_device_handle_t dev,
                           const void *tx, void *rx,
                           int n_bytes);
