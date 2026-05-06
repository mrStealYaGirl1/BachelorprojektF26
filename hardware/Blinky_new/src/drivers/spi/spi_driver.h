#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>
#include <zephyr/drivers/spi.h>

int spi_driver_init(void);

int spi_driver_transceive(uint8_t *tx_buf, uint8_t *rx_buf, size_t len);

#endif