#ifndef BMI270_H
#define BMI270_H

#include <stdint.h>
#include "esp_err.h"   // hvis du bruger ESP-IDF

typedef struct
{
    void *intf_ptr;

    esp_err_t (*read)(uint8_t reg, uint8_t *data, uint16_t len, void *intf_ptr);
    esp_err_t (*write)(uint8_t reg, const uint8_t *data, uint16_t len, void *intf_ptr);
    void (*delay_us)(uint32_t period, void *intf_ptr);

} bmi270_dev_t;


/* Public API */
esp_err_t bmi270_init(bmi270_dev_t *dev);
esp_err_t bmi270_enable_accel(bmi270_dev_t *dev);
esp_err_t bmi270_enable_gyro(bmi270_dev_t *dev);
esp_err_t bmi270_read_accel(bmi270_dev_t *dev, int16_t *x, int16_t *y, int16_t *z);
esp_err_t bmi270_read_gyro(bmi270_dev_t *dev, int16_t *x, int16_t *y, int16_t *z);

#endif