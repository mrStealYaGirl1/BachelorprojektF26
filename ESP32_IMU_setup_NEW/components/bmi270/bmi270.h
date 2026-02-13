#ifndef BMI270_H
#define BMI270_H

#include <stdint.h>

/* ===== Return codes ===== */
typedef enum {
    BMI270_OK = 0,
    BMI270_E_COMM = -1,
    BMI270_E_INVALID = -2
} bmi270_status_t;

/* ===== Device structure ===== */
typedef struct
{
    void *intf_ptr;

    bmi270_status_t (*read)(uint8_t reg,
                            uint8_t *data,
                            uint16_t len,
                            void *intf_ptr);

    bmi270_status_t (*write)(uint8_t reg,
                             const uint8_t *data,
                             uint16_t len,
                             void *intf_ptr);

    void (*delay_us)(uint32_t period, void *intf_ptr);

} bmi270_dev_t;

/* ===== Public API ===== */

bmi270_status_t bmi270_init(bmi270_dev_t *dev);
bmi270_status_t bmi270_enable_accel(bmi270_dev_t *dev);
bmi270_status_t bmi270_enable_gyro(bmi270_dev_t *dev);
bmi270_status_t bmi270_read_accel(bmi270_dev_t *dev,
                                  int16_t *x,
                                  int16_t *y,
                                  int16_t *z);
bmi270_status_t bmi270_read_gyro(bmi270_dev_t *dev,
                                 int16_t *x,
                                 int16_t *y,
                                 int16_t *z);

#endif