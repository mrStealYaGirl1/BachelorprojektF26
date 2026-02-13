#include <stdint.h>
#include <string.h>

#include "bmi270.h"
#include "bmi270_config.h"

/* ===================== Registers ===================== */

#define BMI270_REG_CHIP_ID        0x00
#define BMI270_REG_CMD            0x7E
#define BMI270_CMD_SOFTRESET      0xB6

#define BMI270_REG_PWR_CONF       0x7C
#define BMI270_REG_INIT_CTRL      0x59
#define BMI270_REG_INIT_ADDR_0    0x5B
#define BMI270_REG_INIT_DATA      0x5E
#define BMI270_REG_INTERNAL_STAT  0x21

#define BMI270_ACCEL_DATA_ADDR    0x0C
#define BMI270_GYRO_DATA_ADDR     0x12

#define BMI270_CHIP_ID_VALUE      0x24

#define BMI270_CONFIG_CHUNK_SIZE  16


/* ===================== Internal Functions ===================== */

static bmi270_status_t bmi270_soft_reset(bmi270_dev_t *dev)
{
    uint8_t cmd = BMI270_CMD_SOFTRESET;

    if (dev->write(BMI270_REG_CMD, &cmd, 1, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    dev->delay_us(2000, dev->intf_ptr);

    return BMI270_OK;
}


/* ===== Upload Bosch config file ===== */

static bmi270_status_t bmi270_upload_config(bmi270_dev_t *dev)
{
    uint16_t index = 0;
    uint8_t addr[2];
    uint8_t data;

    /* Disable advanced power save */
    data = 0x00;
    if (dev->write(BMI270_REG_PWR_CONF, &data, 1, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    dev->delay_us(450, dev->intf_ptr);

    /* Set INIT_CTRL = 0 */
    data = 0x00;
    if (dev->write(BMI270_REG_INIT_CTRL, &data, 1, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    /* Upload config in 16-byte chunks */
    while (index < bmi270_config_size)
    {
        uint16_t chunk = BMI270_CONFIG_CHUNK_SIZE;

        if (index + chunk > bmi270_config_size)
            chunk = bmi270_config_size - index;

        /* Set init address (index / 2 per Bosch requirement) */
        uint16_t addr_val = index / 2;

        addr[0] = addr_val & 0xFF;
        addr[1] = (addr_val >> 8) & 0xFF;

        if (dev->write(BMI270_REG_INIT_ADDR_0, addr, 2, dev->intf_ptr) != BMI270_OK)
            return BMI270_E_COMM;

        if (dev->write(BMI270_REG_INIT_DATA,
                       &bmi270_config_file[index],
                       chunk,
                       dev->intf_ptr) != BMI270_OK)
            return BMI270_E_COMM;

        index += chunk;
    }

    /* Set INIT_CTRL = 1 */
    data = 0x01;
    if (dev->write(BMI270_REG_INIT_CTRL, &data, 1, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    dev->delay_us(20000, dev->intf_ptr);

    /* Check init complete */
    if (dev->read(BMI270_REG_INTERNAL_STAT, &data, 1, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    if ((data & 0x01) == 0)
        return BMI270_E_INVALID;

    return BMI270_OK;
}


/* ===================== Public API ===================== */

bmi270_status_t bmi270_init(bmi270_dev_t *dev)
{
    uint8_t chip_id;

    if (dev == NULL)
        return BMI270_E_INVALID;

    if (bmi270_soft_reset(dev) != BMI270_OK)
        return BMI270_E_COMM;

    if (dev->read(BMI270_REG_CHIP_ID, &chip_id, 1, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    if (chip_id != BMI270_CHIP_ID_VALUE)
        return BMI270_E_INVALID;

    if (bmi270_upload_config(dev) != BMI270_OK)
        return BMI270_E_INVALID;

    return BMI270_OK;
}


/* ===== Enable accelerometer ===== */

bmi270_status_t bmi270_enable_accel(bmi270_dev_t *dev)
{
    uint8_t data = 0x04;  // Example: normal mode

    return dev->write(0x40, &data, 1, dev->intf_ptr);
}


/* ===== Enable gyroscope ===== */

bmi270_status_t bmi270_enable_gyro(bmi270_dev_t *dev)
{
    uint8_t data = 0x0E;  // Example: normal mode

    return dev->write(0x42, &data, 1, dev->intf_ptr);
}


/* ===== Read accelerometer ===== */

bmi270_status_t bmi270_read_accel(bmi270_dev_t *dev,
                                  int16_t *x,
                                  int16_t *y,
                                  int16_t *z)
{
    uint8_t buffer[6];

    if (dev->read(BMI270_ACCEL_DATA_ADDR, buffer, 6, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return BMI270_OK;
}


/* ===== Read gyroscope ===== */

bmi270_status_t bmi270_read_gyro(bmi270_dev_t *dev,
                                 int16_t *x,
                                 int16_t *y,
                                 int16_t *z)
{
    uint8_t buffer[6];

    if (dev->read(BMI270_GYRO_DATA_ADDR, buffer, 6, dev->intf_ptr) != BMI270_OK)
        return BMI270_E_COMM;

    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return BMI270_OK;
}