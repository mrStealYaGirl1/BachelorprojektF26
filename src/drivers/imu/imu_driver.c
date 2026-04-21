// imu_driver.c - BMI270 IMU driver

#include "imu_driver.h"
#include "spi/spi_driver.h"

#include "bmi270/bmi2.h"
#include "bmi270/bmi270.h"

#include "imu_processing.h"
#include "ble/ble_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* =====================================================
   BMI270 REGISTERS
===================================================== */

#define BMI270_CHIP_ID_REG   0x00
#define BMI270_ACC_START     0x0C

static struct bmi2_dev s_bmi;

static int8_t bmi_spi_write(uint8_t reg_addr,
                        const uint8_t *data,
                        uint32_t len,
                        void *intf_ptr)
{
    uint8_t tx[1 + len];

    tx[0] = reg_addr & 0x7F;
    memcpy(&tx[1], data, len);

    spi_driver_transceive(tx, NULL, 1 + len);

    return BMI2_OK;
}

static int8_t bmi_spi_read(uint8_t reg_addr,
                       uint8_t *data,
                       uint32_t len,
                       void *intf_ptr)
{
    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    tx[0] = reg_addr | 0x80;
    memset(&tx[1], 0, len);

    spi_driver_transceive(tx, rx, 1 + len);

    memcpy(data, &rx[1], len);

    return BMI2_OK;
}

static void spi_delay(uint32_t period_us, void *intf_ptr)
{
    k_usleep(period_us);
}


/* =====================================================
   RINGBUFFER
===================================================== */

static imu_ringbuffer_t imu_rb;

void imu_ringbuffer_init(void)
{
    imu_rb.write_index = 0;
    imu_rb.wrapped = 0;
}

void imu_ringbuffer_push(const imu_sample_t *sample)
{
    imu_rb.buffer[imu_rb.write_index] = *sample;

    imu_rb.write_index++;

    if (imu_rb.write_index >= IMU_BUFFER_SIZE)
    {
        imu_rb.write_index = 0;
        imu_rb.wrapped = 1;
    }
}

imu_ringbuffer_t* imu_get_ringbuffer(void)
{
    return &imu_rb;
}

/* =====================================================
   LOW LEVEL SPI
===================================================== */

// static void bmi270_read(uint8_t reg, uint8_t *data, uint8_t len)
// {
//     uint8_t tx[1 + len];
//     uint8_t rx[1 + len];

//     tx[0] = reg | 0x80;
//     memset(&tx[1], 0, len);

//     spi_driver_transceive(tx, rx, 1 + len);

//     memcpy(data, &rx[1], len);
// }

/* =====================================================
   CHIP ID
===================================================== */

static uint8_t imu_read_chip_id(void)
{
    uint8_t tx[3];
    uint8_t rx[3];

    /* 1) Dummy read to switch interface to SPI */
    tx[0] = 0x80 | BMI270_CHIP_ID_REG;
    tx[1] = 0x00;
    tx[2] = 0x00;
    memset(rx, 0, sizeof(rx));
    spi_driver_transceive(tx, rx, 3);

    k_usleep(100);

    /* 2) Real read of CHIP_ID */
    memset(rx, 0, sizeof(rx));
    spi_driver_transceive(tx, rx, 3);

    printk("CHIP_ID raw bytes: rx[0]=0x%02X rx[1]=0x%02X rx[2]=0x%02X\n",
           rx[0], rx[1], rx[2]);

    return rx[2];
}

/* =====================================================
   INIT
===================================================== */

// int imu_init(void)
// {
//     uint8_t chip_id;

//     // 🔥 INIT SPI FIRST
//     if (spi_driver_init() != 0)
//     {
//         printk("SPI init failed\n");
//         return -1;
//     }

//     printk("SPI ready\n");

//     // Setup BMI interface
//     s_bmi.read = bmi_spi_read;
//     s_bmi.write = bmi_spi_write;
//     s_bmi.delay_us = spi_delay;
//     s_bmi.intf = BMI2_SPI_INTF;
//     s_bmi.intf_ptr = NULL;

//     printk("Starting BMI270 init...\n");

//     printk("Before bmi270_init\n");

//     uint8_t raw_chip_id = imu_read_chip_id();
//     printk("Raw CHIP_ID before init: 0x%02X\n", raw_chip_id);
//     k_msleep(10);

//     int8_t rslt = bmi270_init(&s_bmi);
//     printk("bmi270_init rslt = %d\n", rslt);

//     if (rslt != BMI2_OK)
//     {
//         printk("BMI270 init failed\n");
//         return -1;
//     }

//     // if (bmi270_init(&s_bmi) != BMI2_OK)
//     // {
//     //     printk("BMI270 init failed\n");
//     //     return -1;
//     // }

//     printk("After bmi270_init\n");

//     bmi2_get_regs(0x00, &chip_id, 1, &s_bmi);
//     printk("CHIP_ID: 0x%02X\n", chip_id);

//     uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
//     bmi2_sensor_enable(sens_list, 2, &s_bmi);

//     printk("BMI270 ready\n");

//     return 0;
// }

int imu_init(void)
{
    uint8_t chip_id;

    // 🔥 INIT SPI FIRST
    if (spi_driver_init() != 0)
    {
        printk("SPI init failed\n");
        return -1;
    }

    printk("SPI ready\n");

    k_msleep(10);

    // Setup BMI interface
    s_bmi.read = bmi_spi_read;
    s_bmi.write = bmi_spi_write;
    s_bmi.delay_us = spi_delay;
    s_bmi.intf = BMI2_SPI_INTF;
    s_bmi.intf_ptr = NULL;

    printk("Starting BMI270 init...\n");

    uint8_t raw_chip_id = imu_read_chip_id();
    printk("Raw CHIP_ID before init: 0x%02X\n", raw_chip_id);

    if (raw_chip_id != 0x24) {
        printk("BMI270 chip-id check failed before init\n");
        return -1;
    }

    k_msleep(10);

    int8_t rslt = bmi270_init(&s_bmi);
    printk("bmi270_init rslt = %d\n", rslt);

    if (rslt != BMI2_OK)
    {
        printk("BMI270 init failed\n");
        return -1;
    }

    bmi2_get_regs(0x00, &chip_id, 1, &s_bmi);
    printk("CHIP_ID: 0x%02X\n", chip_id);

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, &s_bmi);
    printk("bmi2_sensor_enable rslt = %d\n", rslt);

    if (rslt != BMI2_OK) {
        printk("Sensor enable failed\n");
        return -1;
    }

    printk("BMI270 ready\n");
    return 0;
}

/* =====================================================
   READ ACC + GYRO (RAW)
===================================================== */

static void imu_read_raw(imu_sample_t *sample)
{
    struct bmi2_sens_data sensor_data;

    if (bmi2_get_sensor_data(&sensor_data, &s_bmi) == BMI2_OK)
    {
        sample->ax = sensor_data.acc.x;
        sample->ay = sensor_data.acc.y;
        sample->az = sensor_data.acc.z;

        sample->gx = sensor_data.gyr.x;
        sample->gy = sensor_data.gyr.y;
        sample->gz = sensor_data.gyr.z;

        sample->timestamp_ms = k_uptime_get();
    }
}

void imu_get_latest(imu_sample_t *sample)
{
    imu_read_raw(sample);
}

/* =====================================================
   THREAD (200 Hz)
===================================================== */

void imu_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    imu_sample_t sample;

    while (1)
    {
        imu_read_raw(&sample);

        imu_ringbuffer_push(&sample);

        uint32_t idx = (imu_rb.write_index == 0)
                     ? (IMU_BUFFER_SIZE - 1)
                     : (imu_rb.write_index - 1);

        imu_process_sample(&sample, idx);

        ble_send_imu_sample(&sample);

        static int print_counter = 0;

        if (++print_counter >= 100)
        {
            print_counter = 0;

            printk("ACC: %d %d %d | GYRO: %d %d %d\n",
                sample.ax, sample.ay, sample.az,
                sample.gx, sample.gy, sample.gz);
        }

        k_msleep(5);
    }
}