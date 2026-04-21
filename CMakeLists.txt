cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_project)

target_sources(app PRIVATE
    src/main.c
    src/drivers/spi/spi_driver.c
    src/drivers/imu/imu_driver.c
    src/drivers/imu/imu_processing.c
    src/drivers/bmi270/bmi270.c
    src/drivers/bmi270/bmi2.c
    src/drivers/ble/ble_driver.c
)

zephyr_include_directories(src/drivers) 
zephyr_include_directories(src/drivers/bmi270)