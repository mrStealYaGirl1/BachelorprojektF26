idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_spi esp_driver_gpio log freertos
)
