idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "."
    REQUIRES log freertos bmi270 esp_timer swing_manager
)