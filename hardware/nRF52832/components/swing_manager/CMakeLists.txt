idf_component_register(
    SRCS "swing_manager.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES imu ble_manager freertos log
)