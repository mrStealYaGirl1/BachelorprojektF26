idf_component_register(
    SRCS "bmi270.c" "bmi270_config.c"
    INCLUDE_DIRS "."
    REQUIRES driver log freertos
)
