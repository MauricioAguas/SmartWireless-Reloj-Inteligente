idf_component_register(
    SRCS "max30102.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_common shared actuators
)
