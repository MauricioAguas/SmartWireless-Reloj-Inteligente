idf_component_register(
    SRCS "actuators.c"
    INCLUDE_DIRS "."
    REQUIRES driver led_strip freertos shared
)
