idf_component_register(
    SRCS "mpu6050_fall.c"
    INCLUDE_DIRS "."
    REQUIRES driver mpu6050 freertos shared actuators
)
