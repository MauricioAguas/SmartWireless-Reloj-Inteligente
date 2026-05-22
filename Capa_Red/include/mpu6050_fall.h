#pragma once

#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

// =========================================================================
//  API publica
// =========================================================================

/**
 * Crea e inicializa el driver MPU6050 sobre el bus I2C dado.
 * Realiza calibracion automatica de offsets.
 * @param out_mpu  [out] handle del driver inicializado.
 */
esp_err_t mpu6050_fall_init(i2c_master_bus_handle_t i2c_bus,
                             mpu6050_handle_t       *out_mpu);

/**
 * Tarea FreeRTOS: ejecuta la maquina de estados de deteccion de caidas.
 * arg debe ser el mpu6050_handle_t obtenido de mpu6050_fall_init().
 * Prioridad recomendada: 6.
 */
void task_mpu6050(void *arg);
