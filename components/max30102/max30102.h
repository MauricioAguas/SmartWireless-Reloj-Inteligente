#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

// =========================================================================
//  API publica
// =========================================================================

/**
 * Registra el MAX30102 en el bus I2C y lo configura.
 * Llamar una vez desde app_main antes de lanzar task_max30102.
 */
esp_err_t max30102_init_device(i2c_master_bus_handle_t i2c_bus);

/**
 * Tarea FreeRTOS: lee el FIFO del sensor, calcula BPM y SpO2
 * y actualiza las variables globales de shared.h.
 * Prioridad recomendada: 5.
 */
void task_max30102(void *arg);
