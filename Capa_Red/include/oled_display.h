#pragma once

#include "driver/i2c_master.h"
#include <stdint.h>

// =========================================================================
//  Parametros del panel SSD1306 0.96"
// =========================================================================

#define OLED_I2C_ADDR   0x3C
#define OLED_H_RES      128
#define OLED_V_RES      64

// =========================================================================
//  API publica
// =========================================================================

/** Inicializa el panel SSD1306 sobre el bus I2C dado. */
void oled_init(i2c_master_bus_handle_t i2c_bus);

/** Borra el framebuffer en memoria. */
void oled_fb_clear(void);

/** Escribe una cadena en el framebuffer (col en pixeles, row en paginas 0-7). */
void oled_fb_string(uint8_t col, uint8_t row, const char *s);

/** Envia el framebuffer al panel fisico. */
void oled_flush(void);

/** Tarea FreeRTOS: refresca la pantalla cada 500 ms con estado vital. */
void task_oled(void *arg);
