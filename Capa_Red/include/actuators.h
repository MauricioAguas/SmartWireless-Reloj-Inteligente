#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

// =========================================================================
//  Pines y parametros de actuadores
// =========================================================================

#define RGB_LED_PIN         GPIO_NUM_48
#define BUZZER_PIN          GPIO_NUM_7
#define SERVO_PIN           GPIO_NUM_10

// LEDC - buzzer pasivo
#define BUZZER_CHANNEL      LEDC_CHANNEL_0
#define BUZZER_TIMER        LEDC_TIMER_0
#define BUZZER_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define BUZZER_RESOLUTION   LEDC_TIMER_10_BIT
#define BUZZER_FREQ_ALERT   3000
#define BUZZER_FREQ_JOLT    1500
#define BUZZER_FREQ_WARN    2000

// LEDC - servo SG90
#define SERVO_CHANNEL       LEDC_CHANNEL_1
#define SERVO_TIMER         LEDC_TIMER_1
#define SERVO_SPEED_MODE    LEDC_LOW_SPEED_MODE
#define SERVO_RESOLUTION    LEDC_TIMER_14_BIT
#define SERVO_FREQ_HZ       50
#define SERVO_DUTY_MIN      410
#define SERVO_DUTY_MAX      2048

// =========================================================================
//  API publica
// =========================================================================

/** Inicializa buzzer (LEDC), servo (LEDC) y LED RGB (RMT). */
void actuators_init(void);

/** Actualiza LED y servo segun el nivel de alerta global g_alert. */
void actuators_update(void);

/** Secuencia de alerta de caida: servo 180, LED rojo parpadeante. */
void actuators_fall_alert(void);

/** Pitido y LED naranja breve ante movimiento brusco. */
void actuators_jolt(void);

/** Doble pitido de advertencia vital (BPM o SpO2 fuera de rango). */
void beep_vital_warn(void);

// Primitivas expuestas para uso en otros modulos
void buzzer_on(uint32_t freq_hz);
void buzzer_off(void);
void rgb_set(uint8_t r, uint8_t g, uint8_t b);
void rgb_off(void);
void servo_set_angle(int angle_deg);
