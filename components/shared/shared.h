#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>

// =========================================================================
//  Tipos compartidos entre modulos
// =========================================================================

typedef enum {
    VITAL_NORMAL = 0,
    VITAL_WARN_BPM,
    VITAL_WARN_SPO2,
    VITAL_FALL,
} alert_level_t;

typedef enum {
    STATE_IDLE,
    STATE_FREEFALL,
    STATE_IMPACT_WAIT,
    STATE_POSTURE_CHECK,
    STATE_FALL_CONFIRMED,
    STATE_COOLDOWN,
} fall_state_t;

// =========================================================================
//  Variables globales compartidas entre tareas FreeRTOS
// =========================================================================

extern volatile alert_level_t  g_alert;
extern SemaphoreHandle_t       g_alert_mutex;

extern volatile int            g_bpm_display;
extern volatile int            g_spo2_display;
extern volatile bool           g_finger_oled;
extern volatile fall_state_t   g_fall_state_display;
