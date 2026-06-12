#include "shared.h"

// =========================================================================
//  Definiciones de variables globales compartidas
// =========================================================================

volatile alert_level_t  g_alert              = VITAL_NORMAL;
SemaphoreHandle_t       g_alert_mutex        = NULL;

volatile int            g_bpm_display        = 0;
volatile int            g_spo2_display       = 0;
volatile bool           g_finger_oled        = false;
volatile fall_state_t   g_fall_state_display = STATE_IDLE;

// Flags de arranque
volatile bool           g_wifi_ready         = false;
volatile bool           g_ntp_ready          = false;
