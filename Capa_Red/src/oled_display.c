#include "oled_display.h"
#include "shared.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

static const char *TAG_OLED = "OLED";

static esp_lcd_panel_io_handle_t oled_io_hdl    = NULL;
static esp_lcd_panel_handle_t    oled_panel_hdl = NULL;

// Framebuffer principal (se escribe normalmente)
static uint8_t oled_fb[OLED_H_RES * OLED_V_RES / 8];
// Framebuffer rotado 180 que se envia al panel
static uint8_t oled_fb_rot[OLED_H_RES * OLED_V_RES / 8];

// =========================================================================
//  Fuente 5x8
// =========================================================================

static const uint8_t font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},
};

static void oled_fb_char(uint8_t col, uint8_t row, char c)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *glyph = font5x8[c - 32];
    for (int i = 0; i < 5; i++) {
        int x = col + i;
        if (x >= OLED_H_RES) break;
        oled_fb[row * OLED_H_RES + x] = glyph[i];
    }
}

void oled_fb_string(uint8_t col, uint8_t row, const char *s)
{
    while (*s) {
        oled_fb_char(col, row, *s++);
        col += 6;
        if (col + 5 >= OLED_H_RES) break;
    }
}

void oled_fb_clear(void)
{
    memset(oled_fb, 0, sizeof(oled_fb));
}

// =========================================================================
//  Rotacion 180 por software + flush al panel
//
//  El SSD1306 organiza la VRAM en paginas de 8 filas (1 byte = 8 pixeles
//  verticales). Para rotar 180 hay que:
//    1. Invertir el orden de las paginas  (ultima pag -> primera)
//    2. Invertir el orden de las columnas (ultima col -> primera)
//    3. Invertir los bits del byte        (reflejo vertical de la pagina)
// =========================================================================

static uint8_t reverse_byte(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void oled_flush(void)
{
    const int pages = OLED_V_RES / 8;   // 64 / 8 = 8 paginas
    const int cols  = OLED_H_RES;       // 128 columnas

    for (int p = 0; p < pages; p++) {
        int p_src = (pages - 1) - p;    // pagina simetrica
        for (int c = 0; c < cols; c++) {
            int c_src = (cols - 1) - c; // columna simetrica
            oled_fb_rot[p * cols + c] =
                reverse_byte(oled_fb[p_src * cols + c_src]);
        }
    }

    esp_lcd_panel_draw_bitmap(oled_panel_hdl, 0, 0,
                              OLED_H_RES, OLED_V_RES, oled_fb_rot);
}

// =========================================================================
//  Hora NTP: aplica TZ Colombia y devuelve "HH:MM:SS" o "Sin hora"
// =========================================================================

static void get_time_string(char *out, size_t len)
{
    setenv("TZ", "COT5", 1);
    tzset();

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2024 - 1900)) {
        snprintf(out, len, "Sin hora");
        return;
    }
    strftime(out, len, "%H:%M:%S", &timeinfo);
}

// =========================================================================
//  Inicializacion del panel
// =========================================================================

void oled_init(i2c_master_bus_handle_t i2c_bus)
{
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr             = OLED_I2C_ADDR,
        .scl_speed_hz         = 100000,
        .control_phase_bytes  = 1,
        .dc_bit_offset        = 6,
        .lcd_cmd_bits         = 8,
        .lcd_param_bits       = 8,
        .flags = { .disable_control_phase = 0 },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cfg, &oled_io_hdl));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = GPIO_NUM_NC,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(oled_io_hdl, &panel_cfg, &oled_panel_hdl));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(oled_panel_hdl));
    ESP_ERROR_CHECK(esp_lcd_panel_init(oled_panel_hdl));
    // Sin mirror() en el panel: la rotacion se hace por software en oled_flush()
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(oled_panel_hdl, true));

    ESP_LOGI(TAG_OLED, "SSD1306 OK (rotacion 180 por software)");
}

// =========================================================================
//  Tarea FreeRTOS
// =========================================================================

void task_oled(void *arg)
{
    char line[22];
    char hora[12];

    static const char *alert_str[] = { "OK  ", "BPM!", "SpO2", "FALL" };
    static const char *fall_str[]  = {
        "Idle    ", "FreeFall", "ImpWait ",
        "Posture ", "CAIDA!! ", "Cooldown",
    };

    ESP_LOGI(TAG_OLED, "Tarea OLED iniciada");

    // ---------------------------------------------------------------
    //  Fase boot: esperar WiFi y NTP mostrando progreso en pantalla
    // ---------------------------------------------------------------
    while (!g_wifi_ready) {
        oled_fb_clear();
        oled_fb_string(0, 0, "SmartWireless");
        oled_fb_string(0, 2, "Conectando");
        oled_fb_string(0, 3, "WiFi...");
        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    oled_fb_clear();
    oled_fb_string(0, 0, "SmartWireless");
    oled_fb_string(0, 2, "WiFi OK!");
    oled_fb_string(0, 3, "Sync NTP...");
    oled_flush();

    while (!g_ntp_ready) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    oled_fb_clear();
    oled_fb_string(0, 0, "SmartWireless");
    oled_fb_string(0, 2, "NTP OK!");
    oled_fb_string(0, 3, "Iniciando...");
    oled_flush();
    vTaskDelay(pdMS_TO_TICKS(800));

    // ---------------------------------------------------------------
    //  Loop principal de monitoreo
    // ---------------------------------------------------------------
    while (true) {
        alert_level_t lv;
        xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
        lv = g_alert;
        xSemaphoreGive(g_alert_mutex);

        int          bpm  = g_bpm_display;
        int          spo2 = g_spo2_display;
        bool         fing = g_finger_oled;
        fall_state_t fst  = g_fall_state_display;

        // Linea 0: Hora NTP
        get_time_string(hora, sizeof(hora));
        snprintf(line, sizeof(line), "%s", hora);
        oled_fb_clear();
        oled_fb_string(0, 0, line);

        // Linea 1: BPM
        if (!fing)        snprintf(line, sizeof(line), "BPM: ---");
        else if (bpm > 0) snprintf(line, sizeof(line), "BPM: %3d bpm", bpm);
        else              snprintf(line, sizeof(line), "BPM: calcul...");
        oled_fb_string(0, 1, line);

        // Linea 2: SpO2
        if (!fing)         snprintf(line, sizeof(line), "SpO2: ---");
        else if (spo2 > 0) snprintf(line, sizeof(line), "SpO2: %2d%%", spo2);
        else               snprintf(line, sizeof(line), "SpO2: calcul...");
        oled_fb_string(0, 2, line);

        // Linea 3: Estado MPU6050
        snprintf(line, sizeof(line), "MPU: %s", fall_str[fst < 6 ? fst : 0]);
        oled_fb_string(0, 3, line);

        // Linea 4: Alerta + angulo servo
        int servo_ang = 0;
        switch (lv) {
            case VITAL_WARN_BPM:  servo_ang = 45;  break;
            case VITAL_WARN_SPO2: servo_ang = 90;  break;
            case VITAL_FALL:      servo_ang = 180; break;
            default:              servo_ang = 0;   break;
        }
        snprintf(line, sizeof(line), "Alrt:%s Srv:%3d", alert_str[lv], servo_ang);
        oled_fb_string(0, 4, line);

        // Linea 5: Presencia de dedo
        oled_fb_string(0, 5, fing ? "Dedo: presente " : "Dedo: ausente  ");

        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
