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

static uint8_t oled_fb[OLED_H_RES * OLED_V_RES / 8];
static uint8_t oled_fb_rot[OLED_H_RES * OLED_V_RES / 8];

// =========================================================================
//  Fuente 5x8 — 95 entradas, ASCII 32 (' ') a 126 ('~')
//  Cada fila de comentario indica el rango ASCII cubierto
// =========================================================================
static const uint8_t font5x8[][5] = {
    /* 32  ' '  */ {0x00,0x00,0x00,0x00,0x00},
    /* 33  '!'  */ {0x00,0x00,0x5F,0x00,0x00},
    /* 34  '"'  */ {0x00,0x07,0x00,0x07,0x00},
    /* 35  '#'  */ {0x14,0x7F,0x14,0x7F,0x14},
    /* 36  '$'  */ {0x24,0x2A,0x7F,0x2A,0x12},
    /* 37  '%'  */ {0x23,0x13,0x08,0x64,0x62},
    /* 38  '&'  */ {0x36,0x49,0x55,0x22,0x50},
    /* 39  '\'' */ {0x00,0x05,0x03,0x00,0x00},
    /* 40  '('  */ {0x00,0x1C,0x22,0x41,0x00},
    /* 41  ')'  */ {0x00,0x41,0x22,0x1C,0x00},
    /* 42  '*'  */ {0x14,0x08,0x3E,0x08,0x14},
    /* 43  '+'  */ {0x08,0x08,0x3E,0x08,0x08},
    /* 44  ','  */ {0x00,0x50,0x30,0x00,0x00},
    /* 45  '-'  */ {0x08,0x08,0x08,0x08,0x08},
    /* 46  '.'  */ {0x00,0x60,0x60,0x00,0x00},
    /* 47  '/'  */ {0x20,0x10,0x08,0x04,0x02},
    /* 48  '0'  */ {0x3E,0x51,0x49,0x45,0x3E},
    /* 49  '1'  */ {0x00,0x42,0x7F,0x40,0x00},
    /* 50  '2'  */ {0x42,0x61,0x51,0x49,0x46},
    /* 51  '3'  */ {0x21,0x41,0x45,0x4B,0x31},
    /* 52  '4'  */ {0x18,0x14,0x12,0x7F,0x10},
    /* 53  '5'  */ {0x27,0x45,0x45,0x45,0x39},
    /* 54  '6'  */ {0x3C,0x4A,0x49,0x49,0x30},
    /* 55  '7'  */ {0x01,0x71,0x09,0x05,0x03},
    /* 56  '8'  */ {0x36,0x49,0x49,0x49,0x36},
    /* 57  '9'  */ {0x06,0x49,0x49,0x29,0x1E},
    /* 58  ':'  */ {0x00,0x36,0x36,0x00,0x00},
    /* 59  ';'  */ {0x00,0x56,0x36,0x00,0x00},
    /* 60  '<'  */ {0x08,0x14,0x22,0x41,0x00},
    /* 61  '='  */ {0x14,0x14,0x14,0x14,0x14},
    /* 62  '>'  */ {0x00,0x41,0x22,0x14,0x08},
    /* 63  '?'  */ {0x02,0x01,0x51,0x09,0x06},
    /* 64  '@'  */ {0x32,0x49,0x79,0x41,0x3E},
    /* 65  'A'  */ {0x7E,0x11,0x11,0x11,0x7E},
    /* 66  'B'  */ {0x7F,0x49,0x49,0x49,0x36},
    /* 67  'C'  */ {0x3E,0x41,0x41,0x41,0x22},
    /* 68  'D'  */ {0x7F,0x41,0x41,0x22,0x1C},
    /* 69  'E'  */ {0x7F,0x49,0x49,0x49,0x41},
    /* 70  'F'  */ {0x7F,0x09,0x09,0x09,0x01},
    /* 71  'G'  */ {0x3E,0x41,0x49,0x49,0x7A},
    /* 72  'H'  */ {0x7F,0x08,0x08,0x08,0x7F},
    /* 73  'I'  */ {0x00,0x41,0x7F,0x41,0x00},
    /* 74  'J'  */ {0x20,0x40,0x41,0x3F,0x01},
    /* 75  'K'  */ {0x7F,0x08,0x14,0x22,0x41},
    /* 76  'L'  */ {0x7F,0x40,0x40,0x40,0x40},
    /* 77  'M'  */ {0x7F,0x02,0x0C,0x02,0x7F},
    /* 78  'N'  */ {0x7F,0x04,0x08,0x10,0x7F},
    /* 79  'O'  */ {0x3E,0x41,0x41,0x41,0x3E},
    /* 80  'P'  */ {0x7F,0x09,0x09,0x09,0x06},
    /* 81  'Q'  */ {0x3E,0x41,0x51,0x21,0x5E},
    /* 82  'R'  */ {0x7F,0x09,0x19,0x29,0x46},
    /* 83  'S'  */ {0x46,0x49,0x49,0x49,0x31},
    /* 84  'T'  */ {0x01,0x01,0x7F,0x01,0x01},
    /* 85  'U'  */ {0x3F,0x40,0x40,0x40,0x3F},
    /* 86  'V'  */ {0x1F,0x20,0x40,0x20,0x1F},
    /* 87  'W'  */ {0x3F,0x40,0x38,0x40,0x3F},
    /* 88  'X'  */ {0x63,0x14,0x08,0x14,0x63},
    /* 89  'Y'  */ {0x07,0x08,0x70,0x08,0x07},
    /* 90  'Z'  */ {0x61,0x51,0x49,0x45,0x43},
    /* 91  '['  */ {0x00,0x7F,0x41,0x41,0x00},
    /* 92  '\\' */ {0x02,0x04,0x08,0x10,0x20},
    /* 93  ']'  */ {0x00,0x41,0x41,0x7F,0x00},
    /* 94  '^'  */ {0x04,0x02,0x01,0x02,0x04},
    /* 95  '_'  */ {0x40,0x40,0x40,0x40,0x40},
    /* 96  '`'  */ {0x00,0x01,0x02,0x04,0x00},
    /* 97  'a'  */ {0x20,0x54,0x54,0x54,0x78},
    /* 98  'b'  */ {0x7F,0x48,0x44,0x44,0x38},
    /* 99  'c'  */ {0x38,0x44,0x44,0x44,0x20},
    /* 100 'd'  */ {0x38,0x44,0x44,0x48,0x7F},  // circulo izq + palo dcho completo
    /* 101 'e'  */ {0x38,0x54,0x54,0x54,0x18},
    /* 102 'f'  */ {0x08,0x7E,0x09,0x01,0x02},
    /* 103 'g'  */ {0x0C,0x52,0x52,0x52,0x3E},
    /* 104 'h'  */ {0x7F,0x08,0x04,0x04,0x78},
    /* 105 'i'  */ {0x00,0x44,0x7D,0x40,0x00},
    /* 106 'j'  */ {0x20,0x40,0x44,0x3D,0x00},
    /* 107 'k'  */ {0x7F,0x10,0x28,0x44,0x00},
    /* 108 'l'  */ {0x00,0x41,0x7F,0x40,0x00},
    /* 109 'm'  */ {0x7C,0x04,0x18,0x04,0x78},
    /* 110 'n'  */ {0x7C,0x08,0x04,0x04,0x78},
    /* 111 'o'  */ {0x38,0x44,0x44,0x44,0x38},
    /* 112 'p'  */ {0x7C,0x14,0x14,0x14,0x08},
    /* 113 'q'  */ {0x08,0x14,0x14,0x18,0x7C},
    /* 114 'r'  */ {0x7C,0x08,0x04,0x04,0x08},
    /* 115 's'  */ {0x48,0x54,0x54,0x54,0x20},
    /* 116 't'  */ {0x04,0x3F,0x44,0x40,0x20},
    /* 117 'u'  */ {0x3C,0x40,0x40,0x20,0x7C},
    /* 118 'v'  */ {0x1C,0x20,0x40,0x20,0x1C},
    /* 119 'w'  */ {0x3C,0x40,0x30,0x40,0x3C},
    /* 120 'x'  */ {0x44,0x28,0x10,0x28,0x44},
    /* 121 'y'  */ {0x0C,0x50,0x50,0x50,0x3C},
    /* 122 'z'  */ {0x44,0x64,0x54,0x4C,0x44},
    /* 123 '{'  */ {0x00,0x08,0x36,0x41,0x00},
    /* 124 '|'  */ {0x00,0x00,0x7F,0x00,0x00},
    /* 125 '}'  */ {0x00,0x41,0x36,0x08,0x00},
    /* 126 '~'  */ {0x10,0x08,0x08,0x10,0x08},
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
    const int pages = OLED_V_RES / 8;
    const int cols  = OLED_H_RES;

    for (int p = 0; p < pages; p++) {
        int p_src = (pages - 1) - p;
        for (int c = 0; c < cols; c++) {
            int c_src = (cols - 1) - c;
            oled_fb_rot[p * cols + c] =
                reverse_byte(oled_fb[p_src * cols + c_src]);
        }
    }

    esp_lcd_panel_draw_bitmap(oled_panel_hdl, 0, 0,
                              OLED_H_RES, OLED_V_RES, oled_fb_rot);
}

// =========================================================================
//  Hora NTP
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
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(oled_panel_hdl, true));

    ESP_LOGI(TAG_OLED, "SSD1306 OK (font5x8 completa, 95 entradas)");
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

    while (true) {
        alert_level_t lv;
        xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
        lv = g_alert;
        xSemaphoreGive(g_alert_mutex);

        int          bpm  = g_bpm_display;
        int          spo2 = g_spo2_display;
        bool         fing = g_finger_oled;
        fall_state_t fst  = g_fall_state_display;

        get_time_string(hora, sizeof(hora));
        snprintf(line, sizeof(line), "%s", hora);
        oled_fb_clear();
        oled_fb_string(0, 0, line);

        if (!fing)        snprintf(line, sizeof(line), "BPM: ---");
        else if (bpm > 0) snprintf(line, sizeof(line), "BPM: %3d bpm", bpm);
        else              snprintf(line, sizeof(line), "BPM: calcul...");
        oled_fb_string(0, 1, line);

        if (!fing)         snprintf(line, sizeof(line), "SpO2: ---");
        else if (spo2 > 0) snprintf(line, sizeof(line), "SpO2: %2d%%", spo2);
        else               snprintf(line, sizeof(line), "SpO2: calcul...");
        oled_fb_string(0, 2, line);

        snprintf(line, sizeof(line), "MPU: %s", fall_str[fst < 6 ? fst : 0]);
        oled_fb_string(0, 3, line);

        int servo_ang = 0;
        switch (lv) {
            case VITAL_WARN_BPM:  servo_ang = 45;  break;
            case VITAL_WARN_SPO2: servo_ang = 90;  break;
            case VITAL_FALL:      servo_ang = 180; break;
            default:              servo_ang = 0;   break;
        }
        snprintf(line, sizeof(line), "Alrt:%s Srv:%3d", alert_str[lv], servo_ang);
        oled_fb_string(0, 4, line);

        oled_fb_string(0, 5, fing ? "Dedo: presente " : "Dedo: ausente  ");

        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
