#include "max30102.h"
#include "shared.h"
#include "actuators.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>
#include <stdbool.h>

static const char *TAG_MAX = "MAX30102";

// =========================================================================
//  Registros y constantes del sensor
// =========================================================================

#define MAX30102_ADDR    0x57
#define REG_FIFO_WR_PTR  0x04
#define REG_OVF_COUNTER  0x05
#define REG_FIFO_RD_PTR  0x06
#define REG_FIFO_DATA    0x07
#define REG_FIFO_CONFIG  0x08
#define REG_MODE_CONFIG  0x09
#define REG_SPO2_CONFIG  0x0A
#define REG_LED1_PA      0x0C
#define REG_LED2_PA      0x0D
#define REG_PART_ID      0xFF

#define SAMPLE_RATE      13
#define WINDOW_SIZE      130
#define FINGER_THRESH    50000
#define STABLE_SAMPLES   104
#define DISCARD_ON_PLACE 26
#define PI_VALID_MAX     0.18f
#define PI_VALID_MIN     0.003f
#define MAX_INVALID      6
#define MAX_PEAKS        30
#define MIN_PEAK_DIST    3
#define MAX_PEAK_DIST    20
#define PEAK_THRESH_PCT  0.35f
#define CROSS_MARGIN     0.10f
#define MIN_VALID_PEAKS  2
#define IQR_FENCE        1.5f
#define SPO2_R_MIN       0.40f
#define SPO2_R_MAX       2.0f
#define EMA_BPM          0.25f
#define EMA_SPO2         0.20f
#define DBG_LEVEL        1
#define BPM_LOW          50
#define BPM_HIGH         120
#define SPO2_LOW         93

typedef enum {
    REJ_NONE = 0, REJ_POCAS_MUESTRAS, REJ_DC_BAJO, REJ_AMPLITUD_BAJA,
    REJ_PI_FUERA_RANGO, REJ_POCOS_PICOS, REJ_POCOS_VALIDOS,
    REJ_POCOS_IQR, REJ_BPM_FUERA_RANGO, REJ_SPO2_R_FUERA, REJ_SPO2_BAJO,
} reject_t;

static const char *rej_str[] = {
    "ninguno","pocas muestras","DC muy bajo","amplitud<100",
    "PI fuera de rango","pocos picos","intervalos invalidos",
    "IQR elimino todo","BPM fuera de 30-220","R fuera de rango","SpO2<80",
};

static i2c_master_dev_handle_t dev_hdl = NULL;

static uint32_t ir_buf[WINDOW_SIZE];
static uint32_t red_buf[WINDOW_SIZE];
static int  buf_idx       = 0;
static int  buf_count     = 0;
static int  clean_samples = 0;
static int  discard_count = 0;
static bool finger_present = false;

// =========================================================================
//  Helpers I2C
// =========================================================================

static esp_err_t max_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev_hdl, buf, 2, pdMS_TO_TICKS(200));
}

static esp_err_t max_read_reg(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev_hdl, &reg, 1, out, len, pdMS_TO_TICKS(200));
}

static esp_err_t max30102_init_sensor(void)
{
    uint8_t part_id = 0;
    esp_err_t ret = max_read_reg(REG_PART_ID, &part_id, 1);
    if (ret != ESP_OK || part_id != 0x15) {
        ESP_LOGE(TAG_MAX, "Part ID error: 0x%02X (esperado 0x15)", part_id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG_MAX, "Part ID OK: 0x%02X", part_id);
    max_write_reg(REG_MODE_CONFIG, 0x40);
    uint8_t mode = 0x40; int t = 20;
    while ((mode & 0x40) && t--) {
        vTaskDelay(pdMS_TO_TICKS(10));
        max_read_reg(REG_MODE_CONFIG, &mode, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    max_write_reg(REG_FIFO_CONFIG,  0x50);
    max_write_reg(REG_MODE_CONFIG,  0x03);
    max_write_reg(REG_SPO2_CONFIG,  0x43);
    max_write_reg(REG_LED1_PA,      0x7F);
    max_write_reg(REG_LED2_PA,      0x7F);
    max_write_reg(REG_FIFO_WR_PTR,  0x00);
    max_write_reg(REG_OVF_COUNTER,  0x00);
    max_write_reg(REG_FIFO_RD_PTR,  0x00);
    ESP_LOGI(TAG_MAX, "MAX30102 listo");
    return ESP_OK;
}

esp_err_t max30102_init_device(i2c_master_bus_handle_t i2c_bus)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MAX30102_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_hdl));
    return max30102_init_sensor();
}

static esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t raw[6];
    esp_err_t ret = max_read_reg(REG_FIFO_DATA, raw, 6);
    if (ret != ESP_OK) return ret;
    *red = ((uint32_t)(raw[0] & 0x03) << 16) | ((uint32_t)raw[1] << 8) | raw[2];
    *ir  = ((uint32_t)(raw[3] & 0x03) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
    return ESP_OK;
}

// =========================================================================
//  DSP helpers
// =========================================================================

static void linearize_buffer(uint32_t *ir_out, uint32_t *red_out, int count)
{
    int start = (count < WINDOW_SIZE) ? 0 : buf_idx;
    for (int i = 0; i < count; i++) {
        int idx = (start + i) % WINDOW_SIZE;
        ir_out[i]  = ir_buf[idx];
        red_out[i] = red_buf[idx];
    }
}

static void median3_filter(uint32_t *in, float *out, int count)
{
    out[0] = (float)in[0];
    if (count < 2) return;
    out[count - 1] = (float)in[count - 1];
    for (int i = 1; i < count - 1; i++) {
        float a = (float)in[i-1], b = (float)in[i], c = (float)in[i+1], t;
        if (a > b) { t = a; a = b; b = t; }
        if (b > c) { t = b; b = c; c = t; }
        if (a > b) { t = a; a = b; b = t; }
        out[i] = b;
    }
}

static void sort_float(float *arr, int n)
{
    for (int i = 1; i < n; i++) {
        float key = arr[i]; int j = i - 1;
        while (j >= 0 && arr[j] > key) { arr[j+1] = arr[j]; j--; }
        arr[j+1] = key;
    }
}

static void hard_reset(float *bpms, float *spo2s, int *inv)
{
    buf_count = buf_idx = clean_samples = 0;
    discard_count = DISCARD_ON_PLACE;
    finger_present = false;
    *bpms = *spo2s = 0.0f; *inv = 0;
    g_bpm_display = 0; g_spo2_display = 0; g_finger_oled = false;
    for (int i = 0; i < WINDOW_SIZE; i++) ir_buf[i] = red_buf[i] = 0;
}

static void soft_reset(float *bpms, float *spo2s, int *inv)
{
    buf_count = buf_idx = clean_samples = 0;
    *bpms = *spo2s = 0.0f; *inv = 0;
    g_bpm_display = 0; g_spo2_display = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) ir_buf[i] = red_buf[i] = 0;
}

static int calc_bpm(uint32_t *ir, int count, float *pi_out,
                    int *dbg_peaks, int *dbg_valid, int *dbg_used,
                    float *dbg_meaniv, float *dbg_dc, float *dbg_amp,
                    reject_t *reason)
{
    *pi_out = 0; *dbg_peaks = *dbg_valid = *dbg_used = 0;
    *dbg_meaniv = *dbg_dc = *dbg_amp = 0; *reason = REJ_NONE;

    if (count < STABLE_SAMPLES) { *reason = REJ_POCAS_MUESTRAS; return 0; }

    static float irf[WINDOW_SIZE];
    median3_filter(ir, irf, count);

    float dc = 0;
    for (int i = 0; i < count; i++) dc += irf[i];
    dc /= count; *dbg_dc = dc;
    if (dc < 1.0f) { *reason = REJ_DC_BAJO; return 0; }

    float acmin = 1e9f, acmax = -1e9f;
    for (int i = 0; i < count; i++) {
        float v = irf[i] - dc;
        if (v < acmin) acmin = v;
        if (v > acmax) acmax = v;
    }
    float amplitude = acmax - acmin; *dbg_amp = amplitude;
    if (amplitude < 100.f) { *reason = REJ_AMPLITUD_BAJA; return 0; }

    float pi = amplitude / dc; *pi_out = pi;
    if (pi < PI_VALID_MIN || pi > PI_VALID_MAX) { *reason = REJ_PI_FUERA_RANGO; return 0; }

    float threshold  = acmin + amplitude * PEAK_THRESH_PCT;
    float crosslevel = -amplitude * CROSS_MARGIN;
    static int peak_idx[MAX_PEAKS];
    int npeaks = 0; bool above = false;
    float peak_val = -1e9f; int peak_pos = -1;

    for (int i = 0; i < count; i++) {
        float v = irf[i] - dc;
        if (!above && v > threshold)        { above = true; peak_val = v; peak_pos = i; }
        else if (above && v > peak_val)      { peak_val = v; peak_pos = i; }
        else if (above && v < crosslevel) {
            int last = npeaks > 0 ? peak_idx[npeaks-1] : -MIN_PEAK_DIST-1;
            if (peak_pos - last >= MIN_PEAK_DIST && npeaks < MAX_PEAKS)
                peak_idx[npeaks++] = peak_pos;
            above = false; peak_val = -1e9f;
        }
    }
    *dbg_peaks = npeaks;
    if (npeaks < MIN_VALID_PEAKS + 1) { *reason = REJ_POCOS_PICOS; return 0; }

    int nintervals = npeaks - 1;
    static float intervals[MAX_PEAKS];
    for (int i = 0; i < nintervals; i++) {
        int dist = peak_idx[i+1] - peak_idx[i];
        intervals[i] = (dist >= MIN_PEAK_DIST && dist <= MAX_PEAK_DIST)
                       ? (float)dist / SAMPLE_RATE : -1.f;
    }
    static float validivs[MAX_PEAKS], sortedivs[MAX_PEAKS];
    int nvalid = 0;
    for (int i = 0; i < nintervals; i++)
        if (intervals[i] > 0) validivs[nvalid++] = intervals[i];
    *dbg_valid = nvalid;
    if (nvalid < MIN_VALID_PEAKS) { *reason = REJ_POCOS_VALIDOS; return 0; }

    for (int i = 0; i < nvalid; i++) sortedivs[i] = validivs[i];
    sort_float(sortedivs, nvalid);
    float q1  = sortedivs[nvalid/4], q3 = sortedivs[nvalid*3/4];
    float iqr = q3 - q1, lo = q1 - IQR_FENCE*iqr, hi = q3 + IQR_FENCE*iqr;
    float sum = 0; int used = 0;
    for (int i = 0; i < nvalid; i++)
        if (validivs[i] >= lo && validivs[i] <= hi) { sum += validivs[i]; used++; }
    *dbg_used = used;
    if (used < MIN_VALID_PEAKS) { *reason = REJ_POCOS_IQR; return 0; }

    float meaniv = sum / used; *dbg_meaniv = meaniv;
    int bpm = (int)roundf(60.f / meaniv);
    if (bpm < 30 || bpm > 220) { *reason = REJ_BPM_FUERA_RANGO; return 0; }
    return bpm;
}

static int calc_spo2(uint32_t *red, uint32_t *ir, int count, float pi,
                     float *dbg_R, reject_t *reason)
{
    *dbg_R = 0; *reason = REJ_NONE;
    if (count < STABLE_SAMPLES || pi < PI_VALID_MIN || pi > PI_VALID_MAX) {
        *reason = REJ_PI_FUERA_RANGO; return 0;
    }
    static float redf[WINDOW_SIZE], irf[WINDOW_SIZE];
    median3_filter(red, redf, count);
    median3_filter(ir,  irf,  count);
    float reddc = 0, irdc = 0;
    for (int i = 0; i < count; i++) { reddc += redf[i]; irdc += irf[i]; }
    reddc /= count; irdc /= count;
    if (reddc < 1.f || irdc < 1.f) { *reason = REJ_DC_BAJO; return 0; }
    float rr = 0, ss = 0;
    for (int i = 0; i < count; i++) {
        float r = redf[i] - reddc, s = irf[i] - irdc;
        rr += r*r; ss += s*s;
    }
    float redrms = sqrtf(rr/count), irrms = sqrtf(ss/count);
    if (irrms < 1.f || redrms < 1.f) { *reason = REJ_AMPLITUD_BAJA; return 0; }
    float R = (redrms/reddc) / (irrms/irdc); *dbg_R = R;
    if (R < SPO2_R_MIN || R > SPO2_R_MAX) { *reason = REJ_SPO2_R_FUERA; return 0; }
    float spo2 = -45.060f*R*R + 30.354f*R + 94.845f;
    if (spo2 > 100.f) spo2 = 100.f;
    if (spo2 < 80.f)  { *reason = REJ_SPO2_BAJO; return 0; }
    return (int)roundf(spo2);
}

// =========================================================================
//  Tarea FreeRTOS
// =========================================================================

void task_max30102(void *arg)
{
    float bpm_smooth = 0.f, spo2_smooth = 0.f;
    int   inv_count  = 0;
    TickType_t last_report = xTaskGetTickCount();
    uint32_t ir_lin[WINDOW_SIZE], red_lin[WINDOW_SIZE];

    while (true) {
        xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
        bool falling = (g_alert == VITAL_FALL);
        xSemaphoreGive(g_alert_mutex);
        if (falling) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }

        uint8_t wr = 0, rd = 0, ovf = 0;
        max_read_reg(REG_FIFO_WR_PTR, &wr,  1);
        max_read_reg(REG_FIFO_RD_PTR, &rd,  1);
        max_read_reg(REG_OVF_COUNTER, &ovf, 1);
        if (ovf) max_write_reg(REG_OVF_COUNTER, 0x00);

        uint8_t ns = (wr - rd) & 0x1F;
        for (uint8_t i = 0; i < ns; i++) {
            uint32_t red = 0, ir = 0;
            if (max30102_read_fifo(&red, &ir) != ESP_OK) continue;

            if (ir < FINGER_THRESH) {
                if (finger_present) {
                    ESP_LOGI(TAG_MAX, "Dedo retirado - reiniciando...");
                    hard_reset(&bpm_smooth, &spo2_smooth, &inv_count);
                    xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
                    if (g_alert == VITAL_WARN_BPM || g_alert == VITAL_WARN_SPO2)
                        g_alert = VITAL_NORMAL;
                    xSemaphoreGive(g_alert_mutex);
                    actuators_update();
                }
                continue;
            }
            if (!finger_present) {
                hard_reset(&bpm_smooth, &spo2_smooth, &inv_count);
                finger_present = true;
                g_finger_oled  = true;
                ESP_LOGI(TAG_MAX, "Dedo detectado - descartando %d muestras...", DISCARD_ON_PLACE);
            }
            if (discard_count > 0) { discard_count--; continue; }
            ir_buf[buf_idx]  = ir;
            red_buf[buf_idx] = red;
            buf_idx = (buf_idx + 1) % WINDOW_SIZE;
            if (buf_count  < WINDOW_SIZE) buf_count++;
            if (clean_samples < WINDOW_SIZE) clean_samples++;
        }

        if ((xTaskGetTickCount() - last_report) < pdMS_TO_TICKS(2000)) {
            vTaskDelay(pdMS_TO_TICKS(20)); continue;
        }
        last_report = xTaskGetTickCount();

        if (!finger_present || buf_count == 0) {
            ESP_LOGI(TAG_MAX, "Pon el dedo sobre el sensor...");
            vTaskDelay(pdMS_TO_TICKS(20)); continue;
        }
        if (clean_samples < STABLE_SAMPLES) {
            ESP_LOGI(TAG_MAX, "Estabilizando... %d/%d", clean_samples, STABLE_SAMPLES);
            vTaskDelay(pdMS_TO_TICKS(20)); continue;
        }

        linearize_buffer(ir_lin, red_lin, buf_count);
        float pi = 0, dbg_dc = 0, dbg_amp = 0, dbg_meaniv = 0, dbg_R = 0;
        int dbg_peaks = 0, dbg_valid = 0, dbg_used = 0;
        reject_t rej_bpm = REJ_NONE, rej_spo2 = REJ_NONE;
        int bpm  = calc_bpm(ir_lin, buf_count, &pi,
                            &dbg_peaks, &dbg_valid, &dbg_used,
                            &dbg_meaniv, &dbg_dc, &dbg_amp, &rej_bpm);
        int spo2 = calc_spo2(red_lin, ir_lin, buf_count, pi, &dbg_R, &rej_spo2);

#if DBG_LEVEL >= 1
        if (!bpm)  ESP_LOGW(TAG_MAX, "BPM rechazado: %s | picos=%d valid=%d pi=%.4f amp=%.0f",
                            rej_str[rej_bpm], dbg_peaks, dbg_valid, pi, dbg_amp);
        if (!spo2 && rej_spo2 != REJ_PI_FUERA_RANGO)
            ESP_LOGW(TAG_MAX, "SpO2 rechazado: %s | R=%.4f", rej_str[rej_spo2], dbg_R);
#endif

        alert_level_t new_alert = VITAL_NORMAL;
        if (bpm > 0) {
            inv_count = 0;
            bpm_smooth  = (bpm_smooth  == 0.f) ? (float)bpm  : EMA_BPM  * (float)bpm  + (1.f - EMA_BPM)  * bpm_smooth;
            if (spo2 > 0)
                spo2_smooth = (spo2_smooth == 0.f) ? (float)spo2 : EMA_SPO2 * (float)spo2 + (1.f - EMA_SPO2) * spo2_smooth;

            int bpm_r  = (int)roundf(bpm_smooth);
            int spo2_r = (spo2_smooth > 0.f) ? (int)roundf(spo2_smooth) : 0;
            g_bpm_display  = bpm_r;
            g_spo2_display = spo2_r;

            if (spo2_r > 0 && spo2_r < SPO2_LOW)          new_alert = VITAL_WARN_SPO2;
            else if (bpm_r < BPM_LOW || bpm_r > BPM_HIGH)  new_alert = VITAL_WARN_BPM;

            if (spo2_r > 0)
                ESP_LOGI(TAG_MAX, "BPM: %3d  SpO2: %2d%%  PI: %.4f", bpm_r, spo2_r, pi);
            else
                ESP_LOGI(TAG_MAX, "BPM: %3d  SpO2: ---   PI: %.4f", bpm_r, pi);
        } else {
            inv_count++;
            if (bpm_smooth > 0.f && inv_count < MAX_INVALID) {
                ESP_LOGI(TAG_MAX, "BPM: %3d  [senal debil %d/%d]",
                         (int)roundf(bpm_smooth), inv_count, MAX_INVALID);
            } else {
                soft_reset(&bpm_smooth, &spo2_smooth, &inv_count);
                finger_present = true; g_finger_oled = true;
                ESP_LOGW(TAG_MAX, "Recalibrando - manten el dedo quieto %ds",
                         STABLE_SAMPLES / SAMPLE_RATE);
            }
        }

        xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
        if (g_alert != VITAL_FALL) {
            bool changed = (g_alert != new_alert);
            g_alert = new_alert;
            xSemaphoreGive(g_alert_mutex);
            if (changed) {
                actuators_update();
                if (new_alert == VITAL_WARN_BPM || new_alert == VITAL_WARN_SPO2)
                    beep_vital_warn();
            }
        } else {
            xSemaphoreGive(g_alert_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
