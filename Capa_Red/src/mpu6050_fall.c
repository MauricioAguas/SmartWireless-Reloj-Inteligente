#include "mpu6050_fall.h"
#include "shared.h"
#include "actuators.h"

#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG_MPU = "MPU6050";

#define CALIB_SAMPLES       200
#define FREEFALL_THRESH_G   0.8f
#define FREEFALL_TIME_MS    30
#define IMPACT_THRESH_G     1.6f
#define IMPACT_WINDOW_MS    1000
#define POSTURE_THRESH_G    1.2f
#define POSTURE_WINDOW_MS   1500
#define JOLT_THRESH_G       2.2f
#define COOLDOWN_MS         5000
#define TASK_PERIOD_MS      10
#define EMA_ALPHA           0.4f

static float offset_ax = 0, offset_ay = 0, offset_az = 0;
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;

static void calibrate_mpu(mpu6050_handle_t mpu)
{
    float sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
    float temp = 0;
    mpu6050_gyro_data_axes_t gyro;
    mpu6050_accel_data_axes_t accel;

    ESP_LOGI(TAG_MPU, "Calibrando MPU6050 (%d muestras) - no mover...", CALIB_SAMPLES);

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        esp_err_t ret = mpu6050_get_motion(mpu, &gyro, &accel, &temp);
        if (ret == ESP_OK) {
            sax += accel.x_axis;
            say += accel.y_axis;
            saz += accel.z_axis;
            sgx += gyro.x_axis;
            sgy += gyro.y_axis;
            sgz += gyro.z_axis;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    offset_ax = sax / CALIB_SAMPLES;
    offset_ay = say / CALIB_SAMPLES;
    offset_az = (saz / CALIB_SAMPLES) - 1.0f;
    offset_gx = sgx / CALIB_SAMPLES;
    offset_gy = sgy / CALIB_SAMPLES;
    offset_gz = sgz / CALIB_SAMPLES;

    ESP_LOGI(TAG_MPU, "Offsets - ax:%.3f ay:%.3f az:%.3f | gx:%.3f gy:%.3f gz:%.3f",
             offset_ax, offset_ay, offset_az, offset_gx, offset_gy, offset_gz);
}

esp_err_t mpu6050_fall_init(i2c_master_bus_handle_t i2c_bus, mpu6050_handle_t *out_mpu)
{
    if (!out_mpu) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_config_t dev_cfg = I2C_MPU6050_CONFIG_DEFAULT;
    mpu6050_handle_t mpu = NULL;

    esp_err_t ret = mpu6050_init(i2c_bus, &dev_cfg, &mpu);
    if (ret != ESP_OK || mpu == NULL) {
        ESP_LOGE(TAG_MPU, "Error inicializando driver MPU6050: %s", esp_err_to_name(ret));
        return ret != ESP_OK ? ret : ESP_FAIL;
    }

    calibrate_mpu(mpu);
    *out_mpu = mpu;
    return ESP_OK;
}

void task_mpu6050(void *arg)
{
    mpu6050_handle_t mpu = (mpu6050_handle_t)arg;
    fall_state_t state = STATE_IDLE;
    TickType_t state_ts = xTaskGetTickCount();
    float ema_g = 1.0f;

    mpu6050_gyro_data_axes_t gyro;
    mpu6050_accel_data_axes_t accel;
    float temp = 0;

    ESP_LOGI(TAG_MPU, "Tarea MPU6050 iniciada");

    while (true) {
        esp_err_t ret = mpu6050_get_motion(mpu, &gyro, &accel, &temp);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
            continue;
        }

        float ax = accel.x_axis - offset_ax;
        float ay = accel.y_axis - offset_ay;
        float az = accel.z_axis - offset_az;

        float g_mag = sqrtf(ax * ax + ay * ay + az * az);
        ema_g = EMA_ALPHA * g_mag + (1.f - EMA_ALPHA) * ema_g;

        uint32_t dt_ms = (uint32_t)((xTaskGetTickCount() - state_ts) * portTICK_PERIOD_MS);

        if (state == STATE_IDLE && ema_g > JOLT_THRESH_G) {
            ESP_LOGI(TAG_MPU, "Jolt detectado: %.2f g", ema_g);
            actuators_jolt();
        }

        switch (state) {
        case STATE_IDLE:
            if (ema_g < FREEFALL_THRESH_G) {
                state = STATE_FREEFALL;
                state_ts = xTaskGetTickCount();
                ESP_LOGI(TAG_MPU, "Caida libre: %.2f g", ema_g);
            }
            break;

        case STATE_FREEFALL:
            if (ema_g >= FREEFALL_THRESH_G) {
                state = STATE_IDLE;
            } else if (dt_ms >= FREEFALL_TIME_MS) {
                state = STATE_IMPACT_WAIT;
                state_ts = xTaskGetTickCount();
                ESP_LOGI(TAG_MPU, "Libre confirmada, esperando impacto...");
            }
            break;

        case STATE_IMPACT_WAIT:
            if (ema_g > IMPACT_THRESH_G) {
                state = STATE_POSTURE_CHECK;
                state_ts = xTaskGetTickCount();
                ESP_LOGI(TAG_MPU, "Impacto detectado: %.2f g", ema_g);
            } else if (dt_ms > IMPACT_WINDOW_MS) {
                state = STATE_IDLE;
            }
            break;

        case STATE_POSTURE_CHECK:
            if (ema_g < POSTURE_THRESH_G && dt_ms >= POSTURE_WINDOW_MS) {
                state = STATE_FALL_CONFIRMED;
                state_ts = xTaskGetTickCount();
                ESP_LOGW(TAG_MPU, "CAIDA CONFIRMADA!");

                xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
                g_alert = VITAL_FALL;
                xSemaphoreGive(g_alert_mutex);

                actuators_fall_alert();
            } else if (dt_ms > POSTURE_WINDOW_MS) {
                state = STATE_IDLE;
            }
            break;

        case STATE_FALL_CONFIRMED:
            if (dt_ms >= COOLDOWN_MS) {
                state = STATE_COOLDOWN;
                state_ts = xTaskGetTickCount();
            }
            break;

        case STATE_COOLDOWN:
            if (dt_ms >= COOLDOWN_MS) {
                state = STATE_IDLE;
                xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
                if (g_alert == VITAL_FALL) {
                    g_alert = VITAL_NORMAL;
                }
                xSemaphoreGive(g_alert_mutex);
                actuators_update();
                ESP_LOGI(TAG_MPU, "Cooldown terminado, sistema normal");
            }
            break;
        }

        g_fall_state_display = state;
        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}