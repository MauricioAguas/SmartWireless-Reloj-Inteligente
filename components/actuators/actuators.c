#include "actuators.h"
#include "shared.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static led_strip_handle_t rgb_strip = NULL;

// =========================================================================
//  Servo
// =========================================================================

void servo_set_angle(int angle_deg)
{
    if (angle_deg < 0)   angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;
    uint32_t duty = SERVO_DUTY_MIN +
                    (uint32_t)((angle_deg * (SERVO_DUTY_MAX - SERVO_DUTY_MIN)) / 180);
    ledc_set_duty(SERVO_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_SPEED_MODE, SERVO_CHANNEL);
}

// =========================================================================
//  Buzzer
// =========================================================================

void buzzer_on(uint32_t freq_hz)
{
    ledc_set_freq(BUZZER_SPEED_MODE, BUZZER_TIMER, freq_hz);
    ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 512);
    ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

void buzzer_off(void)
{
    ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

// =========================================================================
//  LED RGB WS2812
// =========================================================================

void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(rgb_strip, 0, r, g, b);
    led_strip_refresh(rgb_strip);
}

void rgb_off(void)
{
    led_strip_clear(rgb_strip);
    led_strip_refresh(rgb_strip);
}

// =========================================================================
//  Inicializacion
// =========================================================================

void actuators_init(void)
{
    // Buzzer
    ledc_timer_config_t tmr_buz = {
        .speed_mode      = BUZZER_SPEED_MODE,
        .timer_num       = BUZZER_TIMER,
        .duty_resolution = BUZZER_RESOLUTION,
        .freq_hz         = BUZZER_FREQ_ALERT,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr_buz);

    ledc_channel_config_t ch_buz = {
        .speed_mode = BUZZER_SPEED_MODE,
        .channel    = BUZZER_CHANNEL,
        .timer_sel  = BUZZER_TIMER,
        .gpio_num   = BUZZER_PIN,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch_buz);

    // Servo
    ledc_timer_config_t tmr_srv = {
        .speed_mode      = SERVO_SPEED_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr_srv);

    ledc_channel_config_t ch_srv = {
        .speed_mode = SERVO_SPEED_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .gpio_num   = SERVO_PIN,
        .duty       = SERVO_DUTY_MIN,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch_srv);
    servo_set_angle(0);

    // LED RGB WS2812 via RMT
    led_strip_config_t sc = {
        .strip_gpio_num         = RGB_LED_PIN,
        .max_leds               = 1,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rc = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &rgb_strip));

    rgb_set(0, 40, 0);
    buzzer_off();
}

// =========================================================================
//  Logica de alertas
// =========================================================================

void actuators_update(void)
{
    alert_level_t lv;
    xSemaphoreTake(g_alert_mutex, portMAX_DELAY);
    lv = g_alert;
    xSemaphoreGive(g_alert_mutex);

    switch (lv) {
    case VITAL_NORMAL:
        rgb_set(0, 255, 0);
        servo_set_angle(0);
        break;
    case VITAL_WARN_BPM:
        rgb_set(255, 80, 0);
        servo_set_angle(45);
        break;
    case VITAL_WARN_SPO2:
        rgb_set(0, 0, 255);
        servo_set_angle(90);
        break;
    case VITAL_FALL:
        break;
    }
}

void actuators_fall_alert(void)
{
    servo_set_angle(180);
    for (int i = 0; i < 5; i++) {
        rgb_set(255, 0, 0);
        buzzer_on(BUZZER_FREQ_ALERT);
        vTaskDelay(pdMS_TO_TICKS(300));
        rgb_off();
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    rgb_set(255, 0, 0);
}

void actuators_jolt(void)
{
    rgb_set(255, 80, 0);
    buzzer_on(BUZZER_FREQ_JOLT);
    vTaskDelay(pdMS_TO_TICKS(120));
    buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(300));
    actuators_update();
}

void beep_vital_warn(void)
{
    buzzer_on(BUZZER_FREQ_WARN);
    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_on(BUZZER_FREQ_WARN);
    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_off();
}
