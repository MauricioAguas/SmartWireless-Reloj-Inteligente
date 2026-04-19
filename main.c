#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mpu6050.h"

// Componentes modulares
#include "shared.h"
#include "max30102.h"
#include "mpu6050_fall.h"
#include "oled_display.h"
#include "actuators.h"

#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_SPEED_HZ        400000

static const char *TAG = "APP_MAIN";

// Definición única de las variables globales compartidas (declaradas extern en shared.h)
volatile alert_level_t  g_alert              = VITAL_NORMAL;
SemaphoreHandle_t       g_alert_mutex        = NULL;
volatile int            g_bpm_display        = 0;
volatile int            g_spo2_display       = 0;
volatile bool           g_finger_oled        = false;
volatile fall_state_t   g_fall_state_display = STATE_IDLE;

void app_main(void)
{
    // Mutex compartido entre tareas
    g_alert_mutex = xSemaphoreCreateMutex();
    configASSERT(g_alert_mutex);

    // Bus I2C compartido (MAX30102 + MPU6050 + SSD1306)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = I2C_MASTER_PORT,
        .sda_io_num                   = I2C_MASTER_SDA_IO,
        .scl_io_num                   = I2C_MASTER_SCL_IO,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
    ESP_LOGI(TAG, "Bus I2C inicializado");

    // MAX30102
    if (max30102_init_device(i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init fallida. Revisa el cableado.");
        return;
    }

    // MPU6050
    mpu6050_handle_t mpu = NULL;
    if (mpu6050_fall_init(i2c_bus, &mpu) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init fallida.");
        return;
    }

    // OLED SSD1306
    oled_init(i2c_bus);

    // Actuadores (buzzer, servo, LED RGB)
    actuators_init();
    actuators_update();
    ESP_LOGI(TAG, "Actuadores OK");

    // Pantalla de arranque
    oled_fb_clear();
    oled_fb_string(0, 0, "=Monitor Vital=");
    oled_fb_string(0, 2, "Boot OK!");
    oled_flush();

    // Lanzar tareas FreeRTOS
    // Prioridades: task_mpu6050(6) > task_max30102(5) > task_oled(4)
    xTaskCreate(task_max30102, "max30102", 4096, NULL,        5, NULL);
    xTaskCreate(task_mpu6050,  "mpu6050",  4096, (void *)mpu, 6, NULL);
    xTaskCreate(task_oled,     "oled",     3072, NULL,        4, NULL);

    ESP_LOGI(TAG, "Todas las tareas lanzadas.");
}#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mpu6050.h"

// Componentes modulares
#include "shared.h"
#include "max30102.h"
#include "mpu6050_fall.h"
#include "oled_display.h"
#include "actuators.h"

#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_SPEED_HZ        400000

static const char *TAG = "APP_MAIN";

// Definición única de las variables globales compartidas (declaradas extern en shared.h)
volatile alert_level_t  g_alert              = VITAL_NORMAL;
SemaphoreHandle_t       g_alert_mutex        = NULL;
volatile int            g_bpm_display        = 0;
volatile int            g_spo2_display       = 0;
volatile bool           g_finger_oled        = false;
volatile fall_state_t   g_fall_state_display = STATE_IDLE;

void app_main(void)
{
    // Mutex compartido entre tareas
    g_alert_mutex = xSemaphoreCreateMutex();
    configASSERT(g_alert_mutex);

    // Bus I2C compartido (MAX30102 + MPU6050 + SSD1306)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = I2C_MASTER_PORT,
        .sda_io_num                   = I2C_MASTER_SDA_IO,
        .scl_io_num                   = I2C_MASTER_SCL_IO,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
    ESP_LOGI(TAG, "Bus I2C inicializado");

    // MAX30102
    if (max30102_init_device(i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init fallida. Revisa el cableado.");
        return;
    }

    // MPU6050
    mpu6050_handle_t mpu = NULL;
    if (mpu6050_fall_init(i2c_bus, &mpu) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init fallida.");
        return;
    }

    // OLED SSD1306
    oled_init(i2c_bus);

    // Actuadores (buzzer, servo, LED RGB)
    actuators_init();
    actuators_update();
    ESP_LOGI(TAG, "Actuadores OK");

    // Pantalla de arranque
    oled_fb_clear();
    oled_fb_string(0, 0, "=Monitor Vital=");
    oled_fb_string(0, 2, "Boot OK!");
    oled_flush();

    // Lanzar tareas FreeRTOS
    // Prioridades: task_mpu6050(6) > task_max30102(5) > task_oled(4)
    xTaskCreate(task_max30102, "max30102", 4096, NULL,        5, NULL);
    xTaskCreate(task_mpu6050,  "mpu6050",  4096, (void *)mpu, 6, NULL);
    xTaskCreate(task_oled,     "oled",     3072, NULL,        4, NULL);

    ESP_LOGI(TAG, "Todas las tareas lanzadas.");
}