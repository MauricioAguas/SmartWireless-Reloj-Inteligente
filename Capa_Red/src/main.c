#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_sntp.h"

#include "mqtt_client.h"

#include "shared.h"
#include "max30102.h"
#include "mpu6050_fall.h"
#include "oled_display.h"
#include "actuators.h"

/* =========================================================
   PINES I2C SENSORES
   ========================================================= */

#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define I2C_MASTER_PORT     I2C_NUM_0

/* =========================================================
   WIFI + MQTT
   ========================================================= */

#define WIFI_SSID        "RaspberryIoT420"
#define WIFI_PASS        "12345678"

#define MQTT_BROKER_URI  "mqtts://192.168.50.1:8883"
#define MQTT_TOPIC_DATA  "monitor/paciente/data"

// NTP: misma IP que el broker MQTT — la Raspberry sirve su hora local
#define NTP_SERVER       "192.168.50.1"

static const char *TAG = "APP_MAIN";

/* =========================================================
   CERTIFICADO CA
   ========================================================= */

static const char *ca_cert =
"-----BEGIN CERTIFICATE-----\n"
"MIIDrTCCApWgAwIBAgIUZmuAXTVcePGNYGrec9baZoN2e5AwDQYJKoZIhvcNAQEL\n"
"BQAwZjELMAkGA1UEBhMCQ08xEjAQBgNVBAgMCUFudGlvcXVpYTERMA8GA1UEBwwI\n"
"TWVkZWxsaW4xDTALBgNVBAoMBFVkZUExDDAKBgNVBAsMA0lvVDETMBEGA1UEAwwK\n"
"TUktQ0EtUkFJWjAeFw0yNjA1MjIwMTQ0NDJaFw0zNjA1MTkwMTQ0NDJaMGYxCzAJ\n"
"BgNVBAYTAkNPMRIwEAYDVQQIDAlBbnRpb3F1aWExETAPBgNVBAcMCE1lZGVsbGlu\n"
"MQ0wCwYDVQQKDARVZGVBMQwwCgYDVQQLDANJb1QxEzARBgNVBAMMCk1JLUNBLVJB\n"
"SVowggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCra3LoJ67raga4xzqQ\n"
"QBAjfkLuS7ykoT97m6Ae70eONCaztAsITrmgdj3o/BCSsUUTa3tdAWS4fHpA7zmc\n"
"6ETnnP3WSRfDTaDTuzcnNZ+fW3zVobffSbAza4bWZ9/vEXSVUWU7LewmJtamHAPm\n"
"8i+gXGwgBG2grGL9MZBW1EKuE2nr8sIStyNJpMB8J35Xfq1Wmnug1FMYs33d51HM\n"
"1nSKEqL7P9BzcRhhi/rWHS0MDcmPv9rW+HoT+OSeTeIvYzoV4wRF8NR6Uc25AX+C\n"
"hSxg0+jOnwOdniYsk4SHrNE8h4g32FXIFomFzfUe5krRel0MD0NXK/+XIXb4i7kz\n"
"EK+RAgMBAAGjUzBRMB0GA1UdDgQWBBRllPPlF4GPaLFYfdE4uKY3pQg+ITAfBgNV\n"
"HSMEGDAWgBRllPPlF4GPaLFYfdE4uKY3pQg+ITAPBgNVHRMBAf8EBTADAQH/MA0G\n"
"CSqGSIb3DQEBCwUAA4IBAQB2NsgHlkUwX4+XngLBUJC1XukFCltfYHcmKE4cGNwr\n"
"16KQ0AtBihkYFLptBaXHYn5S4IkyRkDi0vvuuXmviiPmf39vOU047EWk8kt+4Bc7\n"
"087USuSQYj+bSWrPfa9g/Ws7dW+y+9TVhFGBO5fGl1MCga763CTnaYIi/xG3mCVQ\n"
"QhB5/+XfrtkU1XfS4tSx9qjNo+Q4EPoRnPgf8ENYUGuVukL87P31KWuMxfjS84hL\n"
"+wLlmITP0f7+AIiK1LLQ3rcVZF790TESIuJppbXObJyHQfcwTKKOdTU9CUykU46h\n"
"L7HhPtP1n4/uNG2CQ4RDoc/p2NKTSkskJPQtZ/F+GeXi\n"
"-----END CERTIFICATE-----\n";

/* =========================================================
   CERTIFICADO CLIENTE
   ========================================================= */

static const char *client_cert =
"-----BEGIN CERTIFICATE-----\n"
"MIIDlzCCAn+gAwIBAgIUAKy67EiAhlw2EljxEmFQp3oRVrAwDQYJKoZIhvcNAQEL\n"
"BQAwZjELMAkGA1UEBhMCQ08xEjAQBgNVBAgMCUFudGlvcXVpYTERMA8GA1UEBwwI\n"
"TWVkZWxsaW4xDTALBgNVBAoMBFVkZUExDDAKBgNVBAsMA0lvVDETMBEGA1UEAwwK\n"
"TUktQ0EtUkFJWjAeFw0yNjA1MjIwMTU0MDdaFw0yNzA1MjIwMTU0MDdaMGExCzAJ\n"
"BgNVBAYTAkNPMRIwEAYDVQQIDAlBbnRpb3F1aWExETAPBgNVBAcMCE1lZGVsbGlu\n"
"MQ0wCwYDVQQKDARVZGVBMQwwCgYDVQQLDANJb1QxDjAMBgNVBAMMBUVTUDMyMIIB\n"
"IjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAqvOGjqqfuAJcTY0oHn/dKQvS\n"
"WvSP51qvHinbjknbHdVaKm8CILO7rd+BJ4lyUhPOfQZQ6UKRCyIplptmcnmDtRIw\n"
"gFxJeoVt//jnf9Hd2qMMvJ48Qk4riQhJNtSqmKX1+XBn6pHPhVm+2qoiiMnt5Yuw\n"
"2xKyycvxGHFRxMVWUZlhU5nBZCAFpC6Oxj1ODdVKw3E+tHnNiQSxIOjGqPpMcP59\n"
"0omeSFy931HBH7+J4MS4pF9JTI0zg7GBiJy6F9lhm5pQnR+D6Y5+qR+0b+HrJvuA\n"
"EmZR95SmUN3rBlzZEhWBJUsCLFrXA20CvOpU82ob0ZxJt1iUaswlQ4tTfqcPoQID\n"
"AQABo0IwQDAdBgNVHQ4EFgQUWnUMDvjlkbNTxCeirSyQWdYfCeswHwYDVR0jBBgw\n"
"FoAUZZTz5ReBj2ixWH3ROLimN6UIPiEwDQYJKoZIhvcNAQELBQADggEBADMvNDAE\n"
"7RaIT9q1Cv45NzvoFsg5mNXMWe5Vy1AOBRmLWfm/+iO9YP5F8ZVcwGPTpy0SYanC\n"
"wi7GH/o+FYJHhQIAi0TTnL9AtguE/arC2vvR7WDlFpvKf6MOc6RGXZQGIbMIl7zQ\n"
"OurYwI41zIiQ2vwSExmi9zJSKvg3xJ+U9+49+1PXcN9h4xxVErjutULW0lrjbMKM\n"
"4VNIW9eA/mHDNSai0Bi8VLcDM1DpuoZTyb4ItU/WsEpeiFFNkVDqK5jdWpijz/cA\n"
"icW8PrP0fpUWM+qUJqP0tpbwnkzlFDyonZ6GTNLXtmX8l1ANvVzO1Ii69P8LrmyG\n"
"aE4OtjfUY4g+s7E=\n"
"-----END CERTIFICATE-----\n";

/* =========================================================
   PRIVATE KEY CLIENTE
   ========================================================= */

static const char *client_key =
"-----BEGIN PRIVATE KEY-----\n"
"MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCq84aOqp+4AlxN\n"
"jSgef90pC9Ja9I/nWq8eKduOSdsd1VoqbwIgs7ut34EniXJSE859BlDpQpELIimW\n"
"m2ZyeYO1EjCAXEl6hW3/+Od/0d3aowy8njxCTiuJCEk21KqYpfX5cGfqkc+FWb7a\n"
"qiKIye3li7DbErLJy/EYcVHExVZRmWFTmcFkIAWkLo7GPU4N1UrDcT60ec2JBLEg\n"
"6Mao+kxw/n3SiZ5IXL3fUcEfv4ngxLikX0lMjTODsYGInLoX2WGbmlCdH4Ppjn6p\n"
"H7Rv4esm+4ASZlH3lKZQ3esGXNkSFYElSwIsWtcDbQK86lTzahvRnEm3WJRqzCVD\n"
"i1N+pw+hAgMBAAECggEAIFZ9UW8O1BrkNYpaeLyEzE8vbWO+dgwGPN07qmSntbLG\n"
"gejqXN/LBKbXMnCyZrrW8HkZpKpiMBo4BkSiwkxgwQeuE5YBk1L6vjVqK1h6kHGQ\n"
"Bv0i1oMdgqhF20Cq/cHFMDiWe7215RqXOxt8eWZDYY0FeUeF+RziYFaNWnze6YPb\n"
"4RzMO7Mo4cLlweDatx6wqa0SX+9wHuRGT3vG20k0MumPcDeGPZm5rrnNJ33DnKNS\n"
"P2lhtHx+pyDcymU0ml47AkK0ciTeyejOpMQ0FC7v+tTraoz39IqSxoZbtO4134FE\n"
"7N0XwcGMa8wNEz3cwyodGi9QwM2se/svy+sETrBw3QKBgQDnt62Q6F6nd0LgKVrd\n"
"v0Earbn+J4EUZCf2WgGSUMPsMM6n4lSn7RzE+mLZ4bPNngOKzGFRo85pmZHnd73p\n"
"+rUxVjoib/9lFQdkZ/XcxG8yIIJe1Io51h4CAiep0ym/RmzQCPY3n1yglfQV8qMA\n"
"9QYRxNz48MWinBYKmPKU5YLUbQKBgQC83ameKM67Uqrx27GNvQ/lA/UWNU6VbGfJ\n"
"M1018ZJVskF+ivo67S6sF/7FGjBy0Rrfo2pHLx06OddpeMOzKqJdve38+zv9F9LB\n"
"x3pJZod/iTgVTTHOWVdPQ7/Tg0/eslrb92oNhAzv9/8etHFcTf+CxGXCRkdXBNtJ\n"
"4pEEUVufhQKBgQCjw+8JURFErkc3gkLUIc1ze4DOHUFfFgIgXDBsJmSx0zTa9lz6\n"
"adxBYuzmLFwVYC4EtLm7J1hEzeKOgtRYP3Y7rkNb/2ezGw+kaM0dAD/OX6eEOhaP\n"
"FcMTjE5X+gOxSaaxyQOrABhI7nIZ6OhHTuTBPi8mSZSEfmgdiUc48JRsaQKBgGmh\n"
"V/J7RFSEgdNPWli5uyANPJA1NERiIxHmxmUbPQrs7bCGrjky2n2p1fYLFbnBtdQK\n"
"o7A4a5JbM11sC5gzaigfx/FL2ltNbbSvindu/q2X42QWjpqoYSqV672ynYMiIasR\n"
"D6GLj2jPPULBDP3hKdzLV1Z21AOZVcRXEWBm2GW5AoGAHVPISv8vv+5ifgl1BsjF\n"
"S/UvbEu10Y55UFawQHcUBcFW0Ep1JlyczKofKtT8NI/AEnCUsQn6+fZD7S/N8aZL\n"
"SG86EfwAEfbxH8cbNmlRF3es6/Wadm4j0P3AVoVD8KVZmEaYKN9qCzmKKd9eOpfk\n"
"wX2VCyBpSoJxxFR9gBDAHzA=\n"
"-----END PRIVATE KEY-----\n";

/* =========================================================
   MQTT GLOBAL
   ========================================================= */

static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;

/* =========================================================
   WIFI EVENT HANDLER
   ========================================================= */

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WiFi desconectado, reconectando...");
        g_wifi_ready = false;
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi conectado. IP: " IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_ready = true;
    }
}

/* =========================================================
   WIFI INIT
   ========================================================= */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Conectando a WiFi '%s'...", WIFI_SSID);
}

/* =========================================================
   NTP local apuntando a la Raspberry (192.168.50.1)

   REQUISITO en la Raspberry (ejecutar una sola vez):

     sudo apt install chrony -y
     echo "local stratum 8 orphan" | sudo tee -a /etc/chrony/chrony.conf
     echo "allow 192.168.50.0/24"  | sudo tee -a /etc/chrony/chrony.conf
     sudo systemctl restart chrony

   Verificar que funciona:
     chronyc tracking          <- muestra hora local
     chronyc clients           <- debe aparecer 192.168.50.x del ESP32

   Si no tienes chrony instalado aun, verifica con:
     timedatectl               <- debe decir NTP service: active
   ========================================================= */

static void obtain_time(void)
{
    ESP_LOGI(TAG, "Sincronizando hora con Raspberry: %s", NTP_SERVER);

    // Zona horaria Colombia (UTC-5, sin DST)
    setenv("TZ", "COT5", 1);
    tzset();

    if (esp_sntp_enabled()) {
        esp_sntp_stop();
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_max = 20;   // 20 x 2 s = 40 s maximo

    while (retry < retry_max) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);

        ESP_LOGI(TAG, "NTP intento %d/%d - anio=%d",
                 retry + 1, retry_max, timeinfo.tm_year + 1900);

        if (timeinfo.tm_year >= (2024 - 1900)) {
            g_ntp_ready = true;
            ESP_LOGI(TAG, "Hora OK: %04d-%02d-%02d %02d:%02d:%02d (COT)",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                     timeinfo.tm_mday, timeinfo.tm_hour,
                     timeinfo.tm_min,  timeinfo.tm_sec);
            return;
        }
        retry++;
    }

    // Timeout: continua sin hora para no bloquear el sistema
    g_ntp_ready = true;
    ESP_LOGE(TAG, "Timeout NTP. Verifica chrony en la Raspberry (%s).", NTP_SERVER);
}

/* =========================================================
   MQTT EVENT HANDLER
   ========================================================= */

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT conectado");
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT desconectado");
            break;
        case MQTT_EVENT_ERROR:
            mqtt_connected = false;
            ESP_LOGE(TAG, "Error MQTT");
            break;
        default:
            break;
    }
}

/* =========================================================
   MQTT START
   ========================================================= */

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri                     = MQTT_BROKER_URI,
        .broker.verification.certificate        = ca_cert,
        .credentials.authentication.certificate = client_cert,
        .credentials.authentication.key         = client_key,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "No se pudo crear cliente MQTT");
        return;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

/* =========================================================
   TASK PUBLICAR MQTT
   ========================================================= */

static void mqtt_publish_task(void *pvParameters)
{
    char payload[160];

    while (1) {
        if (!mqtt_connected || mqtt_client == NULL) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int bpm = 0, spo2 = 0, fall_state = 0, alert = 0;
        bool finger = false;

        if (xSemaphoreTake(g_alert_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bpm        = g_bpm_display;
            spo2       = g_spo2_display;
            finger     = g_finger_oled;
            fall_state = (int)g_fall_state_display;
            alert      = (int)g_alert;
            xSemaphoreGive(g_alert_mutex);
        } else {
            ESP_LOGW(TAG, "No se pudo tomar mutex para publicar");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        snprintf(payload, sizeof(payload),
                 "{\"bpm\":%d,\"spo2\":%d,\"finger\":%s,\"fall\":%d,\"alert\":%d}",
                 bpm, spo2, finger ? "true" : "false", fall_state, alert);

        int msg_id = esp_mqtt_client_publish(
            mqtt_client, MQTT_TOPIC_DATA, payload, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT pub id=%d %s", msg_id, payload);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* =========================================================
   APP MAIN
   ========================================================= */

void app_main(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Mutex primero, antes de cualquier tarea
    g_alert_mutex = xSemaphoreCreateMutex();
    configASSERT(g_alert_mutex);

    // WiFi (asincrono, g_wifi_ready se activa en el event handler)
    wifi_init();

    // Bloquear hasta tener IP real
    ESP_LOGI(TAG, "Esperando IP de la Raspberry...");
    while (!g_wifi_ready) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // NTP desde la Raspberry local
    obtain_time();

    // MQTT TLS
    mqtt_app_start();

    // Bus I2C compartido
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

    if (max30102_init_device(i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init fallida");
        return;
    }

    mpu6050_handle_t mpu = NULL;
    if (mpu6050_fall_init(i2c_bus, &mpu) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init fallida");
        return;
    }

    oled_init(i2c_bus);
    actuators_init();
    actuators_update();

    xTaskCreate(task_max30102,     "max30102",     4096, NULL,        5, NULL);
    xTaskCreate(task_mpu6050,      "mpu6050",      4096, (void *)mpu, 6, NULL);
    xTaskCreate(task_oled,         "oled",         3072, NULL,        4, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_publish", 4096, NULL,        5, NULL);

    ESP_LOGI(TAG, "Todas las tareas lanzadas");
}
