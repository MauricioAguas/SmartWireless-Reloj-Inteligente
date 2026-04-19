# Wireless Smart: Dispositivo Wearable para Asistencia Autónoma (IoT)

**Wireless Smart** es un prototipo de dispositivo wearable diseñado para mejorar la autonomía y seguridad de personas con movilidad reducida o riesgos cardíacos. Utiliza un microcontrolador ESP32-S3 y sensores biométricos/inerciales para detectar emergencias en tiempo real.

Este proyecto corresponde a la **Fase 1: Capa de Percepción**.

## 🚀 Funcionalidades Principales

- **Monitoreo Biométrico:** Medición de frecuencia cardíaca (BPM) y saturación de oxígeno (SpO2) mediante el sensor MAX30102.
- **Detección de Caídas:** Algoritmo basado en el acelerómetro MPU-6050 que identifica impactos y falta de movimiento posterior.
- **Protocolo de Emergencia:** Ante una caída confirmada, se activa una alarma sonora (Buzzer) y un servomotor para asistencia mecánica.
- **Interfaz Visual:** Pantalla OLED que muestra telemetría constante y LED RGB que indica el estado del sistema mediante colores.
- **Procesamiento en Tiempo Real:** Implementado con **FreeRTOS** para gestionar tareas simultáneas sin bloqueos.

## 🛠️ Hardware Utilizado

- **Microcontrolador:** ESP32-S3 (WROOM).
- **Sensor de Pulso y Oxígeno:** MAX30102.
- **Unidad de Medición Inercial (IMU):** MPU-6050.
- **Pantalla:** OLED I2C.
- **Actuadores:** Servomotor SG90, Buzzer activo y LED RGB integrado.
- **Plataforma:** Montaje en Protoboard.

## 🔌 Conexiones (Pinout)

Todos los sensores comparten el bus **I2C** principal:
- **SDA:** GPIO 8
- **SCL:** GPIO 9
- **Buzzer:** GPIO 7
- **Servo:** GPIO 10
- **LED RGB:** GPIO 48

## 🗂️ Estructura del Proyecto

```
SmartWireless-Reloj-Inteligente/
├── main/
│   ├── main.c               ← Solo app_main() + inicialización
│   └── CMakeLists.txt
├── components/
│   ├── shared/              ← Estado global (mutex, alertas, tipos)
│   ├── actuators/           ← Buzzer, servo SG90, LED RGB WS2812
│   ├── oled_display/        ← Framebuffer SSD1306 + tarea de refresco
│   ├── max30102/            ← Driver I2C + cálculo BPM/SpO2 + tarea
│   └── mpu6050_fall/        ← Detección de caídas + tarea
├── CMakeLists.txt
└── README.md
```

## ⚙️ Configuración y Compilación

Requiere **ESP-IDF v5.x** instalado y configurado.

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
