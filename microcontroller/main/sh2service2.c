#include "sh2service2.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "sh2.h"
#include "sh2_SensorValue.h"

#define SH2SERVICE_READ_LEN 1024

#define BNO085_RESET_PIN GPIO_NUM_26

static const char *TAG = "sh2service";

static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_dev_handle;
static sh2_Hal_t s_hal;

static sh2service_config_t s_config;
static sh2service_callback_t s_callback;
static void *s_callback_ctx;

static TaskHandle_t s_task_handle;

static volatile int s_stop_requested;
static volatile int s_running;
static volatile int s_reset_seen;
static volatile int s_sensors_enabled;

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    return (uint32_t)esp_timer_get_time();
}

static int hal_open(sh2_Hal_t *self)
{
    return 0;
}

static void hal_close(sh2_Hal_t *self)
{
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    if (gpio_get_level(s_config.int_pin) != 0) {
        return 0;
    }

    unsigned read_len = SH2SERVICE_READ_LEN;

    if (len < read_len) {
        read_len = len;
    }

    esp_err_t err = i2c_master_receive(s_dev_handle, pBuffer, read_len, 100);
    if (err != ESP_OK) {
        return 0;
    }

    uint16_t packet_len = (uint16_t)pBuffer[0] | ((uint16_t)(pBuffer[1] & 0x7F) << 8);

    if (packet_len == 0 || packet_len == 0x7FFF || packet_len > read_len) {
        return 0;
    }

    *t_us = hal_get_time_us(self);
    return packet_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    esp_err_t err = i2c_master_transmit(s_dev_handle, pBuffer, len, 100);
    if (err != ESP_OK) {
        return 0;
    }

    return len;
}

static esp_err_t hard_reset_bno085(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BNO085_RESET_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(BNO085_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(BNO085_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(750));

    return ESP_OK;
}

static void sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    sh2_SensorValue_t value;

    int rc = sh2_decodeSensorEvent(&value, event);
    if (rc != 0) {
        return;
    }

    if (s_callback == NULL) {
        return;
    }

    sh2service_event_t out;
    memset(&out, 0, sizeof(out));

    if (value.sensorId == SH2_LINEAR_ACCELERATION) {
        out.type = SH2SERVICE_LINEAR_ACCELERATION;
        out.timestamp_us = value.timestamp;

        out.data.linear_acceleration.x = value.un.linearAcceleration.x;
        out.data.linear_acceleration.y = value.un.linearAcceleration.y;
        out.data.linear_acceleration.z = value.un.linearAcceleration.z;

        s_callback(&out, s_callback_ctx);
        return;
    }

    if (value.sensorId == SH2_ROTATION_VECTOR) {
        out.type = SH2SERVICE_ROTATION_VECTOR;
        out.timestamp_us = value.timestamp;

        out.data.rotation_vector.i = value.un.rotationVector.i;
        out.data.rotation_vector.j = value.un.rotationVector.j;
        out.data.rotation_vector.k = value.un.rotationVector.k;
        out.data.rotation_vector.real = value.un.rotationVector.real;
        out.data.rotation_vector.accuracy = value.un.rotationVector.accuracy;

        s_callback(&out, s_callback_ctx);
        return;
    }
}

static void event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    if (event->eventId == SH2_RESET) {
        s_reset_seen = 1;
        s_sensors_enabled = 0;
    }
}

static esp_err_t init_int_pin(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << s_config.int_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&cfg);
}

static esp_err_t init_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = s_config.i2c_port,
        .scl_io_num = s_config.scl_pin,
        .sda_io_num = s_config.sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_config.i2c_addr,
        .scl_speed_hz = s_config.i2c_speed_hz,
    };

    err = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
    if (err != ESP_OK) {
        i2c_del_master_bus(s_bus_handle);
        s_bus_handle = NULL;
        return err;
    }

    return ESP_OK;
}

static void cleanup_i2c(void)
{
    if (s_dev_handle != NULL) {
        i2c_master_bus_rm_device(s_dev_handle);
        s_dev_handle = NULL;
    }

    if (s_bus_handle != NULL) {
        i2c_del_master_bus(s_bus_handle);
        s_bus_handle = NULL;
    }
}

static void service_for_ms(uint32_t ms)
{
    uint32_t steps = ms / 10;

    for (uint32_t i = 0; i < steps && !s_stop_requested; i++) {
        sh2_service();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static int wait_for_reset(uint32_t ms)
{
    uint32_t steps = ms / 10;

    for (uint32_t i = 0; i < steps && !s_reset_seen && !s_stop_requested; i++) {
        sh2_service();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return s_reset_seen;
}

static int enable_sensor(sh2_SensorId_t id, uint32_t interval_us)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.reportInterval_us = interval_us;

    return sh2_setSensorConfig(id, &cfg);
}

static int configure_sensors(void)
{
    int rc;

    rc = enable_sensor(SH2_LINEAR_ACCELERATION, s_config.report_interval_us);
    if (rc != 0) {
        ESP_LOGE(TAG, "linear acceleration enable failed rc=%d", rc);
        return rc;
    }

    service_for_ms(100);

    rc = enable_sensor(SH2_ROTATION_VECTOR, s_config.report_interval_us);
    if (rc != 0) {
        ESP_LOGE(TAG, "rotation vector enable failed rc=%d", rc);
        return rc;
    }

    s_sensors_enabled = 1;
    return 0;
}

static esp_err_t open_sh2(void)
{
    s_hal.open = hal_open;
    s_hal.close = hal_close;
    s_hal.read = hal_read;
    s_hal.write = hal_write;
    s_hal.getTimeUs = hal_get_time_us;

    int rc = sh2_open(&s_hal, event_callback, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "sh2_open failed rc=%d", rc);
        return ESP_FAIL;
    }

    rc = sh2_setSensorCallback(sensor_callback, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "sh2_setSensorCallback failed rc=%d", rc);
        sh2_close();
        return ESP_FAIL;
    }

    s_reset_seen = 0;
    s_sensors_enabled = 0;

    vTaskDelay(pdMS_TO_TICKS(10));
    rc = sh2_devReset();
    if (rc != 0) {
        ESP_LOGE(TAG, "sh2_devReset failed rc=%d", rc);
        sh2_close();
        return ESP_FAIL;
    }

    if (!wait_for_reset(s_config.reset_wait_ms)) {
        ESP_LOGE(TAG, "no SH2 reset seen");
        sh2_close();
        return ESP_ERR_TIMEOUT;
    }

    service_for_ms(s_config.startup_delay_ms);

    rc = configure_sensors();
    if (rc != 0) {
        sh2_close();
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void sh2service_task(void *arg)
{
    s_running = 1;

    while (!s_stop_requested) {
        sh2_service();

        if (s_reset_seen && !s_sensors_enabled) {
            s_reset_seen = 0;
            service_for_ms(s_config.startup_delay_ms);
            configure_sensors();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    sh2_close();
    cleanup_i2c();

    s_running = 0;
    s_task_handle = NULL;

    vTaskDelete(NULL);
}

void sh2service_default_config2(sh2service_config_t *config)
{
    memset(config, 0, sizeof(*config));

    config->i2c_port = I2C_NUM_0; // Use ESP32 I2C controller number 0.
    config->i2c_addr = 0x4A;

    // These are the ESP32 GPIO pins connected to the BNO085.
    config->scl_pin = GPIO_NUM_22;
    config->sda_pin = GPIO_NUM_21;
    config->int_pin = GPIO_NUM_27;

    config->i2c_speed_hz = 100000; // This sets the I2C bus speed to 100 kHz.
    config->report_interval_us = 5000; // This asks the BNO085 to produce sensor reports every 5000 microseconds.

    config->reset_wait_ms = 8000; // Wait 3000 milliseconds, or 3 seconds, after reset/startup-related setup.
    config->startup_delay_ms = 2000; // Wait 1000 milliseconds, or 1 second, during startup before continuing.

    config->task_stack_size = 4096; // This gives the FreeRTOS service task a stack size of 4096 bytes. (Size of packet we read from the IMU)
    config->task_priority = 5; // This sets the FreeRTOS task priority to 5.
}

esp_err_t sh2service_start2(const sh2service_config_t *config, sh2service_callback_t callback, void *ctx)
{
    if (s_task_handle != NULL || s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        sh2service_default_config2(&s_config);
    } else {
        memcpy(&s_config, config, sizeof(s_config));
    }

    s_callback = callback;
    s_callback_ctx = ctx;

    s_stop_requested = 0;
    s_reset_seen = 0;
    s_sensors_enabled = 0;

    vTaskDelay(pdMS_TO_TICKS(10));

    esp_err_t err = init_int_pin();
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = hard_reset_bno085();
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = init_i2c();
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    err = open_sh2();
    if (err != ESP_OK) {
        cleanup_i2c();
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    BaseType_t ok = xTaskCreate(
        sh2service_task,
        "sh2service",
        s_config.task_stack_size,
        NULL,
        s_config.task_priority,
        &s_task_handle
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    if (ok != pdPASS) {
        sh2_close();
        cleanup_i2c();
        s_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t sh2service_stop2(void)
{
    if (s_task_handle == NULL && !s_running) {
        return ESP_OK;
    }

    s_stop_requested = 1;

    if (xTaskGetCurrentTaskHandle() == s_task_handle) {
        return ESP_OK;
    }

    while (s_task_handle != NULL || s_running) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

bool sh2service_is_running2(void)
{
    return s_running != 0;
}