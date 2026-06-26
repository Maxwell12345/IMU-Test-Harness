#include "sh2service.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "esp_err.h"

#include "driver/gpio.h"

#include "sh2.h"
#include "sh2_SensorValue.h"

#define SH2SERVICE_READ_LEN 128

#define BNO085_RESET_PIN GPIO_NUM_26
#define SH2SERVICE_TIMEOUT_US 10000000
#define SH2SERVICE_RECOVERY_ATTEMPTS 100

static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_dev_handle;
static sh2_Hal_t s_hal;

static sh2service_config_t s_config = {
    I2C_NUM_0,
    0x4B,
    GPIO_NUM_22,
    GPIO_NUM_21,
    GPIO_NUM_27,
    400000,
    5000,
    8000,
    2000,
    4096,
    20
};

static sh2service_callback_t s_callback;
static void *s_callback_ctx;

static TaskHandle_t s_task_handle;
static TaskHandle_t s_recovery_task_handle;

static volatile int s_stop_requested;
static volatile int s_running;
static volatile int s_reset_seen;
static volatile int s_sensors_enabled;
static volatile int s_sh2_ready;
static int64_t s_last_event_us;
static volatile int s_recovering;
static volatile int s_valid_measurements = 0;
static volatile int s_num_valid_acc_measurements;
static volatile int s_num_valid_rot_measurements;

static esp_err_t open_sh2(void);
static void sh2service_task(void *arg);
static void sh2service_recovery_task(void *arg);

static int wdt_add_current(void)
{
    return esp_task_wdt_add(NULL) == ESP_OK;
}

static void wdt_reset_current(int added)
{
    if (added) {
        esp_task_wdt_reset();
    }
}

static void wdt_delete_current(int added)
{
    if (added) {
        esp_task_wdt_delete(NULL);
    }
}

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
    if (pBuffer == NULL || t_us == NULL || s_dev_handle == NULL || len < 4) {
        return 0;
    }

    // if (gpio_get_level(s_config.int_pin) != 0) {
    //     return 0;
    // }

    unsigned read_len = SH2SERVICE_READ_LEN;

    if (len < read_len) {
        read_len = len;
    }

    esp_err_t err = i2c_master_receive(s_dev_handle, pBuffer, read_len, 10);
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
    if (pBuffer == NULL || s_dev_handle == NULL || len == 0) {
        return 0;
    }

    esp_err_t err = i2c_master_transmit(s_dev_handle, pBuffer, len, 10);
    if (err != ESP_OK) {
        return 0;
    }

    return len;
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
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(BNO085_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    return ESP_OK;
}

static void recover_i2c_bus(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << s_config.scl_pin) | (1ULL << s_config.sda_pin),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&cfg);

    gpio_set_level(s_config.sda_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(s_config.sda_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(s_config.sda_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(s_config.scl_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(s_config.scl_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(s_config.scl_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    for (int i = 0; i < 5; i++) {
        gpio_set_level(s_config.scl_pin, 0);
        esp_rom_delay_us(5);
        gpio_set_level(s_config.scl_pin, 1);
        esp_rom_delay_us(5);
    }

    gpio_set_level(s_config.sda_pin, 0);
    esp_rom_delay_us(5);
    gpio_set_level(s_config.scl_pin, 1);
    esp_rom_delay_us(5);
    gpio_set_level(s_config.sda_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static int wait_for_reset(uint32_t ms)
{
    uint32_t steps = ms / 10;

    for (uint32_t i = 0; i < steps && !s_reset_seen && !s_stop_requested; i++) {
        esp_task_wdt_reset();
        sh2_service();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return s_reset_seen;
}

static void sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    s_last_event_us = esp_timer_get_time();

    sh2_SensorValue_t value;

    int rc = sh2_decodeSensorEvent(&value, event);
    if (rc != 0) {
        return;
    }

    uint8_t acc = value.status & 0x03;

    if (acc == 0 || acc == 1) {
        // printf("Bad status: %d.\n", value.status);
        return;
    }

    if (s_callback == NULL) {
        return;
    }

    sh2service_event_t out;
    memset(&out, 0, sizeof(out));

    if (value.sensorId == SH2_LINEAR_ACCELERATION) {
        if(!s_valid_measurements && acc == 3) {
            s_num_valid_acc_measurements++;
        }
        else if (!s_valid_measurements && (acc == 0 || acc == 1)) {
            s_num_valid_acc_measurements = 0;
        }

        out.type = SH2SERVICE_LINEAR_ACCELERATION;
        out.timestamp_us = esp_timer_get_time();

        out.data.linear_acceleration.x = value.un.linearAcceleration.x;
        out.data.linear_acceleration.y = value.un.linearAcceleration.y;
        out.data.linear_acceleration.z = value.un.linearAcceleration.z;

        s_callback(&out, s_callback_ctx);
        return;
    }

    if (value.sensorId == SH2_ROTATION_VECTOR) {
        if(!s_valid_measurements && acc == 3) {
            s_num_valid_rot_measurements++;
        }
        else if (!s_valid_measurements && (acc == 0 || acc == 1)) {
            s_num_valid_rot_measurements = 0;
        }

        out.type = SH2SERVICE_ROTATION_VECTOR;
        out.timestamp_us = esp_timer_get_time();

        out.data.rotation_vector.i = value.un.rotationVector.i;
        out.data.rotation_vector.j = value.un.rotationVector.j;
        out.data.rotation_vector.k = value.un.rotationVector.k;
        out.data.rotation_vector.real = value.un.rotationVector.real;
        out.data.rotation_vector.accuracy = value.un.rotationVector.accuracy;

        s_callback(&out, s_callback_ctx);
        return;
    }

    if (s_num_valid_rot_measurements > 500 && s_num_valid_acc_measurements > 500) {
        sh2_saveDcdNow();
        s_valid_measurements = 1;
    }
}

static void event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    if (event->eventId == SH2_RESET) {
        s_last_event_us = esp_timer_get_time();
        s_reset_seen = 1;
        s_sensors_enabled = 0;
    }
}


static int enable_sensor(sh2_SensorId_t id, uint32_t interval_us)
{
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.reportInterval_us = interval_us;

    return sh2_setSensorConfig(id, &cfg);
}

static void service_for_ms(uint32_t ms)
{
    uint32_t steps = ms / 10;

    for (uint32_t i = 0; i < steps && !s_stop_requested; i++) {
        esp_task_wdt_reset();
        sh2_service();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static int configure_sensors(void)
{
    int rc;

    rc = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
    if (rc != 0) {
        return rc;
    }

    rc = enable_sensor(SH2_LINEAR_ACCELERATION, s_config.report_interval_us);
    if (rc != 0) {
        return rc;
    }

    service_for_ms(100);

    rc = enable_sensor(SH2_ROTATION_VECTOR, s_config.report_interval_us);
    if (rc != 0) {
        return rc;
    }

    s_sensors_enabled = 1;
    return 0;
}

static int sh2service_open_and_configure(void)
{
    sh2_close();
    cleanup_i2c();
    hard_reset_bno085();

    vTaskDelay(pdMS_TO_TICKS(100));

    printf("sh2service_open_and_configure!\n");

    esp_err_t err = open_sh2();

    if (err != ESP_OK) {
        sh2_close();
        cleanup_i2c();
        hard_reset_bno085();
        s_sh2_ready = 0;
        return err;
    }

    s_last_event_us = esp_timer_get_time();
    s_sh2_ready = 1;
    return 0;
}

static esp_err_t sh2service_create_service_task(void)
{
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
        s_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void sh2service_request_recovery(void)
{
    if (s_stop_requested || s_recovering || s_recovery_task_handle != NULL) {
        return;
    }

    s_recovering = 1;
    s_sh2_ready = 0;
    s_sensors_enabled = 0;
    s_reset_seen = 0;

    BaseType_t ok = xTaskCreate(
        sh2service_recovery_task,
        "sh2recover",
        s_config.task_stack_size,
        NULL,
        s_config.task_priority,
        &s_recovery_task_handle
    );

    if (ok != pdPASS) {
        printf("SH2 recovery task create failed, restarting\n");
        esp_restart();
    }
}

static void sh2service_recovery_task(void *arg)
{
    int wdt_added = wdt_add_current();

    printf("recovering SH2\n");

    for (int i = 0; i < SH2SERVICE_RECOVERY_ATTEMPTS && !s_stop_requested; i++) {
        s_valid_measurements = 0;

        wdt_reset_current(wdt_added);

        if (sh2service_open_and_configure() == 0) {
            wdt_reset_current(wdt_added);

            if (sh2service_create_service_task() == ESP_OK) {
                printf("SH2 recovered\n");
                s_recovering = 0;
                s_recovery_task_handle = NULL;
                wdt_delete_current(wdt_added);
                vTaskDelete(NULL);
            }

            printf("SH2 service task recreate failed, restarting\n");
            esp_restart();
        }

        wdt_reset_current(wdt_added);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    s_recovering = 0;
    s_recovery_task_handle = NULL;

    if (!s_stop_requested) {
        printf("SH2 recovery failed, restarting\n");
        esp_restart();
    }

    sh2_close();
    cleanup_i2c();
    hard_reset_bno085();

    wdt_delete_current(wdt_added);
    vTaskDelete(NULL);
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

static esp_err_t open_sh2(void)
{
    cleanup_i2c();

    vTaskDelay(pdMS_TO_TICKS(10));

    esp_err_t err = hard_reset_bno085();
    if (err != ESP_OK) {
        return err;
    }

    recover_i2c_bus();

    err = init_int_pin();
    if (err != ESP_OK) {
        hard_reset_bno085();
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = init_i2c();
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    s_hal.open = hal_open;
    s_hal.close = hal_close;
    s_hal.read = hal_read;
    s_hal.write = hal_write;
    s_hal.getTimeUs = hal_get_time_us;

    int rc = sh2_open(&s_hal, event_callback, NULL);
    if (rc != 0) {
        return ESP_FAIL;
    }

    rc = sh2_setSensorCallback(sensor_callback, NULL);
    if (rc != 0) {
        sh2_close();
        return ESP_FAIL;
    }

    s_reset_seen = 0;
    s_sensors_enabled = 0;

    vTaskDelay(pdMS_TO_TICKS(10));
    rc = sh2_devReset();
    if (rc != 0) {
        sh2_close();
        return ESP_FAIL;
    }

    if (!wait_for_reset(s_config.reset_wait_ms)) {
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
    int wdt_added = wdt_add_current();

    s_running = 1;
    s_sh2_ready = 1;
    s_last_event_us = esp_timer_get_time();

    while (!s_stop_requested) {
        wdt_reset_current(wdt_added);

        sh2_service();

        if (s_reset_seen && !s_sensors_enabled) {
            s_reset_seen = 0;
            s_last_event_us = esp_timer_get_time();
            service_for_ms(s_config.startup_delay_ms);
            wdt_reset_current(wdt_added);

            int rc = configure_sensors();
            if (rc != 0) {
                s_running = 0;
                s_task_handle = NULL;
                sh2service_request_recovery();
                wdt_delete_current(wdt_added);
                vTaskDelete(NULL);
            }

            s_last_event_us = esp_timer_get_time();
        }

        int64_t now = esp_timer_get_time();

        if (s_sh2_ready && s_sensors_enabled && now - s_last_event_us > SH2SERVICE_TIMEOUT_US) {
            printf("SH2 timeout\n");
            s_running = 0;
            s_task_handle = NULL;
            sh2service_request_recovery();
            wdt_delete_current(wdt_added);
            vTaskDelete(NULL);
        }
    }

    s_sh2_ready = 0;
    s_recovering = 0;

    sh2_close();
    cleanup_i2c();
    hard_reset_bno085();

    s_callback = NULL;
    s_callback_ctx = NULL;
    s_reset_seen = 0;
    s_sensors_enabled = 0;
    s_stop_requested = 0;

    s_running = 0;
    s_task_handle = NULL;

    wdt_delete_current(wdt_added);
    vTaskDelete(NULL);
}

esp_err_t sh2service_start(sh2service_callback_t callback, void *ctx)
{
    esp_err_t timer_err = esp_timer_early_init();

    if (timer_err != ESP_OK) {
        printf("Timer init error.\n");
        return timer_err;
    }

    if (s_task_handle != NULL || s_running || s_recovering || s_recovery_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_callback = callback;
    s_callback_ctx = ctx;

    s_stop_requested = 0;
    s_reset_seen = 0;
    s_sensors_enabled = 0;
    s_sh2_ready = 0;
    s_recovering = 0;

    esp_err_t err = open_sh2();
    if (err != ESP_OK) {
        cleanup_i2c();
        hard_reset_bno085();
        return err;
    }

    s_last_event_us = esp_timer_get_time();
    s_sh2_ready = 1;

    vTaskDelay(pdMS_TO_TICKS(10));

    err = sh2service_create_service_task();
    if (err != ESP_OK) {
        s_sh2_ready = 0;
        sh2_close();
        cleanup_i2c();
        hard_reset_bno085();
        return err;
    }

    return ESP_OK;
}

esp_err_t sh2service_stop(void)
{
    if (s_task_handle == NULL && !s_running && s_recovery_task_handle == NULL && !s_recovering) {
        return ESP_OK;
    }

    s_stop_requested = 1;

    TaskHandle_t current = xTaskGetCurrentTaskHandle();

    if (current == s_task_handle || current == s_recovery_task_handle) {
        return ESP_OK;
    }

    while (s_task_handle != NULL || s_running || s_recovery_task_handle != NULL || s_recovering) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

bool sh2service_is_running(void)
{
    return s_running != 0 || s_recovering != 0;
}
