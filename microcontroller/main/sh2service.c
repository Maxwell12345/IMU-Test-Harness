#include "sh2service.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "serial.h"

#include "driver/gpio.h"

#include "sh2.h"
#include "sh2_SensorValue.h"

#define SH2SERVICE_READ_LEN 128

#define BNO085_RESET_PIN GPIO_NUM_26
#define SH2SERVICE_TIMEOUT_US 10000000
#define SH2SERVICE_RECOVERY_ATTEMPTS 100

#define SH2SERVICE_EVENT_QUEUE_LENGTH 64
#define SH2SERVICE_CALLBACK_TASK_PRIORITY 8
#define SH2SERVICE_CALLBACK_RECEIVE_TIMEOUT_MS 100
#define SH2SERVICE_HARD_YIELD_EVERY_LOOPS 16

#define SH2SERVICE_ENABLE_CALLBACK_TIMING_LOG 0
#define SH2SERVICE_SLOW_CALLBACK_US 20000

#define RED_TEXT "\033[31m"
#define WHITE_TEXT "\033[37m"

#if SH2SERVICE_ENABLE_CALLBACK_TIMING_LOG
static const char *TAG = "sh2service";
#endif

static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_dev_handle;
static sh2_Hal_t s_hal;

static sh2service_config_t s_config = {
    I2C_NUM_0,
    0x4B,
    GPIO_NUM_22,
    GPIO_NUM_21,
    GPIO_NUM_27,
    GPIO_NUM_26,
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
static TaskHandle_t s_callback_task_handle;
static QueueHandle_t s_event_queue;

static volatile int s_stop_requested;
static volatile int s_running;
static volatile int s_reset_seen;
static volatile int s_sensors_enabled;
static volatile int s_sh2_ready;
static int64_t s_last_packet_us;
static int64_t s_last_valid_event_us;
static volatile int s_recovering;
static volatile int s_valid_measurements = 0;
static volatile int s_num_valid_acc_measurements;
static volatile int s_num_valid_rot_measurements;
static volatile int s_save_dcd_requested;
static volatile int s_callback_task_stop_requested;
static volatile int s_callback_task_running;
static volatile uint32_t s_dropped_events;

static volatile enum imu_status
s_last_imu_status = NO_DETECTED_IMU;
static volatile int64_t s_last_status_time = 0;

static esp_err_t open_sh2(void);
static esp_err_t hard_recover_and_open_sh2(void);
static esp_err_t soft_reset_sh2(void);
static void sh2service_task(void *arg);
static void sh2service_recovery_task(void *arg);
static void sh2service_callback_task(void *arg);
static esp_err_t sh2service_create_callback_task(void);
static void sh2service_stop_callback_task(void);
static void enqueue_service_event(const sh2service_event_t *event);

static void reset_event_timestamps(void)
{
    int64_t now = esp_timer_get_time();

    s_last_packet_us = now;
    s_last_valid_event_us = now;
}

static void reset_valid_measurement_state(void)
{
    s_valid_measurements = 0;
    s_num_valid_acc_measurements = 0;
    s_num_valid_rot_measurements = 0;
    s_save_dcd_requested = 0;
}

static void update_imu_status(enum imu_status status)
{
    if (s_last_imu_status == status) {
        return;
    }

    int64_t now = esp_timer_get_time();
    const processor_status_t stat = {status, now};
    esp_err_t err = send_status_t(&stat);

    s_last_imu_status = status;
    if (err == ESP_OK) {
        s_last_status_time = now;
    }
}

void handle_micro_commands(enum micro_command_id cmd) {
    switch (cmd) {
        case IMU_RESET_CMD:
            

        default:
            update_imu_status(MICRO_COMMAND_ERROR);
            return;
    }
}

static void update_valid_counts(sh2_SensorValue_t *value, uint8_t acc)
{
    if (s_valid_measurements) {
        return;
    }

    volatile int *counter = NULL;

    if (value->sensorId == SH2_LINEAR_ACCELERATION) {
        counter = &s_num_valid_acc_measurements;
    } else if (value->sensorId == SH2_ROTATION_VECTOR) {
        counter = &s_num_valid_rot_measurements;
    } else {
        return;
    }

    if (acc == 3) {
        (*counter)++;
    } else if (acc == 0 || acc == 1) {
        *counter = 0;
    }

    if (s_num_valid_rot_measurements > 5000 &&
        s_num_valid_acc_measurements > 5000) {
        s_valid_measurements = 1;
        s_save_dcd_requested = 1;
    }
}

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
        update_imu_status(IMU_RESET_FAILURE);
        return err;
    }

    update_imu_status(IMU_UNAVAILABLE);

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

static void enqueue_service_event(const sh2service_event_t *event)
{
    QueueHandle_t queue = s_event_queue;

    if (queue == NULL || event == NULL) {
        return;
    }

    if (xQueueSend(queue, event, 0) != pdTRUE) {
        s_dropped_events++;
    }
}

static void sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    int64_t now = esp_timer_get_time();
    s_last_packet_us = now;

    sh2_SensorValue_t value;

    int rc = sh2_decodeSensorEvent(&value, event);
    if (rc != 0) {
        return;
    }

    uint8_t acc = value.status & 0x03;

    update_valid_counts(&value, acc);
    update_imu_status(HEALTHY);

    // if (acc == 0 || acc == 1) {
    //     return;
    // }

    sh2service_event_t out;
    memset(&out, 0, sizeof(out));
    out.timestamp_us = now;
    out.type = value.sensorId;

    if (value.sensorId == SH2_LINEAR_ACCELERATION) {
        out.type = SH2_LINEAR_ACCELERATION;

        out.data.linear_acceleration.x = value.un.linearAcceleration.x;
        out.data.linear_acceleration.y = value.un.linearAcceleration.y;
        out.data.linear_acceleration.z = value.un.linearAcceleration.z;

        enqueue_service_event(&out);
        return;
    }

    if (value.sensorId == SH2_ROTATION_VECTOR) {
        s_last_valid_event_us = now;

        out.type = SH2_ROTATION_VECTOR;

        out.data.rotation_vector.i = value.un.rotationVector.i;
        out.data.rotation_vector.j = value.un.rotationVector.j;
        out.data.rotation_vector.k = value.un.rotationVector.k;
        out.data.rotation_vector.real = value.un.rotationVector.real;
        out.data.rotation_vector.accuracy = value.un.rotationVector.accuracy;

        enqueue_service_event(&out);
        return;
    }

    if (value.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR) {
        s_last_valid_event_us = now;

        out.type = SH2_GEOMAGNETIC_ROTATION_VECTOR;

        out.data.rotation_vector.i = value.un.geoMagRotationVector.i;
        out.data.rotation_vector.j = value.un.geoMagRotationVector.j;
        out.data.rotation_vector.k = value.un.geoMagRotationVector.k;
        out.data.rotation_vector.real = value.un.geoMagRotationVector.real;
        out.data.rotation_vector.accuracy = value.un.geoMagRotationVector.accuracy;

        enqueue_service_event(&out);
        return;
    }
}

static void sh2service_callback_task(void *arg)
{
    sh2service_event_t event;

    s_callback_task_running = 1;

    while (!s_callback_task_stop_requested) {
        QueueHandle_t queue = s_event_queue;

        int64_t now = esp_timer_get_time();
        if (1000 <= now - s_last_status_time) {
            const processor_status_t repeatUpdate = {s_last_imu_status, now};
            esp_err_t err= send_status_t(&repeatUpdate);
            if (err == ESP_OK) {
                s_last_status_time = now;
            }
        }

        if (queue == NULL) {
            break;
        }

        if (xQueueReceive(queue, &event, pdMS_TO_TICKS(SH2SERVICE_CALLBACK_RECEIVE_TIMEOUT_MS)) != pdTRUE) {
            continue;
        }

        sh2service_callback_t callback = s_callback;
        void *callback_ctx = s_callback_ctx;

        if (callback == NULL) {
            continue;
        }

#if SH2SERVICE_ENABLE_CALLBACK_TIMING_LOG
        int64_t callback_start_us = esp_timer_get_time();
#endif

        callback(&event, callback_ctx);

#if SH2SERVICE_ENABLE_CALLBACK_TIMING_LOG
        int64_t callback_elapsed_us = esp_timer_get_time() - callback_start_us;

        if (callback_elapsed_us > SH2SERVICE_SLOW_CALLBACK_US) {
            ESP_LOGW(TAG, "slow callback: %lld us, type=%d", callback_elapsed_us, event.type);
        }
#endif
    }

    s_callback_task_running = 0;
    s_callback_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t sh2service_create_callback_task(void)
{
    if (s_event_queue == NULL) {
        s_event_queue = xQueueCreate(SH2SERVICE_EVENT_QUEUE_LENGTH, sizeof(sh2service_event_t));
        if (s_event_queue == NULL) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        xQueueReset(s_event_queue);
    }

    if (s_callback_task_handle != NULL || s_callback_task_running) {
        return ESP_OK;
    }

    s_callback_task_stop_requested = 0;

    BaseType_t ok = xTaskCreate(
        sh2service_callback_task,
        "sh2cb",
        s_config.task_stack_size,
        NULL,
        SH2SERVICE_CALLBACK_TASK_PRIORITY,
        &s_callback_task_handle
    );

    if (ok != pdPASS) {
        s_callback_task_handle = NULL;
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

static void sh2service_stop_callback_task(void)
{
    s_callback_task_stop_requested = 1;

    TaskHandle_t current = xTaskGetCurrentTaskHandle();

    if (current != s_callback_task_handle) {
        while (s_callback_task_handle != NULL || s_callback_task_running) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    if (s_event_queue != NULL) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
    }

    s_callback_task_stop_requested = 0;
    s_dropped_events = 0;
}

static void event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    if (event->eventId == SH2_RESET) {
        reset_event_timestamps();
        s_reset_seen = 1;
        s_sensors_enabled = 0;
        update_imu_status(IMU_UNAVAILABLE);
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
        update_imu_status(SENSOR_INITIALIZATION_ERROR);
        return rc;
    }

    rc = enable_sensor(SH2_GEOMAGNETIC_ROTATION_VECTOR, s_config.report_interval_us);
    if (rc != 0) {
        update_imu_status(SENSOR_INITIALIZATION_ERROR);
        return rc;
    }

    service_for_ms(100);

    rc = enable_sensor(SH2_ROTATION_VECTOR, s_config.report_interval_us);
    if (rc != 0) {
        update_imu_status(SENSOR_INITIALIZATION_ERROR);
        return rc;
    }

    s_sensors_enabled = 1;
    return 0;
}

static esp_err_t soft_reset_sh2(void)
{
    reset_event_timestamps();
    s_reset_seen = 0;
    s_sensors_enabled = 0;
    update_imu_status(IMU_UNAVAILABLE);

    vTaskDelay(pdMS_TO_TICKS(10));

    int rc = sh2_devReset();
    if (rc != 0) {
        update_imu_status(IMU_RESET_FAILURE);
        return ESP_FAIL;
    }

    if (!wait_for_reset(s_config.reset_wait_ms)) {
        update_imu_status(IMU_RESET_FAILURE);
        return ESP_ERR_TIMEOUT;
    }

    service_for_ms(s_config.startup_delay_ms);

    rc = configure_sensors();
    if (rc != 0) {
        update_imu_status(IMU_RESET_FAILURE);
        return ESP_FAIL;
    }

    reset_event_timestamps();
    return ESP_OK;
}

static int sh2service_open_and_configure(void)
{
    sh2_close();
    cleanup_i2c();

    vTaskDelay(pdMS_TO_TICKS(100));

    // printf("sh2service_open_and_configure!\n");

    esp_err_t err = hard_recover_and_open_sh2();

    if (err != ESP_OK) {
        sh2_close();
        cleanup_i2c();
        s_sh2_ready = 0;
        return err;
    }

    reset_event_timestamps();
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
        esp_restart();
    }
}

static void sh2service_recovery_task(void *arg)
{
    int wdt_added = wdt_add_current();


    for (int i = 0; i < SH2SERVICE_RECOVERY_ATTEMPTS && !s_stop_requested; i++) {
        reset_valid_measurement_state();

        wdt_reset_current(wdt_added);

        if (sh2service_open_and_configure() == 0) {
            wdt_reset_current(wdt_added);

            if (sh2service_create_service_task() == ESP_OK) {
                s_recovering = 0;
                s_recovery_task_handle = NULL;
                wdt_delete_current(wdt_added);
                vTaskDelete(NULL);
            }

            esp_restart();
        }

        wdt_reset_current(wdt_added);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    s_recovering = 0;
    s_recovery_task_handle = NULL;

    if (!s_stop_requested) {
        esp_restart();
    }

    sh2_close();
    cleanup_i2c();

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

    esp_err_t err = init_int_pin();
    if (err != ESP_OK) {
        update_imu_status(NO_DETECTED_IMU);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = init_i2c();
    if (err != ESP_OK) {
        update_imu_status(NO_DETECTED_IMU);
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
        update_imu_status(NO_DETECTED_IMU);
        return ESP_FAIL;
    }

    rc = sh2_setSensorCallback(sensor_callback, NULL);
    if (rc != 0) {
        sh2_close();
        update_imu_status(NO_DETECTED_IMU);
        return ESP_FAIL;
    }

    err = soft_reset_sh2();
    if (err != ESP_OK) {
        sh2_close();
        cleanup_i2c();
        return err;
    }

    return ESP_OK;
}

static esp_err_t hard_recover_and_open_sh2(void)
{
    esp_err_t err = hard_reset_bno085();
    if (err != ESP_OK) {
        return err;
    }

    recover_i2c_bus();
    err = open_sh2();
    if (err != ESP_OK && s_last_imu_status == IMU_UNAVAILABLE) {
        update_imu_status(NO_DETECTED_IMU);
    }

    return err;
}

static void sh2service_task(void *arg)
{
    int wdt_added = wdt_add_current();
    uint32_t loop_count = 0;

    s_running = 1;
    s_sh2_ready = 1;
    reset_event_timestamps();

    while (!s_stop_requested) {
        wdt_reset_current(wdt_added);

        sh2_service();

        if (s_save_dcd_requested) {
            s_save_dcd_requested = 0;

            wdt_reset_current(wdt_added);
            int rc = sh2_saveDcdNow();
            wdt_reset_current(wdt_added);

            if (rc != 0) {
                reset_valid_measurement_state();
            }
        }

        if (s_reset_seen && !s_sensors_enabled) {
            s_reset_seen = 0;
            reset_event_timestamps();
            service_for_ms(s_config.startup_delay_ms);
            wdt_reset_current(wdt_added);

            int rc = configure_sensors();
            if (rc != 0) {
                update_imu_status(IMU_RESET_FAILURE);
                s_running = 0;
                s_task_handle = NULL;
                sh2service_request_recovery();
                wdt_delete_current(wdt_added);
                vTaskDelete(NULL);
            }

            reset_event_timestamps();
        }

        int64_t now = esp_timer_get_time();

        if (s_sh2_ready && s_sensors_enabled && now - s_last_packet_us > SH2SERVICE_TIMEOUT_US) {
            if (soft_reset_sh2() != ESP_OK) {
                s_running = 0;
                s_task_handle = NULL;
                sh2service_request_recovery();
                wdt_delete_current(wdt_added);
                vTaskDelete(NULL);
            }

            continue;
        }

        if (s_sh2_ready && s_sensors_enabled && now - s_last_valid_event_us > SH2SERVICE_TIMEOUT_US) {
            if (soft_reset_sh2() != ESP_OK) {
                s_running = 0;
                s_task_handle = NULL;
                sh2service_request_recovery();
                wdt_delete_current(wdt_added);
                vTaskDelete(NULL);
            }

            continue;
        }

        loop_count++;
        if ((loop_count % SH2SERVICE_HARD_YIELD_EVERY_LOOPS) == 0) {
            vTaskDelay(1);
        } else {
            taskYIELD();
        }
    }

    s_sh2_ready = 0;
    s_recovering = 0;

    sh2_close();
    cleanup_i2c();

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
        return timer_err;
    }

    if (s_task_handle != NULL || s_running || s_recovering ||
        s_recovery_task_handle != NULL || s_callback_task_handle != NULL ||
        s_callback_task_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_callback = callback;
    s_callback_ctx = ctx;

    s_stop_requested = 0;
    s_reset_seen = 0;
    s_sensors_enabled = 0;
    s_sh2_ready = 0;
    s_recovering = 0;
    reset_valid_measurement_state();
    reset_event_timestamps();

    esp_err_t err = sh2service_create_callback_task();
    if (err != ESP_OK) {
        s_callback = NULL;
        s_callback_ctx = NULL;
        return err;
    }

    err = sh2service_open_and_configure();
    if (err != ESP_OK) {
        cleanup_i2c();
        sh2service_stop_callback_task();
        s_callback = NULL;
        s_callback_ctx = NULL;
        return err;
    }

    reset_event_timestamps();
    s_sh2_ready = 1;

    vTaskDelay(pdMS_TO_TICKS(10));

    err = sh2service_create_service_task();
    if (err != ESP_OK) {
        s_sh2_ready = 0;
        sh2_close();
        cleanup_i2c();
        sh2service_stop_callback_task();
        s_callback = NULL;
        s_callback_ctx = NULL;
        return err;
    }

    return ESP_OK;
}

esp_err_t sh2service_stop(void)
{
    if (s_task_handle == NULL && !s_running &&
        s_recovery_task_handle == NULL && !s_recovering &&
        s_callback_task_handle == NULL && !s_callback_task_running) {
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

    sh2service_stop_callback_task();

    return ESP_OK;
}

bool sh2service_is_running(void)
{
    return s_running != 0 || s_recovering != 0;
}
