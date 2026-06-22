#ifndef SH2SERVICE_H
#define SH2SERVICE_H

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

typedef enum {
    SH2SERVICE_LINEAR_ACCELERATION = 1,
    SH2SERVICE_ROTATION_VECTOR = 2,
} sh2service_event_type_t;

typedef struct {
    float x;
    float y;
    float z;
} sh2service_linear_acceleration_t;

typedef struct {
    float i;
    float j;
    float k;
    float real;
    float accuracy;
} sh2service_rotation_vector_t;

typedef struct {
    sh2service_event_type_t type;
    uint64_t timestamp_us;

    union {
        sh2service_linear_acceleration_t linear_acceleration;
        sh2service_rotation_vector_t rotation_vector;
    } data;
} sh2service_event_t;

typedef void (*sh2service_callback_t)(const sh2service_event_t *event, void *ctx);

typedef struct {
    i2c_port_num_t i2c_port;
    uint8_t i2c_addr;

    gpio_num_t scl_pin;
    gpio_num_t sda_pin;
    gpio_num_t int_pin;

    uint32_t i2c_speed_hz;
    uint32_t report_interval_us;

    uint32_t reset_wait_ms;
    uint32_t startup_delay_ms;

    uint32_t task_stack_size;
    UBaseType_t task_priority;
} sh2service_config_t;

esp_err_t sh2service_start(sh2service_callback_t callback, void *ctx);

esp_err_t sh2service_stop(void);

bool sh2service_is_running(void);

#endif