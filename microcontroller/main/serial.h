/******************************************************************************
 * File:             serial.h
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/18/26 2:10 PM
 * Description:      Briefly describe the purpose of this file and its role within
 *                   the project. Mention key functions or classes it Contains.
 *
 ******************************************************************************/
#ifndef IMU_TEST_HARNESS_SERIAL_H
#define IMU_TEST_HARNESS_SERIAL_H

#include "crc.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct acceleration_t {
    float x;
    float y;
    float z;
    uint64_t timestamp;
} acceleration_t;

esp_err_t host_serial_init(void);

esp_err_t host_serial_write_all(const void *data, size_t length);

esp_err_t send_acceleration_t(const acceleration_t *acceleration);

#endif //IMU_TEST_HARNESS_SERIAL_H
