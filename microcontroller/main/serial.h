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

#define HOST_UART        UART_NUM_0
#define HOST_UART_BAUD   115200

#define RX_BUFFER_SIZE   512
#define TX_BUFFER_SIZE   512

// magic (1B) + type (1B) + payloadLen (1B) + payload (N B) + checksum (2B)
#define ACCELERATION_PAYLOAD_BYTES (3*sizeof(float)+sizeof(uint64_t))
#define ACCELERATION_MSG_BYTES (1+1+1+ACCELERATION_PAYLOAD_BYTES+2)
#define ROTATION_PAYLOAD_BYTES (5*sizeof(float)+sizeof(uint64_t))
#define ROTATION_MSG_BYTES (1+1+1+ROTATION_PAYLOAD_BYTES+2)
#define ROTATION_RATE_PAYLOAD_BYTES (3*sizeof(float)+sizeof(uint64_t))
#define ROTATION_RATE_MSG_BYTES (1+1+1+ROTATION_RATE_PAYLOAD_BYTES+2)
#define STATUS_PAYLOAD_BYTES (1+8)
#define STATUS_MSG_BYTES (1+1+1+STATUS_PAYLOAD_BYTES+2)

#define MAGIC_BYTE_IDX 0
#define MSG_TYPE_IDX 1
#define PAYLOAD_LEN_IDX 2
#define PAYLOAD_START_IDX 3
#define ACCELERATION_CRC_IDX (PAYLOAD_START_IDX+ACCELERATION_PAYLOAD_BYTES)
#define ROTATION_CRC_IDX (PAYLOAD_START_IDX+ROTATION_PAYLOAD_BYTES)

enum message_type_id {
    ACCELERATION_ID = 1,
    ROTATION_VECTOR_ID = 2,
    ROTATION_RATE_ID = 3,
    STATUS_ID = 4,
    COMMAND_ID = 5
};

#pragma pack(push, 1)
typedef struct acceleration_t {
    float x;
    float y;
    float z;
    uint64_t timestamp;
} acceleration_t;

typedef struct rotation_t {
    float i;
    float j;
    float k;
    float real;
    float accuracy;
    uint64_t timestamp;
} rotation_t;

typedef struct rotation_rate_t {
    float d_roll;
    float d_pitch;
    float d_raw;
    uint64_t timestamp;
} rotation_rate_t;
#pragma pack(pop)

enum micro_command_id {
    IMU_RESET_CMD = 0,
};

enum imu_status {
    INITIALIZING=0,
    SENSOR_INITIALIZATION_ERROR,
    HEALTHY,
    IMU_UNAVAILABLE,
    NO_DATA_RECEIVED_AFTER_BOOT,
    IMU_RESET_FAILURE,
    NO_DETECTED_IMU,
    MICRO_COMMAND_ERROR,
};

typedef struct processor_status_t {
    enum imu_status status;
    uint64_t timestamp;
} processor_status_t;

typedef void (*receive_data_callback)(uint8_t command);

esp_err_t start_listening(receive_data_callback);

esp_err_t host_serial_init(void);

esp_err_t host_serial_write_all(const void *data, size_t length);

esp_err_t send_fieldwise_acceleration_t(const acceleration_t *acceleration);

esp_err_t send_fieldwise_rotation_t(const rotation_t *rotation);

esp_err_t send_acceleration_t(const acceleration_t *acceleration);

esp_err_t send_rotation_t(const rotation_t *rotation);

esp_err_t send_rotation_rate_t(const rotation_rate_t *rotationRate);

esp_err_t send_status_t(const processor_status_t *processor_status);

void serial_command_callback(uint8_t command);

#endif //IMU_TEST_HARNESS_SERIAL_H
