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
#define ROTATION_PAYLOAD_BYTES (5*sizeof(float)+sizeof(uint64_t)+1)
#define ROTATION_MSG_BYTES (1+1+1+ROTATION_PAYLOAD_BYTES+2)

#define ACCELERATION_T_ID 1
#define ROTATION_T_ID 2
#define ERROR_MSG_ID 3

#define MAGIC_BYTE_IDX 0
#define MSG_TYPE_IDX 1
#define PAYLOAD_LEN_IDX 2
#define PAYLOAD_START_IDX 3
#define ACCELERATION_CRC_IDX (PAYLOAD_START_IDX+ACCELERATION_PAYLOAD_BYTES)

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
