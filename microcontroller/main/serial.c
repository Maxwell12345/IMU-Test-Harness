/******************************************************************************
 * File:             serial.c
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/18/26 2:10 PM
 *
 ******************************************************************************/
#include "serial.h"

#define MAGIC_ENCODER 0xA5

static uint16_t calculate_crc16_ccitt_false(const unsigned char *data, size_t length)
{
    cm_t cm;

    cm.cm_width = 16;
    cm.cm_poly = 0x1021L;
    cm.cm_init = 0xFFFFL;
    cm.cm_refin = FALSE;
    cm.cm_refot = FALSE;
    cm.cm_xorot = 0x0000L;

    cm_ini(&cm);
    cm_blk(&cm, data, (unsigned long)length);

    return (uint16_t)(cm_crc(&cm) & 0xFFFFu);
}

esp_err_t host_serial_init(void)
{
    const uart_config_t config = {
        .baud_rate = HOST_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        HOST_UART,
        RX_BUFFER_SIZE,
        TX_BUFFER_SIZE,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(HOST_UART, &config));

    /*
     * For UART0 on most ESP32 dev boards, leave pins unchanged.
     * UART0 TX/RX are already wired to the USB-to-UART bridge.
     */
    ESP_ERROR_CHECK(uart_set_pin(
        HOST_UART,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    return ESP_OK;
}

esp_err_t host_serial_write_all(const void *data, size_t length)
{
    const uint8_t *cursor = (const uint8_t *)data;
    size_t remaining = length;

    while (remaining > 0) {
        int written = uart_write_bytes(HOST_UART, cursor, remaining);
        if (written <= 0) {
            return ESP_FAIL;
        }

        cursor += written;
        remaining -= (size_t)written;
    }

    return uart_wait_tx_done(HOST_UART, pdMS_TO_TICKS(1000));
}

esp_err_t send_fieldwise_acceleration_t(const acceleration_t *acceleration) {
    if (acceleration == NULL) return ESP_ERR_INVALID_ARG;
    unsigned char buffer[ACCELERATION_MSG_BYTES] = {0};
    size_t length = 0;

    buffer[length++] = MAGIC_ENCODER;
    buffer[length++] = ACCELERATION_T_ID;
    buffer[length++] = ACCELERATION_PAYLOAD_BYTES;

    memcpy(buffer+length, &acceleration->x, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &acceleration->y, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &acceleration->z, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &acceleration->timestamp, sizeof(uint64_t));
    length += sizeof(uint64_t);

    uint16_t crc = calculate_crc16_ccitt_false(buffer, length);

    buffer[length++] = (uint8_t)((crc >> 8) & 0xFFu);
    buffer[length++] = (uint8_t)(crc & 0xFFu);

    return host_serial_write_all(buffer, length);
}

esp_err_t send_fieldwise_rotation_t(const rotation_t *rotation) {
    if (rotation == NULL) return ESP_ERR_INVALID_ARG;
    unsigned char buffer[ROTATION_MSG_BYTES] = {0};
    size_t length = 0;

    buffer[length++] = MAGIC_ENCODER;
    buffer[length++] = ROTATION_T_ID;
    buffer[length++] = ROTATION_PAYLOAD_BYTES;

    memcpy(buffer+length, &rotation->i, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &rotation->j, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &rotation->k, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &rotation->real, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &rotation->accuracy, sizeof(float));
    length += sizeof(float);
    memcpy(buffer+length, &rotation->timestamp, sizeof(uint64_t));
    length += sizeof(uint64_t);

    uint16_t crc = calculate_crc16_ccitt_false(buffer, length);

    buffer[length++] = (uint8_t)((crc >> 8) & 0xFFu);
    buffer[length++] = (uint8_t)(crc & 0xFFu);

    return host_serial_write_all(buffer, length);
}

esp_err_t send_acceleration_t(const acceleration_t *acceleration) {
    if (acceleration == NULL) return ESP_ERR_INVALID_ARG;
    unsigned char buffer[50] = {0};
    size_t length = 0;

    buffer[length++] = MAGIC_ENCODER;
    buffer[length++] = ACCELERATION_T_ID;
    buffer[length++] = ACCELERATION_PAYLOAD_BYTES;

    memcpy(buffer+length, acceleration, sizeof(acceleration_t));
    length += sizeof(acceleration_t);

    uint16_t crc = calculate_crc16_ccitt_false(buffer, length);

    buffer[length++] = (uint8_t)((crc >> 8) & 0xFFu);
    buffer[length++] = (uint8_t)(crc & 0xFFu);

    return host_serial_write_all(buffer, length);
}

esp_err_t send_rotation_t(const rotation_t *rotation) {
    if (rotation == NULL) return ESP_ERR_INVALID_ARG;
    unsigned char buffer[50] = {0};
    size_t length = 0;

    buffer[length++] = MAGIC_ENCODER;
    buffer[length++] = ROTATION_T_ID;
    buffer[length++] = ROTATION_PAYLOAD_BYTES;

    memcpy(buffer+length, rotation, sizeof(rotation_t));
    length += sizeof(rotation_t);

    uint16_t crc = calculate_crc16_ccitt_false(buffer, length);

    buffer[length++] = (uint8_t)((crc >> 8) & 0xFFu);
    buffer[length++] = (uint8_t)(crc & 0xFFu);

    return host_serial_write_all(buffer, length);
}