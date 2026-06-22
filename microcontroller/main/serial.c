/******************************************************************************
 * File:             serial.c
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/18/26 2:10 PM
 *
 ******************************************************************************/
#include "serial.h"




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

esp_err_t send_acceleration_t(const acceleration_t *acceleration) {
    if (acceleration == NULL) return ESP_ERR_INVALID_ARG;
    unsigned char buffer[ACCELERATION_MSG_BYTES] = {0};

    buffer[MAGIC_BYTE_IDX] = 0xFF;
    buffer[MSG_TYPE_IDX] = ACCELERATION_T_ID;
    buffer[PAYLOAD_LEN_IDX] = ACCELERATION_PAYLOAD_BYTES;

    memcpy(buffer+PAYLOAD_START_IDX, &acceleration->x, sizeof(float));
    memcpy(buffer+PAYLOAD_START_IDX+sizeof(float), &acceleration->y, sizeof(float));
    memcpy(buffer+PAYLOAD_START_IDX+(2*sizeof(float)), &acceleration->z, sizeof(float));
    memcpy(buffer+PAYLOAD_START_IDX+(3*sizeof(float)), &acceleration->timestamp, sizeof(uint64_t));

    uint16_t crc = calculate_crc16_ccitt_false(buffer, ACCELERATION_CRC_IDX);

    buffer[ACCELERATION_CRC_IDX] = (uint8_t)((crc >> 8) & 0xFFu);
    buffer[ACCELERATION_CRC_IDX+1] = (uint8_t)(crc & 0xFFu);

    return host_serial_write_all(buffer, ACCELERATION_MSG_BYTES);
}

