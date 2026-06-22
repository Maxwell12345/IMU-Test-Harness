#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_util.h"
#include "shtp.h"
#include "esp_log_level.h"
#include <serial.h>


void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);

    // printf("ESP32 SH2 mock startup\n");
    //
    // sh2_ProductIds_t productIds{};
    // sh2_SensorValue_t sensorValue{};
    //
    // printf("sh2_ProductIds_t size: %u\n", static_cast<unsigned>(sizeof(productIds)));
    // printf("sh2_SensorValue_t size: %u\n", static_cast<unsigned>(sizeof(sensorValue)));

    ESP_ERROR_CHECK(host_serial_init());

    while (true) {
        acceleration_t acceleration = {
            1.0f,
            2.0f,
            3.0f,
            56
        };

        // 0x00 00 80 3f
        // 0x00 00 00 40
        // 0x00 00 40 40,
        // 0x00 00 00 00 00 00 00 38

        send_acceleration_t(&acceleration);

        // uint8_t buffer[ACCELERATION_MSG_BYTES] = {
        //     0xff, 0x01, 0x14,
        //     0x00, 0x00, 0x80, 0x3f,
        //     0x00, 0x00, 0x00, 0x40,
        //     0x00, 0x00, 0x40, 0x40,
        //     0xbb, 0xaa, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44,
        //     0x0a, 0x20
        // };

        // ESP_ERROR_CHECK(host_serial_write_all(buffer, ACCELERATION_MSG_BYTES));



        // printf("ff00140000803f0000004000004040bbaa99887766554467f8");
        // printf("%s", buffer);
        // ESP_ERROR_CHECK(send_acceleration_t(&acceleration));

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
