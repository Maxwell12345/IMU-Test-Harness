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

    acceleration_t acceleration = {
        1.0f,
        2.0f,
        3.0f,
        0x445566778899AABB
    };

    ESP_ERROR_CHECK(send_acceleration_t(&acceleration));
}
