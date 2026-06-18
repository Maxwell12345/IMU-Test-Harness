#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_util.h"
#include "shtp.h"

void app_main(void) {
    printf("ESP32 SH2 mock startup\n");

    sh2_ProductIds_t productIds{};
    sh2_SensorValue_t sensorValue{};

    printf("sh2_ProductIds_t size: %u\n", static_cast<unsigned>(sizeof(productIds)));
    printf("sh2_SensorValue_t size: %u\n", static_cast<unsigned>(sizeof(sensorValue)));

    while (true) {
        printf("SH2 linked and app running\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}