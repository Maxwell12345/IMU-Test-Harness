#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
    while (true) {
        printf("Hello from ESP32\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}