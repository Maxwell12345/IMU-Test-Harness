#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "sh2service.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_util.h"
#include "shtp.h"
#include "esp_log_level.h"
#include <serial.h>

static void imu_callback(const sh2service_event_t *event, void *ctx)
{
    if (event->type == SH2SERVICE_LINEAR_ACCELERATION) {
        printf("LA,%llu,%f,%f,%f\n",
               (unsigned long long)event->timestamp_us,
               event->data.linear_acceleration.x,
               event->data.linear_acceleration.y,
               event->data.linear_acceleration.z);
        return;
    }

    if (event->type == SH2SERVICE_ROTATION_VECTOR) {
        printf("RV,%llu,%f,%f,%f,%f,%f\n",
               (unsigned long long)event->timestamp_us,
               event->data.rotation_vector.i,
               event->data.rotation_vector.j,
               event->data.rotation_vector.k,
               event->data.rotation_vector.real,
               event->data.rotation_vector.accuracy);
        return;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(host_serial_init());
    setvbuf(stdout, NULL, _IONBF, 0);
    esp_log_level_set("*", ESP_LOG_NONE);

    printf("\n\n\n");
    printf("BOOT,APP_MAIN\n");

    esp_err_t err = sh2service_start(imu_callback, NULL);
    if (err != ESP_OK) {
        printf("ERR,SH2SERVICE_START,%d\n", err);
        return;
    }

    printf("BEGIN_IMU_CSV\n");
    printf("TYPE,TIME_US,A,B,C,D,E\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}