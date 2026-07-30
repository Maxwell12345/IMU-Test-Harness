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
#include <math.h>

static double current_roll;
static double current_pitch;
static int have_attitude;

static void imu_callback(const sh2service_event_t *event, void *ctx)
{
    if (event->type == SH2SERVICE_LINEAR_ACCELERATION) {
        acceleration_t accel = {
            event->data.linear_acceleration.x,
            event->data.linear_acceleration.y,
            event->data.linear_acceleration.z,
            (unsigned long long)event->timestamp_us
        };
        // send_acceleration_t(&accel);
        printf("LA,%llu,%f,%f,%f\n",
               (unsigned long long)event->timestamp_us,
               event->data.linear_acceleration.x,
               event->data.linear_acceleration.y,
               event->data.linear_acceleration.z);
        return;
    }

    if (event->type == SH2SERVICE_ROTATION_VECTOR) {
        const double x = event->data.rotation_vector.i;
        const double y = event->data.rotation_vector.j;
        const double z = event->data.rotation_vector.k;
        const double w = event->data.rotation_vector.real;

        const double norm = sqrt(x * x + y * y + z * z + w * w);
        if (norm <= 0.0) {
            return;
        }

        const double qx = x / norm;
        const double qy = y / norm;
        const double qz = z / norm;
        const double qw = w / norm;

        const double sin_roll = 2.0 * (qw * qx + qy * qz);
        const double cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy);
        current_roll = atan2(sin_roll, cos_roll);

        const double sin_pitch = 2.0 * (qw * qy - qz * qx);
        if (fabs(sin_pitch) >= 1.0) {
            current_pitch = copysign(M_PI / 2.0, sin_pitch);
        } else {
            current_pitch = asin(sin_pitch);
        }

        have_attitude = 1;
        // send_rotation_t(&rotation);
        printf("RV,%llu,%f,%f,%f,%f,%f\n",
            (unsigned long long)event->timestamp_us,
            event->data.rotation_vector.i,
            event->data.rotation_vector.j,
            event->data.rotation_vector.k,
            event->data.rotation_vector.real,
            event->data.rotation_vector.accuracy);

        return;
    }
            

    if (event->type == SH2SERVICE_GYROSCOPE) {
        if (!have_attitude) {
            return;
        }

        const double p = event->data.gyroscope.x;
        const double q = event->data.gyroscope.y;
        const double r = event->data.gyroscope.z;

        const double sin_roll = sin(current_roll);
        const double cos_roll = cos(current_roll);
        const double cos_pitch = cos(current_pitch);

        if (fabs(cos_pitch) < 1e-6) {
            return;
        }

        const double tan_pitch = tan(current_pitch);

        const double d_roll = p + (q * sin_roll + r * cos_roll) * tan_pitch;
        const double d_pitch = q * cos_roll - r * sin_roll;
        const double d_yaw = (q * sin_roll + r * cos_roll) / cos_pitch;

        const rotation_rate_t rotationRate {
            d_roll,
            d_pitch,
            d_yaw
        };
        send_rotation_rate_t(rotationRate);

        printf("DRPY,%llu,%f,%f,%f\n",
            (unsigned long long)event->timestamp_us,
            d_roll,
            d_pitch,
            d_yaw);

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

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}