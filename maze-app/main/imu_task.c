#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "imu_task.h"

static const char *TAG = "IMU_TASK";

void init_imu(void)
{
    xImuSemaphore = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(imu_handler_task, "ImuTask", 2048, NULL, 1, &IMU_handle, 0);
    xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
    ax_buf = get_buffer();
    ay_buf = get_buffer();
    az_buf = get_buffer();
    gx_buf = get_buffer();
    gy_buf = get_buffer();
    gz_buf = get_buffer();
    xSemaphoreGive(xImuSemaphore);
}

void imu_handler_task(void *pvParameters)
{
    ESP_LOGI(TAG, "recording IMU");
    float gx, gy, gz;
    float ax, ay, az;
    for (;;)
    {
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
        push(ax_buf, ax);
        push(ay_buf, ay);
        push(az_buf, az);

        push(gx_buf, gx);
        push(gy_buf, gy);
        push(gz_buf, gz);
        xSemaphoreGive(xImuSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}
