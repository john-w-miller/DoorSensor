/*
 * main.c
 *
 * Standalone test harness for the door-angle EKF module.
 * Requirements:
 *   - FreeRTOS + ESP-IDF
 *   - imu.c / imu.h
 *   - door_angle_ekf.c / door_angle_ekf.h
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "ekf.h"

static const char* TAG = "MAIN";

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "=== Door Angle EKF Demo ===");

    // 1) Initialize IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed (%d)", ret);
        return;
    }

    // 2) Calibrate gyro bias: 2000 samples, 5-second warmup
    ESP_LOGI(TAG, "Calibrating gyro bias...");
    imu_calibrate(50, 1);    // flushing garbage
    imu_calibrate(2000, 5);

    // 3) Initialize EKF
    ESP_LOGI(TAG, "Initializing EKF...");
    ekf_init();

    // 4) Spawn the door-position task
    ESP_LOGI(TAG, "Starting door_position_task...");
    BaseType_t ok = xTaskCreate(
        door_position_task,   // Task entry function
        "door_pos",         // Task name
        4096,                // Stack size (bytes)
        NULL,                // Task parameter
        tskIDLE_PRIORITY + 5,// Priority
        NULL                 // Task handle
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create door_position_task");
        return;
    }

    // app_main returns; RTOS scheduler is already running
    ESP_LOGI(TAG, "Setup complete. Running...");
}
