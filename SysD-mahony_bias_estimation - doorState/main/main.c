// main.c
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "mahony.h"
#include "door_state.h"

static const char *TAG = "APP_MAIN";

// I²C & IMU configuration (from your original main.c) :contentReference[oaicite:0]{index=0}
#define IMU_I2C_NUM I2C_NUM_0
#define IMU_I2C_TIMEOUT_MS pdMS_TO_TICKS(1000)

#define GRAVITY 9.80665f
#define RAD_TO_DEG (180.0f / M_PI)

// Sampling configuration :contentReference[oaicite:1]{index=1}
#define SAMPLE_RATE_HZ 50
#define SAMPLE_PERIOD_S (1.0f / SAMPLE_RATE_HZ)
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)

/**
 * @brief  FreeRTOS task: read IMU, update Mahony filter,
 *         update door state & publish/log.
 */
static void imu_task(void *arg)
{
    esp_err_t ret;

    // 1) Initialize IMU (LSM6DS3TR + LSM303AGR)
    ret = imu_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU init failed: %d", ret);
        vTaskDelete(NULL);
        // TODO: send mqtt error message 
        return;
    }

    // 2) Mahony AHRS setup (6-DOF)
    mahony_init(SAMPLE_PERIOD_S,
                /*Kp=*/0.5f,
                /*Ki=*/0.0f);

    // 3) IMU calibration (throw away settling readings)
    imu_calibrate(50, 1);
    imu_calibrate(2000, 5);
    ESP_LOGI(TAG, "IMU calibration complete");

    // 4) Initialize door‐state FSM
    ret = door_state_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "door_state init failed: %d", ret);
        vTaskDelete(NULL);
        // TODO: send mqtt error message 
        return;
    }

    float last_bx = 0.0f;
    float last_by = 0.0f;
    float last_bz = 0.0f;
    float prev_yaw = 0.0f;
    float yaw, pitch, roll;

    // --- hourly yaw publish timer ---
    TickType_t last_hourly_ts    = xTaskGetTickCount();

    while (1) {

        TickType_t now_time = xTaskGetTickCount();

        // 1) Hourly yaw publish (instead of reset)
        if ((now_time - last_hourly_ts) >= pdMS_TO_TICKS(3600000)) {
            // get the latest filtered yaw
            mahony_get_euler_lpf(&yaw, &pitch, &roll);

            // format as JSON (or plain) and publish over MQTT
            char msg[64];
            int len = snprintf(msg, sizeof(msg), "{\"yaw\":%.2f}", yaw * RAD_TO_DEG);
            // esp_mqtt_client_publish(mqtt_client,
            //                         "door/yaw",   // your topic
            //                         msg,
            //                         len,
            //                         1,            // QoS
            //                         0);           // non-retained
            ESP_LOGI(TAG, "Hourly Yaw Sent: %s", msg);

            last_hourly_ts = now_time;
        }

        imu_l6_comp_norm_t imu;
        if (imu_read_l6_combined(&imu) == ESP_OK)
        {
            //  1) FSM first, on last cycle’s yaw and imu.comp_gyro[]
            door_state_update(prev_yaw,
                              imu.norm_accel[0],
                              imu.norm_accel[1],
                              imu.norm_accel[2],
                              imu.comp_gyro[0] - last_bx,
                              imu.comp_gyro[1] - last_by,
                              imu.comp_gyro[2] - last_bz);

            // 2) Mahony fusion with imu.norm_gyro[]
            mahony_ahrs_6d_update(imu.norm_gyro[0],
                                  imu.norm_gyro[1],
                                  imu.norm_gyro[2],
                                  imu.norm_accel[0],
                                  imu.norm_accel[1],
                                  imu.norm_accel[2]);

            // 3) Extract new yaw & bias
            mahony_get_euler_lpf(&yaw, &pitch, &roll);

            mahony_get_gyro_bias(&last_bx, &last_by, &last_bz);

            // f) Log for debug
            // ESP_LOGI(TAG,
            //          "Yaw=%6.2f°  Pitch=%6.2f°  Roll=%6.2f°",
            //          yaw*RAD_TO_DEG,
            //          pitch*RAD_TO_DEG,
            //          roll*RAD_TO_DEG
            // );

            // 4) Prepare for next iteration
            prev_yaw = yaw;
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        }
        else
        {
            // TOD set some flag to indicate IMU read failure
            ESP_LOGE(TAG, "IMU read failed");
            vTaskDelay(pdMS_TO_TICKS(1000)); // wait before retrying
        }
    }
}

void app_main(void)
{

    esp_log_level_set("imu_filter", ESP_LOG_DEBUG);
    
    // Create the IMU task on core 0, priority 5, 4 KiB stack
    xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        NULL,
        5,
        NULL,
        0);
}
