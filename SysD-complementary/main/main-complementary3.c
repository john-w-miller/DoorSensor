#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"

static const char *TAG = "APP_MAIN";

// look good at 0
// 45 degree is long, ie 50 but not bad
// 90+ looks good

// requires the user to calibrate at 90 degrees

#define SAMPLE_RATE_HZ    50
#define SAMPLE_PERIOD_S   (1.0f / SAMPLE_RATE_HZ)
#define SAMPLE_PERIOD_MS  (1000 / SAMPLE_RATE_HZ)
#define RAD_TO_DEG        (180.0f / M_PI)

void app_main(void)
{
    esp_err_t ret;
    imu_comp_data_t comp;

    // 1) Initialize IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed: %d", ret);
        return;
    }

    // 2) Calibrate biases (gyro & accel)
    imu_calibrate(50, 1);
    imu_calibrate(2000, 5);
    ESP_LOGI(TAG, "IMU calibration complete");

    // 3) Read initial magnetometer heading as zero reference
    ret = imu_read_normalized_data(&comp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read normalized data: %d", ret);
        return;
    }
//    float mag0 = atan2f(comp.norm_mag[1], comp.norm_mag[0]) * RAD_TO_DEG;
//    ESP_LOGI(TAG, "Initial mag heading= %6.2f°", mag0);
///////////////
    printf("Close door to 0° and press Enter to calibrate...\n");
    
    ret = imu_read_normalized_data(&comp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read for 0° cal: %d", ret);
        return;
    }
    float mag0 = atan2f(comp.norm_mag[1], comp.norm_mag[0]) * RAD_TO_DEG;
    ESP_LOGI(TAG, "Calibrated mag0 = %6.2f°", mag0);

    //    b) Fully open (90°)
    printf("Open door to 90° and press Enter to calibrate...\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for user to open door
    ret = imu_read_normalized_data(&comp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read for 90° cal: %d", ret);
        return;
    }
    float mag90 = atan2f(comp.norm_mag[1], comp.norm_mag[0]) * RAD_TO_DEG;
    ESP_LOGI(TAG, "Calibrated mag90 = %6.2f°", mag90);

    //    c) Compute fixed scale
    float scale = 90.0f / (mag90 - mag0);
    ESP_LOGI(TAG, "Using static scale = %5.3f", scale);
/////////////////////////

    // 4) Dynamic in-field scale calibration variables
    // float mag_min =  360.0f;
    // float mag_max = -360.0f;
    // float scale   = 1.0f;  // maps raw mag span to 90°

    // 5) Complementary filter parameters <<<<<<<<<<<
    const float dt    = SAMPLE_PERIOD_S;
    const float tau   = 0.5f;                       // time constant (s)
    const float alpha = tau / (tau + dt);
    float theta_est   = 0.0f;                       // estimated door angle (°)

    // 6) Main loop: fuse gyro & mag with dynamic calibration
    while (true) {
        ret = imu_read_normalized_data(&comp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Sensor read error: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        // a) Gyro integration (Z axis)
        float gyro_dps = comp.norm_gyro[2] * RAD_TO_DEG;        // deg/s
        float pred     = theta_est + gyro_dps * dt;

        // b) Magnetometer heading
        float mag_heading = atan2f(comp.norm_mag[1], comp.norm_mag[0]) * RAD_TO_DEG;
        float raw_delta   = mag_heading - mag0;
        // wrap raw_delta to [-180,180]
        if      (raw_delta > 180.0f) raw_delta -= 360.0f;
        else if (raw_delta < -180.0f) raw_delta += 360.0f;

        // c) Update min/max for dynamic calibration
        // if (raw_delta < mag_min) mag_min = raw_delta;
        // if (raw_delta > mag_max) mag_max = raw_delta;
        // float span = mag_max - mag_min;
        // if (span > 1e-3f) {
        //     scale = 90.0f / span;
        // }

        // d) Apply scale to delta_mag
        float delta_mag = raw_delta * scale;

        // e) Complementary blend
        theta_est = alpha * pred + (1.0f - alpha) * delta_mag;

        // f) Log calibration and result
        // ESP_LOGI(TAG,
        //     "raw= %6.2f, span= %6.2f, scale= %5.3f, angle= %6.2f°",
        //     raw_delta, span, scale, theta_est );

         ESP_LOGI(TAG,
            "raw= %6.2f, scale= %5.3f, angle= %6.2f°",
            raw_delta,  scale, theta_est
        );       

        // g) Delay to maintain sample rate
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
