#include <stdio.h>
#include <math.h>
#include <float.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"

static const char *TAG = "APP_MAIN";

// starts @ 170 door closed
// hits -900 and 90 degrees
// continues to increase past 90 degrees up 1100

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

    // 3) Read initial magnetometer for zero reference
    ret = imu_read_normalized_data(&comp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read normalized data: %d", ret);
        return;
    }
    // Closed-door reference heading
    float mag0 = atan2f(comp.norm_mag[1], comp.norm_mag[0]) * RAD_TO_DEG;
    if (mag0 < 0) mag0 += 360.0f;
    ESP_LOGI(TAG, "Initial mag heading= %.2f°", mag0);

    // 4) Prepare in-field mag calibration (hard & soft iron)
    float mag_min[3] = { comp.norm_mag[0], comp.norm_mag[1], comp.norm_mag[2] };
    float mag_max[3] = { comp.norm_mag[0], comp.norm_mag[1], comp.norm_mag[2] };
    float field_min  =  FLT_MAX;
    float field_max  = -FLT_MAX;

    // 5) Complementary filter parameters
    const float dt    = SAMPLE_PERIOD_S;
    const float tau   = 0.5f;                       // time constant (s)
    const float alpha = tau / (tau + dt);
    float theta_est   = 0.0f;                       // estimated door angle (°)

    // 6) Main loop: gyro + tilt-compensated & calibrated mag
    while (true) {
        ret = imu_read_normalized_data(&comp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Sensor read error: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        // a) Gyro integration (Z axis)
        float gyro_dps = comp.norm_gyro[2] * RAD_TO_DEG;
        float pred     = theta_est + gyro_dps * dt;

        // b) Hard-iron: update mag_min/max per axis
        mag_min[0] = fminf(mag_min[0], comp.norm_mag[0]);
        mag_min[1] = fminf(mag_min[1], comp.norm_mag[1]);
        mag_min[2] = fminf(mag_min[2], comp.norm_mag[2]);
        mag_max[0] = fmaxf(mag_max[0], comp.norm_mag[0]);
        mag_max[1] = fmaxf(mag_max[1], comp.norm_mag[1]);
        mag_max[2] = fmaxf(mag_max[2], comp.norm_mag[2]);
        // Offset removal
        float mx_off = comp.norm_mag[0] - (mag_min[0] + mag_max[0]) * 0.5f;
        float my_off = comp.norm_mag[1] - (mag_min[1] + mag_max[1]) * 0.5f;
        float mz_off = comp.norm_mag[2] - (mag_min[2] + mag_max[2]) * 0.5f;

        // c) Soft-iron: track field magnitude extremes
        float field = sqrtf(mx_off*mx_off + my_off*my_off + mz_off*mz_off);
        field_min = fminf(field_min, field);
        field_max = fmaxf(field_max, field);
        float scale = (field_max - field_min) > 1e-6f ? 90.0f / (field_max - field_min) : 1.0f;

        // d) Tilt-compensate calibrated mag
        float ax = comp.norm_accel1[0];
        float ay = comp.norm_accel1[1];
        float az = comp.norm_accel1[2];
        float roll  = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
        float cos_r = cosf(roll), sin_r = sinf(roll);
        float cos_p = cosf(pitch), sin_p = sinf(pitch);
        float mx_t  = mx_off * cos_p + mz_off * sin_p;
        float my_t  = mx_off * sin_r * sin_p + my_off * cos_r - mz_off * sin_r * cos_p;

        // e) Compute heading delta
        float mag_heading = atan2f(my_t, mx_t) * RAD_TO_DEG;
        if (mag_heading < 0) mag_heading += 360.0f;
        float raw_delta   = mag_heading - mag0;
        if      (raw_delta > 180.0f) raw_delta -= 360.0f;
        else if (raw_delta < -180.0f) raw_delta += 360.0f;
        float angle_mag = raw_delta * scale;

        // f) Complementary blend
        theta_est = alpha * pred + (1.0f - alpha) * angle_mag;

        // g) Log calibration & result
        ESP_LOGI(TAG,
            "raw=%.2f off=(%.2f,%.2f) span=(%.2f) scale=%.3f angle=%.2f°",
            raw_delta,
            (mag_min[0]+mag_max[0])*0.5f - comp.norm_mag[0],
            (mag_min[1]+mag_max[1])*0.5f - comp.norm_mag[1],
            field_max - field_min,
            scale,
            theta_est
        );

        // h) Delay
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
