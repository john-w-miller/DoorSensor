// door_position.c
// Provides a production-ready API for initializing the door-angle AHRS,
// reporting the current door angle relative to the closed position.

#include "door_position.h"
#include "imu.h"                // imu_init, imu_calibrate, imu_read_compensated_data
#include "mahony.h"             // mahony_init, mahony_update, get_yaw, normalize_angle
#include <esp_log.h>
#include <math.h>

static const char *TAG = "door_position";

// Filter and timing parameters
#define FILTER_INIT_ITERS      200      // number of initial AHRS update iterations
#define DT_SEC                 (10e-3f) // loop period in seconds (10 ms)

// Closed-door yaw reference (radians)
static float calibration_offset = 0.0f;

esp_err_t door_position_init(void)
{

    ESP_LOGI(TAG, "Initializing door position...");

    esp_err_t err = imu_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed (0x%02X)", err);
        return err;
    }

    // Full sensor calibration and alignment
    err = imu_calibrate(DEFAULT_CAL_SAMPLES, DEFAULT_CAL_DELAY_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU calibration failed (0x%02X)", err);
        return err;
    }

    // Initialize the Mahony AHRS filter with default gains
    mahony_init(DEFAULT_MAHONY_KP, DEFAULT_MAHONY_KI);

    // Let the filter converge
    imu_comp_data_t cd;
    for (int i = 0; i < FILTER_INIT_ITERS; ++i) {
        if (imu_read_compensated_data(&cd) == ESP_OK) {
            mahony_update(
                cd.gyro_x, cd.gyro_y, cd.gyro_z,
                cd.accel1_x, cd.accel1_y, cd.accel1_z,
                cd.mag_x, cd.mag_y, cd.mag_z,
                DT_SEC
            );
        }
        float raw_yaw = get_yaw();
        ESP_LOGD(TAG, "init[%3d]: raw_yaw=%.3f°", i, raw_yaw*180.0f/M_PI);

        ESP_LOGD(TAG, "INI: ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f | mx=%.3f my=%.3f mz=%.3f",
                cd.accel1_x, cd.accel1_y, cd.accel1_z,
                cd.gyro_x, cd.gyro_y, cd.gyro_z,
                cd.mag_x, cd.mag_y, cd.mag_z);
    }

    // Capture closed-door yaw as reference
    calibration_offset = get_yaw();
    ESP_LOGI(TAG, "Door reference calibrated: offset=%.3f rad (%.1f°)",
             calibration_offset,
             calibration_offset * (180.0f/M_PI));
    return ESP_OK;
}

float door_position_get_angle(void)
{


    imu_comp_data_t cd;
    if (imu_read_compensated_data(&cd) != ESP_OK) {
        ESP_LOGW(TAG, "IMU read failed");
        return NAN;
    }

    // DEBUG: raw sensor data
    ESP_LOGD(TAG, "RAW: ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f | mx=%.3f my=%.3f mz=%.3f",
             cd.accel1_x, cd.accel1_y, cd.accel1_z,
             cd.gyro_x, cd.gyro_y, cd.gyro_z,
             cd.mag_x, cd.mag_y, cd.mag_z);


    // AHRS filter update
    mahony_update(
        cd.gyro_x, cd.gyro_y, cd.gyro_z,
        cd.accel1_x, cd.accel1_y, cd.accel1_z,
        cd.mag_x, cd.mag_y, cd.mag_z,
        DT_SEC
    );

    // DEBUG: raw yaw from filter
    float raw_yaw = get_yaw();
    ESP_LOGD(TAG, "RAW_YAW=%.3f rad (%.1f°)", raw_yaw, raw_yaw * (180.0f/M_PI));

    // Compute door angle relative to reference
    float angle = normalize_angle(raw_yaw - calibration_offset);
    return angle;
}

void door_position_reset_reference(void)
{
    calibration_offset = get_yaw();
    ESP_LOGI(TAG, "Door reference reset: offset=%.3f rad (%.1f°)",
             calibration_offset,
             calibration_offset * (180.0f/M_PI));
}
