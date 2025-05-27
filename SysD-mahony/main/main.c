#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "mahony.h"
#include <math.h>

static const char *TAG = "APP_MAIN";


#define IMU_I2C_NUM        I2C_NUM_0
#define IMU_I2C_TIMEOUT_MS pdMS_TO_TICKS(1000)
#define RAD_TO_DEG  (180.0f / M_PI)

// -- Sampling configuration --
#define SAMPLE_RATE_HZ    50
#define SAMPLE_PERIOD_S   (1.0f / SAMPLE_RATE_HZ)
#define SAMPLE_PERIOD_MS  (1000 / SAMPLE_RATE_HZ)

void app_main(void)
{

    // Initialize both LSM6DS3TR and LSM303AGR
    esp_err_t ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed: %d", ret);
        return;
    }

    // configure Euler-angle low-pass filter (0=no smoothing, 1=hold)
    // initialize Mahony filter with configured sample period
    //mahony_init(SAMPLE_PERIOD_S, /*Kp=*/ 25.0f, /*Ki=*/ 0.01f);
    mahony_init(SAMPLE_PERIOD_S, /*Kp=*/ 0.5f, /*Ki=*/ 0.0f);
    imu_calibrate(50, 1); //   IMPORTANT!!!! allow IMU to settle, throw away cal  
    imu_calibrate(2000, 5);

    ESP_LOGI(TAG, "IMU calibration complete");

    float qw, qx, qy, qz;
    imu_get_calibrated_quaternion(&qw, &qx, &qy, &qz);
    ESP_LOGI(TAG, "IMU quaternion: w=%6.2f  x=%6.2f  y=%6.2f  z=%6.2f",
             qw, qx, qy, qz);
    
    mahony_set_quaternion(qw, qx, qy, qz);

    imu_comp_data_t comp_data;
    while (true) {


        ret = imu_read_compensated_data(&comp_data);

        imu_comp_data_t comp;
        imu_read_normalized_data(&comp);
            // fuse into Mahony AHRS

            // ESP_LOGI(TAG, "Gyro  [comp]: X=%7.3f  Y=%7.3f  Z=%7.3f [norm]:  X=%7.3f  Y=%7.3f  Z=%7.3f",
            //         comp_data.gyro_x,   comp_data.gyro_y,   comp_data.gyro_z, \comp.norm_gyro[0], comp.norm_gyro[1], comp.norm_gyro[2]);

            // mahony_ahrs_6d_update(
            //     comp.norm_gyro[0], comp.norm_gyro[1], comp.norm_gyro[2],
            //     comp.norm_accel1[0], comp.norm_accel1[1], comp.norm_accel1[2]
            // );

            mahony_ahrs_9d_update(
                comp.norm_gyro[0], comp.norm_gyro[1], comp.norm_gyro[2],
                comp.norm_accel1[0], comp.norm_accel1[1], comp.norm_accel1[2],
                comp.norm_mag[0], comp.norm_mag[1], comp.norm_mag[2]
            );
            // extract yaw/pitch/roll
            float yaw_filt, pitch_filt, roll_filt;
            mahony_get_euler_lpf(&yaw_filt, &pitch_filt, &roll_filt);

            ESP_LOGI(TAG, "Door yaw=%6.2f°  Pitch=%6.2f°  Roll=%6.2f°",
                       yaw_filt*RAD_TO_DEG, pitch_filt*RAD_TO_DEG, roll_filt*RAD_TO_DEG);


        

        // loop on mahony code here
        // delay to maintain SAMPLE_RATE_HZ
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
