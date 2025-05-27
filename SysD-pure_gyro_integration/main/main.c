/* main.c
 *
 * @brief Test driver for pure_gyro_integration.c
 *        Calibrates bias, then runs a fixed loop printing angle.
 */

#include <stdio.h>
#include <math.h>

// Replace this with your real IMU read routine:
extern float read_gyro_z(void);

/* Prototypes from pure_gyro_integration.c */
void   pgi_init(float initial_bias);
float  pgi_update(float raw_gyro_z);

#define CALIB_SAMPLES  100
#define TEST_UPDATES   500

int main(void)
{

    // Initialize both LSM6DS3TR and LSM303AGR
    esp_err_t ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed: %d", ret);
        return;
    }

    // // 2) Now that I²C is live, scan for every slave address
    // scan_i2c_bus(I2C_NUM_0, pdMS_TO_TICKS(500));

        // init Mahony filter: sample period=1/100s (100 Hz)
    mahony_init(0.01f, /*Kp=*/ 6.0f, /*Ki=*/ 0.0f);


    imu_calibrate(50, 1); // allow for the IMU to settle, throw away cal
    imu_calibrate(2000, 5);

    imu_comp_data_t comp;

    // --- 1) Calibrate gyro bias at startup ---
    float sum = 0.0f;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        imu_read_normalized_data(&comp);

        sum += read_gyro_z();
        // insert delay here matching SAMPLE_PERIOD if needed
    }
    float bias = sum / CALIB_SAMPLES;
    printf("Calibrated gyro bias: %.6f rad/s\n", bias);

    // --- 2) Init the integrator ---
    pgi_init(bias);

    // --- 3) Run the test loop ---
    for (int i = 0; i < TEST_UPDATES; i++) {
        float raw_gz = read_gyro_z();
        float angle_rad = pgi_update(raw_gz);
        float angle_deg = angle_rad * (180.0f / M_PI);
        printf("Step %3d: Door = %+6.2f°\n", i, angle_deg);
        // insert a SAMPLE_PERIOD delay or vTaskDelay here
    }

    return 0;
}
