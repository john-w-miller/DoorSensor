#ifndef IMU_H
#define IMU_H

/**
 * @file imu.h
 * @brief IMU driver interface for LSM6DS3TR and LSM303AGR sensors.
 *
 * Provides:
 *  - I²C initialization and sensor presence checks
 *  - Sensor resets and configuration with retry logic
 *  - Raw and compensated data reads (accel, gyro, mag, temp)
 *  - Full and gyro‐only calibration routines
 *  
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup imu IMU Driver
 * @{
 */


 // LSM6DS3TR: 0.061 mg/LSB → ~0.000061 g per LSB
#define ACC_SENS_G_PER_LSB       (0.000061f)

// LSM303AGR accel: 0.001 mg/LSB → ~0.000001 g per LSB
#define ACC2_SENS_G_PER_LSB      (0.000001f)

// LSM6DS3TR gyro in ±250 dps mode: 8.75 mdps per LSB → 0.00875 dps/LSB
#define GYRO_SENS_DPS_PER_LSB   (0.00875f)

// Magnetometer: e.g. 0.1 µT/LSB
#define MAG_SENS_UT_PER_LSB      (0.1f)

// Temp: 256 LSB/°C, offset = 25°C at raw 0
#define TEMP_SENS_C_PER_LSB      (1.0f/256.0f)
#define TEMP_OFFSET              25.0f


 /**  
  * @brief  Body→world tilt-only rotation from the LSM303 accel  
  * (built in imu_init via compute_orientation)  
  */
extern float rot_mat_xl[3][3];

/**  
 * @brief  Body→world tilt-only rotation from the LSM6 accel  
 * (built in imu_init via compute_rodrigues)  
 */
 extern float rot_mat_l6[3][3];

 
/**
 * @brief Raw IMU measurements.
 *
 * Contains 16-bit, unscaled counts directly read from the sensors:
 *  - LSM6DS3TR accelerometer & gyroscope
 *  - LSM303AGR accelerometer & magnetometer
 *  - LSM6DS3TR internal temperature sensor
 */
typedef struct {
    int16_t accel1_x; /**< LSM6DS3TR accel X (LSB) */
    int16_t accel1_y; /**< LSM6DS3TR accel Y (LSB) */
    int16_t accel1_z; /**< LSM6DS3TR accel Z (LSB) */
    int16_t gyro_x;   /**< LSM6DS3TR gyro  X (LSB) */
    int16_t gyro_y;   /**< LSM6DS3TR gyro  Y (LSB) */
    int16_t gyro_z;   /**< LSM6DS3TR gyro  Z (LSB) */
    int16_t accel2_x; /**< LSM303AGR accel X (LSB) */
    int16_t accel2_y; /**< LSM303AGR accel Y (LSB) */
    int16_t accel2_z; /**< LSM303AGR accel Z (LSB) */
    int16_t mag_x;    /**< LSM303AGR mag   X (LSB) */
    int16_t mag_y;    /**< LSM303AGR mag   Y (LSB) */
    int16_t mag_z;    /**< LSM303AGR mag   Z (LSB) */
    int16_t temp_raw; /**< LSM6DS3TR internal temperature raw (LSB) */
} imu_data_t;

/**
 * @brief Compensated IMU measurements.
 *
 * Contains bias-subtracted, body-frame-rotated, unit-scaled readings:
 *  - Accelerations in g
 *  - Angular rates in rad/s
 *  - Magnetic field in gauss
 *  - Temperature in degrees Celsius
 */
typedef struct {
    float accel1_x; /**< LSM6DS3TR accel X, in g */
    float accel1_y; /**< LSM6DS3TR accel Y, in g */
    float accel1_z; /**< LSM6DS3TR accel Z, in g */
    float gyro_x;   /**< LSM6DS3TR gyro  X, in rad/s */
    float gyro_y;   /**< LSM6DS3TR gyro  Y, in rad/s */
    float gyro_z;   /**< LSM6DS3TR gyro  Z, in rad/s */
    float accel2_x; /**< LSM303AGR accel X, in g */
    float accel2_y; /**< LSM303AGR accel Y, in g */
    float accel2_z; /**< LSM303AGR accel Z, in g */
    float mag_x;    /**< LSM303AGR mag   X, in gauss */
    float mag_y;    /**< LSM303AGR mag   Y, in gauss */
    float mag_z;    /**< LSM303AGR mag   Z, in gauss */
    float temp_c;   /**< LSM6DS3TR temperature, in °C */
    float norm_accel1[3];  /** @brief Normalized body-frame accel (g) */
    float norm_accel2[3];  /** @brief Normalized body-frame accel from LSM303 (g) */
    float norm_gyro[3];    /** @brief Gyro rates in rad/s (body-frame) */
    float norm_mag[3];     /** @brief Magnetometer in µT (body-frame) */
} imu_comp_data_t;


/**
 * @brief  Combined LSM6DS3TR read: raw accel + gyro → normalize & bias-compensate.
 */
typedef struct {
    float norm_accel[3];  /**< Accelerometer [g] in body frame */
    float norm_gyro[3];   /**< Gyro [rad/s] in body frame */
    float comp_gyro[3];   /**< Bias-removed gyro [rad/s] in sensor frame */
} imu_l6_comp_norm_t;


/**
 * @brief Initialize both IMU sensors (LSM6DS3TR & LSM303AGR).
 *
 * Configures I²C port, performs device reset, sets data rates,
 * full-scale ranges, and enables block data update for both
 * accelerometer/gyro and magnetometer.
 *
 * @return
 *      - ESP_OK on successful initialization
 *      - ESP_ERR_INVALID_STATE if already initialized
 *      - other ESP_ERR_* codes on failure
 */
esp_err_t imu_init(void);

/**
 * @brief Read raw sensor registers from both IMUs.
 *
 * Populates the provided imu_data_t with raw counts.
 *
 * @param[out] data Pointer to imu_data_t to receive raw measurements.
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if data is NULL
 *      - other ESP_ERR_* codes on I²C failure
 */
esp_err_t imu_read_data(imu_data_t *data);

/**
 * @brief Read raw accelerometer data from LSM6DS3TR.
 *
 * @param[out] out 3-element array to receive X, Y, Z counts.
 * @return ESP_OK on success, or an ESP_ERR code on failure.
 */
esp_err_t imu_read_accel_lsm6ds3tr(int16_t out[3]);

/**
 * @brief Read raw gyroscope data from LSM6DS3TR.
 *
 * @param[out] out 3-element array to receive X, Y, Z counts.
 * @return ESP_OK on success, or an ESP_ERR code on failure.
 */
esp_err_t imu_read_gyro_lsm6ds3tr(int16_t out[3]);

/**
 * @brief Read raw accelerometer data from LSM303AGR.
 *
 * @param[out] out 3-element array to receive X, Y, Z counts.
 * @return ESP_OK on success, or an ESP_ERR code on failure.
 */
esp_err_t imu_read_accel_lsm303agr(int16_t out[3]);

/**
 * @brief Read raw magnetometer data from LSM303AGR.
 *
 * @param[out] out 3-element array to receive X, Y, Z counts.
 * @return ESP_OK on success, or an ESP_ERR code on failure.
 */
esp_err_t imu_read_mag_lsm303agr(int16_t out[3]);

/**
 * @brief Verify LSM6DS3TR WHO_AM_I register.
 *
 * @return true if the chip responds with the expected device ID.
 */
bool imu_lsm6ds3tr_whoami_ok(void);

/**
 * @brief Verify LSM303AGR accelerometer & magnetometer WHO_AM_I.
 *
 * @return true if both sub-devices respond with the expected IDs.
 */
bool imu_lsm303agr_whoami_ok(void);

/**
 * @brief Read compensated IMU data.
 *
 * Applies bias subtraction, body-frame rotation, and unit-scaling
 * to all sensor axes (accel in g, gyro in rad/s, mag in gauss, temp in °C).
 *
 * @param[out] d Pointer to imu_comp_data_t to receive compensated data.
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if d is NULL
 *      - other ESP_ERR_* codes on failure
 */
esp_err_t imu_read_compensated_data(imu_comp_data_t *d);

/**
 * @brief Calibrate sensor biases and orientation alignment.
 *
 * Collects @p samples readings at rest, with @p delay_ms milliseconds
 * between each, to compute:
 *  - Gyroscope zero offset and temperature-drift slope
 *  - Accelerometer and magnetometer hard-iron offsets
 *  - Rotation matrices to align each sensor to the board frame
 *
 * @param[in] samples Number of samples to average for calibration.
 * @param[in] delay_ms Delay in milliseconds between samples.
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if samples==0
 *      - other ESP_ERR_* codes on I²C failure
 */
esp_err_t imu_calibrate(uint32_t samples, uint32_t delay_ms);

/**
 * @brief Fast, gyro‐only calibration.
 *
 * Recomputes only the gyroscope zero-offset (for drift compensation).
 * Much faster than full calibration, intended for periodic runtime use.
 *
 * @param[in] samples  Number of readings to average.
 * @param[in] delay_ms Delay between readings, in milliseconds.
 * @return ESP_OK on success, or an ESP_ERR_* code on failure.
 */
esp_err_t imu_calibrate_gyro(uint32_t samples, uint32_t delay_ms);


/**
 * @brief Read raw & calibrated sensors and produce normalized body-frame
 *        values (g, rad/s, µT, °C) suitable for Mahony filter inputs.
 *
 * @param[out] comp  Destination struct (zeroed on entry).
 * @return ESP_OK or an ESP_ERR_* code on failure.
 */
esp_err_t imu_read_normalized_data(imu_comp_data_t *comp);


esp_err_t imu_get_calibrated_quaternion(float *w_out, float *x_out, float *y_out, float *z_out);



/**
 * @brief Read LSM6 accel+gyro once, compute both normalized & compensated outputs.
 * @param[out] out  Destination struct (must be non-NULL).
 * @return ESP_OK on success, or an ESP_ERR_* on I²C failure/invalid arg.
 */
esp_err_t imu_read_l6_combined(imu_l6_comp_norm_t *out);

/** @} (end defgroup imu) */

#ifdef __cplusplus
}
#endif

#endif // IMU_H
