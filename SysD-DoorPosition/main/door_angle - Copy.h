// door_position.h
// Public API for door position (angle) tracking using an AHRS filter.

#ifndef DOOR_POSITION_H
#define DOOR_POSITION_H

#include <esp_err.h>
#include <math.h>

// Default calibration parameters
#define DEFAULT_CAL_SAMPLES     1000    // number of samples for initial gyro/accel calibration
#define DEFAULT_CAL_DELAY_MS    5      // delay between samples in ms

// Default Mahony filter gains
#define DEFAULT_MAHONY_KP       2.0f   // proportional gain for AHRS
#define DEFAULT_MAHONY_KI       0.1f   // integral gain for AHRS

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the door-position subsystem:
 *         - Initializes and calibrates the IMU
 *         - Initializes the Mahony AHRS filter with default gains
 *         - Runs initial filter iterations to converge
 *         - Captures the closed-door (zero-angle) reference
 *
 * @return ESP_OK on success, or an ESP_ERR_* code on failure.
 */
esp_err_t door_position_init(void);

/**
 * @brief  Get the current door angle relative to the closed-door reference.
 *
 * @return Angle in radians (in [-π, +π]). Returns NAN if the IMU read fails.
 */
float door_position_get_angle(void);

/**
 * @brief  Reset the closed-door reference to the current door position.
 *         Call this when you know the door is returned to the closed state.
 */
void door_position_reset_reference(void);

#ifdef __cplusplus
}
#endif

#endif // DOOR_POSITION_H
