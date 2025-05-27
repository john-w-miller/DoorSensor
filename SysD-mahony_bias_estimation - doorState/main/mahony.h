// mahony.h
#ifndef MAHONY_H
#define MAHONY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mahony.h
 * @brief Mahony Attitude and Heading Reference System (AHRS) filter interface.
 *
 * This module provides functions to initialize, update, and manage a Mahony
 * AHRS filter, including optional gyro bias estimation and integration with
 * external events (e.g., door open/close) to control bias learning windows.
 */

/**
 * @brief Minimum squared norm for accel/mag vectors to be considered valid.
 */
#define VECTOR_NORM_EPSILON 1e-6f

/** @name Gain limits
 * @{ */
#define KP_MIN 0.0f
#define KP_MAX 10.0f
#define KI_MIN 0.0f
#define KI_MAX 1.0f
/** @} */

/**
 * @brief Initialize the Mahony AHRS filter.
 *
 * @param dt Sample period in seconds (e.g., 0.01 for 100 Hz).
 * @param Kp Proportional gain (typically ~0.1).
 * @param Ki Integral gain (used to correct steady-state drift).
 */
void mahony_init(float dt, float Kp, float Ki);

/**
 * @brief 6-axis (gyro + accel) Mahony update.
 *
 * @param gx Gyro X [rad/s]
 * @param gy Gyro Y [rad/s]
 * @param gz Gyro Z [rad/s]
 * @param ax Accel X [g]
 * @param ay Accel Y
 * @param az Accel Z
 *
 * @return true if update succeeded; false if accel vector invalid.
 */
bool mahony_ahrs_6d_update(float gx, float gy, float gz,
                            float ax, float ay, float az);

/**
 * @brief Extract Euler angles from the current quaternion.
 *
 * @param[out] yaw   Yaw angle in radians.
 * @param[out] pitch Pitch angle in radians.
 * @param[out] roll  Roll angle in radians.
 */
void mahony_get_euler(float *yaw, float *pitch, float *roll);

/**
 * @brief Get the yaw angle from the current quaternion.
 *
 * @param[out] yaw Yaw angle in radians.
 */
void mahony_get_yaw(float *yaw);

/**
 * @brief Reset internal state (quaternion + integrator) to identity.
 */
void mahony_reset(void);

/**
 * @brief Reset only the filter’s attitude (quaternion) to identity.
 *
 * @details Sets the quaternion to [1, 0, 0, 0] and clears any LPF state
 *          for Euler-angle smoothing, but retains gyro bias estimates.
 */
void mahony_reset_attitude(void);

/**
 * @brief Change proportional & integral gains on‐the‐fly.
 *
 * @param kp New proportional gain (0 ≤ kp ≤ 10).
 * @param ki New integral gain (0 ≤ ki ≤ 1).
 */
void mahony_set_gains(float kp, float ki);

/**
 * @brief Retrieve current proportional & integral gains.
 *
 * @param[out] kp Pointer to receive current Kp.
 * @param[out] ki Pointer to receive current Ki.
 */
void mahony_get_gains(float *kp, float *ki);

/**
 * @brief Set LPF alpha for Euler-angle smoothing (0..1).
 *
 * @details Controls the smoothing factor applied to computed Euler angles.
 *          A higher alpha retains more history, a lower alpha responds faster.
 *
 * @param alpha New smoothing factor.
 */
void mahony_set_lpf_alpha(float alpha);

/**
 * @brief Get low-pass filtered Euler angles.
 *
 * @param[out] yaw   Filtered yaw [rad]
 * @param[out] pitch Filtered pitch [rad]
 * @param[out] roll  Filtered roll [rad]
 */
void mahony_get_euler_lpf(float *yaw, float *pitch, float *roll);

/**
 * @brief Seed the filter’s internal orientation quaternion.
 *
 * @details Use this to initialize the filter state at startup with a known
 *          quaternion. Not part of the normal update loop—intended for
 *          calibration or test alignment.
 *
 * @param w Scalar (real) component of the quaternion.
 * @param x X component of the vector part.
 * @param y Y component of the vector part.
 * @param z Z component of the vector part.
 */
void mahony_set_quaternion(float w, float x, float y, float z);

/**
 * @brief Retrieve current gyro bias estimates (rad/s) for each axis.
 *
 * @param[out] bx X-axis bias estimate.
 * @param[out] by Y-axis bias estimate.
 * @param[out] bz Z-axis bias estimate.
 */
void mahony_get_gyro_bias(float *bx, float *by, float *bz);


/**
 * @brief Notify filter that the door has just closed.
 *
 * @details Triggers restoration of the last saved bias offset (if any)
 *          and starts the warm-up timer before new bias estimation.
 */
void mahony_on_door_closed_event(void);

/**
 * @brief Notify filter that the door has just opened.
 *
 * @details Rolls back to the last saved bias snapshot and freezes
 *          bias estimation until the next close event.
 */
void mahony_on_door_open_event(void);

#ifdef __cplusplus
}
#endif

#endif // MAHONY_H
