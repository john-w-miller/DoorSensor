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
 * @brief Mahony Attitude and Heading Reference System 
 *  (AHRS) filter interface
 */


/** 
 * @brief Minimum squared norm for accel/mag vectors to be considered valid.
 */
#define VECTOR_NORM_EPSILON 1e-6f

#define KP_MIN 0.0f
#define KP_MAX 10.0f
#define KI_MIN 0.0f
#define KI_MAX 1.0f

/**
 * @brief Initialize the Mahony AHRS filter.
 * @param dt     Sample period in seconds (e.g. 0.01 for 100 Hz)
 * @param Kp     Proportional gain (typically ~0.1)
 * @param Ki     Integral gain (used to correct steady-state drift)
 */
void mahony_init(float dt, float Kp, float Ki);

/**
 * @brief 9-axis (gyro+accel+mag) Mahony update
 * @param gx Gyro X [rad/s]
 * @param gy Gyro Y [rad/s]
 * @param gz Gyro Z [rad/s]
 * @param ax Accel X [g]
 * @param ay Accel Y
 * @param az Accel Z
 * @param mx Mag X [µT]
 * @param my Mag Y
 * @param mz Mag Z
 * @return true if update succeeded; false if accel or mag vector was invalid.
 */
bool mahony_ahrs_9d_update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz);

/**
 * @brief 6-axis (gyro+accel) Mahony update
 * @param gx Gyro X [rad/s]
 * @param gy Gyro Y [rad/s]
 * @param gz Gyro Z [rad/s]
 * @param ax Accel X [g] (normalized or m/s²)
 * @param ay Accel Y
 * @param az Accel Z
 * @return true if update succeeded; false if accel vector was invalid.
 */
bool mahony_ahrs_6d_update(float gx, float gy, float gz,
                            float ax, float ay, float az);

/**
 * @brief Extract Euler angles from the current quaternion.
 * @param[out] yaw    Yaw angle in radians
 * @param[out] pitch  Pitch angle in radians
 * @param[out] roll   Roll angle in radians
 */
void mahony_get_euler(float *yaw, float *pitch, float *roll);

/**
 * @brief Get the yaw angle from the current quaternion.
 * @param[out] yaw Yaw angle in radians
 */
void mahony_get_yaw(float *yaw);

/** 
 * @brief Reset internal state (quaternion + integrator) to identity. 
 */
void mahony_reset(void);

/**
 * @brief Change proportional & integral gains on‐the‐fly.
 * @param kp   New proportional gain.
 * @param ki   New integral gain.
 */
void mahony_set_gains(float kp, float ki);

/**
 * @brief Retrieve current proportional & integral gains.
 * @param[out] kp   Pointer to receive current Kp.
 * @param[out] ki   Pointer to receive current Ki.
 */
void mahony_get_gains(float *kp, float *ki);

/**
 * @brief Set LPF alpha for Euler-angle smoothing (0..1). Default is 0.9.
 * @param alpha smoothing factor
 */
void mahony_set_lpf_alpha(float alpha);

/**
 * @brief Get low-pass filtered Euler angles.
 * @param[out] yaw    Filtered yaw [rad]
 * @param[out] pitch  Filtered pitch [rad]
 * @param[out] roll   Filtered roll [rad]
 */
void mahony_get_euler_lpf(float *yaw, float *pitch, float *roll);

/**
 * @brief Seed the filter’s internal orientation quaternion.
 * @details
 *   Use this to initialize the filter state at startup with a calibrated quaternion.  
 *   Not part of the normal update loop—intended for testing or first-time alignment.
 * @note
 *   • The quaternion must be normalized (unit length).  
 *   • It is in the form q = [w, x, y, z], where w is the scalar part.
 * @param w  Scalar (real) component of the quaternion.
 * @param x  X component of the vector part.
 * @param y  Y component of the vector part.
 * @param z  Z component of the vector part.
 */

void mahony_set_quaternion(float w, float x, float y, float z);


#ifdef __cplusplus
}
#endif

#endif // MAHONY_H
