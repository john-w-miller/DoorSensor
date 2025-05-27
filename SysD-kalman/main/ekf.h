/*
 * door_angle_ekf.h
 *
 * Header for EKF-based door angle estimator (gyro + magnetometer fusion).
 * Estimates door angle, gyro bias, and mag bias online.
 *
 * D. A. Engineer — May 2025
 */

#ifndef DOOR_ANGLE_EKF_H
#define DOOR_ANGLE_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

/**
 * @brief Initialize the EKF state, covariance, and noise parameters.
 */
void ekf_init(void);

/**
 * @brief Predict step of the EKF using gyro input.
 * @param gyro_z_dps  Z-axis gyro rate in degrees per second.
 * @param dt          Time step in seconds.
 */
void ekf_predict(float gyro_z_dps, float dt);

/**
 * @brief Update step of the EKF using tilt-compensated magnetometer heading.
 * @param mag_heading_deg  Magnetometer-derived heading in degrees.
 */
void ekf_update(float mag_heading_deg);

/**
 * @brief FreeRTOS task that runs the EKF loop and logs/publishes the door angle.
 * @param arg  Task argument (unused).
 */
void door_position_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // DOOR_ANGLE_EKF_H
