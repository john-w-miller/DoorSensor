/* pure_gyro_integration.c
 *
 * @brief Pure‐Gyro Integration for door‐hinge angle estimation.
 *        No magnetometer, just Z‐axis integrate + auto‐bias + smoothing.
 */

#include <math.h>
#include <stdint.h>

/** Current estimated door angle [rad] */
static float door_angle_rad = 0.0f;
/** Current gyro Z‐bias [rad/s] */
static float gyro_bias_z = 0.0f;
/** Smoothed angle for presentation [rad] */
static float smooth_angle_rad = 0.0f;

/** Sample period [s] – match your IMU update rate */
#define SAMPLE_PERIOD   0.01f
/** Smoothing factor (0…1): closer to 1 = more inertia */
#define ALPHA           0.95f
/** “Still” threshold [rad/s] for auto‐bias correction */
#define BIAS_THRESHOLD  0.02f

/**
 * @brief Initialize the pure‐gyro integrator state.
 * @param initial_bias  Initial gyro Z‐bias [rad/s] (from calibration).
 */
void pgi_init(float initial_bias)
{
    door_angle_rad   = 0.0f;
    smooth_angle_rad = 0.0f;
    gyro_bias_z      = initial_bias;
}

/**
 * @brief Perform one update step.
 * @param raw_gyro_z  Raw Z‐axis gyro rate [rad/s].
 * @return             Smoothed door angle [rad].
 */
float pgi_update(float raw_gyro_z)
{
    // 1) Remove bias
    float gz = raw_gyro_z - gyro_bias_z;

    // 2) Integrate → door angle
    door_angle_rad += gz * SAMPLE_PERIOD;

    // 3) Auto‐bias when nearly still
    if (fabsf(gz) < BIAS_THRESHOLD) {
        gyro_bias_z = 0.99f * gyro_bias_z + 0.01f * raw_gyro_z;
    }

    // 4) Smooth the angle
    smooth_angle_rad = ALPHA * smooth_angle_rad
                     + (1.0f - ALPHA) * door_angle_rad;

    return smooth_angle_rad;
}
