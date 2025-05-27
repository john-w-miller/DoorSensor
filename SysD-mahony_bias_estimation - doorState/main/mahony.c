// mahony.c

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // for xTaskGetTickCount(), pdMS_TO_TICKS()
#include <esp_log.h>
#include "mahony.h"

/**
 * @file mahony.c
 * @brief Mahony AHRS implementation with gyro‐bias estimation.
 *
 * Provides 6-axis updates, quaternion integration, Euler conversion,
 * low-pass smoothing, and gated LPF bias estimation triggered by door events.
 */

 static const char *TAG = "imu_filter";

/* LPF state for Euler angles */
static float lpf_alpha = 0.9f;
static float lpf_yaw = 0.0f, lpf_pitch = 0.0f, lpf_roll = 0.0f;

// Internal quaternion representation
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float samplePeriod;
static float twoKp; // 2 * proportional gain
static float twoKi; // 2 * integral gain

// Integral feedback terms
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

/** @name Bias Estimation Configuration
 * @{
 */

/** LPF weight for gyro bias estimation; lower = faster adaptation. */
#define BIAS_LPF_ALPHA 0.9995f // adjust for desired time constant

/** Delay before taking initial bias snapshot after door-close [ms]. */
#define BIAS_SNAPSHOT_DELAY_MS 10000UL

/** @} */

// State for snapshot rollback
/** Timestamp of last door‐close; zero while door is open. */
static TickType_t bias_close_ts = 0;
/** True once the 10 s warm-up snapshot has been taken. */
static bool bias_snapshot_taken = false;
/** Stored bias at the end of warm-up (rad/s). */
static float snapBiasX = 0, snapBiasY = 0, snapBiasZ = 0;
/** Current LPF estimate of gyro bias (rad/s). */
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
/** True if we have a valid snapshot to restore on next close. */
static bool bias_has_snapshot = false;

void mahony_init(float dt, float Kp, float Ki)
{
    mahony_set_gains(Kp, Ki);
    mahony_reset();
    samplePeriod = dt;
}

// Not-so-fast inverse square-root -use quake hack if you need speed
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/** @addtogroup BiasEstimation
 * @{
 */

/**
 * @brief  Subtract current bias and optionally update it via LPF,
 *         gated by door-open/close events and a 10s warm-up snapshot.
 * @param[in,out] gx  Raw gyro X [rad/s]; on return contains bias‐compensated rate.
 * @param[in,out] gy  Raw gyro Y [rad/s]; on return contains bias‐compensated rate.
 * @param[in,out] gz  Raw gyro Z [rad/s]; on return contains bias‐compensated rate.
 */
static void compensate_and_update_bias(float *gx, float *gy, float *gz)
{
    static int trace_ctr = 0;
    TickType_t now = xTaskGetTickCount();

    // 1) Door is open if bias_close_ts==0 → freeze bias
    if (bias_close_ts == 0)
    {

        if ((trace_ctr++ % 200) == 0)
            ESP_LOGD(TAG,
                     "FROZEN (OPEN): gyroBias=%.5f, ts=0",
                     gyroBiasX);

        *gx -= gyroBiasX;
        *gy -= gyroBiasY;
        *gz -= gyroBiasZ;
        return;
    }

    // 2) Still within the 10 s warm-up after close? → subtract old bias only
    if (!bias_snapshot_taken)
    {
        TickType_t dt = now - bias_close_ts;
        if ((trace_ctr++ % 200) == 0)
            ESP_LOGD(TAG,
                     "WARMUP: dt=%lums < %lums, gyroBias=%.5f",
                     dt, BIAS_SNAPSHOT_DELAY_MS, gyroBiasX);

        if ((now - bias_close_ts) < pdMS_TO_TICKS(BIAS_SNAPSHOT_DELAY_MS))
        {
            ESP_LOGD(TAG, "Not Warm");
            *gx -= gyroBiasX;
            *gy -= gyroBiasY;
            *gz -= gyroBiasZ;
            return;
        }
        // 3) Warm-up just ended → take a one-time snapshot of the bias
        snapBiasX = gyroBiasX;
        snapBiasY = gyroBiasY;
        snapBiasZ = gyroBiasZ;
        bias_snapshot_taken = true;
        // fall through so we update LPF on this first post-warm-up sample
    }

    // 4) Normal LPF update of bias
    float raw_gx = *gx, raw_gy = *gy, raw_gz = *gz;

    if ((trace_ctr++ % 200) == 0)
        ESP_LOGD(TAG,
                 "LPF UPDATE: raw=%.5f, oldBias=%.5f -> ",
                 raw_gx, gyroBiasX);

    // subtract current bias for the filter input
    *gx = raw_gx - gyroBiasX;
    *gy = raw_gy - gyroBiasY;
    *gz = raw_gz - gyroBiasZ;
    // update LPF estimate
    gyroBiasX = BIAS_LPF_ALPHA * gyroBiasX + (1.0f - BIAS_LPF_ALPHA) * raw_gx;
    gyroBiasY = BIAS_LPF_ALPHA * gyroBiasY + (1.0f - BIAS_LPF_ALPHA) * raw_gy;
    gyroBiasZ = BIAS_LPF_ALPHA * gyroBiasZ + (1.0f - BIAS_LPF_ALPHA) * raw_gz;

    if ((trace_ctr % 200) == 1)
        ESP_LOGD(TAG, "newBias=%.5f, yaw=%.5f", gyroBiasX, 
            atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));
}

/** @} */ // end of BiasEstimation

/**
 * @brief 6-axis (gyro+accel) Mahony AHRS update.
 *
 * Applies bias compensation, normalizes accel, computes feedback errors,
 * integrates the quaternion, and normalizes it.
 *
 * @param gx Raw gyro X [rad/s]
 * @param gy Raw gyro Y [rad/s]
 * @param gz Raw gyro Z [rad/s]
 * @param ax Accel X [g]
 * @param ay Accel Y [g]
 * @param az Accel Z [g]
 * @return true if accel normalization succeeded; false if accel vector was too small.
 */
bool mahony_ahrs_6d_update(float gx, float gy, float gz,
                           float ax, float ay, float az)
{
    compensate_and_update_bias(&gx, &gy, &gz);

    static TickType_t last_dump = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - last_dump) >= pdMS_TO_TICKS(1000))
    {
        ESP_LOGD(TAG,
                 "STATE: close_ts=%lu, snapTaken=%d, snap=%.5f, bias=%.5f",
                 bias_close_ts, bias_snapshot_taken,
                 snapBiasX, gyroBiasX);
        last_dump = now;
    }

    // normalize accelerometer
    // Protect against zero/near-zero accel vector
    float normSq = ax * ax + ay * ay + az * az;
    if (normSq < VECTOR_NORM_EPSILON)
    {
        return false;
    }
    float norm = invSqrt(normSq);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // estimated direction of gravity (from quaternion)
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // error = cross(estimated, measured)
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // Apply integral feedback if enabled
    if (twoKi > 0.0f)
    {
        integralFBx += twoKi * ex * samplePeriod;
        integralFBy += twoKi * ey * samplePeriod;
        integralFBz += twoKi * ez * samplePeriod;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }

    // Apply proportional feedback
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    // integrate quaternion rate
    float halfPeriod = 0.5f * samplePeriod;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * halfPeriod;
    q1 += (qa * gx + qc * gz - q3 * gy) * halfPeriod;
    q2 += (qa * gy - qb * gz + q3 * gx) * halfPeriod;
    q3 += (qa * gz + qb * gy - qc * gx) * halfPeriod;

    // normalize quaternion
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    return true;
}

/**
 * @brief Convert current quaternion to Euler angles.
 *
 * roll  = rotation about X axis
 * pitch = rotation about Y axis
 * yaw   = rotation about Z axis
 *
 * @param[out] yaw   Z-axis rotation [rad]
 * @param[out] pitch Y-axis rotation [rad]
 * @param[out] roll  X-axis rotation [rad]
 */
void mahony_get_euler(float *yaw, float *pitch, float *roll)
{
    // roll (x-axis rotation)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    // pitch (y-axis rotation)
    *pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    // yaw (z-axis rotation)
    *yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                  q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
}

/**
 * @brief Get the yaw angle from the current quaternion.
 *
 * Yaw is the rotation about the Z axis.
 *
 * @param[out] yaw Yaw angle in radians.
 */
void mahony_get_yaw(float *yaw)
{
    // yaw (z-axis rotation)
    *yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                  q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
}

/**
 * @brief Reset full filter state.
 *
 * Sets quaternion to identity [1,0,0,0] and clears integral feedback.
 */
void mahony_reset(void)
{
    // reset orientation to identity quaternion
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    // clear integrator memory
    integralFBx = integralFBy = integralFBz = 0.0f;
}

/**
 * @brief Reset only the attitude (quaternion & LPF Euler), keep bias.
 *
 * Leaves gyroBiasX/Y/Z untouched, zeroes quaternion and LPF buffers.
 */
void mahony_reset_attitude(void)
{
    // quaternion → identity
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    // clear the Euler LPFs
    lpf_yaw = 0.0f;
    lpf_pitch = 0.0f;
    lpf_roll = 0.0f;
}

/**
 * @brief Update proportional (Kp) and integral (Ki) gains.
 *
 * @param kp New Kp (clamped to [0,10])
 * @param ki New Ki (clamped to [0,1])
 */
void mahony_set_gains(float kp, float ki)
{
    kp = fminf(fmaxf(kp, KP_MIN), KP_MAX);
    ki = fminf(fmaxf(ki, KI_MIN), KI_MAX);

    // store as twoKp / twoKi for internal usage
    twoKp = 2.0f * kp;
    twoKi = 2.0f * ki;
}

/**
 * @brief Retrieve current Kp/Ki.
 *
 * @param[out] kp Receives current proportional gain.
 * @param[out] ki Receives current integral gain.
 */
void mahony_get_gains(float *kp, float *ki)
{
    if (kp)
        *kp = twoKp * 0.5f; // convert back from twoKp
    if (ki)
        *ki = twoKi * 0.5f; // convert back from twoKi
}

/**
 * @brief Set LPF alpha for Euler-angle smoothing.
 *
 * @param alpha Smoothing factor [0..1], where higher values retain more history.
 */
void mahony_set_lpf_alpha(float alpha)
{
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    lpf_alpha = alpha;
}

/**
 * @brief Get low-pass filtered Euler angles.
 *
 * Applies a one-pole filter to the raw Euler output.
 *
 * @param[out] yaw_out   Filtered yaw [rad]
 * @param[out] pitch_out Filtered pitch [rad]
 * @param[out] roll_out  Filtered roll [rad]
 */
void mahony_get_euler_lpf(float *yaw_out, float *pitch_out, float *roll_out)
{
    float yaw, pitch, roll;
    mahony_get_euler(&yaw, &pitch, &roll);
    /* one-pole lpf */
    lpf_yaw = lpf_alpha * lpf_yaw + (1.0f - lpf_alpha) * yaw;
    lpf_pitch = lpf_alpha * lpf_pitch + (1.0f - lpf_alpha) * pitch;
    lpf_roll = lpf_alpha * lpf_roll + (1.0f - lpf_alpha) * roll;
    if (yaw_out)
        *yaw_out = lpf_yaw;
    if (pitch_out)
        *pitch_out = lpf_pitch;
    if (roll_out)
        *roll_out = lpf_roll;
}

/**
 * @brief Set the current quaternion directly.
 *
 * This replaces the internal quaternion state with the provided values.
 *
 * @param w Real part of quaternion
 * @param x X component of quaternion
 * @param y Y component of quaternion
 * @param z Z component of quaternion
 */
void mahony_set_quaternion(float w, float x, float y, float z)
{
    q0 = w;
    q1 = x;
    q2 = y;
    q3 = z;
}

/**
 * @brief Retrieve current gyro bias estimates.
 *
 * These biases (rad/s) are subtracted from raw gyro inputs in each update.
 *
 * @param[out] bx X-axis bias estimate.
 * @param[out] by Y-axis bias estimate.
 * @param[out] bz Z-axis bias estimate.
 */
void mahony_get_gyro_bias(float *bx, float *by, float *bz)
{
    if (bx)
        *bx = gyroBiasX;
    if (by)
        *by = gyroBiasY;
    if (bz)
        *bz = gyroBiasZ;
}

/**
 * @brief Notify filter of door-closed event.
 *
 * Restores the last bias snapshot (if any) and starts the warm-up timer
 * before new bias estimation begins.
 * @ingroup BiasEstimation
 */
void mahony_on_door_closed_event(void)
{
    // 1) restore last cycle’s bias snapshot (if we have one)
    if (bias_has_snapshot)
    {
        gyroBiasX = snapBiasX;
        gyroBiasY = snapBiasY;
        gyroBiasZ = snapBiasZ;
    }
    // 2) start new warm‐up
    bias_close_ts = xTaskGetTickCount();
    bias_snapshot_taken = false;
    ESP_LOGI(TAG,
             "DOOR CLOSED: snap=%.5f, gyroBias=%.5f, ts=%lu",
             snapBiasX, gyroBiasX, bias_close_ts);
}

/**
 * @brief Notify filter of door-open event.
 *
 * Takes a snapshot of the current bias estimate and freezes further updates
 * until the next close event.
 * @ingroup BiasEstimation
 */
void mahony_on_door_open_event(void)
{
    // snapshot remains valid for next close
    bias_has_snapshot = bias_snapshot_taken;

    // rollback immediately on open
    if (bias_has_snapshot)
    {
        gyroBiasX = snapBiasX;
        gyroBiasY = snapBiasY;
        gyroBiasZ = snapBiasZ;
    }
    // freeze LPF until next close
    bias_close_ts = 0;
    bias_snapshot_taken = false;

    ESP_LOGI(TAG,
             "DOOR OPENED: snap=%.5f, gyroBias=%.5f, ts=0",
             snapBiasX, gyroBiasX);
}
