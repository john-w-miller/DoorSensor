// mahony.c
#include "mahony.h"
#include <math.h>
/* LPF state for Euler angles */
static float lpf_alpha = 0.9f;
static float lpf_yaw = 0.0f, lpf_pitch = 0.0f, lpf_roll = 0.0f;

// Internal quaternion representation
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float samplePeriod;
static float twoKp;    // 2 * proportional gain
static float twoKi;    // 2 * integral gain

// Integral feedback terms
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

void mahony_init(float dt, float Kp, float Ki)
{
    mahony_set_gains(Kp, Ki);
    mahony_reset();
    samplePeriod = dt;
}

// Fast inverse square-root
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

// MAHONY AHRS update
bool mahony_ahrs_9d_update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz)
{
    // Normalize accelerometer
    // Protect against zero/near-zero accel vector
    float normSq = ax*ax + ay*ay + az*az;
    if (normSq < VECTOR_NORM_EPSILON) {
        return false;
    }
    float norm = invSqrt(normSq);
    ax *= norm; ay *= norm; az *= norm;

    // Normalize magnetometer
    // Protect against zero/near-zero mag vector
    float mNormSq = mx*mx + my*my + mz*mz;
    if (mNormSq < VECTOR_NORM_EPSILON) {
        return false;
    }
    norm = invSqrt(mNormSq);
    mx *= norm; my *= norm; mz *= norm;

    // Reference direction of Earth's magnetic field
    float hx = 2.0f*(mx*(0.5f - q2*q2 - q3*q3)
                   + my*(q1*q2 - q0*q3)
                   + mz*(q1*q3 + q0*q2));
    float hy = 2.0f*(mx*(q1*q2 + q0*q3)
                   + my*(0.5f - q1*q1 - q3*q3)
                   + mz*(q2*q3 - q0*q1));
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f*(mx*(q1*q3 - q0*q2)
                   + my*(q2*q3 + q0*q1)
                   + mz*(0.5f - q1*q1 - q2*q2));

    // Estimated direction of gravity and magnetic field
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    float wx = 2.0f*bx*(0.5f - q2*q2 - q3*q3)
             + 2.0f*bz*(q1*q3 - q0*q2);
    float wy = 2.0f*bx*(q1*q2 - q0*q3)
             + 2.0f*bz*(q0*q1 + q2*q3);
    float wz = 2.0f*bx*(q0*q2 + q1*q3)
             + 2.0f*bz*(0.5f - q1*q1 - q2*q2);

    // Error is cross product between estimated and measured direction
    float ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    float ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    // Apply integral feedback if enabled
    if(twoKi > 0.0f) {
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

    // Integrate rate of change of quaternion,
    // note: do not disturb the ordering of calculations
    float pa = q1, pb = q2, pc = q3;
    q0 += (-pa*gx - pb*gy - pc*gz) * (0.5f * samplePeriod);
    q1 += ( q0*gx + pb*gz - pc*gy) * (0.5f * samplePeriod);
    q2 += ( q0*gy - pa*gz + pc*gx) * (0.5f * samplePeriod);
    q3 += ( q0*gz + pa*gy - pb*gx) * (0.5f * samplePeriod);

    // Normalize quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    return true;
}

// Mahony ahrs update 6-axis (gyro+accel) 
bool mahony_ahrs_6d_update(float gx, float gy, float gz,
                         float ax, float ay, float az)
{
    // normalize accelerometer
    // Protect against zero/near-zero accel vector
    float normSq = ax*ax + ay*ay + az*az;
    if (normSq < VECTOR_NORM_EPSILON) {
        return false;
    }
    float norm = invSqrt(normSq);
    ax *= norm; ay *= norm; az *= norm;

    // estimated direction of gravity (from quaternion)
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // error = cross(estimated, measured)
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // Apply integral feedback if enabled
    if(twoKi > 0.0f) {
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
    q1 += ( qa * gx + qc * gz - q3 * gy) * halfPeriod;
    q2 += ( qa * gy - qb * gz + q3 * gx) * halfPeriod;
    q3 += ( qa * gz + qb * gy - qc * gx) * halfPeriod;

    // normalize quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    return true;
}

void mahony_get_euler(float *yaw, float *pitch, float *roll)
{
    // roll (x-axis rotation)
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                    q0*q0 - q1*q1 - q2*q2 + q3*q3);
    // pitch (y-axis rotation)
    *pitch = asinf (-2.0f*(q1*q3 - q0*q2));
    // yaw (z-axis rotation)
    *yaw   = atan2f(2.0f*(q1*q2 + q0*q3),
                    q0*q0 + q1*q1 - q2*q2 - q3*q3);
}

void mahony_get_yaw(float *yaw)
{
    // yaw (z-axis rotation)
    *yaw   = atan2f(2.0f*(q1*q2 + q0*q3),
                    q0*q0 + q1*q1 - q2*q2 - q3*q3);
}

void mahony_reset(void)
{
    // reset orientation to identity quaternion
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    // clear integrator memory
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void mahony_set_gains(float kp, float ki)
{

    kp = fminf(fmaxf(kp, KP_MIN), KP_MAX);
    ki = fminf(fmaxf(ki, KI_MIN), KI_MAX);

    // store as twoKp / twoKi for internal usage
    twoKp = 2.0f * kp;
    twoKi = 2.0f * ki;
}

void mahony_get_gains(float *kp, float *ki)
{
    if (kp) *kp = twoKp * 0.5f;  // convert back from twoKp
    if (ki) *ki = twoKi * 0.5f;  // convert back from twoKi
}

/**
 * @brief Configure Euler-angle low-pass filter alpha (0..1).
 */
void mahony_set_lpf_alpha(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    lpf_alpha = alpha;
}

/**
 * @brief Retrieve low-pass filtered Euler angles.
 */
void mahony_get_euler_lpf(float *yaw_out, float *pitch_out, float *roll_out)
{
    float yaw, pitch, roll;
    mahony_get_euler(&yaw, &pitch, &roll);
    /* one-pole lpf */
    lpf_yaw   = lpf_alpha * lpf_yaw   + (1.0f - lpf_alpha) * yaw;
    lpf_pitch = lpf_alpha * lpf_pitch + (1.0f - lpf_alpha) * pitch;
    lpf_roll  = lpf_alpha * lpf_roll  + (1.0f - lpf_alpha) * roll;
    if (yaw_out)   *yaw_out   = lpf_yaw;
    if (pitch_out) *pitch_out = lpf_pitch;
    if (roll_out)  *roll_out  = lpf_roll;
}

void mahony_set_quaternion(float w, float x, float y, float z)
{
    q0 = w;
    q1 = x;
    q2 = y;
    q3 = z;
}