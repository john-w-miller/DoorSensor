/*
 * door_angle_ekf.c
 *
 * EKF-based door angle estimator using gyro + tilt-compensated magnetometer.
 * Includes 30s circle-fit calibration by swinging the door to map its arc to a circle.
 *
 * D. A. Engineer — May 2025
 */

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"

// Sampling constants
#define SAMPLE_RATE_HZ    50
#define SAMPLE_PERIOD_S   (1.0f / SAMPLE_RATE_HZ)
#define SAMPLE_PERIOD_MS  (1000 / SAMPLE_RATE_HZ)
#define RAD_TO_DEG        (180.0f / M_PI)

// EKF dimensions
#define EKF_DIM   3

// State vector x = [theta; gyro_bias; mag_bias] (degrees, deg/s, degrees)
static float x[EKF_DIM];
static float P[EKF_DIM][EKF_DIM];
static float Q[EKF_DIM][EKF_DIM];
static float R;  // measurement noise (deg^2)

// Circle-fit calibration center (world-frame X,Y)
static float cal_center_x;
static float cal_center_y;

static const char* TAG = "DOOR_EKF";

/**
 * @brief Fit a circle to N points (x_i,y_i) via Kåsa method.
 * @param x    array of X samples
 * @param y    array of Y samples
 * @param n    number of samples
 * @param cx   output circle center X
 * @param cy   output circle center Y
 */
static void circle_fit(const float *x, const float *y, int n, float *cx, float *cy)
{
    // Solves x^2+y^2 + D*x + E*y + F = 0
    double sum_x = 0, sum_y = 0;
    double sum_x2 = 0, sum_y2 = 0, sum_xy = 0;
    double sum_x3 = 0, sum_y3 = 0;
    double sum_x1y2 = 0, sum_x2y1 = 0;
    for (int i = 0; i < n; ++i) {
        double xi = x[i];
        double yi = y[i];
        double xi2 = xi*xi;
        double yi2 = yi*yi;
        sum_x  += xi;
        sum_y  += yi;
        sum_x2 += xi2;
        sum_y2 += yi2;
        sum_xy += xi*yi;
        sum_x3 += xi2*xi;
        sum_y3 += yi2*yi;
        sum_x1y2 += xi*yi2;
        sum_x2y1 += xi2*yi;
    }
    double C = n*sum_x2 - sum_x*sum_x;
    double D = n*sum_xy - sum_x*sum_y;
    double E = n*sum_y2 - sum_y*sum_y;
    double G = 0.5*(n*sum_x1y2 + n*sum_x3 - sum_x*sum_x2 - sum_x*sum_y2);
    double H = 0.5*(n*sum_x2y1 + n*sum_y3 - sum_y*sum_x2 - sum_y*sum_y2);
    double denom = C*E - D*D;
    *cx = (float)((G*E - D*H) / denom);
    *cy = (float)((C*H - D*G) / denom);
}

/**
 * @brief Compute tilt-compensated mag heading [deg], relative to calibrated center.
 */
static float compute_mag_heading_deg(const imu_comp_data_t *comp)
{
    // raw body-frame mag
    float m_s0 = comp->norm_mag[0];
    float m_s1 = comp->norm_mag[1];
    float m_s2 = comp->norm_mag[2];
    // tilt-compensate
    float w0 = rot_mat_xl[0][0]*m_s0 + rot_mat_xl[0][1]*m_s1 + rot_mat_xl[0][2]*m_s2;
    float w1 = rot_mat_xl[1][0]*m_s0 + rot_mat_xl[1][1]*m_s1 + rot_mat_xl[1][2]*m_s2;
    // center it
    float xc = w0 - cal_center_x;
    float yc = w1 - cal_center_y;
    float theta = atan2f(yc, xc);
    return theta * RAD_TO_DEG;
}

/**
 * @brief Initialize EKF and perform 30s circle-fit calibration.
 */
void ekf_init(void)
{
    ESP_LOGI(TAG, "Circle-fit cal: swing door through full arc for 30s...");
    const int N = SAMPLE_RATE_HZ * 30;
    static float samp_x[SAMPLE_RATE_HZ * 30];
    static float samp_y[SAMPLE_RATE_HZ * 30];
    imu_comp_data_t comp;

    for (int i = 0; i < N; ++i) {
        if (imu_read_normalized_data(&comp) == ESP_OK) {
            // tilt-compensated vector
            float m0 = comp.norm_mag[0], m1 = comp.norm_mag[1], m2 = comp.norm_mag[2];
            samp_x[i] = rot_mat_xl[0][0]*m0 + rot_mat_xl[0][1]*m1 + rot_mat_xl[0][2]*m2;
            samp_y[i] = rot_mat_xl[1][0]*m0 + rot_mat_xl[1][1]*m1 + rot_mat_xl[1][2]*m2;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    circle_fit(samp_x, samp_y, N, &cal_center_x, &cal_center_y);
    ESP_LOGI(TAG, "Calib done: center=(%.2f,%.2f)", cal_center_x, cal_center_y);

    // static mag-bias init at closed-door
    x[1] = 0.0f;  // gyro bias
    if (imu_read_normalized_data(&comp) == ESP_OK) {
        x[0] = 0.0f;                             // angle = 0
        x[2] = compute_mag_heading_deg(&comp);  // mag bias = heading at closed
    } else {
        x[0] = x[2] = 0.0f;
    }

    // P covariance
    memset(P, 0, sizeof(P));
    P[0][0] = 10.0f;  P[1][1] = 1.0f;  P[2][2] = 5.0f;
    // Q process noise
    memset(Q, 0, sizeof(Q));
    Q[0][0] = 0.01f;  Q[1][1] = 0.0001f;  Q[2][2] = 0.01f;
    // R measurement noise
    R = 25.0f;
}

// ... ekf_predict, ekf_update, door_position_task remain unchanged ...


/**
 * @brief EKF prediction step using gyro input.
 * @param gyro_z_dps  Z-axis gyro rate [deg/s]
 * @param dt          Time step [s]
 */
void ekf_predict(float gyro_z_dps, float dt)
{
    // 1) State prediction
    float theta = x[0];
    float bg    = x[1];
    float bm    = x[2];
    float dtheta = (gyro_z_dps - bg) * dt;

    float x_pred[EKF_DIM] = {
        theta + dtheta,
        bg,
        bm
    };

    // 2) Jacobian F
    float F[EKF_DIM][EKF_DIM] = {
        {1.0f, -dt,   0.0f},
        {0.0f, 1.0f,  0.0f},
        {0.0f, 0.0f,  1.0f}
    };

    // 3) Covariance prediction: P = F*P*F^T + Q
    float tmp[EKF_DIM][EKF_DIM];
    for (int i = 0; i < EKF_DIM; ++i) {
        for (int j = 0; j < EKF_DIM; ++j) {
            tmp[i][j] = 0.0f;
            for (int k = 0; k < EKF_DIM; ++k) {
                tmp[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    for (int i = 0; i < EKF_DIM; ++i) {
        for (int j = 0; j < EKF_DIM; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_DIM; ++k) {
                sum += tmp[i][k] * F[j][k];
            }
            P[i][j] = sum + Q[i][j];
        }
    }

    // Commit predicted state
    memcpy(x, x_pred, sizeof(x));
}

/**
 * @brief EKF update step using magnetometer heading.
 * @param mag_heading_deg  Tilt-compensated mag heading [deg]
 */
void ekf_update(float mag_heading_deg)
{
    // Measurement model: z = theta + mag_bias + noise
    // H = [1 0 1]
    float z_pred = x[0] + x[2];

    // Innovation (wrap to [-180,180])
    float y = mag_heading_deg - z_pred;
    if (y > 180.0f)  y -= 360.0f;
    else if (y < -180.0f) y += 360.0f;

    // Innovation covariance S = H*P*H^T + R
    float S = P[0][0] + P[2][2] + 2.0f * P[0][2] + R;

    // Kalman gain K = P*H^T / S
    float K[EKF_DIM] = {
        (P[0][0] + P[0][2]) / S,
        (P[1][0] + P[1][2]) / S,
        (P[2][0] + P[2][2]) / S
    };

    // State update: x = x + K*y
    for (int i = 0; i < EKF_DIM; ++i) {
        x[i] += K[i] * y;
    }

    // Covariance update: P = (I - K*H) * P
    float I_KH[EKF_DIM][EKF_DIM] = {
        {1.0f - K[0],    0.0f,       -K[0]},
        {   -K[1],    1.0f,       -K[1]},
        {   -K[2],      0.0f,   1.0f - K[2]}
    };
    float newP[EKF_DIM][EKF_DIM];
    for (int i = 0; i < EKF_DIM; ++i) {
        for (int j = 0; j < EKF_DIM; ++j) {
            newP[i][j] = 0.0f;
            for (int k = 0; k < EKF_DIM; ++k) {
                newP[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }
    memcpy(P, newP, sizeof(P));
}

/**
 * @brief Door position task: runs EKF loop.
 */
void door_position_task(void *arg)
{
    imu_comp_data_t comp;

    while (1) {
        if (imu_read_normalized_data(&comp) == ESP_OK) {
            float gyro_z_dps = comp.norm_gyro[2] * RAD_TO_DEG;  // deg/s
            ekf_predict(gyro_z_dps, SAMPLE_PERIOD_S);

            float mag_heading = compute_mag_heading_deg(&comp);
            ekf_update(mag_heading);

            float door_angle = x[0];  // degrees
            ESP_LOGI(TAG, "Door angle = %.2f°", door_angle/2.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
