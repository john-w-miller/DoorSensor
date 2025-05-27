/* main.c */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "imu.h"
#include "switches.h"

static const char *TAG = "MAIN";

/* Conversion constants – adjust as needed for your sensor configuration.
   These values are generic; consider using the ST driver conversion functions. */
#define ACCEL_SENS (1.0f / 16384.0f)          // 16384 LSB/g → 0.000061 g/LSB
#define GYRO_SENS ((M_PI / 180.0f) / 131.0f)  // 131 LSB/(°/s) → rad/s per count
#define MAG_SENS (0.1f)                       // µT per LSB (adjust to your FS)
#define MOUNTING_ROTATION_RAD (0.7853981634f) // your 45° hardware tilt
#define LOOP_PERIOD_MS 10
#define LOOP_PERIOD_TICKS pdMS_TO_TICKS(LOOP_PERIOD_MS)
//#define CALIBRATION_PERIOD_MS 500 // Recalibration period (ms)
#define CALIBRATION_PERIOD_MS 500 // (ms)

// Gyro bias smoothing (only updated in CALIBRATION state).
#define GYRO_BIAS_ALPHA 0.01f // adjust 0–1 for responsiveness vs. noise

// Gyro bias estimates (rad/s), initialized to zero.
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// Mahony filter global state.
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralErrorX = 0.0f, integralErrorY = 0.0f, integralErrorZ = 0.0f;

//static float Kp = 1.0f; // Proportional gain – tune as needed.
//static float Ki = 0.1f; // Integral gain – tune as needed.

static float Ki = -1.0f; // Integral gain – tune as needed.
static float Kp = 0.0f; // Proportional gain – tune as needed.

static bool calibrated = false;
static uint32_t calib_start_tick;

static uint32_t last_measure_log_tick = 0;
// TODO  normalize both the accel and mag vectors to unit length before feeding it into the mahony filter
// // accel: [ax, ay, az] in g
// float norm_a = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
// ax *= norm_a; ay *= norm_a; az *= norm_a;
// // mag: [mx, my, mz] in G
// float norm_m = 1.0f / sqrtf(mx*mx + my*my + mz*mz);
// mx *= norm_m; my *= norm_m; mz *= norm_m;


// System state.
typedef enum
{
    STATE_CALIBRATION,
    STATE_MEASUREMENT
} system_state_t;
volatile system_state_t system_state = STATE_CALIBRATION;

// Calibration variables.
volatile float calibration_offset = 0.0f;

// Normalize angle to [-pi, pi].
static float normalize_angle(float angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}
unsigned int i = 0;

/* Mahony filter update.
   gx,gy,gz: gyro measurements (rad/s)
   ax,ay,az: accelerometer measurements (in g)
   mx,my,mz: magnetometer measurements (already transformed)
   dt: time step in seconds */
static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt)
{
    float norm;

    // Normalize accelerometer.
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalize magnetometer.
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return;
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Precompute quaternion products.
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Estimated gravity.
    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = q0q0 - q1q1 - q2q2 + q3q3;

    // Error between estimated and measured gravity.
    float errorX = (ay * vz - az * vy);
    float errorY = (az * vx - ax * vz);
    float errorZ = (ax * vy - ay * vx);

    // Magnetometer correction for yaw.
    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    float mag_est_norm = sqrtf(hx * hx + hy * hy);
    if (mag_est_norm > 0.0f)
    {
        float mag_est_x = hx / mag_est_norm;
        float mag_est_y = hy / mag_est_norm;
        float mag_meas_norm = sqrtf(mx * mx + my * my);
        if (mag_meas_norm > 0.0f)
        {
            float mag_meas_x = mx / mag_meas_norm;
            float mag_meas_y = my / mag_meas_norm;
            errorZ += (mag_meas_y * mag_est_x - mag_meas_x * mag_est_y);
        }
    }

    // Integral feedback.
    if (Ki > 0.0f)
    {
        integralErrorX += Ki * errorX * dt;
        integralErrorY += Ki * errorY * dt;
        integralErrorZ += Ki * errorZ * dt;

        // Clamp integral error to prevent windup.
        const float Imax = 0.5f;  // radians
        integralErrorZ = fmaxf(-Imax, fminf(Imax, integralErrorZ));

        gx += integralErrorX;
        gy += integralErrorY;
        gz += integralErrorZ;
    }

    // Proportional feedback.
    gx += Kp * errorX;
    gy += Kp * errorY;
    gz += Kp * errorZ;


    // Integrate quaternion rate.
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    float dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    float dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    float dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    // Normalize quaternion.
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    norm = 1.0f / norm;
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

// Extract current yaw (rotation about vertical axis) from the quaternion.
static float get_yaw(void)
{
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

// SWITCH_WPS callback toggles system state.
static void switch_callback(switch_id_t id, bool pressed, bool long_press)
{
    if (id == SWITCH_WPS && pressed)
    {
        if (system_state == STATE_CALIBRATION)
        {
            // When button is pressed in CALIBRATION, transition to MEASUREMENT
            // only after we’ve captured the offset.
            if (!calibrated)
            {
                ESP_LOGW(TAG, "Still calibrating—please wait.");
                return;
            }
            // freeze everything and go measure
            system_state = STATE_MEASUREMENT;
            last_measure_log_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "Entering MEASUREMENT, offset=%.3f", calibration_offset);
        }
        else
        {
            system_state = STATE_CALIBRATION;
            // begin a fresh calibration cycle
            calibrated = false;
            calib_start_tick = xTaskGetTickCount();
            integralErrorX = integralErrorY = integralErrorZ = 0.0f;
            ESP_LOGI(TAG, "Entering CALIBRATION—keep door closed.");
        }
    }
}

/**
 * @brief  Log both raw counts and converted values from the IMU.
 * @param  d  Pointer to latest imu_data_t
 */
static void log_raw_and_conv(const imu_data_t *d)
{
    // 1) Raw counts
    ESP_LOGI(TAG, "RAW CNT — Accel[%6d, %6d, %6d], Gyro[%6d, %6d, %6d], Mag[%6d, %6d, %6d]",
             d->accel1_x, d->accel1_y, d->accel1_z,
             d->gyro_x, d->gyro_y, d->gyro_z,
             d->mag_x, d->mag_y, d->mag_z);

    // 2) Convert to physical units
    //    (adjust these constants to match your full-scale settings)

    float ax = d->accel1_x * ACCEL_SENS;
    float ay = d->accel1_y * ACCEL_SENS;
    float az = d->accel1_z * ACCEL_SENS;

    float gx = d->gyro_x * GYRO_SENS;
    float gy = d->gyro_y * GYRO_SENS;
    float gz = d->gyro_z * GYRO_SENS;

    float mx = d->mag_x * MAG_SENS;
    float my = d->mag_y * MAG_SENS;
    float mz = d->mag_z * MAG_SENS;

    ESP_LOGI(TAG, " CONV  — Accel[g]=[%.3f, %.3f, %.3f], Gyro[°/s]=[%.1f, %.1f, %.1f], Mag[µT]=[%.2f, %.2f, %.2f]",
             ax, ay, az,
             gx * (180.0f / M_PI), gy * (180.0f / M_PI), gz * (180.0f / M_PI),
             mx, my, mz);
}

void app_main(void)
{
    esp_err_t ret = imu_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU initialization failed");
        return;
    }

    switches_init();
    switches_register_callback(switch_callback);

    uint32_t last_tick = xTaskGetTickCount();
    last_measure_log_tick = last_tick;

    while (1)
    {

        uint32_t current_tick = xTaskGetTickCount();
        float dt = (current_tick - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        last_tick = current_tick;
        //ESP_LOGI(TAG, "dt = %.3f s (%" PRIu32 " ticks)", dt, current_tick - last_tick);

        imu_data_t data;
        ret = imu_read_data(&data);

        if (ret == ESP_OK)
        {
            // log_raw_and_conv(&data);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read IMU data");
            vTaskDelay(LOOP_PERIOD_TICKS);
            continue;
        }

        // --- 1) Convert & reorient raw data into body-frame [accel in g, gyro in rad/s, mag in µT] ---
        // (insert your existing 45° rotation + axis-swap + bias-subtract here)
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        {
            // --- a) convert raw counts to physical units ---
            // accel: counts → g
            float ax0 = data.accel1_x * ACCEL_SENS;
            float ay0 = data.accel1_y * ACCEL_SENS;
            float az0 = data.accel1_z * ACCEL_SENS;

            // gyro: counts → rad/s and subtract bias
            float gx0 = (data.gyro_x * GYRO_SENS);
            float gy0 = (data.gyro_y * GYRO_SENS);
            float gz0 = (data.gyro_z * GYRO_SENS);

            // mag: counts → µT and remap LSM303 → LSM6DS
            //  1) rotate 180° about Z: x'=-x, y'=-y
            //  2) swap X↔Y: x6=y', y6=x'
            float mx0 = -data.mag_y * MAG_SENS; // -y303 → X6
            float my0 = -data.mag_x * MAG_SENS; // -x303 → Y6
            float mz0 = data.mag_z * MAG_SENS;  //  z303 → Z6

            // --- b) rotate by mounting angle so Z_sensor→horizon, X_sensor→vertical ---
            const float ang = MOUNTING_ROTATION_RAD; // e.g. 0.785398f (45°)
            const float c = cosf(ang), s = sinf(ang);

            // rotation about sensor Z by +45° (anticlockwise)
            float axp = c * ax0 - s * ay0;
            float ayp = s * ax0 + c * ay0;
            float azp = az0;

            float gxp = c * gx0 - s * gy0;
            float gyp = s * gx0 + c * gy0;
            float gzp = gz0;

            // 3) Update gyro bias on the rotated axes (in the same frame
            //    that Mahony will consume)
            // Only update the bias estimate when we’re in CALIBRATION mode:
            if (system_state == STATE_CALIBRATION)
            {
                gyro_bias_x = (1.0f - GYRO_BIAS_ALPHA) * gyro_bias_x + GYRO_BIAS_ALPHA * gxp;
                gyro_bias_y = (1.0f - GYRO_BIAS_ALPHA) * gyro_bias_y + GYRO_BIAS_ALPHA * gyp;
                gyro_bias_z = (1.0f - GYRO_BIAS_ALPHA) * gyro_bias_z + GYRO_BIAS_ALPHA * gzp;
            }

            // 4) Subtract the bias — now gx/gy/gz live in the rotated frame
            gxp -= gyro_bias_x;
            gyp -= gyro_bias_y;
            gzp -= gyro_bias_z;

            float mxp = c * mx0 - s * my0;
            float myp = s * mx0 + c * my0;
            float mzp = mz0;

            // --- c) remap so Mahony’s axes map to world [X→forward, Y→side, Z→up] ---
            // here we choose: New_X = sensor Zhorizon, New_Y = rotated Yhorizon, New_Z = –rotated Xvertical
            ax = azp;  // sensor Z (horizon) → Mahony X
            ay = ayp;  // rotated Y (horizon) → Mahony Y
            az = -axp; // -rotated X (vertical) → Mahony Z

            // use your bias-corrected rotated gyros here:
            gx = gzp;  // sensor Z-rate → Mahony X
            gy = gyp;  // rotated Y-rate → Mahony Y
            gz = -gxp; // -rotated X-rate → Mahony Z

            mx = mzp;
            my = myp;
            mz = -mxp;
        }

        if (system_state == STATE_CALIBRATION)
        {

            // 1) run the AHRS filter so q/yaw stay up to date:
            mahony_update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

            // 2) only once, after a settling delay, grab yaw → offset
            if (!calibrated && (xTaskGetTickCount() - calib_start_tick) >= pdMS_TO_TICKS(CALIBRATION_PERIOD_MS)) {
                calibration_offset = get_yaw();
                calibrated         = true;
                ESP_LOGI(TAG, "Calibrated! offset=%.3f  (%.1f°)", calibration_offset,  calibration_offset * (180.0f/M_PI));
                system_state = STATE_MEASUREMENT;
            }

        }
        else /* STATE_MEASUREMENT */
        {
            // Update Mahony filter.
            mahony_update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

            // subtract offset & log 1 Hz
            float door_ang = normalize_angle(get_yaw() - calibration_offset);

            if ((xTaskGetTickCount() - last_measure_log_tick) >= pdMS_TO_TICKS(100)) {
                ESP_LOGI(TAG, "Door angle=%.3f rad (%.1f°)", door_ang, door_ang * (180.0f/M_PI));
                ESP_LOGI(TAG, "ax=%.3f ay=%.3f az=%.3f", ax, ay, az);

                last_measure_log_tick = xTaskGetTickCount();
            }
        }

        vTaskDelay(LOOP_PERIOD_TICKS);
    }
}
