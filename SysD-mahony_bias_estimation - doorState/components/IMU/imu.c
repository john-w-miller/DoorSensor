/**
 * @file imu.c
 * @brief IMU driver implementation for LSM6DS3TR & LSM303AGR sensors.
 * @ingroup IMU_Driver
 *
 * This module provides:
 *  - I²C initialization and sensor presence checks
 *  - Sensor resets and configuration with retry logic
 *  - Raw and compensated data reads (accel, gyro, mag, temp)
 *  - Full and gyro‐only calibration routines
 *
 * @author  J. Miller
 * @date    2025-5-16
 * @version 1.0
 */

#include <math.h>   // sqrtf(), M_PI
#include <string.h> // memset(), memcpy()
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lsm6ds3tr-c_reg.h"
#include "lsm303agr_reg.h"
#include "imu.h"

static const char *TAG = "imu";

/**
 * @defgroup IMU_Config IMU Configuration Macros
 * @ingroup IMU_Driver
 * @{
 */

/** I2C port number for IMU bus */
#define IMU_I2C_NUM I2C_NUM_0
/** I2C SCL pin */
#define IMU_I2C_SCL_IO GPIO_NUM_5
/** I2C SDA pin */
#define IMU_I2C_SDA_IO GPIO_NUM_4
/** I2C clock frequency (Hz) */
#define IMU_I2C_FREQ_HZ 100000
/** I2C transaction timeout (ticks) */
#define IMU_I2C_TIMEOUT_TICKS pdMS_TO_TICKS(1000)

/** Reset settle */
#define IMU_RESET_SETTLE_TICKS pdMS_TO_TICKS(10)

/** gauss per LSB @ ±50gauss */
#define MAG_SENS_XL 0.0015f
/**  g’s per LSB @ ±2g */
#define ACC_SENS_XL 0.000061f
#define ACC_SENS_L6 0.000061f
/**  dps/LSB */
#define GYRO_SENS 0.00875f
/**  rad/s per count */
#define GYRO_SENS_RAD (GYRO_SENS * (M_PI / 180.0f))

/** Number of initialization retries */
#define INIT_RETRIES 3
/** Delay between init retries (ms) */
#define RETRY_DELAY_MS 50

/** Small threshold for floating-point “almost zero” tests */
#define EPSILON 1e-6f

// TODO: move calibration thresholds to NVS for run‑time tuning
/** max std-dev for LSM6 (counts): */
#define STDDEV6_THRESH 400.0f
/** max std-dev for LSM303 (counts): */
#define STDDEV_XL_THRESH 250.0f
/** @} */ // end of IMU_Config

// Sensor I²C addresses (7-bit)
static uint8_t lsm6_addr = 0x6A;
static uint8_t xl_addr = (LSM303AGR_I2C_ADD_XL >> 1);
static uint8_t mg_addr = (LSM303AGR_I2C_ADD_MG >> 1);

// Calibration state for LSM6DS3TR
static float accel_bias_l6[3] = {0}, gyro_bias_l6[3] = {0};
 float rot_mat_l6[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// Calibration state for LSM303AGR
static float accel_bias_xl[3] = {0};
 float rot_mat_xl[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// X-macro definitions for init steps & names
#define LSM6_STEPS(X)     \
    X(OK, "OK")           \
    X(WHOAMI, "WHO_AM_I") \
    X(IF_INC, "IF_INC")   \
    X(BDU, "BDU")         \
    X(XL_ODR, "XL_ODR")   \
    X(XL_FS, "XL_FS")     \
    X(GY_ODR, "GY_ODR")   \
    X(GY_FS, "GY_FS")     \
    X(VERIFY, "VERIFY")

typedef enum
{
#define X(name, str) LSM6_STEP_##name,
    LSM6_STEPS(X)
#undef X
} lsm6_init_step_t;

static const char *lsm6_step_names[] = {
#define X(name, str) [LSM6_STEP_##name] = str,
    LSM6_STEPS(X)
#undef X
};
static inline const char *safe_lsm6_step_name(lsm6_init_step_t step)
{
    int count = sizeof(lsm6_step_names) / sizeof(*lsm6_step_names);
    return (step >= 0 && step < count) ? lsm6_step_names[step] : "<?>";
}

#define LSM303_STEPS(X)            \
    X(OK, "OK")                    \
    X(XL_WHOAMI, "WHO_AM_I accel") \
    X(XL_ODR, "accel ODR")         \
    X(XL_FS, "accel FS")           \
    X(XL_BDU, "accel BDU")         \
    X(MG_WHOAMI, "WHO_AM_I mag")   \
    X(MG_MODE, "mag mode")         \
    X(MG_ODR, "mag ODR")           \
    X(MG_RES, "mag res")           \
    X(MG_BDU, "mag BDU")

typedef enum
{
#define X(name, str) LSM303_STEP_##name,
    LSM303_STEPS(X)
#undef X
} lsm303_init_step_t;

static const char *lsm303_step_names[] = {
#define X(name, str) [LSM303_STEP_##name] = str,
    LSM303_STEPS(X)
#undef X
};
static inline const char *safe_lsm303_step_name(lsm303_init_step_t step)
{
    int count = sizeof(lsm303_step_names) / sizeof(*lsm303_step_names);
    return (step >= 0 && step < count) ? lsm303_step_names[step] : "<?>";
}

// Forward declarations
static esp_err_t i2c_master_init(void);
static esp_err_t lsm6_reset_and_wait(void);
static void compute_rodrigues(const float u[3], float R[3][3]);
static void compute_orientation(const float accel[3], const float mag[3], float R[3][3]);
static void imu_delay_ms(uint32_t ms);
static int32_t i2c_write_reg(void *h, uint8_t r, const uint8_t *d, uint16_t l);
static int32_t lsm6_i2c_read(void *h, uint8_t r, uint8_t *d, uint16_t l);
static int32_t lsm303_i2c_read(void *h, uint8_t r, uint8_t *d, uint16_t l);
static lsm6_init_step_t lsm6_init_once(void);
static lsm303_init_step_t lsm303_init_once(void);

// STMDEV contexts
static stmdev_ctx_t lsm6_ctx = {
    .write_reg = i2c_write_reg,
    .read_reg = lsm6_i2c_read,
    .mdelay = imu_delay_ms,
    .handle = &lsm6_addr};
static stmdev_ctx_t xl_ctx = {
    .write_reg = i2c_write_reg,
    .read_reg = lsm303_i2c_read,
    .mdelay = imu_delay_ms,
    .handle = &xl_addr};
static stmdev_ctx_t mg_ctx = {
    .write_reg = i2c_write_reg,
    .read_reg = lsm303_i2c_read,
    .mdelay = imu_delay_ms,
    .handle = &mg_addr};

/*---------------------------------------------------------------
 * Implementation
 *--------------------------------------------------------------*/

/**
 * @brief I2C master initialization for the IMU bus.
 * @return ESP_OK on success, or an ESP_ERR_* code on failure.
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = IMU_I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = IMU_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(IMU_I2C_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_param_config failed (%d)", ret);
        return ret;
    }
    ret = i2c_driver_install(IMU_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE)
    {
        // Already installed
        ret = ESP_OK;
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_driver_install failed (%d)", ret);
    }
    return ret;
}

/**
 * @brief Reset LSM6 and wait for completion (timeout 100 ms)
 */
static esp_err_t lsm6_reset_and_wait(void)
{
    uint8_t rst = 0;
    TickType_t t0 = xTaskGetTickCount();

    lsm6ds3tr_c_reset_set(&lsm6_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6ds3tr_c_reset_get(&lsm6_ctx, &rst);
        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(100))
        {
            ESP_LOGE(TAG, "LSM6 reset timeout");
            return ESP_ERR_TIMEOUT;
        }
    } while (rst);

    return ESP_OK;
}

/**
 * @brief Rodrigues’ rotation: rotate unit‐vector u → [0,0,1]
 */
static void compute_rodrigues(const float u[3], float R[3][3])
{
    float vx = u[1], vy = -u[0], vz = 0.0f;
    float s = sqrtf(vx * vx + vy * vy + vz * vz);
    float c = u[2];
    float kx = 0, ky = 0, kz = 1;
    if (s > EPSILON)
    {
        kx = vx / s;
        ky = vy / s;
        kz = vz / s;
    }

    float K[3][3] = {
        {0, -kz, ky},
        {kz, 0, -kx},
        {-ky, kx, 0}};
    float K2[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            K2[i][j] = K[i][0] * K[0][j] + K[i][1] * K[1][j] + K[i][2] * K[2][j];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = (i == j ? 1.f : 0.f) + s * K[i][j] + (1.f - c) * K2[i][j];
}

static void compute_orientation(const float accel[3],
                                const float mag[3],
                                float R[3][3])
{
    // 1) Up = –gravity
    float Uz = accel[2];
    float Uy = accel[1];
    float Ux = accel[0];
    float norm = sqrtf(Ux * Ux + Uy * Uy + Uz * Uz);
    Ux /= norm;
    Uy /= norm;
    Uz /= norm;

    // 2) East  = cross(mag, Up)
    float Ex = Uy * mag[2] - Uz * mag[1];
    float Ey = Uz * mag[0] - Ux * mag[2];
    float Ez = Ux * mag[1] - Uy * mag[0];
    norm = sqrtf(Ex * Ex + Ey * Ey + Ez * Ez);
    Ex /= norm;
    Ey /= norm;
    Ez /= norm;

    // 3) North = cross(Up, East)
    float Nx = Uy * Ez - Uz * Ey;
    float Ny = Uz * Ex - Ux * Ez;
    float Nz = Ux * Ey - Uy * Ex;

    // Build rotation: sensor → earth: columns = [East, North, Up]
    R[0][0] = Ex;
    R[0][1] = Nx;
    R[0][2] = Ux;
    R[1][0] = Ey;
    R[1][1] = Ny;
    R[1][2] = Uy;
    R[2][0] = Ez;
    R[2][1] = Nz;
    R[2][2] = Uz;
}

/**
 * @brief Write one or more registers over I²C.
 * @param handle Pointer to the 7-bit I²C address.
 * @param reg    Register address.
 * @param data   Pointer to data to write.
 * @param len    Number of bytes to write.
 * @return        0 on success, negative on failure.
 */
static int32_t i2c_write_reg(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t addr = *(uint8_t *)handle;
    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return (i2c_master_write_to_device(IMU_I2C_NUM, addr, buf, len + 1, IMU_I2C_TIMEOUT_TICKS) == ESP_OK)
               ? 0
               : -1;
}

/**
 * @brief Read registers from LSM6DS3TR (auto-increment via IF_INC).
 */
static int32_t lsm6_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = *(uint8_t *)handle;
    esp_err_t err = i2c_master_write_read_device(
        IMU_I2C_NUM, addr, &reg, 1, data, len, IMU_I2C_TIMEOUT_TICKS);
    return (err == ESP_OK) ? 0 : (int32_t)err;
}

/**
 * @brief Read registers from LSM303AGR (set MSB for auto-increment).
 */
static int32_t lsm303_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = *(uint8_t *)handle;
    if (len > 1)
    {
        reg |= 0x80;
    }
    esp_err_t err = i2c_master_write_read_device(
        IMU_I2C_NUM, addr, &reg, 1, data, len, IMU_I2C_TIMEOUT_TICKS);
    return (err == ESP_OK) ? 0 : (int32_t)err;
}

/**
 * @brief Delay helper for the stmdev context.
 */
static void imu_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief One‐shot init of LSM6DS3TR: config registers in order.
 * @return The step at which it failed (or LSM6_STEP_OK).
 */
static lsm6_init_step_t lsm6_init_once(void)
{
    uint8_t tmp;

    // WHO_AM_I
    if (lsm6ds3tr_c_device_id_get(&lsm6_ctx, &tmp) != 0 || tmp != LSM6DS3TR_C_ID)
    {
        return LSM6_STEP_WHOAMI;
    }
    // Enable auto‐increment
    if (lsm6ds3tr_c_auto_increment_set(&lsm6_ctx, PROPERTY_ENABLE) != 0)
    {
        return LSM6_STEP_IF_INC;
    }
    // Enable block‐data‐update
    if (lsm6ds3tr_c_block_data_update_set(&lsm6_ctx, PROPERTY_ENABLE) != 0)
    {
        return LSM6_STEP_BDU;
    }
    // Set accel ODR to 104 Hz
    if (lsm6ds3tr_c_xl_data_rate_set(&lsm6_ctx, LSM6DS3TR_C_XL_ODR_104Hz) != 0)
    {
        return LSM6_STEP_XL_ODR;
    }
    // Set accel full-scale to ±2 g
    if (lsm6ds3tr_c_xl_full_scale_set(&lsm6_ctx, LSM6DS3TR_C_2g) != 0)
    {
        return LSM6_STEP_XL_FS;
    }
    // Set gyro ODR to 104 Hz
    if (lsm6ds3tr_c_gy_data_rate_set(&lsm6_ctx, LSM6DS3TR_C_GY_ODR_104Hz) != 0)
    {
        return LSM6_STEP_GY_ODR;
    }
    // Set gyro full-scale to ±250 dps
    if (lsm6ds3tr_c_gy_full_scale_set(&lsm6_ctx, LSM6DS3TR_C_250dps) != 0)
    {
        return LSM6_STEP_GY_FS;
    }

    // Verify settings
    vTaskDelay(IMU_RESET_SETTLE_TICKS);
    uint8_t c1, c2;
    if (lsm6ds3tr_c_read_reg(&lsm6_ctx, LSM6DS3TR_C_CTRL1_XL, &c1, 1) != 0 ||
        lsm6ds3tr_c_read_reg(&lsm6_ctx, LSM6DS3TR_C_CTRL2_G, &c2, 1) != 0)
    {
        return LSM6_STEP_VERIFY;
    }
    if (((c1 >> 4) != LSM6DS3TR_C_XL_ODR_104Hz) ||
        ((c2 >> 4) != LSM6DS3TR_C_GY_ODR_104Hz))
    {
        return LSM6_STEP_VERIFY;
    }
    return LSM6_STEP_OK;
}

/**
 * @brief One‐shot init of LSM303AGR (accel + mag).
 * @return The step at which it failed (or LSM303_STEP_OK).
 */
static lsm303_init_step_t lsm303_init_once(void)
{
    uint8_t id;

    // Accel WHO_AM_I
    if (lsm303agr_xl_device_id_get(&xl_ctx, &id) != 0 || id != LSM303AGR_ID_XL)
    {
        return LSM303_STEP_XL_WHOAMI;
    }
    // Accel ODR
    if (lsm303agr_xl_data_rate_set(&xl_ctx, LSM303AGR_XL_ODR_100Hz) != 0)
    {
        return LSM303_STEP_XL_ODR;
    }
    // Accel FS
    if (lsm303agr_xl_full_scale_set(&xl_ctx, LSM303AGR_2g) != 0)
    {
        return LSM303_STEP_XL_FS;
    }
    // Accel BDU
    if (lsm303agr_xl_block_data_update_set(&xl_ctx, PROPERTY_ENABLE) != 0)
    {
        return LSM303_STEP_XL_BDU;
    }

    // Give accel time to settle before mag config
    vTaskDelay(IMU_RESET_SETTLE_TICKS);

    // Mag WHO_AM_I
    if (lsm303agr_mag_device_id_get(&mg_ctx, &id) != 0 || id != LSM303AGR_ID_MG)
    {
        return LSM303_STEP_MG_WHOAMI;
    }
    // Mag mode = continuous
    if (lsm303agr_mag_operating_mode_set(&mg_ctx, LSM303AGR_CONTINUOUS_MODE) != 0)
    {
        return LSM303_STEP_MG_MODE;
    }
    // Mag ODR
    if (lsm303agr_mag_data_rate_set(&mg_ctx, LSM303AGR_MG_ODR_50Hz) != 0)
    {
        return LSM303_STEP_MG_ODR;
    }
    // Mag resolution
    if (lsm303agr_mag_power_mode_set(&mg_ctx, LSM303AGR_HIGH_RESOLUTION) != 0)
    {
        return LSM303_STEP_MG_RES;
    }
    // Mag BDU
    if (lsm303agr_mag_block_data_update_set(&mg_ctx, PROPERTY_ENABLE) != 0)
    {
        return LSM303_STEP_MG_BDU;
    }

    return LSM303_STEP_OK;
}

/**
 * @brief Initialize both IMU sensors with retry logic.
 */
esp_err_t imu_init(void)
{
    // Top‐level retry for the entire IMU init sequence
    for (int attempt = 1; attempt <= INIT_RETRIES; ++attempt)
    {
        ESP_LOGI(TAG, "IMU init attempt %d/%d", attempt, INIT_RETRIES);

        // 1) Bring up I²C master
        if (i2c_master_init() != ESP_OK)
        {
            ESP_LOGW(TAG, "I²C master init failed");
            goto retry;
        }

        // 2) Reset LSM6
        ESP_LOGI(TAG, "Resetting LSM6DS3TR");
        if (lsm6_reset_and_wait() != ESP_OK)
        {
            ESP_LOGW(TAG, "LSM6DS3TR reset failed");
            goto retry;
        }
        vTaskDelay(IMU_RESET_SETTLE_TICKS);

        // 3) Sensor presence checks
        if (!imu_lsm6ds3tr_whoami_ok())
        {
            ESP_LOGW(TAG, "LSM6DS3TR WHO_AM_I failed");
            goto retry;
        }
        if (!imu_lsm303agr_whoami_ok())
        {
            ESP_LOGW(TAG, "LSM303AGR WHO_AM_I failed");
            goto retry;
        }

        // 4) Configure LSM6
        {
            lsm6_init_step_t st6 = lsm6_init_once();
            if (st6 != LSM6_STEP_OK)
            {
                ESP_LOGW(TAG,
                         "LSM6 init failed at %s",
                         safe_lsm6_step_name(st6));
                goto retry;
            }
        }

        // 5) Configure LSM303
        {
            lsm303_init_step_t st3 = lsm303_init_once();
            if (st3 != LSM303_STEP_OK)
            {
                ESP_LOGW(TAG,
                         "LSM303 init failed at %s",
                         safe_lsm303_step_name(st3));
                goto retry;
            }
        }

        // Success!
        ESP_LOGI(TAG, "IMU init complete");
        return ESP_OK;

    retry:
        if (attempt < INIT_RETRIES)
        {
            ESP_LOGI(TAG,
                     "Retrying IMU init in %d ms…",
                     RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }

    ESP_LOGE(TAG, "IMU init failed after %d attempts", INIT_RETRIES);
    return ESP_ERR_NOT_FOUND;
}

/*---------------------------------------------------------------------------*/
/*                            Data Read Functions                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Read raw sensor data into imu_data_t.
 *
 * Reads:
 *  - LSM6DS3TR accel & gyro
 *  - LSM303AGR accel & mag & temperature
 *
 * @param[out] data  Destination struct (zeroed on entry).
 * @return ESP_OK or an ESP_ERR_* code on I²C failure.
 */
esp_err_t imu_read_data(imu_data_t *data)
{
    if (!data)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(data, 0, sizeof(imu_data_t));

    int16_t buf[3];
    esp_err_t ret;

    ret = imu_read_accel_lsm6ds3tr(buf);
    if (ret != ESP_OK)
        return ret;
    data->accel1_x = buf[0];
    data->accel1_y = buf[1];
    data->accel1_z = buf[2];

    ret = imu_read_gyro_lsm6ds3tr(buf);
    if (ret != ESP_OK)
        return ret;
    data->gyro_x = buf[0];
    data->gyro_y = buf[1];
    data->gyro_z = buf[2];

    ret = imu_read_accel_lsm303agr(buf);
    if (ret != ESP_OK)
        return ret;
    data->accel2_x = buf[0];
    data->accel2_y = buf[1];
    data->accel2_z = buf[2];

    ret = imu_read_mag_lsm303agr(buf);
    if (ret != ESP_OK)
        return ret;
    data->mag_x = buf[0];
    data->mag_y = buf[1];
    data->mag_z = buf[2];

    // Read raw temperature (16-bit LSB first)
    int16_t raw_t;
    lsm6ds3tr_c_temperature_raw_get(&lsm6_ctx, &raw_t);
    data->temp_raw = raw_t >> 4; // store if you like

    return ESP_OK;
}

/**
 * @brief Read raw accel from LSM6DS3TR only.
 */
esp_err_t imu_read_accel_lsm6ds3tr(int16_t out[3])
{
    return (lsm6ds3tr_c_acceleration_raw_get(&lsm6_ctx, out) == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read raw gyro from LSM6DS3TR only.
 */
esp_err_t imu_read_gyro_lsm6ds3tr(int16_t out[3])
{
    return (lsm6ds3tr_c_angular_rate_raw_get(&lsm6_ctx, out) == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read raw accel from LSM303AGR only.
 */
esp_err_t imu_read_accel_lsm303agr(int16_t out[3])
{
    return (lsm303agr_acceleration_raw_get(&xl_ctx, out) == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read raw magnetometer from LSM303AGR only.
 */
esp_err_t imu_read_mag_lsm303agr(int16_t out[3])
{
    return (lsm303agr_magnetic_raw_get(&mg_ctx, out) == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Check LSM6DS3TR WHO_AM_I register.
 */
bool imu_lsm6ds3tr_whoami_ok(void)
{
    uint8_t id;
    if (lsm6ds3tr_c_device_id_get(&lsm6_ctx, &id) != 0)
    {
        return false;
    }
    return (id == LSM6DS3TR_C_ID);
}

/**
 * @brief Check LSM303AGR WHO_AM_I registers.
 */
bool imu_lsm303agr_whoami_ok(void)
{
    uint8_t id;
    if (lsm303agr_xl_device_id_get(&xl_ctx, &id) != 0 || id != LSM303AGR_ID_XL)
    {
        return false;
    }
    if (lsm303agr_mag_device_id_get(&mg_ctx, &id) != 0 || id != LSM303AGR_ID_MG)
    {
        return false;
    }
    return true;
}

/*---------------------------------------------------------------------------*/
/*                              Calibration                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief Full IMU calibration: gyro bias + accel/mag orientation.
 */
esp_err_t imu_calibrate(uint32_t samples, uint32_t delay_ms)
{
    if (samples == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    imu_data_t raw;
    int64_t sum_acc_l6[3] = {0}, sum_gyro_l6[3] = {0};
    int64_t sum_acc_xl[3] = {0};
    int64_t sum_mag_xl[3] = {0};

    // running‐variance for accel‐magnitude quality check
    float mean6 = 0, M2_6 = 0;    // LSM6
    float mean_xl = 0, M2_xl = 0; // LSM303

    for (uint32_t i = 0; i < samples; i++)
    {
        esp_err_t ret = imu_read_data(&raw);
        if (ret != ESP_OK)
            return ret;
        sum_acc_l6[0] += raw.accel1_x;
        sum_acc_l6[1] += raw.accel1_y;
        sum_acc_l6[2] += raw.accel1_z;
        sum_gyro_l6[0] += raw.gyro_x;
        sum_gyro_l6[1] += raw.gyro_y;
        sum_gyro_l6[2] += raw.gyro_z;
        sum_acc_xl[0] += raw.accel2_x;
        sum_acc_xl[1] += raw.accel2_y;
        sum_acc_xl[2] += raw.accel2_z;
        sum_mag_xl[0] += raw.mag_x;
        sum_mag_xl[1] += raw.mag_y;
        sum_mag_xl[2] += raw.mag_z;

        // ---- quality check: LSM6 accel magnitude ----
        float mag6 = sqrtf(
            raw.accel1_x * (float)raw.accel1_x +
            raw.accel1_y * (float)raw.accel1_y +
            raw.accel1_z * (float)raw.accel1_z);
        // Welford update
        float delta6 = mag6 - mean6;
        mean6 += delta6 / (i + 1);
        M2_6 += delta6 * (mag6 - mean6);

        // ---- quality check: LSM303 accel magnitude ----
        float magxl = sqrtf(
            raw.accel2_x * (float)raw.accel2_x +
            raw.accel2_y * (float)raw.accel2_y +
            raw.accel2_z * (float)raw.accel2_z);
        float deltaxl = magxl - mean_xl;
        mean_xl += deltaxl / (i + 1);
        M2_xl += deltaxl * (magxl - mean_xl);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // finalize standard deviations
    float var6 = (samples > 1) ? (M2_6 / (samples - 1)) : 0;
    float std6 = sqrtf(var6);
    float varxl = (samples > 1) ? (M2_xl / (samples - 1)) : 0;
    float stdxl = sqrtf(varxl);
    // ESP_LOGI(TAG,
    //          "Calib quality: LSM6 accel mag  mean=%.1f  std=%.1f counts;  "
    //          "LSM303 accel mag mean=%.1f  std=%.1f counts",
    //          mean6, std6,
    //          mean_xl, stdxl);
    if (std6 > STDDEV6_THRESH)
    {
        ESP_LOGW(TAG,
                 "Calibration: accel noise high (std6=%.1f > %.1f)",
                 std6, STDDEV6_THRESH);
    }
    if (stdxl > STDDEV_XL_THRESH)
    {
        ESP_LOGW(TAG,
                 "Calibration: accel noise high (std6=%.1f > %.1f)",
                 stdxl, STDDEV_XL_THRESH);
    }

    // Average
    float avg_acc_l6[3], avg_gyro_l6[3], avg_acc_xl[3], avg_mag_xl[3], mgxl[3];
    for (int i = 0; i < 3; i++)
    {
        avg_acc_l6[i] = sum_acc_l6[i] / (float)samples;
        avg_gyro_l6[i] = sum_gyro_l6[i] / (float)samples;
        avg_acc_xl[i] = sum_acc_xl[i] / (float)samples;
        avg_mag_xl[i] = sum_mag_xl[i] / (float)samples;
        mgxl[i] = avg_mag_xl[i];
        gyro_bias_l6[i] = avg_gyro_l6[i];
    }

    ESP_LOGI(TAG, "gyro bias: %d %d %d counts",
             (int)gyro_bias_l6[0],
             (int)gyro_bias_l6[1],
             (int)gyro_bias_l6[2]);
    // --- compute LSM6 accel_bias_l6 & rot_mat_l6 ---
    // separate avg_acc_l6 into sensor_offset + gravity_counts
    float norm6 = sqrtf(avg_acc_l6[0] * avg_acc_l6[0] + avg_acc_l6[1] * avg_acc_l6[1] + avg_acc_l6[2] * avg_acc_l6[2]);
    if (norm6 < EPSILON)
    {
        ESP_LOGE(TAG, "LSM303 accel norm is zero");
        return ESP_ERR_INVALID_STATE;
    }

    float norm = sqrtf(mgxl[0] * mgxl[0] + mgxl[1] * mgxl[1] + mgxl[2] * mgxl[2]);
    if (norm < EPSILON)
    {
        ESP_LOGE(TAG, "LSM303 mag norm is zero");
        return ESP_ERR_INVALID_STATE;
    }

    // direction of gravity
    float ug6[3] = {avg_acc_l6[0] / norm6,
                    avg_acc_l6[1] / norm6,
                    avg_acc_l6[2] / norm6};

    // direction of mag
    for (int i = 0; i < 3; i++)
        mgxl[i] /= norm;

    // we only care about the rotation—no constant accel bias removal here
    // accel_bias_xl stays at {0,0,0}

    // build rot_mat_l6: rotate unit-gravity ug6 → [0,0,1]
    compute_rodrigues(ug6, rot_mat_l6);

    // --- compute LSM303 accel_bias_xl & rot_mat_xl similarly ---
    float normxl = sqrtf(avg_acc_xl[0] * avg_acc_xl[0] + avg_acc_xl[1] * avg_acc_xl[1] + avg_acc_xl[2] * avg_acc_xl[2]);
    if (normxl < EPSILON)
    {
        ESP_LOGE(TAG, "LSM303 accel norm is zero");
        return ESP_ERR_INVALID_STATE;
    }

    // direction of gravity
    float ugxl[3] = {avg_acc_xl[0] / normxl,
                     avg_acc_xl[1] / normxl,
                     avg_acc_xl[2] / normxl};

    // no DC bias removal for LSM6 accel in this tilt‐only calibration
    // accel_bias_l6 stays at {0,0,0}

    // build rot_mat_xl: rotate unit-gravity ugxl → [0,0,1]
    // compute_rodrigues(ugxl, rot_mat_xl);
    compute_orientation(ugxl, mgxl, rot_mat_xl);

#define IMU_USE_MAG
#ifdef IMU_USE_MAG
    // ------------------------------------------------------------------------
    // Yaw‐alignment: use one LSM303 mag sample to fix the free spin about Z
    {
        imu_data_t raw;
        if (imu_read_data(&raw) == ESP_OK) {
            // 1) rotate mag into world (tilt‐only)
            float m_s[3] = { (float)raw.mag_x,
                             (float)raw.mag_y,
                             (float)raw.mag_z };
            float m_w[3];
            for (int i = 0; i < 3; i++) {
                m_w[i] = rot_mat_xl[i][0]*m_s[0]
                       + rot_mat_xl[i][1]*m_s[1]
                       + rot_mat_xl[i][2]*m_s[2];
            }

            // 2) compute heading θ = atan2(Y, X)
            float theta = atan2f(m_w[1], m_w[0]);
            // rotate by (0°- θ) so that mapped_z → world +X
            float psi = -theta;   // M_PI-theta;
            float c   = cosf(psi);
            float s   = sinf(psi);

            // 3) build Z‐axis yaw matrix R_yaw(-θ)
            float R_yaw[3][3] = {
                {  c, -s, 0 },
                {  s,  c, 0 },
                {  0,  0, 1 }
            };

            // 4) apply R_yaw to both rot_mat_xl and rot_mat_l6
            float tmp[3][3];
            // LSM303AGR
            memcpy(tmp, rot_mat_xl, sizeof(tmp));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot_mat_xl[i][j] = R_yaw[i][0]*tmp[0][j]
                                     + R_yaw[i][1]*tmp[1][j]
                                     + R_yaw[i][2]*tmp[2][j];
            // LSM6DS3TR
            memcpy(tmp, rot_mat_l6, sizeof(tmp));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot_mat_l6[i][j] = R_yaw[i][0]*tmp[0][j]
                                     + R_yaw[i][1]*tmp[1][j]
                                     + R_yaw[i][2]*tmp[2][j];
        }
    }
#else
    // Apriori yaw: align the leveled Z‐axis onto world +X (no mag)
    {
        // 1) see where sensor Z (chip‐top) ended up after tilt
        float rz_x = rot_mat_xl[0][2];
        float rz_y = rot_mat_xl[1][2];

        // 2) compute yaw to undo that spin: send (rz_x,rz_y) → (1,0)
        float yaw_z = -atan2f(rz_y, rz_x);

        // 3) build Z‐axis rotation by yaw_z
        float cz = cosf(yaw_z), sz = sinf(yaw_z);
        float R_z[3][3] = {
            {cz, -sz, 0},
            {sz, cz, 0},
            {0, 0, 1}};

        // 4) apply R_z to both sensor rotation matrices
        float tmp[3][3];
        // LSM303AGR accel/mag
        memcpy(tmp, rot_mat_xl, sizeof(tmp));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rot_mat_xl[i][j] = R_z[i][0] * tmp[0][j] + R_z[i][1] * tmp[1][j] + R_z[i][2] * tmp[2][j];
        // LSM6DS3TR accel/gyro
        memcpy(tmp, rot_mat_l6, sizeof(tmp));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rot_mat_l6[i][j] = R_z[i][0] * tmp[0][j] + R_z[i][1] * tmp[1][j] + R_z[i][2] * tmp[2][j];
    }
#endif
    return ESP_OK;
}

/**
 * @brief Fast, gyro‐only calibration.
 */
esp_err_t imu_calibrate_gyro(uint32_t samples, uint32_t delay_ms)
{
    if (samples == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t sum_g[3] = {0, 0, 0};
    int16_t raw_g[3];

    for (uint32_t i = 0; i < samples; ++i)
    {
        esp_err_t ret = imu_read_gyro_lsm6ds3tr(raw_g);
        if (ret != ESP_OK)
        {
            return ret;
        }
        sum_g[0] += raw_g[0];
        sum_g[1] += raw_g[1];
        sum_g[2] += raw_g[2];
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    for (int i = 0; i < 3; ++i)
    {
        gyro_bias_l6[i] = sum_g[i] / (float)samples;
    }
    return ESP_OK;
}
/*---------------------------------------------------------------------------*/

/**
 * @brief Read compensated (rotated & scaled) IMU data.
 *
 * Fills imu_comp_data_t:
 *  - accel & gyro from LSM6DS3TR (Earth frame, g & rad/s)
 *  - accel & mag from LSM303AGR (Earth frame, g & μT)
 *  - temperature in °C
 */
esp_err_t imu_read_compensated_data(imu_comp_data_t *d)
{
    if (!d)
    {
        return ESP_ERR_INVALID_ARG;
    }

    imu_data_t raw;
    esp_err_t ret = imu_read_data(&raw);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Temperature (LSM303) → °C
    float temp_c = 25.0f + raw.temp_raw * 0.0625f;
    d->temp_c = temp_c;

    // --- Gyro: remove bias, convert to rad/s ---
    d->gyro_x = (raw.gyro_x - gyro_bias_l6[0]) * GYRO_SENS_RAD;
    d->gyro_y = (raw.gyro_y - gyro_bias_l6[1]) * GYRO_SENS_RAD;
    d->gyro_z = (raw.gyro_z - gyro_bias_l6[2]) * GYRO_SENS_RAD;

    // --- LSM6 Accel: subtract bias, rotate, convert to g's ---
    float v6[3] = {
        raw.accel1_x - accel_bias_l6[0],
        raw.accel1_y - accel_bias_l6[1],
        raw.accel1_z - accel_bias_l6[2]};
    float ae6[3];
    for (int i = 0; i < 3; ++i)
    {
        ae6[i] = rot_mat_l6[i][0] * v6[0] + rot_mat_l6[i][1] * v6[1] + rot_mat_l6[i][2] * v6[2];
    }

    // LSM6 accel sensitivity: ±2 g → 0.000061 g/LSB (output in g’s)
    d->accel1_x = ae6[0] * ACC_SENS_L6;
    d->accel1_y = ae6[1] * ACC_SENS_L6;
    d->accel1_z = ae6[2] * ACC_SENS_L6;

    // --- LSM303 Accel + Mag: subtract bias, rotate ---
    float vxl[3] = {
        raw.accel2_x - accel_bias_xl[0],
        raw.accel2_y - accel_bias_xl[1],
        raw.accel2_z - accel_bias_xl[2]};
    float vmg[3] = {
        (float)raw.mag_x,
        (float)raw.mag_y,
        (float)raw.mag_z};
    float ae_xl[3], me_xl[3];
    for (int i = 0; i < 3; ++i)
    {
        ae_xl[i] = rot_mat_xl[i][0] * vxl[0] + rot_mat_xl[i][1] * vxl[1] + rot_mat_xl[i][2] * vxl[2];
        me_xl[i] = rot_mat_xl[i][0] * vmg[0] + rot_mat_xl[i][1] * vmg[1] + rot_mat_xl[i][2] * vmg[2];
    }
    // Invert Y axis on LSM303 to align with LSM6 frame
    ae_xl[1] = -ae_xl[1];
    me_xl[1] = -me_xl[1];

    // Scale LSM303 accel sensitivity: ±2 g → 0.000061 g/LSB (output in g’s)
    d->accel2_x = ae_xl[0] * ACC_SENS_XL;
    d->accel2_y = ae_xl[1] * ACC_SENS_XL;
    d->accel2_z = ae_xl[2] * ACC_SENS_XL;

    // // Scale LSM303 magnetometer to gauss (1.5 mG per LSB at ±50 gauss)
    d->mag_x = me_xl[0] * MAG_SENS_XL;
    d->mag_y = me_xl[1] * MAG_SENS_XL;
    d->mag_z = me_xl[2] * MAG_SENS_XL;

    // // sensor-frame basis vector pointing “out of the chip top”
    // const float ez_s[3] = { 0.0f, 0.0f, 1.0f };

    // // world-frame target
    // float mapped[3];

    // // multiply: mapped = R * ez_s
    // for (int i = 0; i < 3; i++) {
    //     mapped[i] = rot_mat_xl[i][0]*ez_s[0] + rot_mat_xl[i][1]*ez_s[1] + rot_mat_xl[i][2]*ez_s[2];
    // }

    //     // expectR * [0,0,1]_s  =  [ 1.000,  0.000,  0.000 ]
    // ESP_LOGI(TAG,
    //     "R * [0,0,1]_sensor  = [%.3f, %.3f, %.3f]",
    //     mapped[0], mapped[1], mapped[2]);

    // after you’ve read d.mag_x, d.mag_y, d.mag_z in imu_read_compensated_data():
    // float mx = d->mag_x;    // –0.000
    // float my = d->mag_y;    // –0.230
    // // ignore d.mag_z (vertical component)

    // float heading = atan2f(my, mx) * 180.0f / M_PI;
    // if (heading < 0) heading += 360.0f;
    // ESP_LOGI(TAG, "Magnetic heading = %.1f°", heading);

    return ESP_OK;
}

#if 0
//------------------------------------------------------------------------
// imu_read_normalized_data: body‐frame, SI‐units for AHRS filter
//------------------------------------------------------------------------
esp_err_t imu_read_normalized_data( imu_comp_data_t *comp )
{
    if (!comp) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(comp, 0, sizeof(*comp));

    imu_data_t d;
    esp_err_t ret = imu_read_data(&d);
    if (ret != ESP_OK) {
        return ret;
    }

    // 1) Accel1 → [g]
    comp->norm_accel1[0] = d.accel1_x * ACC_SENS_G_PER_LSB;
    comp->norm_accel1[1] = d.accel1_y * ACC_SENS_G_PER_LSB;
    comp->norm_accel1[2] = d.accel1_z * ACC_SENS_G_PER_LSB;

    // 2) Accel2 → [g]
    comp->norm_accel2[0] = d.accel2_x * ACC2_SENS_G_PER_LSB;
    comp->norm_accel2[1] = d.accel2_y * ACC2_SENS_G_PER_LSB;
    comp->norm_accel2[2] = d.accel2_z * ACC2_SENS_G_PER_LSB;

    // 3) Gyro → [rad/s], remove bias then convert:
    //    raw counts → °/s → rad/s
    const float dps2rad = (M_PI / 180.0f);
    comp->norm_gyro[0] = (d.gyro_x - gyro_bias_l6[0])
                      * GYRO_SENS_DPS_PER_LSB
                      * dps2rad;
    comp->norm_gyro[1] = (d.gyro_y - gyro_bias_l6[1])
                      * GYRO_SENS_DPS_PER_LSB
                      * dps2rad;
    comp->norm_gyro[2] = (d.gyro_z - gyro_bias_l6[2])
                      * GYRO_SENS_DPS_PER_LSB
                      * dps2rad;

    // 4) Mag → [µT]
    comp->norm_mag[0] = d.mag_x * MAG_SENS_UT_PER_LSB;
    comp->norm_mag[1] = d.mag_y * MAG_SENS_UT_PER_LSB;
    comp->norm_mag[2] = d.mag_z * MAG_SENS_UT_PER_LSB;

    // 5) Temp → [°C]
    comp->temp_c = d.temp_raw * TEMP_SENS_C_PER_LSB + TEMP_OFFSET;

    return ESP_OK;
}
#endif

esp_err_t imu_read_normalized_data(imu_comp_data_t *comp)
{
    if (!comp)
    {
        return ESP_ERR_INVALID_ARG;
    }
    memset(comp, 0, sizeof(*comp));

    imu_data_t d;
    esp_err_t ret = imu_read_data(&d);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 1) Accel1: raw → [g]
    float ax_g = d.accel1_x * ACC_SENS_G_PER_LSB;
    float ay_g = d.accel1_y * ACC_SENS_G_PER_LSB;
    float az_g = d.accel1_z * ACC_SENS_G_PER_LSB;

    //    then rotate into body frame

    comp->norm_accel1[0] = rot_mat_l6[0][0] * ax_g + rot_mat_l6[0][1] * ay_g + rot_mat_l6[0][2] * az_g;
    comp->norm_accel1[1] = rot_mat_l6[1][0] * ax_g + rot_mat_l6[1][1] * ay_g + rot_mat_l6[1][2] * az_g;
    comp->norm_accel1[2] = rot_mat_l6[2][0] * ax_g + rot_mat_l6[2][1] * ay_g + rot_mat_l6[2][2] * az_g;

    // 2) Accel2 → [g]
    comp->norm_accel2[0] = d.accel2_x * ACC2_SENS_G_PER_LSB;
    comp->norm_accel2[1] = d.accel2_y * ACC2_SENS_G_PER_LSB;
    comp->norm_accel2[2] = d.accel2_z * ACC2_SENS_G_PER_LSB;

    // 3) Gyro: raw counts → remove bias → [°/s] → [rad/s]
    const float dps2rad = (M_PI / 180.0f);
    float gx_rads = (d.gyro_x - gyro_bias_l6[0]) * GYRO_SENS_DPS_PER_LSB * dps2rad;
    float gy_rads = (d.gyro_y - gyro_bias_l6[1]) * GYRO_SENS_DPS_PER_LSB * dps2rad;
    float gz_rads = (d.gyro_z - gyro_bias_l6[2]) * GYRO_SENS_DPS_PER_LSB * dps2rad;

    //    then rotate into body frame

    comp->norm_gyro[0] = rot_mat_l6[0][0] * gx_rads + rot_mat_l6[0][1] * gy_rads + rot_mat_l6[0][2] * gz_rads;
    comp->norm_gyro[1] = rot_mat_l6[1][0] * gx_rads + rot_mat_l6[1][1] * gy_rads + rot_mat_l6[1][2] * gz_rads;
    comp->norm_gyro[2] = rot_mat_l6[2][0] * gx_rads + rot_mat_l6[2][1] * gy_rads + rot_mat_l6[2][2] * gz_rads;

    // 4) Mag: raw → [µT]
    float mx_uT = d.mag_x * MAG_SENS_UT_PER_LSB;
    float my_uT = d.mag_y * MAG_SENS_UT_PER_LSB;
    float mz_uT = d.mag_z * MAG_SENS_UT_PER_LSB;

    //    then rotate into body frame

    comp->norm_mag[0] = rot_mat_xl[0][0] * mx_uT + rot_mat_xl[0][1] * my_uT + rot_mat_xl[0][2] * mz_uT;
    comp->norm_mag[1] = rot_mat_xl[1][0] * mx_uT + rot_mat_xl[1][1] * my_uT + rot_mat_xl[1][2] * mz_uT;
    comp->norm_mag[2] = rot_mat_xl[2][0] * mx_uT + rot_mat_xl[2][1] * my_uT + rot_mat_xl[2][2] * mz_uT;

    // 5) Temp → [°C]
    comp->temp_c = d.temp_raw * TEMP_SENS_C_PER_LSB + TEMP_OFFSET;

    return ESP_OK;
}


/**
 * @brief  Convert the current L6 rotation matrix to a quaternion.
 * @param  w_out Pointer to scalar quaternion output
 * @param  x_out Pointer to X component output
 * @param  y_out Pointer to Y component output
 * @param  z_out Pointer to Z component output
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_get_calibrated_quaternion(float *w_out, float *x_out, float *y_out, float *z_out)
{
    float (*m)[3] = rot_mat_l6;
    float trace = m[0][0] + m[1][1] + m[2][2];
    float qw, qx, qy, qz;
    if (trace > 0.0f) {
        float S = sqrtf(trace + 1.0f) * 2.0f;
        qw = 0.25f * S;
        qx = (m[2][1] - m[1][2]) / S;
        qy = (m[0][2] - m[2][0]) / S;
        qz = (m[1][0] - m[0][1]) / S;
    } else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
        float S = sqrtf(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
        qw = (m[2][1] - m[1][2]) / S;
        qx = 0.25f * S;
        qy = (m[0][1] + m[1][0]) / S;
        qz = (m[0][2] + m[2][0]) / S;
    } else if (m[1][1] > m[2][2]) {
        float S = sqrtf(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
        qw = (m[0][2] - m[2][0]) / S;
        qx = (m[0][1] + m[1][0]) / S;
        qy = 0.25f * S;
        qz = (m[1][2] + m[2][1]) / S;
    } else {
        float S = sqrtf(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
        qw = (m[1][0] - m[0][1]) / S;
        qx = (m[0][2] + m[2][0]) / S;
        qy = (m[1][2] + m[2][1]) / S;
        qz = 0.25f * S;
    }

    /* normalize, to guard against numeric drift in rot_mat_l6 */
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0f) {
        *w_out /= norm;
        *x_out /= norm;
        *y_out /= norm;
        *z_out /= norm;
    }
    *w_out = qw;
    *x_out = qx;
    *y_out = qy;
    *z_out = qz;
    return ESP_OK;
}


esp_err_t imu_read_l6_combined(imu_l6_comp_norm_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw_acc[3], raw_g[3];
    esp_err_t ret;

    // 1) Fetch raw counts from LSM6DS3TR
    ret = imu_read_accel_lsm6ds3tr(raw_acc);
    if (ret != ESP_OK) return ret;
    ret = imu_read_gyro_lsm6ds3tr(raw_g);
    if (ret != ESP_OK) return ret;

    // 2) Normalize accel → [g], rotate into body frame
    float ax = raw_acc[0] * ACC_SENS_L6;
    float ay = raw_acc[1] * ACC_SENS_L6;
    float az = raw_acc[2] * ACC_SENS_L6;
    out->norm_accel[0] = rot_mat_l6[0][0]*ax + rot_mat_l6[0][1]*ay + rot_mat_l6[0][2]*az;
    out->norm_accel[1] = rot_mat_l6[1][0]*ax + rot_mat_l6[1][1]*ay + rot_mat_l6[1][2]*az;
    out->norm_accel[2] = rot_mat_l6[2][0]*ax + rot_mat_l6[2][1]*ay + rot_mat_l6[2][2]*az;

    // 3) Compute bias-removed gyro → [rad/s] in sensor frame
    float bgx = gyro_bias_l6[0], bgy = gyro_bias_l6[1], bgz = gyro_bias_l6[2];
    float cgx = (raw_g[0] - bgx) * GYRO_SENS_RAD;
    float cgy = (raw_g[1] - bgy) * GYRO_SENS_RAD;
    float cgz = (raw_g[2] - bgz) * GYRO_SENS_RAD;
    out->comp_gyro[0] = cgx;
    out->comp_gyro[1] = cgy;
    out->comp_gyro[2] = cgz;

    // 4) Normalize those rates → [rad/s] in body frame
    out->norm_gyro[0] = rot_mat_l6[0][0]*cgx + rot_mat_l6[0][1]*cgy + rot_mat_l6[0][2]*cgz;
    out->norm_gyro[1] = rot_mat_l6[1][0]*cgx + rot_mat_l6[1][1]*cgy + rot_mat_l6[1][2]*cgz;
    out->norm_gyro[2] = rot_mat_l6[2][0]*cgx + rot_mat_l6[2][1]*cgy + rot_mat_l6[2][2]*cgz;

    return ESP_OK;
}
/** @} */ // end of IMU_Driver
