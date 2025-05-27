#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "imu.h"
#include "mahony.h"
#include <math.h>

static const char* TAG = "DOOR_ANGLE";

// Task parameters
#define DOOR_SAMPLE_HZ    50
#define DOOR_PUB_HZ        5
#define SAMPLE_PERIOD_MS  (1000/DOOR_SAMPLE_HZ)
#define PUB_PERIOD_MS     (1000/DOOR_PUB_HZ)

// Event bit to trigger reset & start
#define EV_DOOR_START     BIT0

// Shared event group handle (create in app_main)
extern EventGroupHandle_t g_door_events;

// If you need to send an MQTT handle, you can extend this struct:
typedef struct {
    void *mqtt_session;  // your MQTT session type
} door_task_param_t;

/**
 * @brief Door-angle task: waits for RFID trigger, then runs filter.
 */
static void door_angle_task(void *pv)
{
    door_task_param_t *param = (door_task_param_t*)pv;
    float yaw, pitch, roll;
    TickType_t last_sample = xTaskGetTickCount();
    TickType_t last_pub    = xTaskGetTickCount();

    // initialize Mahony once (you can recalibrate in imu_calibrate as needed)
    mahony_init(1.0f/DOOR_SAMPLE_HZ, /*Kp=*/3.0f, /*Ki=*/0.0f);
    mahony_set_lpf_alpha(0.9f);

    // wait for RFID “go” event
    xEventGroupWaitBits(g_door_events, EV_DOOR_START,
                        pdTRUE,    // clear bit on exit
                        pdFALSE,   // wait any bit
                        portMAX_DELAY);

    // reset filter to zero at the moment of unlock
    mahony_reset();

    while (true) {
        // 1) sample IMU at DOOR_SAMPLE_HZ
        if (xTaskGetTickCount() - last_sample >= pdMS_TO_TICKS(SAMPLE_PERIOD_MS)) {
            imu_comp_data_t comp;
            if (imu_read_normalized_data(&comp) == ESP_OK) {
                mahony_ahrs_update_imu(
                    comp.norm_gyro1[0],
                    comp.norm_gyro1[1],
                    comp.norm_gyro1[2],
                    comp.norm_accel1[0],
                    comp.norm_accel1[1],
                    comp.norm_accel1[2]
                );
            }
            last_sample = xTaskGetTickCount();
        }

        // 2) publish or log at DOOR_PUB_HZ
        if (xTaskGetTickCount() - last_pub >= pdMS_TO_TICKS(PUB_PERIOD_MS)) {
            mahony_get_euler_lpf(&yaw, &pitch, &roll);
            float door_deg = yaw * (180.0f/M_PI);

            // debug logging
            ESP_LOGD(TAG, "DoorAngle=%.2f°", door_deg);

            // example MQTT publish (pseudocode)
            // mqtt_publish(param->mqtt_session, "door/angle", &door_deg, sizeof(door_deg));

            last_pub = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // small yield
    }
}
