#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "imu.h"
#include "mahony.h"
#include <math.h>

static const char* TAG = "door_angle";

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
static void door_angle_task(void *pv);