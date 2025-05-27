// door_state.c

#include <math.h>
#include <stdbool.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "door_state.h"
#include "mahony.h"

static const char *TAG = "door_state";
static door_state_t state;
static TickType_t candidate_ts;
static float yaw_at_candidate;

#define SESSION_STATIC_SAMPLES  50

// thresholds
#define OPEN_ANGLE_THRESHOLD_RAD (20.0f * M_PI / 180.0f)
#define CLOSE_ANGLE_THRESHOLD_RAD (15.0f * M_PI / 180.0f)
#define ACCEL_STATIONARY_THRESH 0.05f // | |a|−1 | < 0.05
#define STATIONARY_TIME_MS 1000       // 1 s
#define RAD_TO_DEG (180.0f / M_PI)
// how much fused‐yaw drift (total) you allow over that time:
#define YAW_STATIONARY_THRESH_RAD (0.5f * M_PI / 180.0f) // 0.5°

EventGroupHandle_t door_event_group = NULL;
static door_state_t state;
static TickType_t   candidate_ts;
static float        yaw_at_candidate;
static int          static_count = 0;

// true once we’ve fired a Door Open event, false after Door Closed
static bool door_opened_flag = false;

static void on_door_open(void)
{
    // only fire once per actual open
    if (!door_opened_flag) {
        door_opened_flag = true;
        ESP_LOGI(TAG, "Door opened");
        mahony_on_door_open_event();
        // TODO: publish “door open” message
    }
}

static void on_door_closed(void)
{
    // only fire once per actual close
    if (door_opened_flag) {
        door_opened_flag = false;
        ESP_LOGI(TAG, "Door closed");
        mahony_on_door_closed_event();
        // TODO: publish “door closed” message
    }
}


void door_signal_session_end(void)
{
    if (door_event_group) {
        xEventGroupSetBits(door_event_group, DOOR_EVENT_SESSION_END);
    }
}

void door_state_init(void)
{
    state = DOOR_STATE_CLOSED;
    candidate_ts = 0;
    yaw_at_candidate = 0;
    // create the event group for session-end signaling
    door_event_group = xEventGroupCreate(); 
    
    // Treat startup as a “closed” event so our bias‐LPF begins warming up immediately
    mahony_on_door_closed_event();
}

void door_state_update(float yaw,
                       float ax, float ay, float az,
                       float gx, float gy, float gz)
{
    // 1) stationarity checks
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    bool accel_ok = fabsf(accel_mag - 1.0f) < ACCEL_STATIONARY_THRESH;

    TickType_t now = xTaskGetTickCount();

    // 1) pull the session-end event bit (atomic) and clear it
    EventBits_t ev = xEventGroupGetBits(door_event_group) & DOOR_EVENT_SESSION_END;
    if (ev) {
        xEventGroupClearBits(door_event_group, DOOR_EVENT_SESSION_END);
        static_count = 0;
    }

    // 2) if session-end signaled & we're not already CLOSED, force-close on static accel
    if (ev && state != DOOR_STATE_CLOSED) {
        if (accel_ok) {
            if (++static_count >= SESSION_STATIC_SAMPLES) {
                state = DOOR_STATE_CLOSED;
                on_door_closed();
                static_count = 0;
            }
        } else {
            static_count = 0;
        }
        return;  // skip normal angle‐based FSM this cycle
    }

    switch (state)
    {
    case DOOR_STATE_CLOSED:
        if (yaw > OPEN_ANGLE_THRESHOLD_RAD)
        {
            state = DOOR_STATE_OPEN;
            on_door_open(); // only here do we fire “door open”
        }
        else
        {
            ESP_LOGI(TAG,
                     "cand: yaw=%.1f°, |Δa|=%.3f, dt=%.0fms",
                     yaw * RAD_TO_DEG,
                     fabsf(accel_mag - 1.0f),
                     (float)(now - candidate_ts) * portTICK_PERIOD_MS);
        }
        break;
    case DOOR_STATE_OPEN:
        // as soon as we drop below the “close” angle, start candidate
        if (yaw < CLOSE_ANGLE_THRESHOLD_RAD)
        {
            state = DOOR_STATE_CLOSED_CANDIDATE;
            candidate_ts = now;
            yaw_at_candidate = yaw;
        }
        else
        {
            ESP_LOGI(TAG,
                     "cand: yaw=%.1f°, |Δa|=%.3f, dt=%.0fms",
                     yaw * RAD_TO_DEG,
                     fabsf(accel_mag - 1.0f),
                     (float)(now - candidate_ts) * portTICK_PERIOD_MS);
        }
        break;
    case DOOR_STATE_CLOSED_CANDIDATE:

        // if we swing back above CLOSE threshold, abort candidate
        if (yaw > CLOSE_ANGLE_THRESHOLD_RAD)
        {
            state = DOOR_STATE_OPEN;
            ESP_LOGI(TAG, "CANDIDATE→OPEN (yaw=%.1f°)", yaw * RAD_TO_DEG);
            break;
        }

        // now we’re truly in the candidate window—log every 100 ms for debugging
        if ((now - candidate_ts) % pdMS_TO_TICKS(100) == 0)
        {
            ESP_LOGI(TAG,
                     "CANDIDATE: yaw=%.1f°, Δyaw=%.2f°, Δa=%.3f, dt=%.0fms",
                     yaw * RAD_TO_DEG,
                     fabsf(yaw - yaw_at_candidate) * RAD_TO_DEG,
                     fabsf(accel_mag - 1.0f),
                     (float)(now - candidate_ts) * portTICK_PERIOD_MS);
        }

        // otherwise check for true rest:
        {
            float dt = now - candidate_ts;
            float dyaw = fabsf(yaw - yaw_at_candidate);
            if (accel_ok && dyaw < YAW_STATIONARY_THRESH_RAD && dt >= pdMS_TO_TICKS(STATIONARY_TIME_MS))
            {
                state = DOOR_STATE_CLOSED;
                on_door_closed();
            }
            else if (!accel_ok || dyaw >= YAW_STATIONARY_THRESH_RAD)
            {
                // bump detected → restart the candidate timer
                ESP_LOGI(TAG,
                         "CANDIDATE reset: Δyaw=%.2f°, Δa=%.3f",
                         dyaw * RAD_TO_DEG,
                         fabsf(accel_mag - 1.0f));
                candidate_ts = now;
                yaw_at_candidate = yaw;
            }
            else
            {
                // still in candidate, but not stationary yet
                ESP_LOGI(TAG,
                         "CANDIDATE: still waiting… Δyaw=%.2f°, Δa=%.3f",
                         dyaw * RAD_TO_DEG,
                         fabsf(accel_mag - 1.0f));
            }
        }
        break;
    }
}
