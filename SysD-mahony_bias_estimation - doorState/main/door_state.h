// door_state.h
#ifndef DOOR_STATE_H
#define DOOR_STATE_H

/**
 * @file door_state.h
 * @brief Door-state finite–state machine for open/closed detection.
 *
 * This module implements a simple FSM that:
 *  - Uses filtered yaw and accel stationarity to detect when the door
 *    has truly closed or opened (with hysteresis and debounce).
 *  - Exposes `door_state_update()` to be called each IMU cycle.
 *  - Supports an external “session end” event to force a close when
 *    the system becomes static.
 *
 * @defgroup DoorState Door State Machine
 * @{
 */

#include <freertos/FreeRTOS.h>

typedef enum {
    DOOR_STATE_CLOSED,
    DOOR_STATE_CLOSED_CANDIDATE,
    DOOR_STATE_OPEN,
} door_state_t;



#define SESSION_STATIC_SAMPLES  50
/**< Number of consecutive accel‐static samples to force-close when session ends */

// thresholds
#define OPEN_ANGLE_THRESHOLD_RAD (20.0f * M_PI / 180.0f)
#define CLOSE_ANGLE_THRESHOLD_RAD (15.0f * M_PI / 180.0f)
#define ACCEL_STATIONARY_THRESH 0.05f // | |a|−1 | < 0.05
#define STATIONARY_TIME_MS 1000       // 1 s
#define RAD_TO_DEG (180.0f / M_PI)
// how much fused‐yaw drift (total) you allow over that time:
#define YAW_STATIONARY_THRESH_RAD (0.5f * M_PI / 180.0f) // 0.5°


/**
 * @brief   Initialize the door-state FSM.
 *
 * Creates the FreeRTOS event group for session‐end signalling and
 * primes the Mahony bias logic as if the door had just closed.
 *
 * @return  ESP_OK          on success
 * @return  ESP_ERR_NO_MEM  if the event group allocation failed
 */
esp_err_t door_state_init(void);

/**
 * @brief   Run one cycle of the door-state FSM.
 *
 * Must be called at each IMU sample (e.g. 50 Hz) with:
 *  - `yaw`  = filtered yaw [rad]
 *  - `ax,ay,az` = accel components (normalized so |a|≈1.0 g)
 *  - `gx,gy,gz` = bias‐corrected gyro rates [rad/s]
 *
 * @param   yaw  current yaw angle (rad)
 * @param   ax   accel X (g)
 * @param   ay   accel Y (g)
 * @param   az   accel Z (g)
 * @param   gx   gyro X (rad/s)
 * @param   gy   gyro Y (rad/s)
 * @param   gz   gyro Z (rad/s)
 */

void door_state_update(float yaw,
                       float ax, float ay, float az,
                       float gx, float gy, float gz);

/**
 * @brief   Signal from another task that the feeding session is ending.
 * @details Sets an event bit; the FSM will force a door‐closed event
 *          once `SESSION_STATIC_SAMPLES` of accel‐static samples occur.
 */
void door_signal_session_end(void);

/** FreeRTOS event group for door‐state signals (session end, future bits). */
extern EventGroupHandle_t door_event_group;

/**
 * @brief   Event‐group bit mask for “session end” signal.
 * @ingroup DoorState
 */
#define DOOR_EVENT_SESSION_END  (1 << 0)

/** @} */ // end of DoorState
#endif // DOOR_STATE_H
