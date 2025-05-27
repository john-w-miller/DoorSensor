// door_state.h
#ifndef DOOR_STATE_H
#define DOOR_STATE_H

#include <freertos/FreeRTOS.h>

typedef enum {
    DOOR_STATE_CLOSED,
    DOOR_STATE_CLOSED_CANDIDATE,
    DOOR_STATE_OPEN,
} door_state_t;

/**
 * @brief  Initialize the door‐state machine.
 */
void door_state_init(void);

/**
 * @brief  Update door state each cycle.
 * @param  yaw  Filtered yaw angle (rad).
 * @param  ax,ay,az  Raw accel components (normalized so |a|≈1.0).
 * @param  gx,gy,gz  Bias-corrected gyro components (rad/s).
 */
void door_state_update(float yaw,
                       float ax, float ay, float az,
                       float gx, float gy, float gz);

/**
 * @brief  Signal from another task that the feeding session is ending.
 *         This sets the SESSION_END bit; the FSM will force-close
 *         once accel is static over several samples.
 */
void door_signal_session_end(void);

// The FreeRTOS event group handle for door signals
extern EventGroupHandle_t door_event_group;

// Bit mask in door_event_group for session-end
#define DOOR_EVENT_SESSION_END  (1 << 0)


#endif // DOOR_STATE_H
