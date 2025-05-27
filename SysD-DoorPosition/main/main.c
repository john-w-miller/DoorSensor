// in app_main.c
#include "freertos/event_groups.h"

// global event group
EventGroupHandle_t g_door_events;

void app_main(void)
{
    // create the event group
    g_door_events = xEventGroupCreate();

    // start door_angle_task
    static door_task_param_t door_params = {
        .mqtt_session = /* your handle here */
    };
    xTaskCreate(
        door_angle_task,
        "door_angle",
        4*1024,
        &door_params,
        tskIDLE_PRIORITY+1,
        NULL
    );

    // ... init IMU, RFID, MQTT, etc. ...
}
