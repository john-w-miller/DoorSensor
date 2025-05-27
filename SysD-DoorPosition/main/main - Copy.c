// main.c
// Example application that exercises the door_position module.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "door_position.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    esp_err_t err;

    esp_log_level_set("door_position", ESP_LOG_DEBUG);
    esp_log_level_set("*", ESP_LOG_DEBUG);


    // Initialize door-position subsystem
    ESP_LOGI(TAG, "Initializing door position...");
    err = door_position_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "door_position_init() failed: 0x%02X", err);
        return;
    }

    ESP_LOGI(TAG, "Initialization complete. Starting angle loop.");

    // Main loop: print door angle once per second
    while (true) {
        float angle = door_position_get_angle();
        if (!isnan(angle)) {
            ESP_LOGI(TAG, "Door angle = %.3f rad (%.1f°)",
                     angle, angle * (180.0f / M_PI));
        } else {
            ESP_LOGW(TAG, "Invalid angle reading");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
