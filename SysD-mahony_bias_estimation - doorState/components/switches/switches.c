/**
 * @file switches.c
 * @brief Implementation of the switch handling module with long press detection.
 *
 * This file configures two hardware switches (WPS and Tag) using GPIO interrupts.
 * It includes debouncing logic and long press detection using FreeRTOS one-shot timers.
 * The GPIO interrupts are configured for ANYEDGE so that both press and release events are captured.
 * When a button is pressed, a timer is started. If the button remains pressed for two seconds,
 * a long press event is generated immediately. Upon release, the callback is also called with
 * the appropriate long press flag.
 */

 #include "switches.h"
 #include "driver/gpio.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/timers.h"
 
 #define WPS_GPIO    GPIO_NUM_6    /**< GPIO for the WPS switch (active low when pressed) */
 #define TAG_GPIO    GPIO_NUM_40   /**< GPIO for the Tag switch (active high when pressed) */
 
 #define DEBOUNCE_TIME_MS 50           /**< Debounce time in milliseconds */
 #define LONG_PRESS_TIME_MS 2000       /**< Duration threshold for a long press in milliseconds */
 
 static const char *TAG = "Switches";
 
 /* Global state variables used for debouncing and long press detection. */
 static TickType_t last_tick[SWITCH_COUNT] = {0};         /**< Last event tick for each switch */
 static bool last_state[SWITCH_COUNT] = { false, false };   /**< Last stable state for each switch */
 static TickType_t press_time[SWITCH_COUNT] = {0};          /**< Tick when the switch was pressed */
 static bool long_press_triggered[SWITCH_COUNT] = { false, false }; /**< Flag indicating a long press event was triggered */
 static TimerHandle_t long_press_timer[SWITCH_COUNT] = { NULL, NULL }; /**< One-shot timers for long press detection */
 
 /**
  * @brief Structure representing a switch event.
  */
 typedef struct {
     switch_id_t id;         /**< Identifier of the switch */
     bool pressed;           /**< Switch state: true if pressed, false if released */
     TickType_t tick;        /**< Tick count when the event occurred */
 } switch_event_t;
 
 static QueueHandle_t switch_queue = NULL;       /**< Queue to transfer switch events from the ISR to the processing task */
 static switch_callback_t user_callback = NULL;    /**< Pointer to the user-registered callback function */
 
 /**
  * @brief Timer callback for long press detection.
  *
  * This callback is invoked when the long press timer expires. It checks if the switch
  * is still pressed and, if so, marks the event as a long press and calls the user callback.
  *
  * @param xTimer Handle to the timer.
  */
 static void long_press_timer_callback(TimerHandle_t xTimer)
 {
     uint32_t id = (uint32_t) pvTimerGetTimerID(xTimer);
     // If the switch is still pressed, mark as long press and call the callback immediately.
     if (last_state[id]) {
         long_press_triggered[id] = true;
         ESP_LOGI(TAG, "Switch %lu long press detected", id);
         if (user_callback) {
             user_callback(id, true, true);
         }
     }
 }
 
 /**
  * @brief Register a callback function to handle debounced switch events.
  *
  * @param callback The function to be called when a debounced switch event occurs.
  */
 void switches_register_callback(switch_callback_t callback)
 {
     user_callback = callback;
 }
 
 /**
  * @brief GPIO ISR handler for switch events.
  *
  * This handler is executed in interrupt context. It reads the current state of the
  * triggered GPIO and posts a switch event to the queue.
  *
  * @param arg Pointer to the GPIO number that triggered the interrupt.
  */
 static void IRAM_ATTR gpio_isr_handler(void* arg)
 {
     uint32_t gpio_num = (uint32_t) arg;
     switch_id_t id;
     bool pressed;
 
     if (gpio_num == WPS_GPIO) {
         id = SWITCH_WPS;
         // For the WPS switch, a low level indicates the switch is pressed.
         pressed = (gpio_get_level(WPS_GPIO) == 0);
     } else if (gpio_num == TAG_GPIO) {
         id = SWITCH_TAG;
         // For the Tag switch, a high level indicates the switch is pressed.
         pressed = (gpio_get_level(TAG_GPIO) == 1);
     } else {
         return;
     }
 
     switch_event_t event;
     event.id = id;
     event.pressed = pressed;
     event.tick = xTaskGetTickCountFromISR();
 
     xQueueSendFromISR(switch_queue, &event, NULL);
 }
 
 /**
  * @brief Task to process switch events, perform debouncing, and detect long presses.
  *
  * This task continuously monitors the switch event queue. It applies a time-based
  * debounce filter, starts and stops timers for long press detection, and triggers a callback
  * with the event details.
  *
  * @param arg Unused parameter.
  */
 static void switch_task(void* arg)
 {
     switch_event_t event;
 
     while (1) {
         if (xQueueReceive(switch_queue, &event, portMAX_DELAY)) {
             // Ignore duplicate events for the same state.
             if (event.pressed == last_state[event.id]) {
                 continue;
             }
             // Debounce: ignore events that occur too soon after the previous one.
             if (((event.tick - last_tick[event.id]) * portTICK_PERIOD_MS) < DEBOUNCE_TIME_MS) {
                 continue;
             }
             last_tick[event.id] = event.tick;
 
             if (event.pressed) {
                 // Button press: record the press time, reset long press flag,
                 // update state, and (re)start the long press timer.
                 last_state[event.id] = true;
                 press_time[event.id] = event.tick;
                 long_press_triggered[event.id] = false;
                 xTimerStop(long_press_timer[event.id], 0);
                 xTimerStart(long_press_timer[event.id], 0);
                 ESP_LOGI(TAG, "Switch %d pressed", event.id);
                 if (user_callback) {
                     user_callback(event.id, true, false);
                 }
             } else {
                 // Button release: update state and stop the long press timer.
                 last_state[event.id] = false;
                 xTimerStop(long_press_timer[event.id], 0);
                 TickType_t duration_ticks = event.tick - press_time[event.id];
                 uint32_t duration_ms = duration_ticks * portTICK_PERIOD_MS;
                 ESP_LOGI(TAG, "Switch %d released after %lu ms (%s press)", 
                          event.id, duration_ms, long_press_triggered[event.id] ? "long" : "short");
                 if (user_callback) {
                     user_callback(event.id, false, long_press_triggered[event.id]);
                 }
             }
         }
     }
 }
 
 /**
  * @brief Initialize the switch module.
  *
  * This function creates the event queue, starts the processing task, configures GPIOs
  * for the switches, installs the ISR service, attaches the ISR handlers, and creates long press timers.
  * The GPIO interrupts are configured for ANYEDGE to capture both press and release events.
  */
 void switches_init(void)
 {
     // Create queue for switch events.
     switch_queue = xQueueCreate(10, sizeof(switch_event_t));
     if (switch_queue == NULL) {
         ESP_LOGE(TAG, "Failed to create switch event queue");
         return;
     }
 
     // Create the task that processes switch events.
     xTaskCreate(switch_task, "switch_task", 4096, NULL, 10, NULL);
 
     // Create long press timers for each switch.
     for (int i = 0; i < SWITCH_COUNT; i++) {
         long_press_timer[i] = xTimerCreate("lp_timer", pdMS_TO_TICKS(LONG_PRESS_TIME_MS), pdFALSE, (void *)(uintptr_t) i, long_press_timer_callback);
         if (long_press_timer[i] == NULL) {
             ESP_LOGE(TAG, "Failed to create long press timer for switch %d", i);
         }
     }
 
     // Configure the WPS switch GPIO (pull-up, any edge trigger).
     gpio_config_t io_conf_wps = {
         .intr_type = GPIO_INTR_ANYEDGE,
         .mode = GPIO_MODE_INPUT,
         .pin_bit_mask = (1ULL << WPS_GPIO),
         .pull_up_en = GPIO_PULLUP_ENABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE
     };
     gpio_config(&io_conf_wps);
 
     // Configure the Tag switch GPIO (pull-down, any edge trigger).
     gpio_config_t io_conf_tag = {
         .intr_type = GPIO_INTR_ANYEDGE,
         .mode = GPIO_MODE_INPUT,
         .pin_bit_mask = (1ULL << TAG_GPIO),
         .pull_up_en = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_ENABLE
     };
     gpio_config(&io_conf_tag);
 
     // Install the GPIO ISR service.
     gpio_install_isr_service(0);
     gpio_isr_handler_add(WPS_GPIO, gpio_isr_handler, (void*) WPS_GPIO);
     gpio_isr_handler_add(TAG_GPIO, gpio_isr_handler, (void*) TAG_GPIO);
 
     ESP_LOGI(TAG, "Switches initialized");
 }
 