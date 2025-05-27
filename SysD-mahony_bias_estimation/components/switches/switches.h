/**
 * @file switches.h
 * @brief Header file for the switch handling module.
 *
 * This module provides functions to initialize and manage two hardware switches
 * using GPIO interrupts, debouncing, and long press detection.
 * A user can register a callback to receive debounced switch events.
 */

 #ifndef SWITCHES_H
 #define SWITCHES_H
 
 #include <stdint.h>
 #include <stdbool.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @enum switch_id_t
  * @brief Enumeration of switch identifiers.
  */
 typedef enum {
     SWITCH_WPS = 0,  /**< WPS switch identifier */
     SWITCH_TAG,      /**< Tag switch identifier */
     SWITCH_COUNT     /**< Total number of switches */
 } switch_id_t;
 
 /**
  * @brief Type definition for the switch event callback function.
  *
  * This callback is invoked after a switch event is debounced.
  *
  * @param id The identifier of the switch.
  * @param pressed The state of the switch: true if pressed, false if released.
  * @param long_press On release events, true if the switch was held long enough to be considered a long press.
  */
 typedef void (*switch_callback_t)(switch_id_t id, bool pressed, bool long_press);
 
 /**
  * @brief Initialize the switch module.
  *
  * Configures GPIOs, interrupts, and the task for processing switch events.
  */
 void switches_init(void);
 
 /**
  * @brief Register a callback function to receive debounced switch events.
  *
  * @param callback Function pointer to the callback to be invoked on a debounced switch event.
  */
 void switches_register_callback(switch_callback_t callback);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // SWITCHES_H
 