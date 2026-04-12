/**
 * @file led_manager.h
 * @brief WS2812 RGB LED state machine for connection status indication
 */
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Connection states reflected by the LED colour
// ---------------------------------------------------------------------------
typedef enum {
    LED_STATE_NO_UART,    // Red blinking  — UART transport not ready
    LED_STATE_NO_AGENT,   // Blue blinking — UART ok, microROS agent not found
    LED_STATE_CONNECTED,  // Green solid   — publishing data
} led_state_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the RMT channel and WS2812 encoder.
 *        Must be called once before led_task() is started.
 */
void led_manager_init(void);

/**
 * @brief Thread-safe state update.
 *        Can be called from any task.
 */
void led_manager_set_state(led_state_t state);

/**
 * @brief FreeRTOS task that drives the LED blink pattern.
 *        Create with xTaskCreate(led_task, ...).
 */
void led_task(void *arg);

#ifdef __cplusplus
}
#endif
