/**
 * @file servo.h
 * @brief PWM driver for the storage hatch servo
 *
 * Uses the ESP32 LEDC peripheral (LEDC_TIMER_0 / LEDC_CHANNEL_0) to generate
 * a standard 50 Hz PWM signal.  Pulse widths are defined in config.h:
 *   SERVO_OPEN_US  — open  position
 *   SERVO_CLOSE_US — closed position
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the LEDC PWM output. Servo starts in the closed position.
 *
 * @return ESP_OK on success, ESP_FAIL if LEDC config fails.
 */
esp_err_t servo_init(void);

/**
 * @brief Command the storage servo.
 *
 * @param open  true = open (SERVO_OPEN_US), false = closed (SERVO_CLOSE_US).
 */
void servo_set(bool open);

#ifdef __cplusplus
}
#endif
