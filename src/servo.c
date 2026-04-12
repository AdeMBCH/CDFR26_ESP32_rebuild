/**
 * @file servo.c
 * @brief PWM driver for the storage hatch servo
 *
 * PWM: LEDC_TIMER_0 / LEDC_CHANNEL_0 at SERVO_FREQ_HZ (50 Hz).
 * Pulse widths (SERVO_OPEN_US / SERVO_CLOSE_US) are defined in config.h.
 */

#include "servo.h"
#include "config.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "servo";

static void set_pulse_us(uint32_t pulse_us)
{
    uint32_t max_duty  = (1u << SERVO_TIMER_RESOLUTION);
    uint32_t period_us = 1000000u / SERVO_FREQ_HZ;
    uint32_t duty      = (pulse_us * max_duty) / period_us;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void servo_set(bool open)
{
    if (open) {
        set_pulse_us(SERVO_OPEN_US);
        ESP_LOGI(TAG, "open (%d us)", SERVO_OPEN_US);
    } else {
        set_pulse_us(SERVO_CLOSE_US);
        ESP_LOGI(TAG, "closed (%d us)", SERVO_CLOSE_US);
    }
}

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = (ledc_timer_bit_t)SERVO_TIMER_RESOLUTION,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed");
        return ESP_FAIL;
    }

    ledc_channel_config_t channel_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_GPIO_PIN,
        .duty           = 0,
        .hpoint         = 0,
    };
    if (ledc_channel_config(&channel_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed");
        return ESP_FAIL;
    }

    set_pulse_us(SERVO_CLOSE_US);
    ESP_LOGI(TAG, "GPIO%d at %d Hz — init closed (%d us)",
             SERVO_GPIO_PIN, SERVO_FREQ_HZ, SERVO_CLOSE_US);
    return ESP_OK;
}
