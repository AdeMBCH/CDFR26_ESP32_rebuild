/**
 * @file led_manager.cpp
 * @brief WS2812 RGB LED state machine implementation
 */

#include "led_manager.h"
#include "config.h"
#include "led_strip_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

static const char *TAG = "led_manager";

// ---------------------------------------------------------------------------
// Module-private state
// ---------------------------------------------------------------------------
static rmt_channel_handle_t s_led_channel = NULL;
static rmt_encoder_handle_t s_led_encoder = NULL;

// Shared between led_task (reader) and micro_ros_task (writer).
// Protected by a FreeRTOS spinlock so both tasks see a consistent value
// without the overhead of a full mutex.
static portMUX_TYPE       s_state_mux    = portMUX_INITIALIZER_UNLOCKED;
static volatile led_state_t s_led_state  = LED_STATE_NO_UART;

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/** Set the WS2812 to the given RGB colour (0–255 per channel). */
static void set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    // WS2812 uses GRB byte order
    uint8_t led_data[3] = { green, red, blue };

    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    esp_err_t err = rmt_transmit(s_led_channel, s_led_encoder,
                                 led_data, sizeof(led_data), &tx_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_transmit failed: %d", err);
        return;
    }
    rmt_tx_wait_all_done(s_led_channel, 100);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void led_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing WS2812 LED on GPIO%d", LED_RGB_PIN);

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num         = LED_RGB_PIN,
        .clk_src          = RMT_CLK_SRC_DEFAULT,
        .resolution_hz    = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma   = false,
        },
    };

    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %d", err);
        return;
    }

    err = rmt_new_led_strip_encoder(&s_led_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip encoder: %d", err);
        return;
    }

    err = rmt_enable(s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %d", err);
        return;
    }

    ESP_LOGI(TAG, "WS2812 LED ready");
}

void led_manager_set_state(led_state_t state)
{
    portENTER_CRITICAL(&s_state_mux);
    s_led_state = state;
    portEXIT_CRITICAL(&s_state_mux);
}

void led_task(void *arg)
{
    (void)arg;
    bool led_on = false;

    while (1) {
        portENTER_CRITICAL(&s_state_mux);
        led_state_t state = s_led_state;
        portEXIT_CRITICAL(&s_state_mux);

        switch (state) {
            case LED_STATE_NO_UART:
                // Red blinking (500 ms on/off)
                set_led_color(led_on ? 255 : 0, 0, 0);
                led_on = !led_on;
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_STATE_NO_AGENT:
                // Blue blinking (500 ms on/off)
                set_led_color(0, 0, led_on ? 255 : 0);
                led_on = !led_on;
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_STATE_CONNECTED:
                // Green solid
                set_led_color(0, 255, 0);
                led_on = true;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }
}
