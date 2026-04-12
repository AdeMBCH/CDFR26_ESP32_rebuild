/**
 * @file led_strip_encoder.h
 * @brief RMT encoder for WS2812 LED strip
 */

#pragma once

#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create RMT encoder for encoding WS2812 strip
 *
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_OK: Create RMT encoder successfully
 *      - ESP_ERR_INVALID_ARG: Create RMT encoder failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Create RMT encoder failed because out of memory
 *      - ESP_FAIL: Create RMT encoder failed because of other error
 */
esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
