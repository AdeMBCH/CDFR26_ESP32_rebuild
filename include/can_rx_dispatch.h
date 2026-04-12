/**
 * @file can_rx_dispatch.h
 * @brief Centralised CAN RX dispatcher for MKS motor response frames.
 *
 * Problem solved:
 *   twai_receive() is a consuming read — whichever task calls it first gets
 *   the frame. With multiple background tasks (homing, phase moves) all
 *   expecting CAN responses, direct twai_receive() calls would race and steal
 *   each other's frames.
 *
 * Solution:
 *   A single dedicated task (can_rx_task, priority 6) is the only caller of
 *   twai_receive(). It inspects each frame's (motor_id, cmd_byte) and routes
 *   it into the matching FreeRTOS queue. Consumers call can_rx_wait() which
 *   blocks on their specific queue — no contention.
 *
 * Registered (motor_id, cmd_byte) pairs:
 *   Mechanism motors 5-8 × {0x91 GoHome, 0xF5 AbsMove, 0x3E Stall, 0x3B HomeStatus}
 *   All other frames are silently discarded.
 *
 * Thread safety:
 *   can_rx_wait() may be called from any task concurrently, provided at most
 *   one consumer waits on each (motor_id, cmd_byte) pair at a time.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A received CAN frame as seen by consumers. */
typedef struct {
    uint8_t motor_id;
    uint8_t dlc;
    uint8_t data[8];
} can_rx_frame_t;

/**
 * @brief Initialise the dispatcher: create queues and start can_rx_task.
 *        Must be called once after can_driver_init(), before any can_rx_wait().
 */
esp_err_t can_rx_dispatch_init(void);

/**
 * @brief Wait for a CAN response frame matching (motor_id, cmd_byte).
 *
 * Blocks until a matching frame is available or the timeout expires.
 * Thread-safe — may be called from any task.
 *
 * @param motor_id    CAN ID of the expected sender (5-8 for mechanism motors).
 * @param cmd_byte    First data byte of the expected response (e.g. 0x91, 0xF5).
 * @param out         Filled with the received frame on success. Must not be NULL.
 * @param timeout_ms  Maximum wait in milliseconds. 0 = non-blocking poll.
 * @return ESP_OK            – frame received and stored in *out.
 *         ESP_ERR_TIMEOUT   – no frame arrived within timeout_ms.
 *         ESP_ERR_NOT_FOUND – (motor_id, cmd_byte) not in the routing table.
 */
esp_err_t can_rx_wait(uint8_t motor_id, uint8_t cmd_byte,
                      can_rx_frame_t *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
