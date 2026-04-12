/**
 * @file phase_node.h
 * @brief micro-ROS subscriber for strategy/atomic_phase + async completion feedback
 *
 * Subscriber:
 *   strategy/atomic_phase — receives phase names, dispatches physical actions
 *                           in a background task (executor never blocked)
 *
 * Publisher:
 *   /phase_status (std_msgs/String) — result published when the action completes:
 *     "PHASE_NAME: OK"
 *     "PHASE_NAME: FAIL:TIMEOUT"
 *     "PHASE_NAME: FAIL:MOTOR_ERR"
 *
 * Handled phases:
 *   STORAGE_UP    — move storage up   (MOTOR_STORAGE_MASTER / SLAVE, waits F5H completion)
 *   STORAGE_DOWN  — move storage down (MOTOR_STORAGE_MASTER / SLAVE, waits F5H completion)
 *   STORAGE_OPEN  — open  storage hatch (servo, immediate OK)
 *   STORAGE_CLOSE — close storage hatch (servo, immediate OK)
 *   GRIPPER_UP    — move gripper up   (MOTOR_GRIPPER_MASTER / SLAVE, waits F5H completion)
 *   GRIPPER_DOWN  — move gripper down (MOTOR_GRIPPER_MASTER / SLAVE, waits F5H completion)
 *
 * All other phase strings are silently ignored.
 *
 * Adds 2 handles to the executor (1 subscriber + 1 status timer).
 */
#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the phase subscriber, background task, and status publisher.
 *
 * Must be called after servo_init() and can_rx_dispatch_init().
 * The executor must have at least 2 spare handles.
 *
 * @param node     Pointer to the already-created rcl_node_t.
 * @param executor Pointer to the executor.
 * @param support  Pointer to rclc_support_t (needed to create the status timer).
 * @return ESP_OK on success, ESP_FAIL if any rcl/FreeRTOS call fails.
 */
esp_err_t phase_node_init(rcl_node_t *node, rclc_executor_t *executor,
                           rclc_support_t *support);

#ifdef __cplusplus
}
#endif
