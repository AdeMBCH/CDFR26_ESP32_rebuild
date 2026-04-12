/**
 * @file motor_node.h
 * @brief micro-ROS entities for MKS motor control
 *
 * Subscribes : /motor_commands  (std_msgs/Float64MultiArray) [id, rad_s, …]
 * Publishes  : /motor_states    (std_msgs/Float64MultiArray) [id, rad_s, …]
 *
 * Omni wheel CAN commands are sent at a fixed rate (MOTOR_OMNI_PERIOD_MS) by
 * an internal timer, independently of the /motor_commands publish rate.
 * Mechanism motor commands (IDs 5-8) are still sent immediately on receipt.
 *
 * This module registers exactly 2 handles into the executor:
 *   - /motor_commands subscription
 *   - omni periodic timer
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
 * @brief Finalize motor micro-ROS entities (timer, subscription, publisher).
 *
 * Must be called before rcl_node_fini() on reconnection to properly release
 * DDS resources. Does NOT stop the motors — call motor_node_emergency_stop() first.
 */
void motor_node_fini(rcl_node_t *node);

/**
 * @brief Send speed 0 to all omni wheels immediately and reset the watchdog.
 *
 * Safe to call from any context where CAN is available (e.g. on agent loss).
 */
void motor_node_emergency_stop(void);

/**
 * @brief Register motor micro-ROS entities (subscriber + publisher + omni timer).
 *
 * Must be called after the rcl_node, rclc_support, and rclc_executor have been
 * created, but before rclc_executor_spin_some().
 *
 * @param node     Pointer to the already-created rcl_node_t.
 * @param executor Pointer to the executor (must have been initialised with
 *                 enough handles to accommodate the 2 handles added here).
 * @param support  Pointer to the rclc_support_t (needed to create the timer).
 * @return ESP_OK on success, ESP_FAIL if any rcl call fails.
 */
esp_err_t motor_node_init(rcl_node_t *node, rclc_executor_t *executor,
                           rclc_support_t *support);

#ifdef __cplusplus
}
#endif
