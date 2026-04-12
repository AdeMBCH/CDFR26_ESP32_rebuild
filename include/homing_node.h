/**
 * @file homing_node.h
 * @brief micro-ROS homing and calibration services for MKS mechanism motors
 *
 * Services (std_srvs/srv/Trigger) — all respond immediately:
 *   /home_gripper             — Home gripper (MOTOR_GRIPPER_MASTER & SLAVE)
 *                               Result published on /homing_status when done.
 *   /home_storage             — Home storage (MOTOR_STORAGE_MASTER & SLAVE)
 *                               Result published on /homing_status when done.
 *   /calibrate_encoder        — Fire-and-forget encoder calibration (mechanism motors)
 *   /calibrate_wheel_encoders — Fire-and-forget encoder calibration (wheel motors)
 *
 * Publisher (std_msgs/String):
 *   /homing_status — "Gripper: OK/FAIL:…" or "Storage: OK/FAIL:…"
 *                    Published only for homing ops, not for calibration.
 *
 * Adds 5 handles to the executor:
 *   4 services + 1 status timer
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
 * @brief Register homing/calibration micro-ROS entities into an existing node and executor.
 *
 * The executor must have been initialised with at least 5 spare handles
 * (4 services + 1 timer).
 *
 * @param node     Pointer to the already-created rcl_node_t.
 * @param executor Pointer to the executor.
 * @param support  Pointer to the rclc_support_t (needed for timer init).
 * @return ESP_OK on success, ESP_FAIL if any rcl call fails.
 */
esp_err_t homing_node_init(rcl_node_t *node, rclc_executor_t *executor,
                            rclc_support_t *support);

#ifdef __cplusplus
}
#endif
