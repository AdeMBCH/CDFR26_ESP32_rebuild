/**
 * @file otos_node.h
 * @brief SparkFun OTOS sensor initialisation and micro-ROS publisher registration
 *
 * Publishes:
 *   /otos/odometry  — nav_msgs/Odometry at PUBLISH_FREQUENCY_HZ
 *   /otos/velocity  — geometry_msgs/Twist at PUBLISH_FREQUENCY_HZ
 *
 * Adds 1 handle to the executor (OTOS timer).
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
 * @brief Initialise the SparkFun OTOS sensor over I2C.
 * @return true on success, false if the sensor is not responding.
 */
bool otos_init(void);

/**
 * @brief Finalize OTOS micro-ROS entities (timer + publishers).
 *
 * Must be called before rcl_node_fini() on reconnection.
 * The reader task and queue are kept alive across reconnections.
 */
void otos_node_fini(rcl_node_t *node);

/**
 * @brief Register OTOS micro-ROS entities (publishers + timer + reader task).
 *
 * Must be called after the rcl_node, rclc_executor, and rclc_support have been
 * created.  The executor must have been initialised with at least 1 spare handle.
 *
 * @param node     Pointer to the already-created rcl_node_t.
 * @param executor Pointer to the executor.
 * @param support  Pointer to the rclc_support_t (needed to create the timer).
 * @return ESP_OK on success, ESP_FAIL if any call fails.
 */
esp_err_t otos_node_register(rcl_node_t *node, rclc_executor_t *executor,
                              rclc_support_t *support);

#ifdef __cplusplus
}
#endif
