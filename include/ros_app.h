/**
 * @file ros_app.h
 * @brief micro-ROS application task — orchestrates all ROS nodes on the ESP32.
 *
 * Owns: agent discovery, rcl_node, rclc_executor, and delegates node
 * registration to otos_node, motor_node, homing_node, servo, phase_node.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS task that manages the micro-ROS connection and spins the
 *        executor.  Create with xTaskCreate(micro_ros_task, ...).
 */
void micro_ros_task(void *arg);

#ifdef __cplusplus
}
#endif
