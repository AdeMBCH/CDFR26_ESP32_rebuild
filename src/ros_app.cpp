/**
 * @file ros_app.cpp
 * @brief micro-ROS application task — agent discovery, node/executor creation,
 *        and delegation to each functional node module.
 *
 * Handle budget (executor):
 *   1  OTOS timer               (otos_node)
 *   2  motor subscriber         (motor_node)
 *   3  omni periodic timer      (motor_node)
 *   4  home_gripper             (homing_node)
 *   5  home_storage             (homing_node)
 *   6  calibrate_encoder        (homing_node)
 *   7  calibrate_wheel_encoders (homing_node)
 *   8  homing status timer      (homing_node)
 *   9  strategy/atomic_phase    (phase_node)
 *  10  phase status timer       (phase_node)
 *  11  (spare)
 *  12  (spare)
 *
 * Reconnection: the task loops forever.  On agent loss it tears down all
 * micro-ROS entities, sends an emergency stop, and re-pings until the
 * agent is back before rebuilding everything.
 */

#include "ros_app.h"
#include "config.h"
#include "led_manager.h"
#include "otos_node.h"
#include "motor_node.h"
#include "homing_node.h"
#include "servo.h"
#include "phase_node.h"

#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

static const char *TAG = "ros_app";

// ---------------------------------------------------------------------------

void micro_ros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();

    while (1) {
        /* ── 1. Wait for agent ─────────────────────────────────────────────── */
        led_manager_set_state(LED_STATE_NO_AGENT);
        ESP_LOGI(TAG, "Pinging micro-ROS agent …");
        for (int attempt = 1; ; ++attempt) {
            if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
                ESP_LOGI(TAG, "Agent found (attempt %d)", attempt);
                break;
            }
            ESP_LOGW(TAG, "Ping attempt %d failed", attempt);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* ── 2. Init micro-ROS entities ────────────────────────────────────── */
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "rcl_init_options_init failed — retrying");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
        ESP_LOGI(TAG, "ROS Domain ID: %d", ROS_DOMAIN_ID);

        rclc_support_t support;
        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "rclc_support_init failed — retrying");
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        rcl_node_t node;
        if (rclc_node_init_default(&node, "esp_node", "", &support) != RCL_RET_OK) {
            ESP_LOGE(TAG, "rclc_node_init_default failed — retrying");
            rclc_support_fini(&support);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "Node 'esp_node' created");

        rclc_executor_t executor;
        if (rclc_executor_init(&executor, &support.context, 12, &allocator) != RCL_RET_OK) {
            ESP_LOGE(TAG, "rclc_executor_init failed — retrying");
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (otos_node_register(&node, &executor, &support) != ESP_OK)
            ESP_LOGW(TAG, "otos_node_register failed — odometry will not be published");
        if (motor_node_init(&node, &executor, &support) != ESP_OK)
            ESP_LOGW(TAG, "motor_node_init failed — motor commands will not work");
        if (homing_node_init(&node, &executor, &support) != ESP_OK)
            ESP_LOGW(TAG, "homing_node_init failed — homing services will not work");
        if (servo_init() != ESP_OK)
            ESP_LOGW(TAG, "servo_init failed — storage servo will not work");
        if (phase_node_init(&node, &executor, &support) != ESP_OK)
            ESP_LOGW(TAG, "phase_node_init failed — strategy/atomic_phase will not work");

        /* ── 3. Spin until agent disconnects ───────────────────────────────── */
        led_manager_set_state(LED_STATE_CONNECTED);
        ESP_LOGI(TAG, "Spinning …");

        /** Ping the agent every N ms to detect disconnection. */
        static const uint32_t AGENT_PING_INTERVAL_MS = 5000;
        static const int MAX_MISSED_PINGS = 3;

        TickType_t last_ping_tick = xTaskGetTickCount();
        int missed_pings = 0;

        bool agent_ok = true;
        while (agent_ok) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

            TickType_t now = xTaskGetTickCount();
            if ((now - last_ping_tick) * portTICK_PERIOD_MS >= AGENT_PING_INTERVAL_MS) {
                if (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
                    missed_pings++;
                    ESP_LOGW(TAG, "Agent ping failed (%d/%d)", missed_pings, MAX_MISSED_PINGS);
                    if (missed_pings >= MAX_MISSED_PINGS) {
                        ESP_LOGW(TAG, "Agent considered lost after repeated ping failures");
                        agent_ok = false;
                    }
                } else {
                    missed_pings = 0;
                }
                last_ping_tick = now;
            }

            taskYIELD();
        }

        /* ── 4. Agent lost — stop motors and tear down ─────────────────────── */
        motor_node_emergency_stop();

        /* Explicitly fini RCL entities before tearing down the node */
        otos_node_fini(&node);
        motor_node_fini(&node);

        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        rcl_init_options_fini(&init_options);

        ESP_LOGI(TAG, "Entities torn down — reconnecting …");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
