/**
 * @file motor_node.c
 * @brief micro-ROS subscriber/publisher for MKS motor control
 *
 * Subscribes : /motor_commands  (std_msgs/Float64MultiArray)
 *   Format: [id0, vel0_rad_s, id1, vel1_rad_s, …]
 *
 * Publishes  : /motor_states    (std_msgs/Float64MultiArray)
 *   Format: [id0, vel0_rad_s, id1, vel1_rad_s, …]  — echo of sent commands
 *
 * Motor behaviour:
 *   - Omni wheels (IDs 1-4)    : targets stored in s_omni_omega[], sent
 *                                 periodically at MOTOR_OMNI_PERIOD_MS by the
 *                                 omni_timer_callback — independent of the
 *                                 ROS2 publish rate.
 *   - Mechanisms (IDs 5-8)     : command sent immediately on receipt, with
 *                                 acc = MKS_ACC_MECHANISM (gentle ramp).
 *   - Symmetric pairs (MOTOR_AUTO_MIRROR = 1):
 *       master 5 → slave 6 with inverted velocity
 *       master 7 → slave 8 with inverted velocity
 *
 * Endstop note: endstops are wired directly to the MKS motor controllers,
 * which stop autonomously.  Homing is performed by the ROS2 controller
 * (send slow velocity, wait for the motor to stall against the endstop).
 */

#include "motor_node.h"
#include "can_driver.h"
#include "config.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>

/** Stop omni wheels if no /motor_commands received within this duration (ms). */
#define MOTOR_WATCHDOG_MS  300

static const char *TAG = "motor_node";

/* ── micro-ROS entities ───────────────────────────────────────────────────── */

static rcl_subscription_t               s_cmd_sub;
static std_msgs__msg__Float64MultiArray s_cmd_msg;
static double                           s_cmd_buf[32]; /* 16 motors × [id, vel] */

static rcl_publisher_t                  s_states_pub;
static std_msgs__msg__Float64MultiArray s_states_msg;
static double                           s_states_buf[32];

/* ── Omni wheel periodic send ─────────────────────────────────────────────── */

/** Ordered list of the 4 omni wheel CAN IDs.
 *  Index matches s_omni_omega[]: 0=FL, 1=FR, 2=RL, 3=RR. */
static const uint8_t OMNI_IDS[4] = {
    MOTOR_OMNI_FL, MOTOR_OMNI_FR,
    MOTOR_OMNI_RL, MOTOR_OMNI_RR,
};

/** Last-received velocity targets for the 4 omni wheels (indexed as OMNI_IDS).
 *  Written by motor_cmd_callback, read by omni_timer_callback.
 *  Both run in the executor context — no lock needed. */
static double s_omni_omega[4] = {0.0, 0.0, 0.0, 0.0};

/** Timestamp (esp_timer_get_time, µs) of the last received /motor_commands.
 *  Used by omni_timer_callback to zero targets on watchdog timeout. */
static int64_t s_last_cmd_us = 0;

static rcl_timer_t s_omni_timer;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/** Return the index of id in OMNI_IDS, or -1 if it is not an omni motor.
 *  Works regardless of the order or values of the MOTOR_OMNI_* constants. */
static int omni_idx(uint8_t id)
{
    for (int i = 0; i < 4; i++) {
        if (OMNI_IDS[i] == id) return i;
    }
    return -1;
}

/** Return the MKS acceleration value appropriate for the given motor ID. */
static inline uint8_t motor_acc(uint8_t id)
{
    return (omni_idx(id) >= 0) ? MKS_ACC_OMNI : MKS_ACC_MECHANISM;
}

/* ── Omni timer callback — fires at MOTOR_OMNI_PERIOD_MS ─────────────────── */

static void omni_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    /* Watchdog: if no command received recently, zero the targets and stop. */
    int64_t now_us   = esp_timer_get_time();
    int64_t elapsed  = (now_us - s_last_cmd_us) / 1000; /* ms */
    if (s_last_cmd_us != 0 && elapsed > MOTOR_WATCHDOG_MS) {
        bool any_nonzero = false;
        for (int i = 0; i < 4; i++) {
            if (s_omni_omega[i] != 0.0) { any_nonzero = true; break; }
        }
        if (any_nonzero) {
            ESP_LOGW(TAG, "Watchdog: no cmd for %" PRId64 " ms — stopping omni wheels", elapsed);
            for (int i = 0; i < 4; i++) s_omni_omega[i] = 0.0;
        }
    }

    const uint8_t ids[4] = {
        MOTOR_OMNI_FL, MOTOR_OMNI_FR,
        MOTOR_OMNI_RL, MOTOR_OMNI_RR,
    };
    for (int i = 0; i < 4; i++) {
        esp_err_t err = mks_send_speed_rads(ids[i], s_omni_omega[i], MKS_ACC_OMNI);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "CAN keepalive failed for omni motor %u omega=%.3f: %s",
                    (unsigned)ids[i], s_omni_omega[i], esp_err_to_name(err));
        }
    }
}

/* ── Subscriber callback ─────────────────────────────────────────────────── */

static void motor_cmd_callback(const void *msg_in)
{
    const std_msgs__msg__Float64MultiArray *msg =
        (const std_msgs__msg__Float64MultiArray *)msg_in;

    size_t n = msg->data.size;
    if (n < 2 || (n & 1u) != 0) {
        ESP_LOGW(TAG, "/motor_commands: unexpected payload size %zu (must be even)", n);
        return;
    }
    s_last_cmd_us = esp_timer_get_time();

    size_t states_idx = 0;

    for (size_t i = 0; i + 1 < n; i += 2) {
        uint8_t id    = (uint8_t)msg->data.data[i];
        double  omega = msg->data.data[i + 1];

        /* The ROS2 controller sends 0-indexed wheel positions [0=FL,1=FR,2=RL,3=RR].
         * Translate to actual CAN IDs using the OMNI_IDS table.
         * Mechanism motors are sent as absolute CAN IDs (>= 5) and pass through. */
        if (id < 4) {
            id = OMNI_IDS[id];
        }

        uint8_t acc   = motor_acc(id);

        int idx = omni_idx(id);
        if (idx >= 0) {
            /* Omni wheels: send immediately for low latency, and update the
             * target so the keep-alive timer can re-send it periodically. */
            esp_err_t err = mks_send_speed_rads(id, omega, MKS_ACC_OMNI);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "CAN send failed for omni motor %u omega=%.3f: %s",
                        (unsigned)id, omega, esp_err_to_name(err));
            }
            s_omni_omega[idx] = omega;
        } else {
            /* Mechanism motors: send immediately */
            esp_err_t err = mks_send_speed_rads(id, omega, acc);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "CAN send failed for motor %u omega=%.3f: %s",
                        (unsigned)id, omega, esp_err_to_name(err));
            }

#if MOTOR_AUTO_MIRROR
            /* Symmetric pairs: slave receives the inverted velocity */
            uint8_t slave_id = 0;
            if      (id == MOTOR_GRIPPER_MASTER) slave_id = MOTOR_GRIPPER_SLAVE;
            else if (id == MOTOR_STORAGE_MASTER) slave_id = MOTOR_STORAGE_SLAVE;

            if (slave_id != 0) {
                esp_err_t err_slave = mks_send_speed_rads(slave_id, -omega, acc);
                if (err_slave != ESP_OK) {
                    ESP_LOGE(TAG, "CAN send failed for slave motor %u omega=%.3f: %s",
                            (unsigned)slave_id, -omega, esp_err_to_name(err_slave));
                }
                if (states_idx + 1 < 32) {
                    s_states_buf[states_idx++] = (double)slave_id;
                    s_states_buf[states_idx++] = -omega;
                }
            }
#endif
        }

        if (states_idx + 1 < 32) {
            s_states_buf[states_idx++] = (double)id;
            s_states_buf[states_idx++] = omega;
        }
    }

    s_states_msg.data.size = states_idx;

    rcl_ret_t rc = rcl_publish(&s_states_pub, &s_states_msg, NULL);
    if (rc != RCL_RET_OK) {
        ESP_LOGW(TAG, "motor_states publish failed: %d", (int)rc);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void motor_node_fini(rcl_node_t *node)
{
    rcl_timer_fini(&s_omni_timer);
    rcl_subscription_fini(&s_cmd_sub, node);
    rcl_publisher_fini(&s_states_pub, node);
}

void motor_node_emergency_stop(void)
{
    for (int i = 0; i < 4; i++) s_omni_omega[i] = 0.0;
    s_last_cmd_us = 0; /* re-arm watchdog for next session */
    for (int i = 0; i < 4; i++) {
        mks_send_speed_rads(OMNI_IDS[i], 0.0, MKS_ACC_OMNI);
    }
    ESP_LOGI(TAG, "Emergency stop: omni wheels zeroed");
}

esp_err_t motor_node_init(rcl_node_t *node, rclc_executor_t *executor,
                           rclc_support_t *support)
{
    rcl_ret_t rc;

    /* Publisher: /motor_states */
    rc = rclc_publisher_init_default(
        &s_states_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "motor_states");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create motor_states publisher: %d", (int)rc);
        return ESP_FAIL;
    }

    s_states_msg.data.data     = s_states_buf;
    s_states_msg.data.size     = 0;
    s_states_msg.data.capacity = 32;

    /* Subscriber: /motor_commands */
    rc = rclc_subscription_init_default(
        &s_cmd_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "motor_commands");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create motor_commands subscriber: %d", (int)rc);
        return ESP_FAIL;
    }

    s_cmd_msg.data.data     = s_cmd_buf;
    s_cmd_msg.data.size     = 0;
    s_cmd_msg.data.capacity = 32;

    rc = rclc_executor_add_subscription(
        executor, &s_cmd_sub, &s_cmd_msg,
        motor_cmd_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add subscription to executor: %d", (int)rc);
        return ESP_FAIL;
    }

    /* Omni periodic timer */
    rc = rclc_timer_init_default(&s_omni_timer, support,
                                 RCL_MS_TO_NS(MOTOR_OMNI_PERIOD_MS),
                                 omni_timer_callback);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create omni timer: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_executor_add_timer(executor, &s_omni_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add omni timer to executor: %d", (int)rc);
        return ESP_FAIL;
    }
    /* après reboot/reset dans tous les cas MKS s'active dès le départ */
    const uint8_t ids[4] = {
        MOTOR_OMNI_FL, MOTOR_OMNI_FR,
        MOTOR_OMNI_RL, MOTOR_OMNI_RR,
    };
    for (int i = 0; i < 4; i++) {
        esp_err_t err = mks_set_enable(ids[i], true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable omni motor %u: %s",
                     (unsigned)ids[i], esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Enabled omni motor %u", (unsigned)ids[i]);
        }
    }
    
    ESP_LOGI(TAG, "motor_node ready — sub: /motor_commands  pub: /motor_states"
                  "  omni timer: %d ms (%d Hz)",
             MOTOR_OMNI_PERIOD_MS, 1000 / MOTOR_OMNI_PERIOD_MS);
    return ESP_OK;
}
