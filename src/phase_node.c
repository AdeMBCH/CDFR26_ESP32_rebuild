/**
 * @file phase_node.c
 * @brief micro-ROS subscriber for strategy/atomic_phase + async completion feedback
 *
 * See phase_node.h for the public interface and phase list.
 *
 * Architecture (identical pattern to homing_node):
 *   Subscriber callback  → enqueue phase_request_t (depth=1, "busy" if full)
 *   phase_bg_task        → execute the action, wait for CAN completion via dispatcher
 *   status_timer_cb      → poll s_result.pending every 200 ms, publish to /phase_status
 *
 * Motor moves (F5H):
 *   Waits for spontaneous [0xF5, 0x02/0x03, CRC] from both master and slave.
 *   status=0x02 → position reached
 *   status=0x03 → stopped at limit switch (also treated as success)
 *   status=0x00 → motor error
 *   Timeout (PHASE_TIMEOUT_MS) → emergency stop + FAIL:TIMEOUT
 *
 * Servo actions: immediate completion, no CAN feedback needed.
 */

#include "phase_node.h"
#include "can_rx_dispatch.h"
#include "can_driver.h"
#include "servo.h"
#include "config.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "phase_node";

/* ── Phase operation types ───────────────────────────────────────────────── */

typedef enum {
    PHASE_STORAGE_UP = 0,
    PHASE_STORAGE_DOWN,
    PHASE_STORAGE_OPEN,
    PHASE_STORAGE_CLOSE,
    PHASE_GRIPPER_UP,
    PHASE_GRIPPER_DOWN,
} phase_op_t;

typedef struct {
    phase_op_t op;
} phase_request_t;

/* ── Shared result ────────────────────────────────────────────────────────
 * Same single-producer/single-consumer atomic handoff as homing_node.
 * Written exclusively by phase_bg_task, read by status_timer_cb.
 * ─────────────────────────────────────────────────────────────────────────*/

typedef struct {
    char          message[128];
    volatile bool pending;
} phase_result_t;

static QueueHandle_t  s_queue;
static phase_result_t s_result;
static bool           s_bg_started = false; /**< queue+task created only once */

/* ── micro-ROS entities ──────────────────────────────────────────────────── */

static rcl_subscription_t    s_phase_sub;
static std_msgs__msg__String s_phase_msg;
static char                  s_phase_buf[64];

static rcl_publisher_t       s_status_pub;
static std_msgs__msg__String s_status_msg;
static char                  s_status_buf[128];
static rcl_timer_t           s_status_timer;

/* ── Motor move helper ───────────────────────────────────────────────────── */

typedef enum {
    MOVE_OK = 0,
    MOVE_MOTOR_ERR,
    MOVE_TIMEOUT,
} move_err_t;

/**
 * Wait for both motors to confirm F5H position reached via CAN RX dispatcher.
 * Polls each motor's queue with a 20 ms window, checking a global deadline.
 * On timeout: sends emergency stop to motors that have not yet completed.
 */
static move_err_t wait_move_done(uint8_t master_id, uint8_t slave_id)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(PHASE_TIMEOUT_MS);
    bool master_done = false, slave_done = false;
    bool master_ok   = false, slave_ok   = false;

    while (!master_done || !slave_done) {
        if ((int32_t)(deadline - xTaskGetTickCount()) <= 0) {
            ESP_LOGE(TAG, "Move timeout — stopping motors %u & %u",
                     (unsigned)master_id, (unsigned)slave_id);
            if (!master_done) mks_emergency_stop(master_id);
            if (!slave_done)  mks_emergency_stop(slave_id);
            return MOVE_TIMEOUT;
        }

        can_rx_frame_t f;
        if (!master_done && can_rx_wait(master_id, 0xF5u, &f, 20) == ESP_OK) {
            /* 0x02 = position reached, 0x03 = stopped at limit (both = success) */
            if      (f.data[1] == 0x02u || f.data[1] == 0x03u) { master_done = true; master_ok = true; }
            else if (f.data[1] == 0x00u)                        { master_done = true; }
            /* 0x01 = starting ACK — keep waiting */
        }
        if (!slave_done && can_rx_wait(slave_id, 0xF5u, &f, 20) == ESP_OK) {
            if      (f.data[1] == 0x02u || f.data[1] == 0x03u) { slave_done = true; slave_ok = true; }
            else if (f.data[1] == 0x00u)                        { slave_done = true; }
        }
    }

    return (master_ok && slave_ok) ? MOVE_OK : MOVE_MOTOR_ERR;
}

static const char *move_err_str(move_err_t r)
{
    if (r == MOVE_OK)        return "OK";
    if (r == MOVE_TIMEOUT)   return "FAIL:TIMEOUT";
    return "FAIL:MOTOR_ERR";
}

/* ── Background task ─────────────────────────────────────────────────────── */

static void phase_bg_task(void *arg)
{
    (void)arg;
    phase_request_t req;

    for (;;) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) continue;

        bool ok = false;

        switch (req.op) {

            case PHASE_STORAGE_UP: {
                mks_move_abs(MOTOR_STORAGE_MASTER,  STORAGE_POS_UP,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                mks_move_abs(MOTOR_STORAGE_SLAVE,  -STORAGE_POS_UP,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                move_err_t r = wait_move_done(MOTOR_STORAGE_MASTER, MOTOR_STORAGE_SLAVE);
                snprintf(s_result.message, sizeof(s_result.message),
                         "STORAGE_UP: %s", move_err_str(r));
                ok = (r == MOVE_OK);
                break;
            }

            case PHASE_STORAGE_DOWN: {
                mks_move_abs(MOTOR_STORAGE_MASTER,  STORAGE_POS_DOWN,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                mks_move_abs(MOTOR_STORAGE_SLAVE,  -STORAGE_POS_DOWN,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                move_err_t r = wait_move_done(MOTOR_STORAGE_MASTER, MOTOR_STORAGE_SLAVE);
                snprintf(s_result.message, sizeof(s_result.message),
                         "STORAGE_DOWN: %s", move_err_str(r));
                ok = (r == MOVE_OK);
                break;
            }

            case PHASE_STORAGE_OPEN:
                servo_set(true);
                snprintf(s_result.message, sizeof(s_result.message), "STORAGE_OPEN: OK");
                ok = true;
                break;

            case PHASE_STORAGE_CLOSE:
                servo_set(false);
                snprintf(s_result.message, sizeof(s_result.message), "STORAGE_CLOSE: OK");
                ok = true;
                break;

            case PHASE_GRIPPER_UP: {
                mks_move_abs(MOTOR_GRIPPER_MASTER,  GRIPPER_POS_UP,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                mks_move_abs(MOTOR_GRIPPER_SLAVE,  -GRIPPER_POS_UP,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                move_err_t r = wait_move_done(MOTOR_GRIPPER_MASTER, MOTOR_GRIPPER_SLAVE);
                snprintf(s_result.message, sizeof(s_result.message),
                         "GRIPPER_UP: %s", move_err_str(r));
                ok = (r == MOVE_OK);
                break;
            }

            case PHASE_GRIPPER_DOWN: {
                mks_move_abs(MOTOR_GRIPPER_MASTER,  GRIPPER_POS_DOWN,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                mks_move_abs(MOTOR_GRIPPER_SLAVE,  -GRIPPER_POS_DOWN,
                             MKS_POSITION_SPEED_RPM, MKS_ACC_MECHANISM);
                move_err_t r = wait_move_done(MOTOR_GRIPPER_MASTER, MOTOR_GRIPPER_SLAVE);
                snprintf(s_result.message, sizeof(s_result.message),
                         "GRIPPER_DOWN: %s", move_err_str(r));
                ok = (r == MOVE_OK);
                break;
            }
        }

        s_result.pending = true;
        ESP_LOGI(TAG, "phase_bg: %s", s_result.message);
    }
}

/* ── Status timer callback (executor context — safe to publish) ───────────── */

static void status_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL)     return;
    if (!s_result.pending) return;

    s_result.pending = false;

    strncpy(s_status_buf, s_result.message, sizeof(s_status_buf) - 1);
    s_status_buf[sizeof(s_status_buf) - 1] = '\0';

    s_status_msg.data.data     = s_status_buf;
    s_status_msg.data.size     = strlen(s_status_buf);
    s_status_msg.data.capacity = sizeof(s_status_buf);

    rcl_publish(&s_status_pub, &s_status_msg, NULL);
    ESP_LOGI(TAG, "/phase_status → \"%s\"", s_status_buf);
}

/* ── Subscriber callback ─────────────────────────────────────────────────── */

static void phase_callback(const void *msg_in)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
    const char *phase = msg->data.data;

    phase_op_t op;
    bool known = true;

    if      (strcmp(phase, "STORAGE_UP")    == 0) { op = PHASE_STORAGE_UP;    }
    else if (strcmp(phase, "STORAGE_DOWN")  == 0) { op = PHASE_STORAGE_DOWN;  }
    else if (strcmp(phase, "STORAGE_OPEN")  == 0) { op = PHASE_STORAGE_OPEN;  }
    else if (strcmp(phase, "STORAGE_CLOSE") == 0) { op = PHASE_STORAGE_CLOSE; }
    else if (strcmp(phase, "GRIPPER_UP")    == 0) { op = PHASE_GRIPPER_UP;    }
    else if (strcmp(phase, "GRIPPER_DOWN")  == 0) { op = PHASE_GRIPPER_DOWN;  }
    else                                          { known = false;             }

    if (!known) return;

    phase_request_t req = { .op = op };
    if (xQueueSend(s_queue, &req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "phase busy — \"%s\" dropped", phase);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t phase_node_init(rcl_node_t *node, rclc_executor_t *executor,
                           rclc_support_t *support)
{
    if (!s_bg_started) {
        s_queue = xQueueCreate(1, sizeof(phase_request_t));
        if (!s_queue) {
            ESP_LOGE(TAG, "Failed to create queue");
            return ESP_FAIL;
        }
        if (xTaskCreate(phase_bg_task, "phase_bg",
                        4096, NULL, 3, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create background task");
            return ESP_FAIL;
        }
        s_bg_started = true;
    } else {
        /* Flush stale requests from the previous session */
        xQueueReset(s_queue);
    }

    rcl_ret_t rc;

    rc = rclc_subscription_init_default(
        &s_phase_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "strategy/atomic_phase");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create subscriber: %d", (int)rc);
        return ESP_FAIL;
    }

    s_phase_msg.data.data     = s_phase_buf;
    s_phase_msg.data.size     = 0;
    s_phase_msg.data.capacity = sizeof(s_phase_buf);

    rc = rclc_executor_add_subscription(executor, &s_phase_sub, &s_phase_msg,
                                        phase_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add subscriber to executor: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_publisher_init_default(
        &s_status_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "phase_status");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create /phase_status publisher: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_timer_init_default(&s_status_timer, support,
                                 RCL_MS_TO_NS(200), status_timer_cb);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create status timer: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_executor_add_timer(executor, &s_status_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add timer to executor: %d", (int)rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ready — strategy/atomic_phase → /phase_status");
    return ESP_OK;
}
