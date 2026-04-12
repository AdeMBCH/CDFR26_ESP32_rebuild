/**
 * @file homing_node.c
 * @brief micro-ROS homing and calibration services for MKS mechanism motors
 *
 * Services (std_srvs/srv/Trigger) — all fire-and-forget, respond immediately:
 *   /home_gripper             — Home gripper: motors MOTOR_GRIPPER_MASTER & SLAVE
 *   /home_storage             — Home storage: motors MOTOR_STORAGE_MASTER & SLAVE
 *   /calibrate_encoder        — Encoder self-calibration on mechanism motors
 *   /calibrate_wheel_encoders — Encoder self-calibration on wheel motors
 *
 * Publisher (std_msgs/String):
 *   /homing_status — Result published when the background operation completes
 *
 * Architecture:
 *   All long-running operations run in a dedicated FreeRTOS task so the rclc
 *   executor is never blocked. Service callbacks enqueue the request and return
 *   immediately with {success=true, message="<op> started"} (or "busy").
 *   When the operation completes, the result is published on /homing_status
 *   via a 200 ms timer that runs inside the executor (thread-safe publish).
 *
 * Homing strategy (noLimit stall mode):
 *   1. Release lingering stall state (3DH).
 *   2. Enable master and slave motors (F3H).
 *   3. Trigger the built-in GoHome sequence on both motors (91H).
 *   4. Wait for spontaneous [0x91, 0x02] completion frames from both motors,
 *      routed from the CAN RX dispatcher via can_rx_wait().
 *   5. On timeout (HOMING_TIMEOUT_MS): emergency-stop both motors and report
 *      FAIL:TIMEOUT. On motor error (status=0x00): report FAIL:MOTOR_ERR.
 *
 * Calibration: fire-and-forget (80H), followed by a fixed CALIBRATE_SETTLE_MS
 * delay — the MKS 80H command never sends a spontaneous completion frame.
 */

#include "homing_node.h"
#include "can_driver.h"
#include "can_rx_dispatch.h"
#include "config.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_srvs/srv/trigger.h>
#include <std_msgs/msg/string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "homing_node";

/* ── Background operation types ──────────────────────────────────────────── */

typedef enum {
    OP_HOME_GRIPPER = 0,
    OP_HOME_STORAGE,
    OP_CAL_ENCODER,
    OP_CAL_WHEELS,
} homing_op_t;

typedef struct {
    homing_op_t op;
} homing_request_t;

/* ── Shared result ────────────────────────────────────────────────────────
 * Written exclusively by homing_bg_task.
 * Read exclusively by status_timer_cb (executor context).
 * `pending` is the handoff flag: background task sets it last (after writing
 * message/success), timer callback reads it first and clears it.
 * On Xtensa LX7, 32-bit aligned bool writes are atomic — safe for this
 * single-producer / single-consumer pattern without a mutex.
 * ─────────────────────────────────────────────────────────────────────────*/

typedef struct {
    char          message[128];
    volatile bool pending;
} homing_result_t;

static QueueHandle_t   s_queue;    /**< depth=1: only one op at a time */
static homing_result_t s_result;
static bool            s_bg_started = false; /**< queue+task created only once */

/* ── micro-ROS service entities ──────────────────────────────────────────── */

static rcl_service_t s_home_gripper_srv;
static rcl_service_t s_home_storage_srv;
static rcl_service_t s_cal_enc_srv;
static rcl_service_t s_cal_wheel_srv;

static std_srvs__srv__Trigger_Request  s_home_gripper_req;
static std_srvs__srv__Trigger_Response s_home_gripper_resp;
static char                            s_gripper_msg[64];

static std_srvs__srv__Trigger_Request  s_home_storage_req;
static std_srvs__srv__Trigger_Response s_home_storage_resp;
static char                            s_storage_msg[64];

static std_srvs__srv__Trigger_Request  s_cal_enc_req;
static std_srvs__srv__Trigger_Response s_cal_enc_resp;
static char                            s_cal_enc_msg[64];

static std_srvs__srv__Trigger_Request  s_cal_wheel_req;
static std_srvs__srv__Trigger_Response s_cal_wheel_resp;
static char                            s_cal_wheel_msg[64];

/* ── /homing_status publisher + polling timer ────────────────────────────── */

static rcl_publisher_t       s_status_pub;
static std_msgs__msg__String s_status_msg;
static char                  s_status_buf[128];
static rcl_timer_t           s_status_timer;

/* ── Homing implementation ───────────────────────────────────────────────── */

typedef enum {
    HOMING_OK = 0,
    HOMING_MOTOR_FAIL,
    HOMING_TIMEOUT,
} homing_err_t;

/**
 * Home a master/slave motor pair using the MKS GoHome (91H) command.
 *
 * Sends GoHome to both motors simultaneously then waits for spontaneous
 * [0x91, 0x02, CRC] completion frames from each via the CAN RX dispatcher.
 * On timeout, sends emergency stop to any motor that has not yet completed.
 */
static homing_err_t run_homing(uint8_t master_id, uint8_t slave_id)
{
    ESP_LOGI(TAG, "Homing motors %u & %u — timeout=%u ms",
             (unsigned)master_id, (unsigned)slave_id,
             (unsigned)HOMING_TIMEOUT_MS);

    mks_release_stall(master_id);
    mks_release_stall(slave_id);
    vTaskDelay(pdMS_TO_TICKS(50));

    mks_set_enable(master_id, true);
    mks_set_enable(slave_id,  true);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (mks_go_home(master_id) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send GoHome to motor %u", (unsigned)master_id);
        return HOMING_MOTOR_FAIL;
    }
    if (mks_go_home(slave_id) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send GoHome to motor %u — stopping master %u",
                 (unsigned)slave_id, (unsigned)master_id);
        mks_emergency_stop(master_id);
        return HOMING_MOTOR_FAIL;
    }

    /* Wait for spontaneous 91H completion frames from both motors.
     * Poll each motor with a 20 ms window per call, checking the global
     * deadline between iterations. Discards status=0x01 (starting ACK). */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(HOMING_TIMEOUT_MS);
    bool master_done = false, slave_done = false;
    bool master_ok   = false, slave_ok   = false;

    while (!master_done || !slave_done) {
        if ((int32_t)(deadline - xTaskGetTickCount()) <= 0) {
            ESP_LOGE(TAG, "Homing timeout — stopping motors %u & %u",
                     (unsigned)master_id, (unsigned)slave_id);
            if (!master_done) mks_emergency_stop(master_id);
            if (!slave_done)  mks_emergency_stop(slave_id);
            return HOMING_TIMEOUT;
        }

        can_rx_frame_t f;
        if (!master_done && can_rx_wait(master_id, 0x91u, &f, 20) == ESP_OK) {
            if      (f.data[1] == 0x02u) { master_done = true; master_ok = true; }
            else if (f.data[1] == 0x00u) { master_done = true; }
            /* 0x01 = starting ACK — keep waiting */
        }
        if (!slave_done && can_rx_wait(slave_id, 0x91u, &f, 20) == ESP_OK) {
            if      (f.data[1] == 0x02u) { slave_done = true; slave_ok = true; }
            else if (f.data[1] == 0x00u) { slave_done = true; }
        }
    }

    ESP_LOGI(TAG, "Motors %u & %u homed: master=%s slave=%s",
             (unsigned)master_id, (unsigned)slave_id,
             master_ok ? "OK" : "FAIL", slave_ok ? "OK" : "FAIL");
    return (master_ok && slave_ok) ? HOMING_OK : HOMING_MOTOR_FAIL;
}

/* ── Background task ─────────────────────────────────────────────────────── */

static void homing_bg_task(void *arg)
{
    (void)arg;
    homing_request_t req;

    for (;;) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) continue;

        switch (req.op) {

            case OP_HOME_GRIPPER: {
                homing_err_t r = run_homing(MOTOR_GRIPPER_MASTER, MOTOR_GRIPPER_SLAVE);
                snprintf(s_result.message, sizeof(s_result.message), "Gripper: %s",
                         r == HOMING_OK      ? "OK" :
                         r == HOMING_TIMEOUT ? "FAIL:TIMEOUT" : "FAIL:MOTOR_ERR");
                s_result.pending = true;
                ESP_LOGI(TAG, "homing_bg: %s", s_result.message);
                break;
            }

            case OP_HOME_STORAGE: {
                homing_err_t r = run_homing(MOTOR_STORAGE_MASTER, MOTOR_STORAGE_SLAVE);
                snprintf(s_result.message, sizeof(s_result.message), "Storage: %s",
                         r == HOMING_OK      ? "OK" :
                         r == HOMING_TIMEOUT ? "FAIL:TIMEOUT" : "FAIL:MOTOR_ERR");
                s_result.pending = true;
                ESP_LOGI(TAG, "homing_bg: %s", s_result.message);
                break;
            }

            case OP_CAL_ENCODER: {
                /* Fire-and-forget: send commands and return immediately.
                 * The motor rotates ~10 s to calibrate — no result published. */
                const uint8_t motors[] = {
                    MOTOR_GRIPPER_MASTER, MOTOR_GRIPPER_SLAVE,
                    MOTOR_STORAGE_MASTER, MOTOR_STORAGE_SLAVE,
                };
                for (size_t i = 0; i < 4; i++) {
                    ESP_LOGI(TAG, "Calibrating encoder — motor %u", (unsigned)motors[i]);
                    mks_calibrate_encoder(motors[i]);
                }
                break;
            }

            case OP_CAL_WHEELS: {
                /* Fire-and-forget: same as OP_CAL_ENCODER. */
                const uint8_t motors[] = {
                    MOTOR_OMNI_FL, MOTOR_OMNI_FR,
                    MOTOR_OMNI_RL, MOTOR_OMNI_RR,
                };
                for (size_t i = 0; i < 4; i++) {
                    ESP_LOGI(TAG, "Calibrating encoder — wheel motor %u", (unsigned)motors[i]);
                    mks_calibrate_encoder(motors[i]);
                }
                break;
            }
        }
    }
}

/* ── Status timer callback (executor context — safe to publish) ───────────── */

static void status_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL)        return;
    if (!s_result.pending)    return;

    s_result.pending = false;

    strncpy(s_status_buf, s_result.message, sizeof(s_status_buf) - 1);
    s_status_buf[sizeof(s_status_buf) - 1] = '\0';

    s_status_msg.data.data     = s_status_buf;
    s_status_msg.data.size     = strlen(s_status_buf);
    s_status_msg.data.capacity = sizeof(s_status_buf);

    rcl_publish(&s_status_pub, &s_status_msg, NULL);
    ESP_LOGI(TAG, "/homing_status → \"%s\"", s_status_buf);
}

/* ── Helper: enqueue an op, reply immediately ─────────────────────────────── */

static void enqueue_op(homing_op_t op,
                       std_srvs__srv__Trigger_Response *resp,
                       const char *svc_name,
                       char *msg_buf, size_t msg_cap)
{
    homing_request_t req = { .op = op };
    bool sent = (xQueueSend(s_queue, &req, 0) == pdTRUE);

    resp->success = sent;
    const char *txt = sent ? "started" : "busy — previous operation still running";
    strncpy(msg_buf, txt, msg_cap - 1);
    msg_buf[msg_cap - 1]   = '\0';
    resp->message.data     = msg_buf;
    resp->message.size     = strlen(msg_buf);
    resp->message.capacity = msg_cap;

    ESP_LOGI(TAG, "/%s → %s", svc_name, txt);
}

/* ── Service callbacks ───────────────────────────────────────────────────── */

static void home_gripper_cb(const void *req, void *resp)
{
    (void)req;
    enqueue_op(OP_HOME_GRIPPER,
               (std_srvs__srv__Trigger_Response *)resp,
               "home_gripper", s_gripper_msg, sizeof(s_gripper_msg));
}

static void home_storage_cb(const void *req, void *resp)
{
    (void)req;
    enqueue_op(OP_HOME_STORAGE,
               (std_srvs__srv__Trigger_Response *)resp,
               "home_storage", s_storage_msg, sizeof(s_storage_msg));
}

static void calibrate_encoder_cb(const void *req, void *resp)
{
    (void)req;
    enqueue_op(OP_CAL_ENCODER,
               (std_srvs__srv__Trigger_Response *)resp,
               "calibrate_encoder", s_cal_enc_msg, sizeof(s_cal_enc_msg));
}

static void calibrate_wheel_encoders_cb(const void *req, void *resp)
{
    (void)req;
    enqueue_op(OP_CAL_WHEELS,
               (std_srvs__srv__Trigger_Response *)resp,
               "calibrate_wheel_encoders", s_cal_wheel_msg, sizeof(s_cal_wheel_msg));
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t homing_node_init(rcl_node_t *node, rclc_executor_t *executor,
                            rclc_support_t *support)
{
    rcl_ret_t rc;
    const rosidl_service_type_support_t *trig_ts =
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);

    if (!s_bg_started) {
        s_queue = xQueueCreate(1, sizeof(homing_request_t));
        if (!s_queue) {
            ESP_LOGE(TAG, "Failed to create queue");
            return ESP_FAIL;
        }
        if (xTaskCreate(homing_bg_task, "homing_bg",
                        4096, NULL, 3, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create background task");
            return ESP_FAIL;
        }
        s_bg_started = true;
    } else {
        /* Flush stale requests from the previous session */
        xQueueReset(s_queue);
    }

    s_home_gripper_resp.message.data     = s_gripper_msg;
    s_home_gripper_resp.message.capacity = sizeof(s_gripper_msg);
    s_home_gripper_resp.message.size     = 0;

    s_home_storage_resp.message.data     = s_storage_msg;
    s_home_storage_resp.message.capacity = sizeof(s_storage_msg);
    s_home_storage_resp.message.size     = 0;

    s_cal_enc_resp.message.data          = s_cal_enc_msg;
    s_cal_enc_resp.message.capacity      = sizeof(s_cal_enc_msg);
    s_cal_enc_resp.message.size          = 0;

    s_cal_wheel_resp.message.data        = s_cal_wheel_msg;
    s_cal_wheel_resp.message.capacity    = sizeof(s_cal_wheel_msg);
    s_cal_wheel_resp.message.size        = 0;

#define INIT_SRV(srv, req, resp, name, cb)                                          \
    rc = rclc_service_init_default(&(srv), node, trig_ts, (name));                  \
    if (rc != RCL_RET_OK) {                                                         \
        ESP_LOGE(TAG, "Failed to create /%s: %d", (name), (int)rc);                 \
        return ESP_FAIL;                                                             \
    }                                                                               \
    rc = rclc_executor_add_service(executor, &(srv), &(req), &(resp), (cb));        \
    if (rc != RCL_RET_OK) {                                                         \
        ESP_LOGE(TAG, "Failed to add /%s to executor: %d", (name), (int)rc);        \
        return ESP_FAIL;                                                             \
    }

    INIT_SRV(s_home_gripper_srv, s_home_gripper_req, s_home_gripper_resp,
             "home_gripper",             home_gripper_cb);
    INIT_SRV(s_home_storage_srv, s_home_storage_req, s_home_storage_resp,
             "home_storage",             home_storage_cb);
    INIT_SRV(s_cal_enc_srv,      s_cal_enc_req,      s_cal_enc_resp,
             "calibrate_encoder",        calibrate_encoder_cb);
    INIT_SRV(s_cal_wheel_srv,    s_cal_wheel_req,    s_cal_wheel_resp,
             "calibrate_wheel_encoders", calibrate_wheel_encoders_cb);

#undef INIT_SRV

    rc = rclc_publisher_init_default(
        &s_status_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "homing_status");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create /homing_status publisher: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_timer_init_default(&s_status_timer, support,
                                 RCL_MS_TO_NS(200),
                                 status_timer_cb);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create status timer: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_executor_add_timer(executor, &s_status_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add status timer to executor: %d", (int)rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ready — /home_gripper /home_storage"
                  " /calibrate_encoder /calibrate_wheel_encoders /homing_status");
    return ESP_OK;
}
