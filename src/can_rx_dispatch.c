/**
 * @file can_rx_dispatch.c
 * @brief Centralised CAN RX dispatcher — see can_rx_dispatch.h for design notes.
 */

#include "can_rx_dispatch.h"
#include "config.h"

#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "can_rx";

/* ── Routing table ────────────────────────────────────────────────────────
 * One entry per (motor_id, cmd_byte) pair we care about.
 * QueueHandle is created at init time; depth=4 buffers the immediate ACK
 * (status=1) and the spontaneous completion frame (status=2) without overflow.
 * ─────────────────────────────────────────────────────────────────────────*/

#define RX_QUEUE_DEPTH   4

typedef struct {
    uint8_t       motor_id;
    uint8_t       cmd_byte;
    QueueHandle_t queue;
} slot_t;

static slot_t s_slots[] = {
    /* 91H GoHome — homing_node */
    { MOTOR_GRIPPER_MASTER, 0x91u, NULL },
    { MOTOR_GRIPPER_SLAVE,  0x91u, NULL },
    { MOTOR_STORAGE_MASTER, 0x91u, NULL },
    { MOTOR_STORAGE_SLAVE,  0x91u, NULL },
    /* F5H AbsMove — phase_node */
    { MOTOR_GRIPPER_MASTER, 0xF5u, NULL },
    { MOTOR_GRIPPER_SLAVE,  0xF5u, NULL },
    { MOTOR_STORAGE_MASTER, 0xF5u, NULL },
    { MOTOR_STORAGE_SLAVE,  0xF5u, NULL },
    /* 3EH Stall query — mks_get_stall_status */
    { MOTOR_GRIPPER_MASTER, 0x3Eu, NULL },
    { MOTOR_GRIPPER_SLAVE,  0x3Eu, NULL },
    { MOTOR_STORAGE_MASTER, 0x3Eu, NULL },
    { MOTOR_STORAGE_SLAVE,  0x3Eu, NULL },
    /* 3BH Home status query — mks_read_home_status */
    { MOTOR_GRIPPER_MASTER, 0x3Bu, NULL },
    { MOTOR_GRIPPER_SLAVE,  0x3Bu, NULL },
    { MOTOR_STORAGE_MASTER, 0x3Bu, NULL },
    { MOTOR_STORAGE_SLAVE,  0x3Bu, NULL },
};

#define NUM_SLOTS  (sizeof(s_slots) / sizeof(s_slots[0]))

/* Linear scan — table is small (16 entries), no hash needed. */
static slot_t *find_slot(uint8_t motor_id, uint8_t cmd_byte)
{
    for (size_t i = 0; i < NUM_SLOTS; i++) {
        if (s_slots[i].motor_id == motor_id &&
            s_slots[i].cmd_byte == cmd_byte) {
            return &s_slots[i];
        }
    }
    return NULL;
}

/* ── Dispatch task ────────────────────────────────────────────────────────
 * Priority 6: higher than micro_ros_task (5) so frames reach their queues
 * before any consumer wakes and calls can_rx_wait().
 * ─────────────────────────────────────────────────────────────────────────*/

#define CAN_RX_TASK_STACK    2048
#define CAN_RX_TASK_PRIORITY 6

static void can_rx_task(void *arg)
{
    (void)arg;
    twai_message_t msg;

    for (;;) {
        /* 20 ms timeout keeps the task responsive without busy-spinning */
        if (twai_receive(&msg, pdMS_TO_TICKS(20)) != ESP_OK) continue;
        if (msg.data_length_code < 1) continue;

        uint8_t motor_id = (uint8_t)msg.identifier;
        uint8_t cmd      = msg.data[0];

        slot_t *slot = find_slot(motor_id, cmd);
        if (slot == NULL) {
            /* Frame not in routing table — unregistered response, ignore. */
            continue;
        }

        can_rx_frame_t frame;
        frame.motor_id = motor_id;
        frame.dlc      = msg.data_length_code;
        memcpy(frame.data, msg.data, msg.data_length_code);

        if (xQueueSend(slot->queue, &frame, 0) != pdTRUE) {
            /* Consumer is not reading fast enough — drop rather than block */
            ESP_LOGW(TAG, "queue full motor=%u cmd=0x%02X — frame dropped",
                     (unsigned)motor_id, (unsigned)cmd);
        }
    }
}

/* ── Public API ───────────────────────────────────────────────────────────*/

esp_err_t can_rx_dispatch_init(void)
{
    for (size_t i = 0; i < NUM_SLOTS; i++) {
        s_slots[i].queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_rx_frame_t));
        if (!s_slots[i].queue) {
            ESP_LOGE(TAG, "xQueueCreate failed at slot %zu", i);
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK,
                    NULL, CAN_RX_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ready — %zu slots registered, task prio=%d",
             NUM_SLOTS, CAN_RX_TASK_PRIORITY);
    return ESP_OK;
}

esp_err_t can_rx_wait(uint8_t motor_id, uint8_t cmd_byte,
                      can_rx_frame_t *out, uint32_t timeout_ms)
{
    slot_t *slot = find_slot(motor_id, cmd_byte);
    if (!slot) {
        ESP_LOGE(TAG, "can_rx_wait: no slot for motor=%u cmd=0x%02X",
                 (unsigned)motor_id, (unsigned)cmd_byte);
        return ESP_ERR_NOT_FOUND;
    }

    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    if (xQueueReceive(slot->queue, out, ticks) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}
