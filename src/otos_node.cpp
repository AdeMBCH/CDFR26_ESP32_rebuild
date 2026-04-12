/**
 * @file otos_node.cpp
 * @brief SparkFun OTOS sensor initialisation and micro-ROS publisher registration
 *
 * Publishes:
 *   /otos/odometry  — nav_msgs/Odometry at PUBLISH_FREQUENCY_HZ
 *   /otos/velocity  — geometry_msgs/Twist at PUBLISH_FREQUENCY_HZ
 *
 * Adds 1 handle to the executor (OTOS timer).
 */

#include "otos_node.h"
#include "config.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

// SparkFun OTOS
#include "SparkFun_Qwiic_OTOS_ESP32.h"
#include "esp32_i2c.h"

// microROS
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// ROS 2 messages
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

static const char *TAG = "otos_node";

// ---------------------------------------------------------------------------
// Error handling
// ---------------------------------------------------------------------------
#define RCSOFTCHECK(fn) \
    do { \
        rcl_ret_t _rc = (fn); \
        if (_rc != RCL_RET_OK) { \
            ESP_LOGW(TAG, "rcl error %d at line %d — continuing", (int)_rc, __LINE__); \
        } \
    } while (0)

// ---------------------------------------------------------------------------
// Module-private state
// ---------------------------------------------------------------------------
static QwiicOTOS_ESP32 s_otos;

static rcl_publisher_t           s_odom_pub;
static rcl_publisher_t           s_vel_pub;
static nav_msgs__msg__Odometry   s_odom_msg;
static geometry_msgs__msg__Twist s_vel_msg;

// ---------------------------------------------------------------------------
// OTOS reader task — decouples blocking I2C reads from the rclc executor.
// The reader runs at PUBLISH_FREQUENCY_HZ in its own FreeRTOS task and
// stores the latest sample in a depth-1 queue via xQueueOverwrite.
// The timer callback reads non-destructively with xQueuePeek (no blocking).
// ---------------------------------------------------------------------------
typedef struct {
    sfe_otos_pose2d_t pos;
    sfe_otos_pose2d_t vel;
} otos_sample_t;

static QueueHandle_t s_otos_queue;  /**< depth=1, writer: otos_reader_task, reader: otos_timer_callback */
static rcl_timer_t   s_otos_timer;  /**< module-level so otos_node_fini() can finalize it */
static bool          s_reader_started = false; /**< queue+task created only once */

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

static void check_i2c_device(void)
{
    ESP_LOGI(TAG, "Checking for OTOS at I2C address 0x17 …");

    ESP32_I2C scanner;
    esp32_i2c_config_t cfg = {
        .port           = OTOS_I2C_PORT,
        .sda_pin        = OTOS_SDA_PIN,
        .scl_pin        = OTOS_SCL_PIN,
        .freq_hz        = OTOS_I2C_FREQ,
        .device_address = 0x17,
    };

    if (scanner.init(cfg) != kESP32I2C_OK) {
        ESP_LOGE(TAG, "I2C bus init failed");
        return;
    }
    if (scanner.ping() == kESP32I2C_OK) {
        ESP_LOGI(TAG, "OTOS detected at 0x17");
    } else {
        ESP_LOGW(TAG, "OTOS not detected — check SDA=GPIO%d SCL=GPIO%d",
                 OTOS_SDA_PIN, OTOS_SCL_PIN);
    }
    scanner.deinit();
}

// ---------------------------------------------------------------------------
// OTOS reader task — priority 4, runs at PUBLISH_FREQUENCY_HZ
// ---------------------------------------------------------------------------
static void otos_reader_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TIMER_PERIOD_MS);

    for (;;) {
        otos_sample_t sample;
        sfe_otos_pose2d_t acc; /* discarded */
        if (s_otos.getPosVelAcc(sample.pos, sample.vel, acc) == kESP32I2C_OK) {
            xQueueOverwrite(s_otos_queue, &sample);
        } else {
            ESP_LOGW(TAG, "OTOS read failed");
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
// Timer callback — 50 Hz (executor context, non-blocking)
// ---------------------------------------------------------------------------
static void otos_timer_callback(rcl_timer_t *timer, int64_t /*last_call_time*/)
{
    if (timer == NULL) return;

    otos_sample_t sample;
    if (xQueuePeek(s_otos_queue, &sample, 0) != pdTRUE) return; /* no data yet */

    int64_t now_ns = rmw_uros_epoch_nanos();

    // --- Odometry ---
    s_odom_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    s_odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    s_odom_msg.pose.pose.position.x = sample.pos.x;
    s_odom_msg.pose.pose.position.y = sample.pos.y;
    s_odom_msg.pose.pose.position.z = 0.0;

    double half_h = sample.pos.h / 2.0;
    s_odom_msg.pose.pose.orientation.x = 0.0;
    s_odom_msg.pose.pose.orientation.y = 0.0;
    s_odom_msg.pose.pose.orientation.z = sin(half_h);
    s_odom_msg.pose.pose.orientation.w = cos(half_h);

    s_odom_msg.twist.twist.linear.x  = sample.vel.x;
    s_odom_msg.twist.twist.linear.y  = sample.vel.y;
    s_odom_msg.twist.twist.linear.z  = 0.0;
    s_odom_msg.twist.twist.angular.x = 0.0;
    s_odom_msg.twist.twist.angular.y = 0.0;
    s_odom_msg.twist.twist.angular.z = sample.vel.h;

    RCSOFTCHECK(rcl_publish(&s_odom_pub, &s_odom_msg, NULL));

    // --- Velocity (convenience topic) ---
    s_vel_msg.linear.x  = sample.vel.x;
    s_vel_msg.linear.y  = sample.vel.y;
    s_vel_msg.linear.z  = 0.0;
    s_vel_msg.angular.x = 0.0;
    s_vel_msg.angular.y = 0.0;
    s_vel_msg.angular.z = sample.vel.h;

    RCSOFTCHECK(rcl_publish(&s_vel_pub, &s_vel_msg, NULL));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool otos_init(void)
{
    ESP_LOGI(TAG, "Initializing OTOS sensor …");
    check_i2c_device();

    if (!s_otos.begin(OTOS_I2C_PORT, OTOS_SDA_PIN, OTOS_SCL_PIN, OTOS_I2C_FREQ)) {
        ESP_LOGE(TAG, "otos.begin() failed — is the sensor present at 0x17?");
        return false;
    }

    s_otos.setLinearUnit(kSfeOtosLinearUnitMeters);
    s_otos.setAngularUnit(kSfeOtosAngularUnitRadians);

    sfe_otos_version_t hw, fw;
    if (s_otos.getVersionInfo(hw, fw) == kESP32I2C_OK) {
        ESP_LOGI(TAG, "OTOS HW v%d.%d  FW v%d.%d",
                 hw.major, hw.minor, fw.major, fw.minor);
    }

    ESP_LOGI(TAG, "Calibrating IMU — keep the sensor still …");
    if (s_otos.calibrateImu() == kESP32I2C_OK) {
        ESP_LOGI(TAG, "IMU calibration done");
    } else {
        ESP_LOGW(TAG, "IMU calibration failed, continuing anyway");
    }

    s_otos.resetTracking();
    ESP_LOGI(TAG, "OTOS ready");
    return true;
}

void otos_node_fini(rcl_node_t *node)
{
    rcl_timer_fini(&s_otos_timer);
    rcl_publisher_fini(&s_odom_pub, node);
    rcl_publisher_fini(&s_vel_pub,  node);
}

esp_err_t otos_node_register(rcl_node_t *node, rclc_executor_t *executor,
                              rclc_support_t *support)
{
    // ── Publishers ───────────────────────────────────────────────────────────
    if (rclc_publisher_init_default(
            &s_odom_pub, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "otos/odometry") != RCL_RET_OK) return ESP_FAIL;

    if (rclc_publisher_init_default(
            &s_vel_pub, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "otos/velocity") != RCL_RET_OK) return ESP_FAIL;

    s_odom_msg.header.frame_id.data     = (char *)"odom";
    s_odom_msg.header.frame_id.size     = strlen("odom");
    s_odom_msg.header.frame_id.capacity = s_odom_msg.header.frame_id.size + 1;

    s_odom_msg.child_frame_id.data     = (char *)"base_link";
    s_odom_msg.child_frame_id.size     = strlen("base_link");
    s_odom_msg.child_frame_id.capacity = s_odom_msg.child_frame_id.size + 1;

    // ── Timer ────────────────────────────────────────────────────────────────
    if (rclc_timer_init_default(&s_otos_timer, support,
                                RCL_MS_TO_NS(TIMER_PERIOD_MS),
                                otos_timer_callback) != RCL_RET_OK) return ESP_FAIL;

    if (rclc_executor_add_timer(executor, &s_otos_timer) != RCL_RET_OK) return ESP_FAIL;
    ESP_LOGI(TAG, "OTOS timer: %d ms (%d Hz)", TIMER_PERIOD_MS, PUBLISH_FREQUENCY_HZ);

    // ── Reader task (priority 4 — below executor, above background) ──────────
    if (!s_reader_started) {
        s_otos_queue = xQueueCreate(1, sizeof(otos_sample_t));
        if (!s_otos_queue) {
            ESP_LOGE(TAG, "Failed to create OTOS queue");
            return ESP_FAIL;
        }
        if (xTaskCreate(otos_reader_task, "otos_reader", 4096, NULL, 4, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create OTOS reader task");
            return ESP_FAIL;
        }
        s_reader_started = true;
        ESP_LOGI(TAG, "OTOS reader task started (%d Hz)", PUBLISH_FREQUENCY_HZ);
    }

    return ESP_OK;
}
