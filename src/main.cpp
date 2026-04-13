/**
 * @file main.cpp
 * @brief Application entry point — microROS OTOS + MKS motor control
 *
 * Hardware:
 *   - SparkFun Qwiic OTOS (I2C)       → odometry at 50 Hz over UART to ROS 2 Humble
 *   - TJA1050 CAN transceiver (TWAI)  → MKS SERVO42D/57D motor commands at 500 kbps
 *   - WS2812 RGB LED                  → connection status indicator
 *
 * @author CDFR 2026
 */

#include "config.h"
#include "led_manager.h"
#include "otos_node.h"
#include "ros_app.h"
#include "can_driver.h"
#include "can_rx_dispatch.h"
#include "esp32_serial_transport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>

static const char *TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "microROS OTOS ESP32-S3 — starting (ROS 2 Humble, USB serial transport)");

    // --- LED ---
    led_manager_init();
    led_manager_set_state(LED_STATE_NO_UART);

    xTaskCreate(led_task, "led_task",
                LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);

    // --- microROS USB serial transport ---
    rmw_uros_set_custom_transport(
        true,
        NULL,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );

    // --- TWAI / CAN bus ---
    if (can_driver_init() != ESP_OK) {
        ESP_LOGW(TAG, "CAN driver init failed — motor commands will not work");
    }
    if (can_rx_dispatch_init() != ESP_OK) {
        ESP_LOGW(TAG, "CAN RX dispatch init failed — motor feedback will not work");
    }

    // --- OTOS sensor ---
    if (!otos_init()) {
        ESP_LOGW(TAG, "OTOS initialisation failed — continuing without sensor");
    }

    // Give the micro-ROS agent time to be ready
    ESP_LOGI(TAG, "Waiting 3 s for micro-ROS agent …");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // --- microROS task ---
    xTaskCreate(micro_ros_task, "micro_ros_task",
                UROS_TASK_STACK_SIZE, NULL, UROS_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "micro_ros_task created");
}
