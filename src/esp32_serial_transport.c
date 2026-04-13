#include <uxr/client/transport.h>

#include <driver/usb_serial_jtag.h>
#include <esp_log.h>
#include "config.h"

static const char *TAG = "USB_TRANSPORT";

// --- micro-ROS Transports ---
#define USB_BUFFER_SIZE (512)

bool esp32_serial_open(struct uxrCustomTransport * transport){
    (void)transport;

    ESP_LOGI(TAG, "Opening USB Serial/JTAG transport at host baudrate %d", UROS_SERIAL_BAUD);

    if (usb_serial_jtag_is_driver_installed()) {
        ESP_LOGI(TAG, "USB Serial/JTAG driver already installed");
        return true;
    }

    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = USB_BUFFER_SIZE,
        .rx_buffer_size = USB_BUFFER_SIZE,
    };

    if (usb_serial_jtag_driver_install(&usb_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Serial/JTAG driver");
        return false;
    }

    ESP_LOGI(TAG, "USB Serial/JTAG transport opened successfully");
    return true;
}

bool esp32_serial_close(struct uxrCustomTransport * transport){
    (void)transport;

    if (!usb_serial_jtag_is_driver_installed()) {
        return true;
    }

    return usb_serial_jtag_driver_uninstall() == ESP_OK;
}

size_t esp32_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    (void)transport;
    if (err != NULL) {
        *err = 0;
    }

    const int tx_bytes = usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
    if (tx_bytes > 0) {
        usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    }
    return tx_bytes > 0 ? (size_t)tx_bytes : 0U;
}

size_t esp32_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    (void)transport;
    if (err != NULL) {
        *err = 0;
    }

    TickType_t ticks_to_wait = timeout > 0 ? pdMS_TO_TICKS(timeout) : 0;
    const int rx_bytes = usb_serial_jtag_read_bytes(buf, len, ticks_to_wait);
    return rx_bytes > 0 ? (size_t)rx_bytes : 0U;
}
