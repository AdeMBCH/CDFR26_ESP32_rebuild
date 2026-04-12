/**
 * @file can_driver.c
 * @brief MKS SERVO42D/57D CAN bus driver — TWAI + TJA1050
 *
 * MKS CAN Bus protocol V1.0.9 quick reference
 * ─────────────────────────────────────────────
 * Frame  : Standard 11-bit ID, DLC ≤ 8 bytes, 500 kbps
 * CRC    : (CAN_ID + Byte1 + … + ByteN) & 0xFF  (last byte of payload)
 * Endian : Big-endian
 *
 * F6H Speed control frame (DLC = 5):
 *   [0xF6] [Dir(b7)|SpeedH4(b3-b0)] [SpeedL8] [acc] [CRC]
 *   Dir=1 → CCW, Dir=0 → CW.  Speed is 12-bit RPM value.
 *
 * F3H Enable/disable (DLC = 3):
 *   [0xF3] [0x01=enable / 0x00=disable] [CRC]
 *
 * F7H Emergency stop (DLC = 2):
 *   [0xF7] [CRC]
 */

#include "can_driver.h"
#include "can_rx_dispatch.h"
#include "config.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

static const char *TAG = "can_driver";

/* ─────────────────────────────────────────────────────────────────────────── */
/* Internal helpers                                                             */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * MKS 8-bit CRC: sum of CAN_ID and all payload bytes (excluding the CRC
 * byte itself), masked to 8 bits.
 */
static uint8_t mks_crc(uint8_t motor_id, const uint8_t *payload, size_t len)
{
    uint16_t sum = motor_id;
    for (size_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/** Transmit a prepared TWAI frame with a 10 ms timeout. */
static esp_err_t twai_send_frame(const twai_message_t *msg)
{
    esp_err_t ret = twai_transmit(msg, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TWAI tx id=0x%03lx failed: %s",
                 (unsigned long)msg->identifier, esp_err_to_name(ret));
    }
    return ret;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/* Public API                                                                   */
/* ─────────────────────────────────────────────────────────────────────────── */

esp_err_t can_driver_init(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,
                                    (gpio_num_t)TWAI_RX_PIN,
                                    TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "TWAI ready — TX=GPIO%d  RX=GPIO%d  500 kbps",
             TWAI_TX_PIN, TWAI_RX_PIN);
    return ESP_OK;
}

/* ── 3DH: Release stall ─────────────────────────────────────────────────── */

esp_err_t mks_release_stall(uint8_t motor_id)
{
    uint8_t payload[1] = {0x3Du};
    uint8_t crc = mks_crc(motor_id, payload, 1);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 2;
    msg.data[0] = 0x3Du;
    msg.data[1] = crc;

    return twai_send_frame(&msg);
}

/* ── 3EH: Query stall status ────────────────────────────────────────────── */

esp_err_t mks_get_stall_status(uint8_t motor_id, bool *stalled, uint32_t timeout_ms)
{
    /* Transmit the 3EH query frame */
    uint8_t payload[1] = {0x3Eu};
    uint8_t crc = mks_crc(motor_id, payload, 1);

    twai_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.identifier       = motor_id;
    tx.data_length_code = 2;
    tx.data[0] = 0x3Eu;
    tx.data[1] = crc;

    esp_err_t ret = twai_send_frame(&tx);
    if (ret != ESP_OK) return ret;

    /* Wait via dispatcher — no direct twai_receive() call */
    can_rx_frame_t frame;
    ret = can_rx_wait(motor_id, 0x3Eu, &frame, timeout_ms);
    if (ret != ESP_OK) return ret;

    *stalled = (frame.data[1] != 0u);
    return ESP_OK;
}

/* ── 80H: Calibrate encoder ─────────────────────────────────────────────── */

esp_err_t mks_calibrate_encoder(uint8_t motor_id)
{
    /* DLC=3: [0x80, 0x00, CRC]
     * Fire-and-forget: the motor rotates several turns (~10-30 s) to calibrate
     * its magnetic encoder.  Like the 91H GoHome command, the motor responds
     * only once (immediate ACK) and never sends a spontaneous completion frame.
     * The caller must wait CALIBRATE_SETTLE_MS after sending before using the motor. */
    uint8_t payload[2] = {0x80u, 0x00u};
    uint8_t crc = mks_crc(motor_id, payload, 2);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 3;
    msg.data[0] = 0x80u;
    msg.data[1] = 0x00u;
    msg.data[2] = crc;

    return twai_send_frame(&msg);
}

/* ── 91H: GoHome ────────────────────────────────────────────────────────── */

esp_err_t mks_go_home(uint8_t motor_id)
{
    /* DLC=3: [0x91, goZeroMode=0x00, CRC]
     * goZeroMode 0x00 = return to origin (zero point).
     *
     * The motor sends two 91H responses (same format, DLC=3):
     *   Immediate ACK  : [0x91, 0x01, CRC]  — starting
     *   Spontaneous ACK: [0x91, 0x02, CRC]  — homing complete  ← key!
     *   Failure        : [0x91, 0x00, CRC]  — failed to return
     *
     * This function only transmits the command (fire-and-forget).
     * The caller is responsible for receiving and interpreting the responses.
     */
    uint8_t payload[2] = {0x91u, 0x00u};
    uint8_t crc = mks_crc(motor_id, payload, 2);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 3;
    msg.data[0] = 0x91u;
    msg.data[1] = 0x00u;
    msg.data[2] = crc;

    return twai_send_frame(&msg);
}

/* ── 3BH: Read zero-return status ───────────────────────────────────────── */

esp_err_t mks_read_home_status(uint8_t motor_id, uint8_t *status, uint32_t timeout_ms)
{
    /* Transmit the 3BH query frame */
    uint8_t payload[1] = {0x3Bu};
    uint8_t crc = mks_crc(motor_id, payload, 1);

    twai_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.identifier       = motor_id;
    tx.data_length_code = 2;
    tx.data[0] = 0x3Bu;
    tx.data[1] = crc;

    esp_err_t ret = twai_send_frame(&tx);
    if (ret != ESP_OK) return ret;

    /* Wait via dispatcher — no direct twai_receive() call.
     * Expected frame: [0x3B, status1, status2, CRC]
     * status1 = single-turn / noLimit stall mode
     * status2 = multi-turn / GoHome mode
     * Both encode: 0=in progress, 1=success, 2=failed. */
    can_rx_frame_t frame;
    ret = can_rx_wait(motor_id, 0x3Bu, &frame, timeout_ms);
    if (ret != ESP_OK) return ret;

    uint8_t s1 = frame.data[1];
    uint8_t s2 = (frame.dlc >= 4) ? frame.data[2] : 0u;
    ESP_LOGD(TAG, "3BH motor %u: status1=%u status2=%u",
             (unsigned)motor_id, s1, s2);
    if      (s1 == 1u || s2 == 1u) { *status = 1u; }
    else if (s1 == 2u || s2 == 2u) { *status = 2u; }
    else                            { *status = 0u; }
    return ESP_OK;
}

/* ── F5H: Absolute coordinate position control ──────────────────────────── */

esp_err_t mks_move_abs(uint8_t motor_id, int32_t position,
                       uint16_t speed_rpm, uint8_t acc)
{
    /* Clamp speed to hardware limit */
    if (speed_rpm > (uint16_t)MKS_MAX_RPM) speed_rpm = (uint16_t)MKS_MAX_RPM;

    uint8_t  dir   = (position >= 0) ? 1u : 0u;       /* 1=CCW, 0=CW */
    uint32_t pos   = (position >= 0)
                     ? (uint32_t)position
                     : (uint32_t)(-position);
    /* Clamp to 24-bit */
    if (pos > 0x7FFFFFu) pos = 0x7FFFFFu;

    /*
     * F5H byte layout (DLC = 8):
     *   Byte 1 : 0xF5
     *   Byte 2 : Dir(bit7) | Speed_high_4bits(bits 3-0)
     *   Byte 3 : Speed_low_8bits
     *   Byte 4 : Acceleration
     *   Byte 5 : Position_high_8bits
     *   Byte 6 : Position_mid_8bits
     *   Byte 7 : Position_low_8bits
     *   Byte 8 : CRC
     */
    uint8_t b1 = (uint8_t)((dir << 7) | ((speed_rpm >> 8) & 0x0Fu));
    uint8_t b2 = (uint8_t)(speed_rpm & 0xFFu);
    uint8_t b4 = (uint8_t)((pos >> 16) & 0xFFu);
    uint8_t b5 = (uint8_t)((pos >>  8) & 0xFFu);
    uint8_t b6 = (uint8_t)(pos & 0xFFu);

    uint8_t payload[7] = {0xF5u, b1, b2, acc, b4, b5, b6};
    uint8_t crc = mks_crc(motor_id, payload, 7);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 8;
    msg.data[0] = 0xF5u;
    msg.data[1] = b1;
    msg.data[2] = b2;
    msg.data[3] = acc;
    msg.data[4] = b4;
    msg.data[5] = b5;
    msg.data[6] = b6;
    msg.data[7] = crc;

    return twai_send_frame(&msg);
}

/* ── F6H: Speed control ─────────────────────────────────────────────────── */

esp_err_t mks_send_speed_rpm(uint8_t motor_id, int16_t rpm, uint8_t acc)
{
    if (motor_id == 0) {
        ESP_LOGE(TAG, "mks_send_speed_rpm: id=0 is CAN broadcast — rejected");
        return ESP_ERR_INVALID_ARG;
    }

    /* Clamp to physical limit */
    if (rpm >  MKS_MAX_RPM) rpm =  (int16_t)MKS_MAX_RPM;
    if (rpm < -MKS_MAX_RPM) rpm = -(int16_t)MKS_MAX_RPM;

    uint8_t  dir   = (rpm >= 0) ? 1u : 0u;           /* 1=CCW, 0=CW */
    uint16_t speed = (uint16_t)(rpm < 0 ? -rpm : rpm);

    /*
     * F6H byte layout:
     *   Byte 1 (b2 in frame): Dir(bit7) | Speed_high_4bits(bits 3-0)
     *   Byte 2 (b3 in frame): Speed_low_8bits
     */
    uint8_t b1 = (uint8_t)((dir << 7) | ((speed >> 8) & 0x0Fu));
    uint8_t b2 = (uint8_t)(speed & 0xFFu);

    uint8_t payload[4] = {0xF6u, b1, b2, acc};
    uint8_t crc = mks_crc(motor_id, payload, 4);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 5;
    msg.data[0] = 0xF6u;
    msg.data[1] = b1;
    msg.data[2] = b2;
    msg.data[3] = acc;
    msg.data[4] = crc;

    return twai_send_frame(&msg);
}

esp_err_t mks_send_speed_rads(uint8_t motor_id, double omega_rads, uint8_t acc)
{
    if (!isfinite(omega_rads)) {
        ESP_LOGE(TAG, "mks_send_speed_rads: non-finite omega (%f) for motor %u — sending 0",
                 omega_rads, (unsigned)motor_id);
        omega_rads = 0.0;
    }
    double rpm_d = omega_rads * (60.0 / (2.0 * M_PI));
    /* Clamp to int16_t range before casting */
    if (rpm_d >  32767.0) rpm_d =  32767.0;
    if (rpm_d < -32767.0) rpm_d = -32767.0;
    return mks_send_speed_rpm(motor_id, (int16_t)rpm_d, acc);
}

/* ── F3H: Enable / disable ──────────────────────────────────────────────── */

esp_err_t mks_set_enable(uint8_t motor_id, bool enable)
{
    uint8_t state = enable ? 0x01u : 0x00u;
    uint8_t payload[2] = {0xF3u, state};
    uint8_t crc = mks_crc(motor_id, payload, 2);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 3;
    msg.data[0] = 0xF3u;
    msg.data[1] = state;
    msg.data[2] = crc;

    return twai_send_frame(&msg);
}

/* ── F7H: Emergency stop ────────────────────────────────────────────────── */

esp_err_t mks_emergency_stop(uint8_t motor_id)
{
    uint8_t payload[1] = {0xF7u};
    uint8_t crc = mks_crc(motor_id, payload, 1);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = motor_id;
    msg.data_length_code = 2;
    msg.data[0] = 0xF7u;
    msg.data[1] = crc;

    return twai_send_frame(&msg);
}
