/**
 * @file can_driver.h
 * @brief MKS SERVO42D/57D CAN bus driver — TWAI peripheral + TJA1050 transceiver
 *
 * Protocol: MKS CAN Bus V1.0.9
 *   - Standard 11-bit CAN frame, 500 kbps
 *   - CRC = (CAN_ID + payload bytes excl. CRC) & 0xFF
 *   - Big-endian byte order
 *
 * Motor IDs and pin assignments are defined in config.h.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the TWAI peripheral at 500 kbps.
 *        Must be called once in app_main() before any motor command.
 */
esp_err_t can_driver_init(void);

/**
 * @brief Send an absolute-coordinate position command to a motor (protocol F5H).
 *
 * The motor moves autonomously to the target position.  The call returns as
 * soon as the CAN frame has been transmitted (non-blocking).
 *
 * Frame layout (DLC = 8):
 *   [0xF5] [Dir(b7)|Speed_H4] [Speed_L8] [Acc] [Pos_H8] [Pos_M8] [Pos_L8] [CRC]
 *
 * @param motor_id   CAN ID of the target motor (1-16).
 * @param position   Absolute target coordinate in encoder units.
 *                   Positive → CCW (Dir=1), negative → CW (Dir=0).
 *                   0x4000 (16384) encoder units = 1 full revolution.
 *                   Range: ±8 388 607 (24-bit absolute value).
 * @param speed_rpm  Travel speed in RPM (clamped to MKS_MAX_RPM).
 * @param acc        Acceleration ramp 0-255 (0 = instant).
 * @return ESP_OK on successful TWAI transmission.
 */
esp_err_t mks_move_abs(uint8_t motor_id, int32_t position,
                       uint16_t speed_rpm, uint8_t acc);

/**
 * @brief Send a speed-control command to a single MKS motor (protocol F6H).
 *
 * @param motor_id  CAN ID of the target motor (1-16).
 * @param rpm       Signed RPM: ≥ 0 → CCW, < 0 → CW, 0 → stop.
 *                  Clamped to ±MKS_MAX_RPM automatically.
 * @param acc       Acceleration ramp 0-255 (0 = instant start/stop).
 * @return ESP_OK on successful TWAI transmission.
 */
esp_err_t mks_send_speed_rpm(uint8_t motor_id, int16_t rpm, uint8_t acc);

/**
 * @brief Convert angular velocity (rad/s) to RPM and send speed command.
 *
 * @param motor_id   CAN ID of the target motor.
 * @param omega_rads Signed velocity in rad/s (sign encodes direction).
 * @param acc        Acceleration ramp 0-255.
 */
esp_err_t mks_send_speed_rads(uint8_t motor_id, double omega_rads, uint8_t acc);

/**
 * @brief Enable or disable a motor (protocol F3H).
 *
 * @param motor_id CAN ID of the target motor.
 * @param enable   true → lock/enable, false → release/disable.
 */
esp_err_t mks_set_enable(uint8_t motor_id, bool enable);

/**
 * @brief Send an immediate emergency stop to a motor (protocol F7H).
 *
 * @param motor_id CAN ID of the target motor.
 */
esp_err_t mks_emergency_stop(uint8_t motor_id);

/**
 * @brief Release the stall protection state of a motor (protocol 3DH).
 *        Call this after the motor has been stopped following a stall event.
 *
 * @param motor_id CAN ID of the target motor.
 */
esp_err_t mks_release_stall(uint8_t motor_id);

/**
 * @brief Query the stall status of a motor (protocol 3EH).
 *
 * Transmits a 3EH request and waits for the motor's CAN response.
 * Frames from other CAN IDs received while waiting are discarded.
 *
 * @param motor_id   CAN ID of the target motor (1-16).
 * @param[out] stalled  Set to true if the motor reports a stall condition.
 * @param timeout_ms Timeout in milliseconds to wait for the response frame.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no response, other on TWAI error.
 */
esp_err_t mks_get_stall_status(uint8_t motor_id, bool *stalled, uint32_t timeout_ms);

/**
 * @brief Run the built-in encoder self-calibration of a motor (protocol 80H).
 *
 * Sends DLC=3 frame [0x80, 0x00, CRC].
 * The motor physically rotates several turns to calibrate its magnetic encoder.
 * WARNING: the motor shaft must be free to rotate — disconnect any mechanism
 * before running this command.  Only needed once during assembly.
 *
 * Fire-and-forget: like GoHome (91H), the motor responds only once with an
 * immediate ACK and never sends a spontaneous completion frame.
 * The caller must delay CALIBRATE_SETTLE_MS (~10 s) after this call
 * before the motor is ready to use.
 *
 * @param motor_id CAN ID of the target motor.
 * @return ESP_OK on successful TWAI transmission, error otherwise.
 */
esp_err_t mks_calibrate_encoder(uint8_t motor_id);

/**
 * @brief Trigger the built-in GoHome sequence of a motor (protocol 91H).
 *
 * Sends DLC=3 frame [0x91, 0x00 (origin return), CRC].
 * The motor runs autonomously toward its configured home position.
 * Homing parameters (Hm_Dir, Hm_Speed, Hm_Mode) must be set in the
 * motor's on-board menu before calling this function.
 *   - Hm_Mode = noLimit : mechanical stall-based homing (no switch needed)
 *   - Hm_Mode = Limited : endstop-switch-based homing
 * For symmetric pairs, each motor must have its own Hm_Dir set
 * (master and slave will be opposite since they are mechanically inverted).
 *
 * After sending, the motor produces two 91H response frames (same DLC=3 format):
 *   [0x91, 0x01, CRC] — immediate ACK: homing started
 *   [0x91, 0x02, CRC] — spontaneous: homing COMPLETE  (sent when done)
 *   [0x91, 0x00, CRC] — failure: motor could not start homing
 *
 * This function is fire-and-forget.  The caller must handle the responses
 * (typically by calling twai_receive in a loop watching for status=2).
 *
 * @param motor_id CAN ID of the target motor.
 * @return ESP_OK on successful frame transmission.
 */
esp_err_t mks_go_home(uint8_t motor_id);

/**
 * @brief Read the non-single-turn zero-return status of a motor (protocol 3BH).
 *
 * Reports the progress of a GoHome (91H) operation.
 * Transmits a 3BH query and waits for the motor's response.
 *
 * @param motor_id   CAN ID of the target motor.
 * @param[out] status  0 = homing in progress, 1 = success, 2 = failed.
 * @param timeout_ms   Timeout in milliseconds to wait for the response frame.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no response, other on TWAI error.
 */
esp_err_t mks_read_home_status(uint8_t motor_id, uint8_t *status, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
