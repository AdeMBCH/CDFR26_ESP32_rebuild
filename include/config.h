/**
 * @file config.h
 * @brief Compile-time configuration for the microROS OTOS node
 */
#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

// ---------------------------------------------------------------------------
// LED (WS2812)
// ---------------------------------------------------------------------------
#define LED_RGB_PIN             GPIO_NUM_48
#define LED_STRIP_RMT_RES_HZ    (10 * 1000 * 1000)  // 10 MHz RMT resolution

// ---------------------------------------------------------------------------
// OTOS sensor (I2C)
// ---------------------------------------------------------------------------
#define OTOS_I2C_PORT           I2C_NUM_0
#define OTOS_SDA_PIN            GPIO_NUM_21
#define OTOS_SCL_PIN            GPIO_NUM_47
#define OTOS_I2C_FREQ           400000   // 400 kHz

// ---------------------------------------------------------------------------
// microROS / ROS 2
// ---------------------------------------------------------------------------
#define ROS_DOMAIN_ID           80        // Must match ROS_DOMAIN_ID on host
#define PUBLISH_FREQUENCY_HZ    50       // Odometry publishing rate
#define TIMER_PERIOD_MS         (1000 / PUBLISH_FREQUENCY_HZ)

// ---------------------------------------------------------------------------
// microROS serial transport
// ---------------------------------------------------------------------------
// The ESP32-S3 talks to the host over the native USB Serial/JTAG device.
// Keep the host-side agent at 921600 so launch files and tooling stay aligned.
#define UROS_SERIAL_BAUD        921600

// ---------------------------------------------------------------------------
// TWAI (CAN Bus) — TJA1050 transceiver
// ---------------------------------------------------------------------------
#define TWAI_TX_PIN             GPIO_NUM_17  // ESP32 TX → TJA1050 TXD
#define TWAI_RX_PIN             GPIO_NUM_18  // ESP32 RX ← TJA1050 RXD

// ---------------------------------------------------------------------------
// MKS SERVO42D/57D — CAN motor IDs (1-16)
// ---------------------------------------------------------------------------
// 4 omni drive motors
#define MOTOR_OMNI_FL           2   // Front-Left
#define MOTOR_OMNI_FR           1   // Front-Right
#define MOTOR_OMNI_RL           3   // Rear-Left
#define MOTOR_OMNI_RR           4   // Rear-Right
// Gripper lift (symmetric pair): slave mirrors master with inverted direction
#define MOTOR_GRIPPER_MASTER    5
#define MOTOR_GRIPPER_SLAVE     6
// Storage module lift (symmetric pair)
#define MOTOR_STORAGE_MASTER    7
#define MOTOR_STORAGE_SLAVE     8

// Acceleration: omni wheels use 0 (instant) for responsive driving;
// mechanisms use a gentle ramp to protect the mechanics.
#define MKS_ACC_OMNI            0    // Instant — no ramp
#define MKS_ACC_MECHANISM       5   // Gentle ramp for gripper / storage
// Safety cap — MKS hardware max is 3000 RPM in vFOC mode
#define MKS_MAX_RPM             3000
// Travel speed for position-control moves (F5H).  Lower values are safer
// for mechanisms; raise if moves are too slow.
#define MKS_POSITION_SPEED_RPM  300
// Set to 1: ESP32 automatically sends inverted command to slave motors.
// Set to 0: ROS2 controller sends explicit commands for every motor.
#define MOTOR_AUTO_MIRROR       1

// ---------------------------------------------------------------------------
// Mechanism homing parameters
// ---------------------------------------------------------------------------
// Homing uses the MKS native GoHome command (91H) + zero-return status (3BH).
// The direction, speed, and mode are configured ONCE in each motor's on-board
// menu (or via 9AH over CAN) — not driven by firmware at runtime:
//
//   Hm_Mode  noLimit → mechanical stall (no endstop switch required)
//            Limited → endstop switch wired to IN_1
//   Hm_Speed 60 RPM (recommended: slow enough to avoid damage)
//   Hm_Dir   Master: direction toward endstop
//            Slave:  opposite of master (mechanically inverted pair)
//
// HOMING_TIMEOUT_MS: how long the ESP32 waits for both motors to report
// success (3BH status2 = 1) before giving up and sending an emergency stop.
#define HOMING_TIMEOUT_MS       15000  // Maximum time per mechanism (ms)

// ---------------------------------------------------------------------------
// PWM Servo — storage open/close
// ---------------------------------------------------------------------------
// Two servos wired in parallel on the same GPIO, mechanically aligned.
// Travel: 25° + 6° offset = 31° total (Arduino Servo lib: 0°=544µs, 180°=2400µs)
//   open  ≈ 55° → 1111 µs
//   close ≈ 86° → 1431 µs
#define SERVO_GPIO_PIN          GPIO_NUM_10  // PWM signal pin (both servos)
#define SERVO_FREQ_HZ           50           // 50 Hz = 20 ms period
#define SERVO_TIMER_RESOLUTION  14           // LEDC 14-bit (LEDC_TIMER_14_BIT)
#define SERVO_OPEN_US           1111         // Pulse for open  position (~55°)
#define SERVO_CLOSE_US          1431         // Pulse for closed position (~86°)

// ---------------------------------------------------------------------------
// Mechanism absolute positions — encoder units (0x4000 = 1 rev)
// TODO: measure actual values on the robot after homing
// ---------------------------------------------------------------------------
#define GRIPPER_POS_UP          0        // TODO: position haute de la pince
#define GRIPPER_POS_DOWN        20000     // TODO: position basse  de la pince
#define STORAGE_POS_UP          0        // TODO: position haute du storage
#define STORAGE_POS_DOWN        20000     // TODO: position basse  du storage

// ---------------------------------------------------------------------------
// Phase node — position move timeout
// ---------------------------------------------------------------------------
// Maximum time phase_bg_task waits for F5H completion from both motors.
// Covers the full travel from one end-stop to the other at MKS_POSITION_SPEED_RPM.
#define PHASE_TIMEOUT_MS        10000  // 10 s per move (ms)

// ---------------------------------------------------------------------------
// Motor omni-wheel periodic CAN send
// ---------------------------------------------------------------------------
/** Rate at which the ESP32 re-sends the last omni-wheel speed targets over CAN.
 *  Decouples the ROS2 publish rate from the CAN send rate.
 *  The subscriber only updates in-memory targets; this timer drives the bus. */
#define MOTOR_OMNI_PERIOD_MS    20   // 50 Hz

// ---------------------------------------------------------------------------
// FreeRTOS tasks
// ---------------------------------------------------------------------------
#define LED_TASK_STACK_SIZE     2048
#define LED_TASK_PRIORITY       3

#define UROS_TASK_STACK_SIZE    16384
#define UROS_TASK_PRIORITY      5
