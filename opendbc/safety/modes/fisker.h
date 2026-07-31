#pragma once

#include "opendbc/safety/declarations.h"

// Fisker Ocean. openpilot impersonates the ADAS/Hydra module: it transmits the lateral takeover
// frames on bus 0 (the car bus) while the stock module is isolated on bus 2 behind the intercept
// relay. See opendbc/car/fisker/. Lateral is a torque command; longitudinal is off by default.
//
//   0x1D0 ADAS_STEER_CONTROL     TX  SecOC-signed steering torque command
//   0x1C0 LKAS_STEER_AUTHORITY   TX  non-SecOC lateral-control validity frame
//   0x121 ADAS_ACCEL_CONTROL     TX  SecOC-signed accel command (only with the long flag)
//
// The SecOC MAC/freshness bytes are opaque to the safety layer -- it constrains the commanded
// torque/accel, not the authentication (openpilot signs with the per-vehicle key it recovers).

#define FISKER_STEER_ADDR      0x1D0U
#define FISKER_AUTHORITY_ADDR  0x1C0U
#define FISKER_ACCEL_ADDR      0x121U

// LKAS_STEERING_TORQUE has a DBC offset of -192, so the on-wire 12-bit value is (torque + 192).
#define FISKER_STEER_OFFSET    192
// ADAS_ACCEL_CONTROL.PAYLOAD placeholder scaling: neutral 2048, 300 payload units per m/s^2.
#define FISKER_ACCEL_NEUTRAL   2048
#define FISKER_ACCEL_PER_MS2   300

static bool fisker_longitudinal = false;

static void fisker_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // vehicle speed from ESP_WHEEL_SPEED (0x115): two 12-bit wheels scaled 0.075 km/h/unit
    if (msg->addr == 0x115U) {
      int ws1 = ((msg->data[2] & 0x0FU) << 8) | msg->data[3];
      int ws2 = ((msg->data[5] & 0x0FU) << 8) | msg->data[6];
      vehicle_moving = (ws1 != 0) || (ws2 != 0);
      UPDATE_VEHICLE_SPEED(((ws1 + ws2) / 2.0f) * 0.075f * KPH_TO_MS);
    }

    // driver input torque from STEERING_WHEEL1 (0x1C4): 12-bit, treated as signed about 0
    if (msg->addr == 0x1C4U) {
      int torque_driver_new = ((msg->data[2] << 4) | (msg->data[3] >> 4)) & 0xFFF;
      torque_driver_new = to_signed(torque_driver_new, 12);
      update_sample(&torque_driver, torque_driver_new);
    }

    // stock ACC engagement from CRUISE_CONTROL_STATUS (0x358): 2 = ACTIVE (speed control engaged)
    if (msg->addr == 0x358U) {
      bool cruise_engaged = (((msg->data[4] >> 4) & 0x3U) == 2U);
      pcm_cruise_check(cruise_engaged);
    }

    // brake from BCM (0x214): BRAKE_PRESSED_1
    if (msg->addr == 0x214U) {
      brake_pressed = GET_BIT(msg, 14U);
    }

    // gas from ACCELERATOR_PEDAL (0x1BA): ACCELERATOR_PEDAL_POSITION_ABSOLUTE
    if (msg->addr == 0x1BAU) {
      gas_pressed = msg->data[2] != 0U;
    }
  }
}

static bool fisker_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits FISKER_STEERING_LIMITS = {
    .max_torque = 192,        // LKAS_STEERING_TORQUE physical range [-192, 192]
    .max_rate_up = 10,        // from values.py CarControllerParams (provisional)
    .max_rate_down = 25,
    .max_rt_delta = 150,      // 250 ms window bound (provisional, below the 192 absolute cap)
    .type = TorqueDriverLimited,
    .driver_torque_allowance = 100,   // provisional, pending driver-torque calibration
    .driver_torque_multiplier = 2,
  };

  const LongitudinalLimits FISKER_LONG_LIMITS = {
    .max_accel = 2000,        // 2.0 m/s^2 in 0.001 m/s^2 units
    .min_accel = -3500,       // -3.5 m/s^2
    .inactive_accel = 0,
  };

  bool tx = true;

  if (msg->bus == 0U) {
    // ADAS_STEER_CONTROL: torque limited
    if (msg->addr == FISKER_STEER_ADDR) {
      int desired_torque = (((msg->data[2] & 0x0FU) << 8) | msg->data[3]) - FISKER_STEER_OFFSET;
      // UNKNOWN_CONSTANT (byte 2 high nibble) is 3 while requesting steer, 1 otherwise
      bool steer_req = (((msg->data[2] >> 4) & 0x0FU) == 3U);
      if (steer_torque_cmd_checks(desired_torque, steer_req, FISKER_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // ADAS_ACCEL_CONTROL: only with openpilot longitudinal; convert PAYLOAD to 0.001 m/s^2
    if (msg->addr == FISKER_ACCEL_ADDR) {
      int payload = ((msg->data[2] & 0x0FU) << 8) | msg->data[3];
      int desired_accel = ((payload - FISKER_ACCEL_NEUTRAL) * 1000) / FISKER_ACCEL_PER_MS2;
      if (!fisker_longitudinal || longitudinal_accel_checks(desired_accel, FISKER_LONG_LIMITS)) {
        tx = false;
      }
    }

    // LKAS_STEER_AUTHORITY (0x1C0) carries no actuation and is allowed unconditionally.
  }

  return tx;
}

static safety_config fisker_init(uint16_t param) {
  // FiskerSafetyFlags in opendbc/car/fisker/values.py
  const uint16_t FISKER_FLAG_LONGITUDINAL = 2U;

  static const CanMsg FISKER_TX_MSGS[] = {
    {FISKER_STEER_ADDR, 0, 8, .check_relay = true},
    {FISKER_AUTHORITY_ADDR, 0, 8, .check_relay = true},
  };

  static const CanMsg FISKER_LONG_TX_MSGS[] = {
    {FISKER_STEER_ADDR, 0, 8, .check_relay = true},
    {FISKER_AUTHORITY_ADDR, 0, 8, .check_relay = true},
    {FISKER_ACCEL_ADDR, 0, 8, .check_relay = true},
  };

  // No checksum/counter/quality signals decoded for these state messages; rely on presence/frequency.
  static RxCheck fisker_rx_checks[] = {
    {.msg = {{0x115, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ESP_WHEEL_SPEED
    {.msg = {{0x1C4, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEERING_WHEEL1
    {.msg = {{0x214, 0, 16, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // BCM
    {.msg = {{0x358, 0, 8,  20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // CRUISE_CONTROL_STATUS
    {.msg = {{0x1BA, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ACCELERATOR_PEDAL
  };

  fisker_longitudinal = GET_FLAG(param, FISKER_FLAG_LONGITUDINAL);

  return fisker_longitudinal ? BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_LONG_TX_MSGS) :
                               BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_TX_MSGS);
}

const safety_hooks fisker_hooks = {
  .init = fisker_init,
  .rx = fisker_rx_hook,
  .tx = fisker_tx_hook,
};
