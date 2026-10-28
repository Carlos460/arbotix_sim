/* servo.c: Virtual AX-12A servo model.
 *
 * */

#include "servo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// AX-12A: 0..1023 ticks = 0-300 degrees; radians are relative to center
// (tick 512 = 0 rad).
#define TICKS_PER_RAD (1023.0 / (300.0 * M_PI / 180.0)) // ~195.38 ticks/rad
#define RAD_PER_TICK (1.0 / TICKS_PER_RAD)

// AX-12A max speed: ~59 rpm = ~354°/s = ~1201 ticks/s.
#define MAX_TICKS_PER_SEC 1200.0
// Speed unit: 0.111 rpm = 0.111 * 1023 ticks/rev * (1rev/60s) * (360°/300°)
#define SPEED_UNIT_TICKS_PER_SEC (0.111 * 60.0 * 1023.0 / 300.0) // ~13.5

void servo_init(ax12_servo_t *servo, uint8_t id) {
  memset(servo, 0, sizeof *servo);
  servo->ctrl[AX_REG_MODEL_L] = (uint8_t)(AX12_MODEL & 0xFF);
  servo->ctrl[AX_REG_MODEL_H] = (uint8_t)(AX12_MODEL >> 8);
  servo->ctrl[AX_REG_FIRMWARE] = AX12_FIRMWARE;
  servo->ctrl[AX_REG_ID] = id;
  servo->ctrl[AX_REG_BAUD_RATE] = 1; // 1 Mbps encoding
  servo->ctrl[AX_REG_RETURN_DELAY] = 250;
  servo->ctrl[AX_REG_CCW_LIMIT_H] = (uint8_t)(AX12_POS_MAX >> 8);
  servo->ctrl[AX_REG_CCW_LIMIT_L] = (uint8_t)(AX12_POS_MAX & 0xFF);
  servo->ctrl[AX_REG_LIMIT_TEMP] = 70;
  servo->ctrl[AX_REG_LOW_VOLTAGE] = 60;
  servo->ctrl[AX_REG_HIGH_VOLTAGE] = 140;
  servo->ctrl[AX_REG_MAX_TORQUE_L] = 0xFF;
  servo->ctrl[AX_REG_MAX_TORQUE_H] = 0x03;
  servo->ctrl[AX_REG_RETURN_LEVEL] = 2; // respond to all packets
  servo->ctrl[AX_REG_TORQUE_ENABLE] = 0;
  servo->ctrl[AX_REG_CW_SLOPE] = 32;
  servo->ctrl[AX_REG_CCW_SLOPE] = 32;
  servo->ctrl[AX_REG_TORQUE_LIMIT_L] = 0xFF;
  servo->ctrl[AX_REG_TORQUE_LIMIT_H] = 0x03;
  servo->ctrl[AX_REG_PRESENT_VOLTAGE] = 120; // 12.0 V
  servo->ctrl[AX_REG_PRESENT_TEMP] = 35;
  servo->ctrl[AX_REG_PUNCH_L] = 32;
  servo_setreg16(servo, AX_REG_GOAL_POS_L, AX12_POS_CENTER);
  servo_setreg16(servo, AX_REG_PRESENT_POS_L, AX12_POS_CENTER);
  servo_setreg16(servo, AX_REG_GOAL_SPEED_L, AX12_SPEED_DEFAULT);
  servo->pos_exact = AX12_POS_CENTER;
}

void servo_update(ax12_servo_t *servo, double dt_seconds) {
  uint16_t goal = servo_reg16(servo, AX_REG_GOAL_POS_L);
  uint16_t speed = servo_reg16(servo, AX_REG_GOAL_SPEED_L);
  if (speed == 0)
    speed = 1023; // 0 = max speed

  double ticks_per_sec = speed * SPEED_UNIT_TICKS_PER_SEC;
  if (ticks_per_sec > MAX_TICKS_PER_SEC)
    ticks_per_sec = MAX_TICKS_PER_SEC;

  double max_step = ticks_per_sec * dt_seconds;
  double remaining = (double)goal - servo->pos_exact;
  if (fabs(remaining) <= max_step)
    servo->pos_exact = goal;
  else
    servo->pos_exact += (remaining > 0 ? max_step : -max_step);

  uint16_t pos = (uint16_t)(servo->pos_exact + 0.5);
  if (pos > AX12_POS_MAX)
    pos = AX12_POS_MAX;
  servo_setreg16(servo, AX_REG_PRESENT_POS_L, pos);

  int moving = (pos != goal);
  servo->ctrl[AX_REG_MOVING] = (uint8_t)moving;
  // Approximate present speed proportional to how fast we're moving.
  servo_setreg16(servo, AX_REG_PRESENT_SPEED_L, moving ? speed : 0);
}

int servo_process(ax12_servo_t *servo, uint8_t instruction,
                  const uint8_t *in_params, uint8_t in_np, uint8_t *out_params,
                  uint8_t *out_np_ptr) {
  uint8_t return_level = servo->ctrl[AX_REG_RETURN_LEVEL];
  *out_np_ptr = 0;

  switch (instruction) {
  case AX_INS_PING:
    // Always respond to PING regardless of return_level.
    return AX_ERR_NONE;

  case AX_INS_READ: {
    if (in_np < 2)
      return AX_ERR_INSTRUCTION;
    uint8_t addr = in_params[0];
    uint8_t len = in_params[1];
    if (addr + len > AX_CTRL_TABLE_SIZE)
      return AX_ERR_RANGE;
    for (uint8_t i = 0; i < len; i++)
      out_params[i] = servo->ctrl[addr + i];
    *out_np_ptr = len;

    if (return_level == 0)
      return -1;
    return AX_ERR_NONE;
  }

  case AX_INS_WRITE: {
    if (in_np < 2)
      return AX_ERR_INSTRUCTION;
    uint8_t addr = in_params[0];
    uint8_t len = (uint8_t)(in_np - 1);
    if (addr + len > AX_CTRL_TABLE_SIZE)
      return AX_ERR_RANGE;
    for (uint8_t i = 0; i < len; i++)
      servo->ctrl[addr + i] = in_params[1 + i];

    // Clamp goal position in case this write touched it.
    uint16_t goal_pos = servo_reg16(servo, AX_REG_GOAL_POS_L);
    if (goal_pos > AX12_POS_MAX)
      servo_setreg16(servo, AX_REG_GOAL_POS_L, AX12_POS_MAX);

    if (return_level < 2)
      return -1;
    return AX_ERR_NONE;
  }

  case AX_INS_REG_WRITE: {
    if (in_np < 2)
      return AX_ERR_INSTRUCTION;
    servo->pending_pos_l = in_params[1];
    servo->pending_pos_h = (in_np > 2) ? in_params[2] : 0;
    servo->has_pending = 1;
    servo->ctrl[AX_REG_REGISTERED_INS] = 1;
    if (return_level < 2)
      return -1;
    return AX_ERR_NONE;
  }

  case AX_INS_ACTION:
    if (servo->has_pending) {
      servo->ctrl[AX_REG_GOAL_POS_L] = servo->pending_pos_l;
      servo->ctrl[AX_REG_GOAL_POS_H] = servo->pending_pos_h;
      servo->has_pending = 0;
      servo->ctrl[AX_REG_REGISTERED_INS] = 0;
    }
    if (return_level < 2)
      return -1;
    return AX_ERR_NONE;

  case AX_INS_RESET:
    servo_init(servo, servo->ctrl[AX_REG_ID]);
    if (return_level < 2)
      return -1;
    return AX_ERR_NONE;

  default:
    return AX_ERR_INSTRUCTION;
  }
}

double servo_pos_radians(const ax12_servo_t *servo) {
  uint16_t tick = servo_reg16(servo, AX_REG_PRESENT_POS_L);
  return ((double)tick - AX12_POS_CENTER) * RAD_PER_TICK;
}

void servo_set_goal_radians(ax12_servo_t *servo, double radians) {
  double tick = radians * TICKS_PER_RAD + AX12_POS_CENTER;
  if (tick < AX12_POS_MIN)
    tick = AX12_POS_MIN;
  if (tick > AX12_POS_MAX)
    tick = AX12_POS_MAX;
  servo_setreg16(servo, AX_REG_GOAL_POS_L, (uint16_t)(tick + 0.5));
}
