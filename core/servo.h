/* servo.h: Virtual AX-12A Dynamixel servo model.
 *
 * Implements Dynamixel Protocol 1.0; 50-register control table.
 * present_position tracks goal_position at
 * goal_speed ticks/second
 * servo_update() advances it each run-loop tick.
 *
 * */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// AX-12A control table register addresses (Protocol 1.0).
// reference:
// https://docs.robotis.com/docs/dxl/model_reference/ax_series/ax-12a/
#define AX_REG_MODEL_L 0
#define AX_REG_MODEL_H 1
#define AX_REG_FIRMWARE 2
#define AX_REG_ID 3
#define AX_REG_BAUD_RATE 4
#define AX_REG_RETURN_DELAY 5
#define AX_REG_CW_LIMIT_L 6
#define AX_REG_CW_LIMIT_H 7
#define AX_REG_CCW_LIMIT_L 8
#define AX_REG_CCW_LIMIT_H 9
#define AX_REG_LIMIT_TEMP 11
#define AX_REG_LOW_VOLTAGE 12
#define AX_REG_HIGH_VOLTAGE 13
#define AX_REG_MAX_TORQUE_L 14
#define AX_REG_MAX_TORQUE_H 15
#define AX_REG_RETURN_LEVEL 16
#define AX_REG_ALARM_LED 17
#define AX_REG_ALARM_SHUTDOWN 18
// RAM area
#define AX_REG_TORQUE_ENABLE 24
#define AX_REG_LED 25
#define AX_REG_CW_MARGIN 26
#define AX_REG_CCW_MARGIN 27
#define AX_REG_CW_SLOPE 28
#define AX_REG_CCW_SLOPE 29
#define AX_REG_GOAL_POS_L 30
#define AX_REG_GOAL_POS_H 31
#define AX_REG_GOAL_SPEED_L 32
#define AX_REG_GOAL_SPEED_H 33
#define AX_REG_TORQUE_LIMIT_L 34
#define AX_REG_TORQUE_LIMIT_H 35
#define AX_REG_PRESENT_POS_L 36
#define AX_REG_PRESENT_POS_H 37
#define AX_REG_PRESENT_SPEED_L 38
#define AX_REG_PRESENT_SPEED_H 39
#define AX_REG_PRESENT_LOAD_L 40
#define AX_REG_PRESENT_LOAD_H 41
#define AX_REG_PRESENT_VOLTAGE 42
#define AX_REG_PRESENT_TEMP 43
#define AX_REG_REGISTERED_INS 44
#define AX_REG_MOVING 46
#define AX_REG_LOCK 47
#define AX_REG_PUNCH_L 48
#define AX_REG_PUNCH_H 49
#define AX_CTRL_TABLE_SIZE 50

// Dynamixel 1.0 instruction codes.
// reference:
// https://docs.robotis.com/docs/dxl/protocol/protocol1/
#define AX_INS_PING 0x01
#define AX_INS_READ 0x02
#define AX_INS_WRITE 0x03
#define AX_INS_REG_WRITE 0x04
#define AX_INS_ACTION 0x05
#define AX_INS_RESET 0x06
#define AX_INS_SYNC_WRITE 0x83

// Error bits returned in status packets.
#define AX_ERR_NONE 0x00
#define AX_ERR_VOLTAGE 0x01
#define AX_ERR_ANGLE_LIMIT 0x02
#define AX_ERR_OVERHEATING 0x04
#define AX_ERR_RANGE 0x08
#define AX_ERR_CHECKSUM 0x10
#define AX_ERR_OVERLOAD 0x20
#define AX_ERR_INSTRUCTION 0x40

// AX-12A hardware constants.
#define AX12_MODEL 12
#define AX12_FIRMWARE 24
#define AX12_POS_MIN 0
#define AX12_POS_MAX 1023
#define AX12_POS_CENTER 512
#define AX12_SPEED_DEFAULT 50 // normal speed

typedef struct {
  uint8_t ctrl[AX_CTRL_TABLE_SIZE]; // control table
  uint8_t pending_pos_l;            // REG_WRITE buffered position L
  uint8_t pending_pos_h;            // REG_WRITE buffered position H
  uint8_t has_pending;              // 1 if REG_WRITE is waiting on ACTION
  double pos_exact;                 // present position (fractional)
} ax12_servo_t;

// Initialize a servo with the given ID and center position.
void servo_init(ax12_servo_t *servo, uint8_t id);

// Advance motion by dt_seconds toward goal position.
void servo_update(ax12_servo_t *servo, double dt_seconds);

// Processes one Dynamixel instruction packet directed at this servo.
int servo_process(ax12_servo_t *servo, uint8_t instruction,
                  const uint8_t *in_params, uint8_t in_np, uint8_t *out_params,
                  uint8_t *out_np_ptr);


/* helper functions */
// present position in radians (0 = center = 150°).
double servo_pos_radians(const ax12_servo_t *servo);

// set goal position from radians.
void servo_set_goal_radians(ax12_servo_t *servo, double radians);

// reg helpers
static inline uint16_t servo_reg16(const ax12_servo_t *servo, uint8_t addr) {
  return (uint16_t)servo->ctrl[addr] | ((uint16_t)servo->ctrl[addr + 1] << 8);
}

static inline void servo_setreg16(ax12_servo_t *servo, uint8_t addr,
                                  uint16_t value) {
  servo->ctrl[addr] = (uint8_t)(value & 0xFF);
  servo->ctrl[addr + 1] = (uint8_t)(value >> 8);
}

#ifdef __cplusplus
}
#endif
