/* simulator.h: public interface to the ArbotiX-M board simulator core.
 *
 * Owns the emulated MCU, host serial bridge, virtual Dynamixel bus, and IPC
 * socket
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Largest number of servos the bus can hold. Must match DYNBUS_MAX_SERVOS
// in dynbus.h.
#define SIM_MAX_SERVOS 24

typedef struct {
  const char *firmware_path; // AVR ELF to load (required)
  const char *link_path;     // stable symlink to the host serial PTY
  const char *ipc_path;      // unix socket for servo state IPC
  const uint8_t *servo_ids;  // servo IDs to create on the bus
  int servo_count;
  int verbose; // hexdump UART and Dynamixel-bus traffic
} sim_config_t;

typedef struct {
  uint8_t id;
  uint16_t present_position; // ticks, 0-1023
  uint16_t goal_position;    // ticks, 0-1023
  double present_radians;
  int moving;
} sim_servo_snapshot_t;

typedef struct {
  char mcu_name[64];
  int servo_count;
  sim_servo_snapshot_t servos[SIM_MAX_SERVOS];
  int ipc_client_connected;
} sim_snapshot_t;

// Loads the firmware and runs.
int sim_run(const sim_config_t *config);

// Asks the simulator to stop.
void sim_request_stop(void);

// Copies the latest published state into *out.
void sim_get_snapshot(sim_snapshot_t *out);

// Requests a new goal position (0-1023 ticks) for the given servo ID.
void sim_set_goal_position(uint8_t servo_id, uint16_t position);

#ifdef __cplusplus
}
#endif
