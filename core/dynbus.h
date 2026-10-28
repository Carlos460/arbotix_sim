/* dynbus.h: Virtual Dynamixel bus on USART1.
 */

#pragma once
#include "servo.h"
#include "sim_avr.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYNBUS_MAX_SERVOS 24
#define DYNBUS_MAX_PKT 64
#define DYNBUS_BCAST_ID 0xFE // broadcast to all servos

typedef struct {
  // Servos keyed by ID
  ax12_servo_t servos[DYNBUS_MAX_SERVOS];
  int n_servos;

  // Instruction packet accumulation
  uint8_t pkt[DYNBUS_MAX_PKT];
  uint8_t pkt_len;

  // Response byte ring buffer, flushed into UART1 in XON hook.
  uint8_t rsp_buf[128];
  uint8_t rsp_r; // read index
  uint8_t rsp_w; // write index
  int xon;       // 1 = UART1 RXEN is active, safe to inject

  avr_t *avr;
  avr_irq_t *uart1_in; // UART1 INPUT IRQ to inject response bytes

  int verbose;
  int ipc_fd; // unix socket fd (-1 if not connected)
} dynbus_t;

// attach the bus controller to avr. called after avr_init().
void dynbus_attach(dynbus_t *bus, avr_t *avr, int verbose, int ipc_fd);

// register a servo on the bus.
void dynbus_add_servo(dynbus_t *bus, uint8_t id);

// advance all servo motion models. call periodically from the run loop.
// dt_seconds: elapsed wallclock time since last call.
void dynbus_update(dynbus_t *bus, double dt_seconds);

// Serialise all servo states to the IPC fd as newline-delimited JSON.
void dynbus_ipc_publish(dynbus_t *bus);

#ifdef __cplusplus
}
#endif
