/* dynbus.c: Virtual Dynamixel bus on USART1 implementation.
 *
 * */

#define _GNU_SOURCE
#include "dynbus.h"
#include "avr_uart.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// response byte queue
#define RSP_BUF 128

static uint8_t dyn_checksum(const uint8_t *pkt) {
  // Sum: id (pkt[2]) + length (pkt[3]) + instruction + params (pkt[4..3+len-1])
  // = pkt[2..3+len-1].  Checksum byte sits at pkt[3+len].
  uint8_t len = pkt[3];
  uint8_t checksum = pkt[2]; // id
  for (uint8_t i = 3; i < (uint8_t)(3u + len); i++)
    checksum = (uint8_t)(checksum + pkt[i]);
  return (uint8_t)(~checksum);
}

// Sends queued response bytes now that RXEN1 is enabled. The read index is
// advanced before avr_raise_irq() so a recursive XON call from that same
// injection doesn't resend the byte.
static void flush_response_queue(dynbus_t *bus) {
  while (bus->xon && bus->rsp_r != bus->rsp_w) {
    uint8_t byte = bus->rsp_buf[bus->rsp_r];
    bus->rsp_r = (uint8_t)((bus->rsp_r + 1) & (RSP_BUF - 1));
    avr_raise_irq(bus->uart1_in, byte);
  }
}

static void queue_response(dynbus_t *bus, uint8_t id, uint8_t error,
                           const uint8_t *params, uint8_t param_count) {
  uint8_t len = (uint8_t)(param_count + 2);
  uint8_t checksum = (uint8_t)(id + len + error);
  for (uint8_t i = 0; i < param_count; i++)
    checksum = (uint8_t)(checksum + params[i]);
  checksum = (uint8_t)(~checksum);

  // setup
  // packet: FF FF id len error params... checksum
  uint8_t packet[64];
  uint8_t packet_len = 0;
  packet[packet_len++] = 0xFF;
  packet[packet_len++] = 0xFF;
  packet[packet_len++] = id;
  packet[packet_len++] = len;
  packet[packet_len++] = error;
  for (uint8_t i = 0; i < param_count; i++)
    packet[packet_len++] = params[i];
  packet[packet_len++] = checksum;

  for (uint8_t i = 0; i < packet_len; i++) {
    uint8_t next = (uint8_t)((bus->rsp_w + 1) & (RSP_BUF - 1));
    if (next == bus->rsp_r)
      continue; // queue full: drop the byte
    bus->rsp_buf[bus->rsp_w] = packet[i];
    bus->rsp_w = next;
  }

  // If RXEN is already on, sends the response now instead of waiting for
  // the next XON pulse.
  flush_response_queue(bus);
}

static ax12_servo_t *find_servo(dynbus_t *bus, uint8_t id) {
  for (int i = 0; i < bus->n_servos; i++)
    if (bus->servos[i].ctrl[AX_REG_ID] == id)
      return &bus->servos[i];
  return NULL;
}

// SYNC_WRITE (0x83): the leader writes different data to several servos in
// one packet, with no status response.
// Params are: addr, data_len, [id, data...] repeated once per servo.
static void handle_sync_write(dynbus_t *bus, const uint8_t *params,
                              uint8_t param_count) {
  if (param_count < 2)
    return;
  uint8_t addr = params[0];
  uint8_t data_len = params[1];
  const uint8_t *entry = params + 2;
  uint8_t remaining = (uint8_t)(param_count - 2);

  while (remaining >= data_len + 1) {
    ax12_servo_t *servo = find_servo(bus, entry[0]);
    if (servo) {
      uint8_t copy_len = (addr + data_len <= AX_CTRL_TABLE_SIZE)
                             ? data_len
                             : (uint8_t)(AX_CTRL_TABLE_SIZE - addr);
      memcpy(&servo->ctrl[addr], entry + 1, copy_len);
    }
    entry += data_len + 1;
    remaining -= data_len + 1;
  }
}

// broadcast instructions (id 0xFE) run on every servo and never get a status
// response.
static void handle_broadcast(dynbus_t *bus, uint8_t instruction,
                             const uint8_t *params, uint8_t param_count) {
  for (int i = 0; i < bus->n_servos; i++) {
    uint8_t discard_params[32];
    uint8_t discard_len = 0;
    servo_process(&bus->servos[i], instruction, params, param_count,
                  discard_params, &discard_len);
  }
}

// process a complete, checksummed instruction packet.
static void process_packet(dynbus_t *bus) {
  if (bus->pkt_len < 6)
    return;
  if (bus->pkt[0] != 0xFF || bus->pkt[1] != 0xFF)
    return;

  uint8_t id = bus->pkt[2];
  uint8_t len = bus->pkt[3];
  uint8_t instruction = bus->pkt[4];

  uint8_t expected_checksum = dyn_checksum(bus->pkt);
  uint8_t received_checksum = bus->pkt[(uint8_t)(3 + len)];
  if (expected_checksum != received_checksum) {
    if (bus->verbose)
      fprintf(stderr, "[dynbus] checksum error id=%u exp=%02X got=%02X\n", id,
              expected_checksum, received_checksum);
    return;
  }

  const uint8_t *params = bus->pkt + 5;
  uint8_t param_count = (uint8_t)(len - 2);

  if (bus->verbose)
    fprintf(stderr, "[dynbus] pkt id=%u ins=%02X np=%u\n", id, instruction,
            param_count);

  if (instruction == AX_INS_SYNC_WRITE) {
    handle_sync_write(bus, params, param_count);
    return;
  }

  if (id == DYNBUS_BCAST_ID) {
    handle_broadcast(bus, instruction, params, param_count);
    return;
  }

  ax12_servo_t *servo = find_servo(bus, id);
  if (!servo) {
    if (bus->verbose)
      fprintf(stderr, "[dynbus] no servo id=%u\n", id);
    return;
  }

  uint8_t response_params[32];
  uint8_t response_len = 0;
  int error = servo_process(servo, instruction, params, param_count,
                            response_params, &response_len);
  if (error >= 0)
    queue_response(bus, id, (uint8_t)error, response_params, response_len);
}

/* simavr IRQ hooks */

// UART1 OUTPUT: byte the firmware is transmitting onto the Dynamixel bus.
static void uart1_out_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
  (void)irq;
  dynbus_t *bus = (dynbus_t *)param;
  uint8_t byte = (uint8_t)value;

  if (bus->verbose)
    fprintf(stderr, "[dynbus TX] %02X\n", byte);

  // Sync: ignore bytes until the FF FF header is seen.
  if (bus->pkt_len == 0 && byte != 0xFF)
    return;
  if (bus->pkt_len == 1 && byte != 0xFF) {
    bus->pkt_len = 0;
    return;
  }

  if (bus->pkt_len < DYNBUS_MAX_PKT)
    bus->pkt[bus->pkt_len++] = byte;

  // Complete when pkt_len == LEN+4 (header:2 + id:1 + len:1 + data:LEN)
  if (bus->pkt_len >= 4 && bus->pkt_len >= (uint8_t)(bus->pkt[3] + 4u)) {
    process_packet(bus);
    bus->pkt_len = 0;
  }
}

// UART1 OUT_XON: simavr signals this (continuously) when the UART1 input
// FIFO is not full.
static void uart1_xon_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
  (void)irq;
  dynbus_t *bus = (dynbus_t *)param;
  bus->xon = (value != 0);
  if (bus->verbose)
    fprintf(stderr, "[dynbus XON] xon=%d rsp_pending=%d\n", bus->xon,
            (bus->rsp_w - bus->rsp_r) & (RSP_BUF - 1));
  flush_response_queue(bus);
}

static void uart1_xoff_hook(struct avr_irq_t *irq, uint32_t value,
                            void *param) {
  (void)irq;
  (void)value;
  ((dynbus_t *)param)->xon = 0;
}

/* public API */
void dynbus_attach(dynbus_t *bus, avr_t *avr, int verbose, int ipc_fd) {
  bus->avr = avr;
  bus->verbose = verbose;
  bus->ipc_fd = ipc_fd;
  bus->pkt_len = 0;
  bus->xon = 0;
  bus->rsp_r = 0;
  bus->rsp_w = 0;

  // Hook UART1 OUTPUT (firmware->bus)
  avr_irq_t *base1 = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), 0);
  bus->uart1_in = base1 + UART_IRQ_INPUT;
  avr_irq_register_notify(base1 + UART_IRQ_OUTPUT, uart1_out_hook, bus);
  avr_irq_register_notify(base1 + UART_IRQ_OUT_XON, uart1_xon_hook, bus);
  avr_irq_register_notify(base1 + UART_IRQ_OUT_XOFF, uart1_xoff_hook, bus);
}

void dynbus_add_servo(dynbus_t *bus, uint8_t id) {
  if (bus->n_servos >= DYNBUS_MAX_SERVOS)
    return;
  servo_init(&bus->servos[bus->n_servos++], id);
}

void dynbus_update(dynbus_t *bus, double dt_seconds) {
  for (int i = 0; i < bus->n_servos; i++)
    servo_update(&bus->servos[i], dt_seconds);
}

void dynbus_ipc_publish(dynbus_t *bus) {
  if (bus->ipc_fd < 0)
    return;
  for (int i = 0; i < bus->n_servos; i++) {
    ax12_servo_t *servo = &bus->servos[i];
    char message[160];
    int len = snprintf(message, sizeof message,
                       "{\"type\":\"servo_state\",\"id\":%u,"
                       "\"present_position\":%u,\"goal_position\":%u,"
                       "\"moving\":%s,\"pos_radians\":%.4f}\n",
                       servo->ctrl[AX_REG_ID],
                       servo_reg16(servo, AX_REG_PRESENT_POS_L),
                       servo_reg16(servo, AX_REG_GOAL_POS_L),
                       servo->ctrl[AX_REG_MOVING] ? "true" : "false",
                       servo_pos_radians(servo));
    if (len > 0) {
      ssize_t written = write(bus->ipc_fd, message, (size_t)len);
      (void)written;
    }
  }
}
