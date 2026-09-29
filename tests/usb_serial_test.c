#include <assert.h>
#include <stdio.h>
#include "tusb.h"
#include "usb_serial.h"

static uint32_t now, events[5], reads, controls;
static bool enumerated, accept = true;
static tuh_xfer_cb_t pending;
static uintptr_t token;
static uint32_t available = 512, writes, flushes, clears;
static char transmitted[33];
static bool in_flight;
static bool burst;
static uint32_t rx_bytes;
uint32_t tuh_cdc_write_available(uint8_t idx) { assert(idx == 0); return available; }
uint32_t tuh_cdc_write(uint8_t idx, const void *data, uint32_t len) {
  assert(idx == 0 && len <= 32 && len <= available);
  memcpy(transmitted, data, len); transmitted[len] = 0;
  available -= len;
  ++writes; return len;
}
uint32_t tuh_cdc_write_flush(uint8_t idx) {
  assert(idx == 0);
  ++flushes;
  if (in_flight) return 0;
  uint32_t count = 512 - available;
  if (count) { available = 512; in_flight = true; }
  return count;
}
bool tuh_cdc_write_clear(uint8_t idx) { assert(idx == 0); ++clears; available = 512; return true; }
static void complete_tx(void) {
  assert(in_flight);
  in_flight = false;
  // Same ordering as cdch_xfer_cb: notify before starting queued output.
  tuh_cdc_tx_complete_cb(0);
  tuh_cdc_write_flush(0);
}

uint32_t tusb_time_millis_api(void) { return now; }
bool tuh_mounted(uint8_t address) { assert(address == 1); return enumerated; }
bool tuh_cdc_itf_get_info(uint8_t idx, tuh_itf_info_t *info) {
  assert(idx == 0);
  *info = (tuh_itf_info_t){ .daddr = 1, .desc.bInterfaceNumber = 2 };
  return true;
}
static bool submit(tuh_xfer_cb_t cb, uintptr_t user) {
  ++controls;
  if (!accept) return false;
  assert(!pending && cb);
  pending = cb;
  token = user;
  return true;
}
bool tuh_cdc_set_line_coding(uint8_t idx, const cdc_line_coding_t *coding,
                           tuh_xfer_cb_t cb, uintptr_t user) {
  assert(idx == 0 && coding->bit_rate == 115200 && coding->data_bits == 8);
  assert(coding->parity == 0 && coding->stop_bits == 0);
  return submit(cb, user);
}
bool tuh_cdc_set_control_line_state(uint8_t idx, uint16_t lines,
                                  tuh_xfer_cb_t cb, uintptr_t user) {
  assert(idx == 0 && lines == CDC_CONTROL_LINE_STATE_DTR);
  return submit(cb, user);
}
uint32_t tuh_cdc_read_available(uint8_t idx) { assert(idx == 0); return rx_bytes; }
uint32_t tuh_cdc_read(uint8_t idx, void *buf, uint32_t size) {
  assert(idx == 0 && size == 64);
  if (burst) {
    uint32_t n = rx_bytes < size ? rx_bytes : size;
    rx_bytes -= n;
    memset(buf, 'a', n);
    if (n) ++reads;
    return n;
  }
  memcpy(buf, "SOFT=20103\n", 11);
  ++reads;
  return 11;
}
void scanner_serial_event(uint8_t event, uint8_t addr, uint8_t itf) {
  assert(event < 5 && addr == 1 && itf == 2);
  ++events[event];
}
void scanner_serial_rx(const uint8_t *data, uint32_t size) {
  if (burst) { assert(size == 64); return; }
  assert(size == 11 && memcmp(data, "SOFT=20103\n", 11) == 0);
}
static void complete(tusb_xfer_result_t result) {
  assert(pending);
  tuh_xfer_t xfer = { .daddr = 1, .result = result, .user_data = token };
  tuh_xfer_cb_t cb = pending;
  pending = NULL;
  cb(&xfer);
}

int main(void) {
  const uint8_t speed[] = "SSPD=16777215\n";
  const uint8_t target[] = "DPOS=-33554432\n";
  assert(!usb_serial_tx_idle());
  assert(!usb_serial_send(speed, sizeof(speed)-1, false));
  usb_serial_task();
  assert(!controls && !reads);
  tuh_cdc_mount_cb(0);
  usb_serial_task();
  assert(!controls); // Wait for device enumeration before taking EP0.
  enumerated = true;
  accept = false;
  usb_serial_task(); // Busy EP0: retry later, never block.
  assert(!pending && controls == 1);
  accept = true;
  usb_serial_task();
  usb_serial_task();
  assert(controls == 2 && !reads);
  complete(XFER_RESULT_SUCCESS);
  usb_serial_task();
  complete(XFER_RESULT_SUCCESS);
  assert(events[1] == 1);
  assert(usb_serial_tx_idle());
  assert(usb_serial_send(speed, sizeof(speed)-1, false));
  assert(writes == 1 && flushes == 1 && strcmp(transmitted, "SSPD=16777215\n") == 0);
  assert(available == 512 && !usb_serial_tx_idle()); // FIFO empty, USB still busy
  tuh_cdc_tx_complete_cb(1); // another interface cannot release our transfer
  assert(!usb_serial_tx_idle());
  assert(usb_serial_send(target, sizeof(target)-1, false));
  assert(writes == 2 && flushes == 2 && strcmp(transmitted, "DPOS=-33554432\n") == 0);
  complete_tx(); // SSPD done; DPOS starts only now
  assert(!usb_serial_tx_idle());
  complete_tx();
  assert(usb_serial_tx_idle());
  available = sizeof(target)-2;
  assert(!usb_serial_tx_idle());
  assert(!usb_serial_send(target, sizeof(target)-1, false));
  assert(writes == 2); // No partial line on backpressure.
  assert(usb_serial_send((const uint8_t *)"STOP\n", 5, true));
  assert(clears == 1 && writes == 3 && strcmp(transmitted, "STOP\n") == 0);
  assert(!usb_serial_tx_idle());
  complete_tx();
  assert(usb_serial_tx_idle());

  // No completion (NAK/stall) must never advertise idle, regardless of time or
  // an empty FIFO. STOP replaces waiting bytes, but cannot recall the packet.
  assert(usb_serial_send(target, sizeof(target)-1, false));
  assert(available == 512 && !usb_serial_tx_idle());
  now += 1000;
  usb_serial_task();
  assert(!usb_serial_tx_idle());
  assert(usb_serial_send(speed, sizeof(speed)-1, false));
  assert(usb_serial_send((const uint8_t *)"STOP\n", 5, true));
  assert(!usb_serial_tx_idle());
  complete_tx(); // outstanding DPOS completes, queued STOP starts
  assert(!usb_serial_tx_idle());
  complete_tx();
  assert(usb_serial_tx_idle());
  assert(!usb_serial_send(speed, 33, false));
  usb_serial_task();
  assert(reads == 2);
  tuh_cdc_umount_cb(0);
  assert(!usb_serial_send(speed, sizeof(speed)-1, false));
  usb_serial_task();
  assert(reads == 2 && events[2] == 1);
  tuh_cdc_tx_complete_cb(0); // late notification cannot reopen disconnected TX
  assert(!usb_serial_tx_idle());

  tuh_cdc_mount_cb(0);
  usb_serial_task();
  complete(XFER_RESULT_STALLED);
  usb_serial_task();
  assert(events[3] == 1 && reads == 2);
  tuh_cdc_umount_cb(0);

  now = UINT32_MAX - 1000;
  tuh_cdc_mount_cb(0);
  usb_serial_task();
  now += 3000; // Timeout must work across tick rollover.
  usb_serial_task();
  assert(events[4] == 1);
  complete(XFER_RESULT_SUCCESS); // Late completion must not reopen the port.
  usb_serial_task();
  assert(events[1] == 1 && reads == 2);
  tuh_cdc_umount_cb(0);

  tuh_cdc_mount_cb(0);
  usb_serial_task();
  tuh_cdc_umount_cb(0);
  tuh_cdc_mount_cb(0);
  complete(XFER_RESULT_SUCCESS); // Ignore completion from the old connection.
  usb_serial_task();
  complete(XFER_RESULT_SUCCESS);
  usb_serial_task();
  complete(XFER_RESULT_SUCCESS);
  assert(events[1] == 2);
  burst = true;
  rx_bytes = 576;
  now = UINT32_MAX - 4;
  usb_serial_timing(true);
  now = 2;
  const uint32_t baseline_reads = reads;
  usb_serial_task();
  assert(reads == baseline_reads + 4 && rx_bytes == 320 && usb_serial_rx_pending());
  usb_serial_task();
  assert(reads == baseline_reads + 8 && rx_bytes == 64);
  usb_serial_task();
  assert(reads == baseline_reads + 9 && !usb_serial_rx_pending());
  assert(usb_serial_send(speed, sizeof(speed)-1, false));
  now = 12;
  complete_tx();
  uint32_t peak, backlog, gap, tx;
  usb_serial_stats(&peak, &backlog, &gap, &tx);
  assert(peak == 576 && backlog == 2 && gap == 7 && tx == 10);
  uint32_t progress[6];
  usb_serial_progress(progress);
  assert(progress[0] == 1 && progress[1] == 1);
  assert(progress[3] == 2 && progress[5] == 12);
  // Ordered zero/direction packets have distinct completion observations.
  now = UINT32_MAX;
  assert(usb_serial_send((const uint8_t *)"SSPD=0\n", 7, false));
  assert(usb_serial_send((const uint8_t *)"SCAN=-1\n", 8, false));
  usb_serial_progress(progress);
  assert(progress[0] == 3 && progress[1] == 1);
  assert(progress[2] == UINT32_MAX && progress[3] == UINT32_MAX);
  now = 0;
  complete_tx();
  assert(!usb_serial_tx_idle());
  now = 1;
  complete_tx();
  usb_serial_progress(progress);
  assert(progress[1] == 3 && progress[4] == 0 && progress[5] == 1);
  assert(usb_serial_tx_idle());
  usb_serial_timing(false);
  assert(usb_serial_send((const uint8_t *)"STOP\n", 5, true));
  now = 2;
  complete_tx();
  usb_serial_progress(progress);
  assert(progress[0] == 3 && progress[1] == 3 && progress[5] == 1);
  puts("USB serial transport tests passed");
}
