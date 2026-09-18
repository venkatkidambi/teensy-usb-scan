#include <assert.h>
#include <stdio.h>
#include "tusb.h"
#include "usb_serial.h"

static uint32_t now, events[5], reads, controls;
static bool enumerated, accept = true;
static tuh_xfer_cb_t pending;
static uintptr_t token;

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
uint32_t tuh_cdc_read(uint8_t idx, void *buf, uint32_t size) {
  assert(idx == 0 && size == 64);
  memcpy(buf, "SOFT=20103\n", 11);
  ++reads;
  return 11;
}
void scanner_serial_event(uint8_t event, uint8_t addr, uint8_t itf) {
  assert(event < 5 && addr == 1 && itf == 2);
  ++events[event];
}
void scanner_serial_rx(const uint8_t *data, uint32_t size) {
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
  usb_serial_task();
  assert(reads == 1);
  tuh_cdc_umount_cb(0);
  usb_serial_task();
  assert(reads == 1 && events[2] == 1);

  tuh_cdc_mount_cb(0);
  usb_serial_task();
  complete(XFER_RESULT_STALLED);
  usb_serial_task();
  assert(events[3] == 1 && reads == 1);
  tuh_cdc_umount_cb(0);

  now = UINT32_MAX - 1000;
  tuh_cdc_mount_cb(0);
  usb_serial_task();
  now += 3000; // Timeout must work across tick rollover.
  usb_serial_task();
  assert(events[4] == 1);
  complete(XFER_RESULT_SUCCESS); // Late completion must not reopen the port.
  usb_serial_task();
  assert(events[1] == 1 && reads == 1);
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
  puts("USB serial transport tests passed");
}
