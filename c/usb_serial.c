#include "tusb.h"
#include "usb_serial.h"

// Bare-metal CDC ACM transport. All functions/callbacks run in main context.
// No Xeryon application commands are sent until its protocol is identified.
extern void scanner_serial_event(uint8_t event, uint8_t daddr, uint8_t itf);
extern void scanner_serial_rx(const uint8_t *data, uint32_t len);

enum { OFF, WAIT_ENUM, LINE_CODING, LINE_STATE, READY, FAILED };
enum { ATTACHED, OPENED, REMOVED, CONTROL_FAILED, CONTROL_TIMEOUT };
static uint8_t state, cdc_index, address, interface;
static bool busy;
static uint32_t started;
static uintptr_t generation;

void tuh_cdc_mount_cb(uint8_t idx) {
  tuh_itf_info_t info;
  if (!tuh_cdc_itf_get_info(idx, &info)) return;
  ++generation;
  cdc_index = idx;
  address = info.daddr;
  interface = info.desc.bInterfaceNumber;
  state = WAIT_ENUM;
  busy = false;
  started = tusb_time_millis_api();
  scanner_serial_event(ATTACHED, address, interface);
}

void tuh_cdc_umount_cb(uint8_t idx) {
  if (state == OFF || cdc_index != idx) return;
  ++generation;
  state = OFF;
  busy = false;
  scanner_serial_event(REMOVED, address, interface);
}

static void control_complete(tuh_xfer_t *xfer) {
  if (xfer->user_data != generation || state == OFF || state == FAILED) return;
  busy = false;
  if (xfer->result != XFER_RESULT_SUCCESS) {
    state = FAILED;
    scanner_serial_event(CONTROL_FAILED, address, interface);
    return;
  }
  if (state == LINE_CODING) {
    state = LINE_STATE;
    started = tusb_time_millis_api();
  } else if (state == LINE_STATE) {
    state = READY;
    scanner_serial_event(OPENED, address, interface);
  }
}

void usb_serial_task(void) {
  if (state == OFF || state == FAILED) return;
  const uint32_t now = tusb_time_millis_api();
  if (state != READY && (uint32_t)(now - started) >= 3000) {
    // Do not recycle a possibly live EP0 request; unplug to retry.
    state = FAILED;
    scanner_serial_event(CONTROL_TIMEOUT, address, interface);
    return;
  }
  if (state == WAIT_ENUM) {
    if (!tuh_mounted(address)) return;
    state = LINE_CODING;
    started = now;
  }
  if (busy) return;
  if (state == LINE_CODING) {
    const cdc_line_coding_t coding = {
      .bit_rate = 115200, .stop_bits = 0, .parity = 0, .data_bits = 8
    };
    busy = true;
    if (!tuh_cdc_set_line_coding(cdc_index, &coding, control_complete, generation)) busy = false;
  } else if (state == LINE_STATE) {
    // DTR marks the port open; no RTS/CTS flow control.
    busy = true;
    if (!tuh_cdc_set_control_line_state(cdc_index, CDC_CONTROL_LINE_STATE_DTR,
                                       control_complete, generation)) busy = false;
  } else if (state == READY) {
    // Bound work per loop so USB1 logging and the LED keep getting serviced.
    uint8_t data[64];
    const uint32_t count = tuh_cdc_read(cdc_index, data, sizeof(data));
    if (count) scanner_serial_rx(data, count);
  }
}
