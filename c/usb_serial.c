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
static bool tx_pending;
static bool transfer_failed;
static uint32_t started;
static uintptr_t generation;
static bool timing;
// RUNNING-only ordered command timestamps, not controller acknowledgements.
static uint32_t tx_queued, tx_completed, queued_ms[2], completed_ms[2];
void usb_serial_progress(uint32_t *values) {
  values[0] = tx_queued; values[1] = tx_completed;
  values[2] = queued_ms[0]; values[3] = queued_ms[1];
  values[4] = completed_ms[0]; values[5] = completed_ms[1];
}
static uint32_t rx_peak, rx_backlog, service_gap, tx_max, last_service, tx_started;

bool usb_serial_rx_pending(void) {
  return state == READY && tuh_cdc_read_available(cdc_index) != 0;
}
void usb_serial_timing(bool on) {
  timing = on;
  if (on) {
    tx_queued = tx_completed = 0;
    queued_ms[0] = queued_ms[1] = completed_ms[0] = completed_ms[1] = 0;
  }
  rx_peak = rx_backlog = service_gap = tx_max = 0;
  last_service = tx_started = tusb_time_millis_api();
}
void usb_serial_stats(uint32_t *peak, uint32_t *backlog, uint32_t *gap, uint32_t *tx_ms) {
  *peak = rx_peak; *backlog = rx_backlog; *gap = service_gap; *tx_ms = tx_max;
}
static void observe_tx(uint32_t now) {
  if (timing && tx_pending && (uint32_t)(now - tx_started) > tx_max)
    tx_max = now - tx_started;
}

// FIFO space alone does not establish completion: flush removes bytes from the
// FIFO before the USB transfer finishes. Experiments require both to be idle.
bool usb_serial_tx_idle(void) {
  return state == READY && !transfer_failed && !tx_pending &&
         tuh_cdc_write_available(cdc_index) == CFG_TUH_CDC_TX_BUFSIZE;
}

bool usb_serial_tx_failed(void) { return transfer_failed; }

void tuh_cdc_xfer_error_cb(uint8_t idx, uint8_t ep_addr, xfer_result_t result) {
  (void)ep_addr;
  (void)result;
  if (state == READY && idx == cdc_index) transfer_failed = true;
}

void tuh_cdc_tx_complete_cb(uint8_t idx) {
  if (state != READY || idx != cdc_index) return;
  observe_tx(tusb_time_millis_api());
  if (timing) {
    ++tx_completed;
    completed_ms[0] = completed_ms[1];
    completed_ms[1] = tusb_time_millis_api();
  }
  // TinyUSB calls this BEFORE flushing the next queued packet. Manual SSPD/DPOS
  // and STOP can queue behind an in-flight command, so retain pending until the
  // final packet completes. A failed/stalled transfer never marks TX idle.
  tx_pending = tuh_cdc_write_available(idx) != CFG_TUH_CDC_TX_BUFSIZE;
}

// Enqueue a complete short command, never a partial command or a blocking write.
// Stop discards queued bytes; an already submitted USB packet cannot be recalled.
bool usb_serial_send(const uint8_t *data, uint32_t len, bool stop) {
  if (state != READY || !data || !len || len > 32) return false;
  // Latch real failures until reconnect. STOP remains eligible for best-effort
  // transmission; clearing a software flag cannot repair a failed USB endpoint.
  if (transfer_failed && !stop) return false;
  if (stop) tuh_cdc_write_clear(cdc_index);
  if (tuh_cdc_write_available(cdc_index) < len) return false;
  if (!tx_pending) tx_started = tusb_time_millis_api();
  tx_pending = true;
  if (tuh_cdc_write(cdc_index, data, len) != len) {
    tuh_cdc_write_clear(cdc_index);
    transfer_failed = true;
    return false;
  }
  if (timing) {
    ++tx_queued;
    queued_ms[0] = queued_ms[1];
    queued_ms[1] = tusb_time_millis_api();
  }
  tuh_cdc_write_flush(cdc_index);
  return true;
}

void tuh_cdc_mount_cb(uint8_t idx) {
  tuh_itf_info_t info;
  if (!tuh_cdc_itf_get_info(idx, &info)) return;
  ++generation;
  cdc_index = idx;
  address = info.daddr;
  interface = info.desc.bInterfaceNumber;
  state = WAIT_ENUM;
  busy = false;
  tx_pending = false;
  transfer_failed = false;
  started = tusb_time_millis_api();
  scanner_serial_event(ATTACHED, address, interface);
}

void tuh_cdc_umount_cb(uint8_t idx) {
  if (state == OFF || cdc_index != idx) return;
  ++generation;
  state = OFF;
  busy = false;
  tx_pending = false;
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
  if (timing) {
    if ((uint32_t)(now - last_service) > service_gap) service_gap = now - last_service;
    last_service = now;
    observe_tx(now);
  }
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
    tuh_cdc_write_flush(cdc_index);
    // Drain a bounded burst, then yield to pressure, motion and USB1/ABORT.
    // Never spin waiting for bytes; the foreground avoids WFI with RX pending.
    uint8_t data[64];
    const uint32_t available = tuh_cdc_read_available(cdc_index);
    if (timing && available > rx_peak) rx_peak = available;
    for (unsigned i = 0; i < 4; ++i) {
      const uint32_t count = tuh_cdc_read(cdc_index, data, sizeof(data));
      if (!count) break;
      scanner_serial_rx(data, count);
      if (count < sizeof(data)) break;
    }
    if (timing && usb_serial_rx_pending()) ++rx_backlog;
  }
}
