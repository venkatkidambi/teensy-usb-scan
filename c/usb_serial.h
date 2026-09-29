#ifndef SCANNER_USB_SERIAL_H
#define SCANNER_USB_SERIAL_H
#include <stdbool.h>
#include <stdint.h>

void usb_serial_task(void);
bool usb_serial_tx_idle(void);
bool usb_serial_tx_failed(void);
bool usb_serial_send(const uint8_t *data, uint32_t len, bool stop);
bool usb_serial_rx_pending(void);
void usb_serial_timing(bool on);
void usb_serial_progress(uint32_t *values);
void usb_serial_stats(uint32_t *peak, uint32_t *backlog, uint32_t *gap, uint32_t *tx_ms);

#endif
