#ifndef SCANNER_USB_SERIAL_H
#define SCANNER_USB_SERIAL_H
#include <stdbool.h>
#include <stdint.h>

void usb_serial_task(void);
bool usb_serial_send(const uint8_t *data, uint32_t len, bool stop);

#endif
