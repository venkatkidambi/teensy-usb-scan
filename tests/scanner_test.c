// Native tests for our bridge; no MMIO or Teensy hardware is involved.
#include <assert.h>
#include <stdio.h>
#include "tusb.h"
#include "host/hcd.h"

bool scanner_init(void);
void scanner_task(void);
void scanner_interrupt(void);
bool scanner_report(void);

static tuh_xfer_cb_t pending;
static uint8_t *buffer;
static uint16_t length;
static uint8_t index_requested;
static uint16_t language_requested;
static unsigned string_calls, device_calls, remove_calls, events;
static bool configured;

void usb_serial_task(void) {}

bool tusb_rhport_init(uint8_t port, const tusb_rhport_init_t *init) {
  assert(port == 1 && init->role == TUSB_ROLE_HOST);
  return true;
}
void hcd_int_handler(uint8_t port, bool in_isr) { assert(port == 1 && in_isr); }
void tuh_task_ext(uint32_t timeout, bool in_isr) { assert(timeout == 0 && !in_isr); }
bool tuh_descriptor_get_string(uint8_t address, uint8_t index, uint16_t lang,
    void *buf, uint16_t len, tuh_xfer_cb_t cb, uintptr_t user) {
  (void)user;
  assert(address == 1 && !pending && len <= 256);
  pending = cb;
  buffer = buf;
  length = len;
  index_requested = index;
  language_requested = lang;
  return true;
}
void scanner_bus_event(uint32_t event) { assert(event <= 1); ++events; }
void scanner_device(uint8_t address, uint16_t vid, uint16_t pid,
    uint8_t cls, uint8_t sub, uint8_t proto, bool mounted) {
  assert(address == 1 && vid == 0x1234 && pid == 0xabcd);
  assert(cls == 0xff && sub == 2 && proto == 3);
  configured = mounted;
  ++device_calls;
}
void scanner_string(uint8_t kind, const uint8_t *bytes, uint16_t n) {
  assert(kind == 0 && n == 4 && bytes[0] == 'A' && bytes[2] == 'B');
  ++string_calls;
}
void scanner_removed(uint8_t address) { assert(address == 1); ++remove_calls; }

static void complete(const uint8_t *data, uint32_t size, tusb_xfer_result_t result) {
  assert(pending && size <= length);
  memcpy(buffer, data, size);
  tuh_xfer_t xfer = { .daddr = 1, .buffer = buffer, .actual_len = size, .result = result };
  tuh_xfer_cb_t cb = pending;
  pending = NULL;
  cb(&xfer);
}

int main(void) {
  assert(scanner_init());
  scanner_interrupt();
  tuh_event_hook_cb(1, HCD_EVENT_DEVICE_ATTACH, true);
  tuh_event_hook_cb(1, HCD_EVENT_XFER_COMPLETE, true);
  assert(events == 1);
  const tusb_desc_device_t desc = {
    .bLength = 18, .bDescriptorType = TUSB_DESC_DEVICE,
    .idVendor = 0x1234, .idProduct = 0xabcd,
    .bDeviceClass = 0xff, .bDeviceSubClass = 2, .bDeviceProtocol = 3,
    .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 0,
  };
  tuh_enum_descriptor_device_cb(1, &desc);
  assert(device_calls == 1 && !configured);
  tuh_mount_cb(1);
  assert(configured);
  scanner_task();
  assert(index_requested == 0 && length == 4);
  const uint8_t language[] = {4, 3, 9, 4};
  complete(language, sizeof(language), XFER_RESULT_SUCCESS);
  scanner_task();
  assert(index_requested == 1 && length == 2 && language_requested == 0x0409);
  const uint8_t header[] = {6, 3};
  complete(header, sizeof(header), XFER_RESULT_SUCCESS);
  scanner_task();
  assert(length == 6);
  const uint8_t name[] = {6, 3, 'A', 0, 'B', 0};
  complete(name, sizeof(name), XFER_RESULT_SUCCESS);
  assert(string_calls == 1);
  scanner_task();
  assert(index_requested == 2);
  // Malformed odd descriptor length must be ignored, not read past the buffer.
  const uint8_t malformed[] = {255, 3};
  complete(malformed, sizeof(malformed), XFER_RESULT_SUCCESS);
  scanner_task(); // absent serial index is skipped
  scanner_task();
  assert(!pending && string_calls == 1);
  scanner_report();
  assert(string_calls == 2); // cached strings are available to late serial clients
  tuh_umount_cb(1);
  assert(remove_calls == 1);
  unsigned before = device_calls;
  scanner_report();
  assert(device_calls == before);

  // Disconnect while an optional request is pending; late completion is ignored.
  tuh_enum_descriptor_device_cb(1, &desc);
  tuh_mount_cb(1);
  scanner_task();
  tuh_umount_cb(1);
  complete(language, sizeof(language), XFER_RESULT_FAILED);
  scanner_task();
  assert(!pending && remove_calls == 2);
  puts("scanner bridge tests passed");
}
