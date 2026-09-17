#include "tusb.h"
#include "host/hcd.h"

// Callbacks into Rust run in main context, except scanner_bus_event, which
// only updates an atomic counter. No logging or allocation in the USB ISR.
extern void scanner_bus_event(uint32_t event);
extern void scanner_device(uint8_t address, uint16_t vid, uint16_t pid,
                           uint8_t cls, uint8_t sub, uint8_t protocol, bool mounted);
extern void scanner_string(uint8_t kind, const uint8_t *utf16le, uint16_t bytes);
extern void scanner_removed(uint8_t address);

static tusb_desc_device_t device;
static uint8_t address;
static bool mounted;
static uint8_t string_index[3];
static uint8_t strings[3][254];
static uint16_t string_len[3];
static uint16_t language;
static uint8_t stage; // 0: language; 1..3: manufacturer/product/serial; 4: done
static bool busy;
static bool get_body;
static uint16_t requested_len;
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t string_buffer[256];

bool scanner_init(void) {
  const tusb_rhport_init_t init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_HIGH };
  return tusb_init(1, &init); // TinyUSB rhport 1 is RT1062 USB2.
}

void scanner_interrupt(void) {
  tuh_int_handler(1, true);
}

void tuh_event_hook_cb(uint8_t rhport, uint32_t event, bool in_isr) {
  (void)rhport;
  (void)in_isr;
  if (event == HCD_EVENT_DEVICE_ATTACH || event == HCD_EVENT_DEVICE_REMOVE) {
    scanner_bus_event(event);
  }
}

bool scanner_report(void) {
  if (!address) return false;
  scanner_device(address, device.idVendor, device.idProduct, device.bDeviceClass,
                 device.bDeviceSubClass, device.bDeviceProtocol, mounted);
  for (unsigned i = 0; i < 3; ++i) {
    if (string_len[i]) scanner_string(i, strings[i], string_len[i]);
  }
  return true;
}

void tuh_enum_descriptor_device_cb(uint8_t daddr, const tusb_desc_device_t *desc) {
  device = *desc;
  address = daddr;
  mounted = false;
  memset(string_len, 0, sizeof(string_len));
  scanner_report();
}

void tuh_mount_cb(uint8_t daddr) {
  address = daddr;
  mounted = true;
  string_index[0] = device.iManufacturer;
  string_index[1] = device.iProduct;
  string_index[2] = device.iSerialNumber;
  stage = 0;
  busy = false;
  get_body = false;
  language = 0;
  scanner_report();
}

void tuh_umount_cb(uint8_t daddr) {
  mounted = false;
  address = 0;
  busy = false;
  stage = 4;
  memset(string_len, 0, sizeof(string_len));
  scanner_removed(daddr);
}

static void string_complete(tuh_xfer_t *xfer) {
  busy = false;
  if (!mounted || xfer->daddr != address) return;
  const uint32_t n = xfer->actual_len;
  if (xfer->result != XFER_RESULT_SUCCESS || n < 2 ||
      string_buffer[1] != TUSB_DESC_STRING || string_buffer[0] < 2 ||
      (string_buffer[0] & 1)) {
    // Strings are optional. Do not prevent device-descriptor reporting.
    stage = stage == 0 ? 4 : stage + 1;
    get_body = false;
    return;
  }
  if (stage == 0) {
    if (n >= 4 && string_buffer[0] >= 4) {
      language = (uint16_t)string_buffer[2] | ((uint16_t)string_buffer[3] << 8);
      stage = 1;
    } else {
      stage = 4;
    }
    return;
  }
  if (!get_body) {
    requested_len = string_buffer[0];
    get_body = true;
    return;
  }
  // Validate the complete returned descriptor before copying or decoding.
  if (n >= string_buffer[0] && stage <= 3) {
    const unsigned slot = stage - 1;
    string_len[slot] = string_buffer[0] - 2u;
    memcpy(strings[slot], string_buffer + 2, string_len[slot]);
    scanner_string(slot, strings[slot], string_len[slot]);
  }
  get_body = false;
  ++stage;
}

void scanner_task(void) {
  tuh_task_ext(0, false);
  if (!mounted || busy || stage >= 4) return;
  if (stage && string_index[stage - 1] == 0) {
    ++stage;
    return;
  }
  // Fully asynchronous: serial polling and LED timing continue during requests.
  // The first request asks only for the length before requesting the body.
  const uint8_t index = stage ? string_index[stage - 1] : 0;
  const uint16_t len = stage == 0 ? 4 : (get_body ? requested_len : 2);
  busy = true;
  if (!tuh_descriptor_get_string(address, index, language, string_buffer, len,
                                 string_complete, 0)) {
    busy = false; // Retry next main-loop iteration if EP0 is occupied.
  }
}
