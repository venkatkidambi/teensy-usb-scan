// Minimal compatibility surface consumed by TinyUSB's unmodified i.MX RT
// port. Startup, NVIC and clocks remain owned by the Rust BSP / cortex-m.
// Values and offsets checked against imxrt-ral 0.6.2 (imxrt1062).
#ifndef SCANNER_FSL_DEVICE_REGISTERS_H
#define SCANNER_FSL_DEVICE_REGISTERS_H
#include <stdint.h>
#include <stddef.h>

#define FSL_FEATURE_SOC_USBHS_COUNT 2
#define FSL_FEATURE_SOC_USBPHY_COUNT 2
#define USB1_BASE 0x402e0000u
#define USB2_BASE 0x402e0200u
typedef int32_t IRQn_Type;
#define USB_OTG1_IRQn 113
#define USB_OTG2_IRQn 112
void scanner_irq_enable(IRQn_Type irq);
void scanner_irq_disable(IRQn_Type irq);
#define NVIC_EnableIRQ scanner_irq_enable
#define NVIC_DisableIRQ scanner_irq_disable

// Only these PHY registers are used by the upstream disconnect workaround.
typedef struct {
  uint32_t reserved[13];
  volatile uint32_t CTRL_SET;
  volatile uint32_t CTRL_CLR;
} scanner_usbphy_t;
_Static_assert(offsetof(scanner_usbphy_t, CTRL_SET) == 0x34, "PHY layout");
_Static_assert(offsetof(scanner_usbphy_t, CTRL_CLR) == 0x38, "PHY layout");
#define USBPHY1 ((scanner_usbphy_t *)0x400d9000u)
#define USBPHY2 ((scanner_usbphy_t *)0x400da000u)
#define USBPHY_CTRL_ENHOSTDISCONDETECT_MASK (1u << 1)
#endif
