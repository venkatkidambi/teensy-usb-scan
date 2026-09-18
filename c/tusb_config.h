#ifndef SCANNER_TUSB_CONFIG_H
#define SCANNER_TUSB_CONFIG_H

#define CFG_TUSB_MCU OPT_MCU_MIMXRT1XXX
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 0
#define CFG_TUH_ENABLED 1
#define CFG_TUH_MAX_SPEED OPT_MODE_HIGH_SPEED
#define CFG_TUH_DEVICE_MAX 1
#define CFG_TUH_HUB 0
#define CFG_TUH_CDC 1
#define CFG_TUH_CDC_RX_BUFSIZE 1024
#define CFG_TUH_CDC_TX_BUFSIZE 512
#define CFG_TUH_ENUMERATION_BUFSIZE 1024

// RT1062 M7 DTCM is USB-DMA accessible and uncached. The BSP puts .bss.*
// here. Do not move this section to cached OCRAM without adding maintenance.
#ifndef SCANNER_NATIVE_TEST
#define CFG_TUH_MEM_SECTION __attribute__((section(".bss.usb_dma")))
#endif
#define CFG_TUH_MEM_ALIGN __attribute__((aligned(32)))
#define CFG_TUH_MEM_DCACHE_ENABLE 0
#define CFG_TUD_MEM_DCACHE_ENABLE 0
#endif
