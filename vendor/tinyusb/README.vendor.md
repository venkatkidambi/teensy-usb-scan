# TinyUSB source subset

Sources from TinyUSB **0.21.0**, commit
`dae3f9a366bfcddbf9dcf1b48d7500286a849539`:
https://github.com/hathach/tinyusb/tree/0.21.0

Included: top-level stack files, common headers/FIFO, the bare-metal OS
abstraction, USB host core, EHCI, i.MX RT ChipIdea HS host support, and CDC host.
Only the C translation units listed in `build.rs` are compiled. The subset matches
`c/tusb_config.h`: one directly attached CDC ACM device on i.MX RT, with no RTOS,
hub, or TinyUSB device stack.

Unused RTOS backends, other ChipIdea platforms, the ChipIdea device driver, and
the hub implementation are omitted. The hub header and vendor-serial headers
remain because upstream includes them unconditionally, even with those drivers
disabled. The CDC host has one local extension: `tuh_cdc_xfer_error_cb` reports
failed transfers to the application before returning from `cdch_xfer_cb`.
It does not retry failures or report them as successful completions.
Other retained upstream files are unmodified. Changing the supported platform
or enabling additional drivers may require restoring files from the pinned commit.
Examples and the NXP SDK are not vendored.
Upstream MIT license is retained in LICENSE and individual source headers.
Project-specific adaptation lives in c/, outside this directory.
