# TinyUSB source subset

Unmodified sources from TinyUSB **0.21.0**, commit
`dae3f9a366bfcddbf9dcf1b48d7500286a849539`:
https://github.com/hathach/tinyusb/tree/0.21.0

Included: top-level stack files, common, OS abstraction, host, EHCI, and
ChipIdea HS directories, plus CDC host sources and their serial headers.
Only the C translation units listed in build.rs are compiled. The CDC ACM
driver is enabled; vendor-specific serial drivers remain disabled. Device
drivers, examples, and the NXP SDK are not vendored.
Upstream MIT license is retained in LICENSE and individual source headers.
Project-specific adaptation lives in c/, outside this directory.
