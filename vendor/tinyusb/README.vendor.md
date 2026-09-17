# TinyUSB source subset

Unmodified sources from TinyUSB **0.21.0**, commit
`dae3f9a366bfcddbf9dcf1b48d7500286a849539`:
https://github.com/hathach/tinyusb/tree/0.21.0

Included: top-level stack files, common, OS abstraction, host, EHCI, and
ChipIdea HS directories. Only the six C translation units listed in build.rs
are compiled. Device/class drivers, examples, and the NXP SDK are not vendored.
Upstream MIT license is retained in LICENSE and individual source headers.
Project-specific adaptation lives in c/, outside this directory.
