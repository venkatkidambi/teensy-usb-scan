# Teensy 4.1 USB host scanner

Bare-metal Rust (`no_std`, `no_main`) plus the TinyUSB 0.21.0 C host stack.
Cargo builds everything; no Arduino, PlatformIO, or operating system is used.
The Rust `teensy4-bsp` supplies startup, clocks, pins, and the linker script.

The onboard LED toggles every 500 ms. The separate **USB2 host header** scans
one directly connected device and reports connection/disconnection, VID, PID,
device class/subclass/protocol, and optional manufacturer/product/serial strings.
The **primary USB1 connector** provides CDC serial logging to your Mac.
Descriptors are repeated every five seconds so you can open the terminal later.
This enumerates devices; it does not send actuator-specific control commands.

## Tools and build

Tested with Rust 1.95.0 and Arm GNU Toolchain 15.3.Rel1.

```sh
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools-preview
cargo install cargo-binutils --locked
brew install teensy_loader_cli
brew install --cask gcc-arm-embedded
```

The C compiler installer may require your administrator password in a terminal.
Alternatively, unpack the official [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
and set `ARM_GNU_TOOLCHAIN` to its root directory. This workspace already has a
portable toolchain in `.tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi`,
which `build.rs` detects automatically. `.tools/` is ignored and is not part of
the source distribution. A normal `arm-none-eabi-gcc`/`arm-none-eabi-ar` on PATH
also works. No downloads happen in the build script.

```sh
cargo check --locked
cargo build --release
```

Cargo defaults to `thumbv7em-none-eabihf`. Keep `Cargo.lock` and `vendor/` in
version control. The normal ELF path is
`target/thumbv7em-none-eabihf/release/teensy-usb-scan`.

## Upload and scan

1. Connect the primary USB connector to the Mac.
2. Run `./upload.sh`, then press the Teensy's Program button when the loader waits.
3. Find the CDC port with `ls /dev/cu.usbmodem*` and open the matching port:
   `screen /dev/cu.usbmodemXXXX 115200` (replace the path; CDC ignores baud rate).
4. Attach the actuator to the separate five-pin host header using the correct
   Teensy 4.1 USB host cable. For this first scanner, connect directly, without a hub.

Typical messages include `USB2 connection detected`, followed by
`VID=1234 PID=abcd class=ff subclass=02 protocol=03 configured=true` and available
strings. A device-level class of `00` means class information is in its interfaces.

The upload script builds, converts the ELF with `cargo objcopy` to
`target/firmware/firmware.hex`, and runs
`teensy_loader_cli --mcu=TEENSY41 -w -v` with that file. It discovers the Cargo
artifact rather than assuming a binary filename. USB software reboot is not implemented.

## Validation and limits

Release build, Cargo check, HEX conversion, native bridge tests, interrupt vectors,
and DMA-buffer placement were checked. **No hardware has been flashed or tested.**
Enumeration, electrical behavior, and timing still need verification on the board.

```sh
./tests/run.sh                   # native C bridge tests; needs a host C compiler
cargo fmt --check
```

Only one directly attached device is supported, with a configuration descriptor
up to 1024 bytes. Strings are optional and best-effort. There are no actuator
class drivers. Control-transfer timeout support is incomplete; a nonresponsive
device may require unplugging/reconnecting or resetting the board to retry.
