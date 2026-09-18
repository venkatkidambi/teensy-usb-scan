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

### Windows upload

Use Windows PowerShell 5.1 or PowerShell 7 with `upload.ps1`. Install Rust
(including its Visual Studio C++ build prerequisites), the ARM Rust target,
`llvm-tools-preview`, and `cargo-binutils` using the commands above. Install the
Windows **arm-none-eabi** Arm GNU Toolchain and put its `bin` directory on PATH.
Do not copy the macOS `.tools` directory to Windows.

The script requires **teensy_loader_cli.exe**, not the graphical `teensy.exe`.
PJRC provides the [command-line loader source and build instructions](https://www.pjrc.com/teensy/loader_cli.html)
(Windows builds use MinGW). `upload.config.json` stores the loader path and is
already set to `C:\msys64\home\vek19004\teensy_loader_cli\teensy_loader_cli.exe`.
Edit its `TeensyLoader` value if the executable moves (JSON requires doubled
backslashes). For normal uploads from PowerShell or Command Prompt, run:

```powershell
.\upload.cmd
```

The launcher invokes `upload.ps1` with an execution-policy bypass scoped to that
PowerShell process, so there is no repeated policy setup or persistent policy
change. Organization-enforced Group Policy still takes precedence.

An explicit loader argument overrides the config. If the config file is absent,
the script searches PATH for `teensy_loader_cli.exe`:

```powershell
.\upload.cmd -TeensyLoader 'C:\Tools\Teensy\teensy_loader_cli.exe'
```

You can also invoke PowerShell directly (equivalent to the launcher):

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\upload.ps1
```

The script builds, creates `target/firmware/firmware.hex`, then flashes using the
same options as `upload.sh`. It stops on any failed command, handles paths with
spaces, works from another directory, and restores the caller's working directory.
After upload, find the Teensy's COM port in Device Manager and open it in a serial
terminal at 115200 baud, 8-N-1, with no flow control. Windows upload and actuator
recognition have been confirmed by the user; the new config and `.cmd` launcher
still need a Windows run.

## Validation and limits

Release build, Cargo check, HEX conversion, native bridge tests, interrupt vectors,
and DMA-buffer placement were checked. The onboard LED blink has been confirmed
on hardware. The user also confirmed USB1 CDC logging on COM6 and recognition
of the Xeryon actuator through USB2. Actuator motion/control is not implemented.

### USB bring-up checklist

1. Upload this firmware with `./upload.sh` (macOS) or `.\upload.cmd` (Windows).
   Leave the host header empty initially.
2. Open the primary connector's CDC port. Within five seconds, expect
   `Scanner alive`, a `USB2 connected=false` register snapshot, and
   `No device descriptor available yet`. The LED should continue blinking.
3. Connect a known-working USB device directly to the host cable. Expect
   `USB2 connection detected`, VID/PID with `configured=false`, then
   `configured=true`. Optional strings may follow. A keyboard or mouse can be
   used to test enumeration; this firmware does not process input reports.
4. Unplug it and check for disconnection and removal messages, then reconnect
   and confirm it configures again. Repeat with the intended actuator.
5. Save the serial output from attachment through at least two five-second
   reports, including VID/PID and the raw register snapshots.

If no CDC port appears, first check the primary USB data cable and that this
firmware was uploaded. If the heartbeat reports arrive but `connected` stays
false with a device attached, check the host cable orientation, device power,
and host VBUS. `connected` reflects the controller's connection-status bit;
it does not prove configuration succeeded. If connected is true but there is
no descriptor, retain the IRQ count and register snapshots for diagnosis.
An IRQ count of zero alone is normal before any device is attached.
If VID/PID appears but `configured=true` never follows, enumeration has not
completed. A stopped LED indicates the main loop is no longer progressing.
The snapshots are read-only and do not clear controller status flags.

```sh
./tests/run.sh                   # native C bridge tests; needs a host C compiler
cargo fmt --check
```

Only one directly attached device is supported, with a configuration descriptor
up to 1024 bytes. Strings are optional and best-effort. There are no actuator
class drivers. Control-transfer timeout support is incomplete; a nonresponsive
device may require unplugging/reconnecting or resetting the board to retry.
