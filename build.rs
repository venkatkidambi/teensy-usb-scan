use std::{env, path::PathBuf};

fn main() {
    for (name, maximum) in [
        ("SINE_ENVELOPE_MARGIN_NM", u32::MAX),
        ("SINE_ENVELOPE_MAX_AGE_MS", 1000),
    ] {
        println!("cargo:rerun-if-env-changed={name}");
        if let Ok(value) = env::var(name) {
            assert!(
                value.parse::<u32>().is_ok_and(|v| v > 0 && v <= maximum),
                "Invalid {name}"
            );
        }
    }
    println!("cargo:rerun-if-changed=c");
    println!("cargo:rerun-if-changed=vendor/tinyusb");
    println!("cargo:rerun-if-env-changed=ARM_GNU_TOOLCHAIN");

    let mut build = cc::Build::new();
    // A normal installation on PATH works too. This fallback is for the
    // portable Arm distribution unpacked locally (no administrator required).
    let local = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap())
        .join(".tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi");
    let toolchain = env::var_os("ARM_GNU_TOOLCHAIN")
        .map(PathBuf::from)
        .or_else(|| local.is_dir().then_some(local));
    if let Some(root) = toolchain {
        build.compiler(root.join("bin/arm-none-eabi-gcc"));
        build.archiver(root.join("bin/arm-none-eabi-ar"));
    }

    build
        .include("c")
        .include("vendor/tinyusb/src")
        .flag("-mcpu=cortex-m7")
        .flag("-mthumb")
        .flag("-mfpu=fpv5-d16")
        .flag("-mfloat-abi=hard")
        .flag("-ffreestanding")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .flag("-fno-common")
        .std("c11")
        .opt_level(2)
        .files([
            "c/scanner.c",
            "c/usb_serial.c",
            "vendor/tinyusb/src/tusb.c",
            "vendor/tinyusb/src/common/tusb_fifo.c",
            "vendor/tinyusb/src/class/cdc/cdc_host.c",
            "vendor/tinyusb/src/host/usbh.c",
            "vendor/tinyusb/src/portable/ehci/ehci.c",
            "vendor/tinyusb/src/portable/chipidea/ci_hs/hcd_ci_hs.c",
        ])
        .compile("tinyusb_host");
}
