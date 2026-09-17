use std::{env, path::PathBuf};

fn main() {
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
        // Upstream compares an unsigned driver index against zero when all
        // class drivers are disabled, as intended for this descriptor scanner.
        .flag("-Wno-type-limits")
        .std("c11")
        .opt_level(2)
        .files([
            "c/scanner.c",
            "vendor/tinyusb/src/tusb.c",
            "vendor/tinyusb/src/common/tusb_fifo.c",
            "vendor/tinyusb/src/host/usbh.c",
            "vendor/tinyusb/src/portable/ehci/ehci.c",
            "vendor/tinyusb/src/portable/chipidea/ci_hs/hcd_ci_hs.c",
        ])
        .compile("tinyusb_host");
}
