#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$project_dir"

command -v cargo >/dev/null || { echo "Install Rust/Cargo first." >&2; exit 1; }
command -v cargo-objcopy >/dev/null || { echo "Run: cargo install cargo-binutils --locked" >&2; exit 1; }
command -v teensy_loader_cli >/dev/null || { echo "Install teensy_loader_cli first." >&2; exit 1; }

cargo build --release --locked

# cargo-objcopy discovers the sole binary's ELF through Cargo metadata, including
# a custom CARGO_TARGET_DIR. No target artifact path or package name is assumed.
firmware_dir="$project_dir/target/firmware"
mkdir -p -- "$firmware_dir"
hex="$firmware_dir/firmware.hex"
cargo objcopy --release --locked -- -O ihex "$hex"
teensy_loader_cli --mcu=TEENSY41 -w -v "$hex"
