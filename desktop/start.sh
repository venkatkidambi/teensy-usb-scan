#!/usr/bin/env bash
set -euo pipefail
desktop_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_host="$(rustc -vV | sed -n 's/^host: //p')"
exec cargo run --manifest-path "$desktop_dir/Cargo.toml" --target "$desktop_host" --release --locked
