#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p target/tests
for name in scanner usb_serial; do
  "${HOST_CC:-cc}" -std=c11 -Wall -Wextra -Werror -DSCANNER_NATIVE_TEST -Ic -Ivendor/tinyusb/src \
    "c/$name.c" "tests/${name}_test.c" -o "target/tests/$name-test"
  "target/tests/$name-test"
done
# rustc defaults to the native host, independent of Cargo's embedded target.
rustc --edition=2024 --test tests/firmware.rs -o target/tests/firmware-test
target/tests/firmware-test
python3 tests/analyze_reversals_test.py
