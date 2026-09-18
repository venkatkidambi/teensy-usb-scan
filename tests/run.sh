#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p target/tests
"${HOST_CC:-cc}" -std=c11 -Wall -Wextra -Werror -DSCANNER_NATIVE_TEST -Ic -Ivendor/tinyusb/src \
  c/scanner.c tests/scanner_test.c -o target/tests/scanner-test
target/tests/scanner-test
"${HOST_CC:-cc}" -std=c11 -Wall -Wextra -Werror -DSCANNER_NATIVE_TEST -Ic -Ivendor/tinyusb/src \
  c/usb_serial.c tests/usb_serial_test.c -o target/tests/usb-serial-test
target/tests/usb-serial-test
# rustc defaults to the native host, independent of Cargo's embedded target.
rustc --edition=2024 --test src/xeryon.rs -o target/tests/xeryon-test
target/tests/xeryon-test
rustc --edition=2024 --test src/protocol.rs -o target/tests/protocol-test
target/tests/protocol-test
